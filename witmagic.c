/* witmagic.c
 * Pure Data external: IMU motion processor (accel + gyro [+ mag] -> cleaned movement/orientation)
 * Uses Madgwick filter (IMU or AHRS with magnetometer), gravity subtraction, velocity integration.
 *
 * Inlet: list [ax ay az gx gy gz], or accel/gyro (3 floats each). Mag via "mag" message.
 * Units: accel in g (1g = gravity). Gyro list: values from [witsensor] scaled by turnrange/360 (forward [witsensor] status turnrange into witmagic). Mag in µT. Frame: x right, y front, z up.
 * Mag at lower rate OK: last mag used until new one arrives.
 * Messages: rate, xyzero, magcal, reset, zzero, fusiongain, veldecay, turnrange, magsmooth.
 * turnrange <float> — same as witsensor status (360 = degrees, 2π = rad, 1 = turns on sensor stream); only affects gyro list decoding. Angle outlet is always turns (roll/pitch/yaw).
 * reset: identity quaternion, zero speed/yaw_offset, clear accel bias and pending xyzero; last mag kept for AHRS (avoids IMU-only yaw drift until next mag). mag cal kept.
 * Outlets: (0) data — (1) status: mag_recal 0|1 outside mag cal box.
 *
 * Prefer raw witsensor mag + witmagic magcal; skip sensor-side magcal if firmware lags.
 * fusiongain 0.001–1 (default 0.99): Madgwick correction strength. zzero only shifts displayed yaw (euler); quat outlet is unchanged — use relative quat in Pd if you need heading-zero there too.
 * magsmooth (default 0.25): EMA on mag after cal — blend weight for new sample; 1 = off.
 * acclin = rotated accel − gravity; veldecay: speed leak per sample.
 *
 * This is free and unencumbered software released into the public domain.
 */

#include "m_pd.h"
#include <math.h>
#include <string.h>

#ifdef _WIN32
#include <windows.h>
#else
#include <sys/time.h>
#endif

#define DEG2RAD (3.14159265358979323846f / 180.0f)
#define RAD2DEG (180.0f / 3.14159265358979323846f)
#define G_TO_MM_S2  9806.65f         /* 1g = 9.80665 m/s² = 9806.65 mm/s² */
#define WITMAGIC_FUSIONGAIN      0.99f   /* sensor fusion gain (accel+mag correction) */
#define WITMAGIC_FUSIONGAIN_MIN  0.001f
#define WITMAGIC_FUSIONGAIN_MAX  1.0f
#define WITMAGIC_VELDECAY        0.99f   /* velocity decay per sample (0.5–0.9999) */

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
/* Internal euler is degrees; angle outlet is always in turns */
#define WITMAGIC_INTERNAL_DEG_TO_TURNS (1.0 / 360.0)
#define WITMAGIC_TURNRANGE_DEFAULT    360.0  /* matches witsensor default (degrees on stream) */
#define WITMAG_MAG_SMOOTH   0.25f   /* EMA: weight of new mag sample; 1 = no smoothing */

static t_class *witmagic_class;

typedef struct _witmagic {
    t_object x_obj;
    
    /* Madgwick state: quaternion q = w + x*i + y*j + z*k (sensor to world) */
    float qw, qx, qy, qz;
    float fusiongain;        /* sensor fusion gain (0.001–1 typical) */
    float veldecay;          /* velocity decay per sample (0.9–0.999 typical) */
    
    /* Calibration: accel bias subtracted before processing */
    float bias_ax, bias_ay, bias_az;
    int bias_valid;
    
    /* Integration state */
    float vx, vy, vz;        /* velocity in world frame (mm/s) */
    
    /* Timing */
    double last_time;
    int first_sample;
    int do_calibrate_next;    /* next list stores accel as bias */
    float rate_hz;            /* sample rate; 0 = use wall-clock dt */
    float last_ax, last_ay, last_az;  /* buffered for accel/gyro split input */
    float mag_x, mag_y, mag_z;        /* last magnetometer (normalized); used when mag_valid */
    int mag_valid;
    float mag_offset_x, mag_offset_y, mag_offset_z;   /* min-max center */
    float mag_scale_x, mag_scale_y, mag_scale_z;     /* min-max half-range */
    int mag_cal_valid;
    float mag_cal_min_x, mag_cal_min_y, mag_cal_min_z;
    float mag_cal_max_x, mag_cal_max_y, mag_cal_max_z;
    int mag_cal_count;
    int mag_cal_collecting;
    float mag_mag_sum;       /* sum |B| during magcal sweep (µT) */
    float mag_ref_mag;       /* avg |B| at calibration (µT) */
    int mag_outside_prev;    /* previous raw-sample outside cal box (for mag_recal edges) */
    float mag_smooth_alpha;  /* EMA blend for new mag (after cal); 1 = bypass */
    float mag_lp_x, mag_lp_y, mag_lp_z;
    int mag_lp_valid;
    float yaw_offset;        /* subtract from yaw so current heading = 0 (internal °) */
    double turnrange;        /* witsensor one-turn span (360, 2π, 1…); gyro_list = °/s × turnrange/360 */
    
    t_outlet *o_out;         /* data: quat angle acclin speed mag */
    t_outlet *o_status;      /* mag_recal 0|1 — suggest remagcal when 1 */
} t_witmagic;

/* Fast inverse sqrt - portable (no strict-aliasing) */
static inline float inv_sqrt(float x) {
    if (x <= 0.f) return 1.f;
    return 1.f / (float)sqrt((double)x);
}

/* Madgwick IMU update (no magnetometer). gyro in rad/s, accel as 3-vector (will be normalized). */
static void madgwick_imu_update(t_witmagic *x, float gx, float gy, float gz,
                                float ax, float ay, float az, float dt) {
    float q0 = x->qw, q1 = x->qx, q2 = x->qy, q3 = x->qz;
    float recip_norm;
    float s0, s1, s2, s3;
    float q_dot1, q_dot2, q_dot3, q_dot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2;
    float q0q0, q1q1, q2q2, q3q3;

    /* Rate of change of quaternion from gyroscope */
    q_dot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    q_dot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    q_dot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    q_dot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    if (dt <= 0.f) dt = 0.001f;

    /* Accelerometer feedback only if valid (non-zero) */
    if (ax != 0.f || ay != 0.f || az != 0.f) {
        recip_norm = inv_sqrt(ax * ax + ay * ay + az * az);
        ax *= recip_norm;
        ay *= recip_norm;
        az *= recip_norm;

        _2q0 = 2.f * q0; _2q1 = 2.f * q1; _2q2 = 2.f * q2; _2q3 = 2.f * q3;
        _4q0 = 4.f * q0; _4q1 = 4.f * q1; _4q2 = 4.f * q2;
        _8q1 = 8.f * q1; _8q2 = 8.f * q2;
        q0q0 = q0 * q0; q1q1 = q1 * q1; q2q2 = q2 * q2; q3q3 = q3 * q3;

        /* Gradient descent corrective step (IMU - no mag) */
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.f * q1q1 * q3 - _2q1 * ax + 4.f * q2q2 * q3 - _2q2 * ay;
        recip_norm = inv_sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        s0 *= recip_norm; s1 *= recip_norm; s2 *= recip_norm; s3 *= recip_norm;

        q_dot1 -= x->fusiongain * s0;
        q_dot2 -= x->fusiongain * s1;
        q_dot3 -= x->fusiongain * s2;
        q_dot4 -= x->fusiongain * s3;
    }

    /* Integrate */
    q0 += q_dot1 * dt;
    q1 += q_dot2 * dt;
    q2 += q_dot3 * dt;
    q3 += q_dot4 * dt;

    recip_norm = inv_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recip_norm; q1 *= recip_norm; q2 *= recip_norm; q3 *= recip_norm;
    /* Quaternion continuity: q and -q represent same rotation; avoid sign flip "jumps" */
    if (q0 * x->qw + q1 * x->qx + q2 * x->qy + q3 * x->qz < 0.f) {
        q0 = -q0; q1 = -q1; q2 = -q2; q3 = -q3;
    }
    x->qw = q0; x->qx = q1; x->qy = q2; x->qz = q3;
}

/* Madgwick AHRS update (accel + gyro + magnetometer). Uses last stored mag when mag_valid.
 * Mag corrects yaw drift. Falls back to IMU-only when mag invalid or zero. */
static void madgwick_ahrs_update(t_witmagic *x, float gx, float gy, float gz,
                                 float ax, float ay, float az, float dt) {
    float mx = x->mag_x, my = x->mag_y, mz = x->mag_z;
    if (!x->mag_valid || (mx == 0.f && my == 0.f && mz == 0.f)) {
        madgwick_imu_update(x, gx, gy, gz, ax, ay, az, dt);
        return;
    }
    float q0 = x->qw, q1 = x->qx, q2 = x->qy, q3 = x->qz;
    float recip_norm;
    float s0, s1, s2, s3;
    float q_dot1, q_dot2, q_dot3, q_dot4;
    float _2q0, _2q1, _2q2, _2q3;
    float _2q0mx, _2q0my, _2q0mz, _2q1mx;
    float _2q0q2, _2q2q3, _2bx, _4bx, _2bz, _4bz;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

    q_dot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    q_dot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    q_dot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    q_dot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    if (dt <= 0.f) dt = 0.001f;

    if (ax != 0.f || ay != 0.f || az != 0.f) {
        recip_norm = inv_sqrt(ax * ax + ay * ay + az * az);
        ax *= recip_norm; ay *= recip_norm; az *= recip_norm;
        recip_norm = inv_sqrt(mx * mx + my * my + mz * mz);
        mx *= recip_norm; my *= recip_norm; mz *= recip_norm;

        _2q0 = 2.f * q0; _2q1 = 2.f * q1; _2q2 = 2.f * q2; _2q3 = 2.f * q3;
        _2q0mx = 2.f * q0 * mx; _2q0my = 2.f * q0 * my; _2q0mz = 2.f * q0 * mz;
        _2q1mx = 2.f * q1 * mx;
        _2q0q2 = 2.f * q0 * q2; _2q2q3 = 2.f * q2 * q3;
        q0q0 = q0 * q0; q0q1 = q0 * q1; q0q2 = q0 * q2; q0q3 = q0 * q3;
        q1q1 = q1 * q1; q1q2 = q1 * q2; q1q3 = q1 * q3;
        q2q2 = q2 * q2; q2q3 = q2 * q3; q3q3 = q3 * q3;

        /* Reference direction of Earth's magnetic field (mag rotated to world frame) */
        {
            float hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
            float hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
            _2bx = (float)sqrt((double)(hx * hx + hy * hy));
            _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
        }
        _4bx = 2.f * _2bx; _4bz = 2.f * _2bz;

        /* Gradient descent (accel + mag) */
        s0 = -_2q2 * (2.f * q1q3 - _2q0q2 - ax) + _2q1 * (2.f * q0q1 + _2q2q3 - ay)
             - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx)
             + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my)
             + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s1 = _2q3 * (2.f * q1q3 - _2q0q2 - ax) + _2q0 * (2.f * q0q1 + _2q2q3 - ay)
             - 4.f * q1 * (1.f - 2.f * q1q1 - 2.f * q2q2 - az)
             + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx)
             + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my)
             + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s2 = -_2q0 * (2.f * q1q3 - _2q0q2 - ax) + _2q3 * (2.f * q0q1 + _2q2q3 - ay)
             - 4.f * q2 * (1.f - 2.f * q1q1 - 2.f * q2q2 - az)
             + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx)
             + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my)
             + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s3 = _2q1 * (2.f * q1q3 - _2q0q2 - ax) + _2q2 * (2.f * q0q1 + _2q2q3 - ay)
             + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx)
             + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my)
             + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        recip_norm = inv_sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        s0 *= recip_norm; s1 *= recip_norm; s2 *= recip_norm; s3 *= recip_norm;

        q_dot1 -= x->fusiongain * s0;
        q_dot2 -= x->fusiongain * s1;
        q_dot3 -= x->fusiongain * s2;
        q_dot4 -= x->fusiongain * s3;
    }

    q0 += q_dot1 * dt;
    q1 += q_dot2 * dt;
    q2 += q_dot3 * dt;
    q3 += q_dot4 * dt;

    recip_norm = inv_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recip_norm; q1 *= recip_norm; q2 *= recip_norm; q3 *= recip_norm;
    if (q0 * x->qw + q1 * x->qx + q2 * x->qy + q3 * x->qz < 0.f) {
        q0 = -q0; q1 = -q1; q2 = -q2; q3 = -q3;
    }
    x->qw = q0; x->qx = q1; x->qy = q2; x->qz = q3;
}

/* Quaternion to Euler (ZYX: yaw pitch roll) in degrees. Z-up: yaw=heading, pitch=tilt, roll=bank. */
static void quat_to_euler(float qw, float qx, float qy, float qz,
                          float *yaw, float *pitch, float *roll) {
    float sinp = 2.f * (qw * qy - qz * qx);
    if (sinp > 1.f) sinp = 1.f;
    if (sinp < -1.f) sinp = -1.f;
    *pitch = (float)asin((double)sinp) * RAD2DEG;
    float sinr_cosp = 2.f * (qw * qx + qy * qz);
    float cosr_cosp = 1.f - 2.f * (qx * qx + qy * qy);
    *roll = (float)atan2((double)sinr_cosp, (double)cosr_cosp) * RAD2DEG;
    float siny_cosp = 2.f * (qw * qz + qx * qy);
    float cosy_cosp = 1.f - 2.f * (qy * qy + qz * qz);
    *yaw = (float)atan2((double)siny_cosp, (double)cosy_cosp) * RAD2DEG;
}

/* Rotate vector v by quaternion q: v' = q * v * q^-1 (vector part only) */
static void quat_rotate_vector(float qw, float qx, float qy, float qz,
                               float vx, float vy, float vz,
                               float *out_x, float *out_y, float *out_z) {
    float tx = 2.f * (qy * vz - qz * vy);
    float ty = 2.f * (qz * vx - qx * vz);
    float tz = 2.f * (qx * vy - qy * vx);
    *out_x = vx + qw * tx + (qy * tz - qz * ty);
    *out_y = vy + qw * ty + (qz * tx - qx * tz);
    *out_z = vz + qw * tz + (qx * ty - qy * tx);
}

#ifdef _WIN32
static double get_time_sec(void) {
    LARGE_INTEGER freq, count;
    if (QueryPerformanceFrequency(&freq) && QueryPerformanceCounter(&count))
        return (double)count.QuadPart / (double)freq.QuadPart;
    return (double)GetTickCount() / 1000.0;
}
#else
static double get_time_sec(void) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (double)tv.tv_sec + (double)tv.tv_usec / 1e6;
}
#endif

static void witmagic_turnrange(t_witmagic *x, t_floatarg f) {
    if (f > 0.f)
        x->turnrange = (double)f;
}

/* Core processing: run pipeline for one sample. Calibration uses ax,ay,az when do_calibrate_next. */
static void witmagic_process(t_witmagic *x, float ax, float ay, float az,
                              float gx, float gy, float gz) {
    /* witsensor list gyro = (internal °/s) × (turnrange/360) → back to internal °/s */
    if (x->turnrange > 1e-20) {
        double gscale = 360.0 / x->turnrange;
        gx = (float)((double)gx * gscale);
        gy = (float)((double)gy * gscale);
        gz = (float)((double)gz * gscale);
    }
    float dt;
    if (x->rate_hz > 0.f) {
        dt = 1.f / x->rate_hz;
    } else {
        double now = get_time_sec();
        if (x->first_sample) {
            x->first_sample = 0;
            x->last_time = now;
            dt = 0.001f;
        } else {
            dt = (float)(now - x->last_time);
            x->last_time = now;
            if (dt <= 0.f || dt > 1.f) dt = 0.001f;
        }
    }

    /* Calibrate: store accel offset (sensor flat, Z up). bias = raw - [0,0,1] so we preserve gravity. */
    if (x->do_calibrate_next) {
        x->bias_ax = ax;
        x->bias_ay = ay;
        x->bias_az = az - 1.f;  /* offset from 1g; assumes Z-up when calibrating */
        x->bias_valid = 1;
        x->do_calibrate_next = 0;
        post("witmagic: bias (offset) set to %.4g %.4g %.4g", x->bias_ax, x->bias_ay, x->bias_az);
    }
    /* Subtract accel bias if calibrated */
    if (x->bias_valid) {
        ax -= x->bias_ax;
        ay -= x->bias_ay;
        az -= x->bias_az;
    }

    /* Gyro: deg/s -> rad/s */
    gx *= DEG2RAD;
    gy *= DEG2RAD;
    gz *= DEG2RAD;

    /* 1. Madgwick orientation filter (AHRS when mag valid, else IMU). Accel corrects pitch/roll; mag corrects yaw. */
    madgwick_ahrs_update(x, gx, gy, gz, ax, ay, az, dt);

    /* 2. Rotate accel into world frame (x/y horizontal, z vertical) */
    float aw_x, aw_y, aw_z;
    quat_rotate_vector(x->qw, x->qx, x->qy, x->qz, ax, ay, az, &aw_x, &aw_y, &aw_z);

    /* 3. Subtract gravity (accel in g, so gravity = [0,0,1]). acclin adapts with orientation;
     * higher fusiongain → faster quaternion → acclin responds faster to motion. */
    float ad_x = aw_x;
    float ad_y = aw_y;
    float ad_z = aw_z - 1.f;

    /* 4. Integrate speed with leak (acclin in g → speed in mm/s via G_TO_MM_S2) */
    x->vx += ad_x * dt * G_TO_MM_S2;
    x->vy += ad_y * dt * G_TO_MM_S2;
    x->vz += ad_z * dt * G_TO_MM_S2;
    x->vx *= x->veldecay;
    x->vy *= x->veldecay;
    x->vz *= x->veldecay;

    /* Output: single outlet with named selectors (use [route] in Pd to dispatch) */
    t_atom out[4];
    SETFLOAT(out + 0, x->qw);
    SETFLOAT(out + 1, x->qx);
    SETFLOAT(out + 2, x->qy);
    SETFLOAT(out + 3, x->qz);
    outlet_anything(x->o_out, gensym("quat"), 4, out);
    float yaw, pitch, roll;
    quat_to_euler(x->qw, x->qx, x->qy, x->qz, &yaw, &pitch, &roll);
    yaw -= x->yaw_offset;
    while (yaw > 180.f) yaw -= 360.f;
    while (yaw < -180.f) yaw += 360.f;
    /* euler: roll pitch yaw (x y z) — always turns */
    SETFLOAT(out + 0, (t_float)(roll * WITMAGIC_INTERNAL_DEG_TO_TURNS));
    SETFLOAT(out + 1, (t_float)(pitch * WITMAGIC_INTERNAL_DEG_TO_TURNS));
    SETFLOAT(out + 2, (t_float)(yaw * WITMAGIC_INTERNAL_DEG_TO_TURNS));
    outlet_anything(x->o_out, gensym("angle"), 3, out);
    SETFLOAT(out + 0, ad_x);
    SETFLOAT(out + 1, ad_y);
    SETFLOAT(out + 2, ad_z);
    outlet_anything(x->o_out, gensym("acclin"), 3, out);
    SETFLOAT(out + 0, x->vx);
    SETFLOAT(out + 1, x->vy);
    SETFLOAT(out + 2, x->vz);
    outlet_anything(x->o_out, gensym("speed"), 3, out);
    /* Calibrated mag (min-max normalized) */
    SETFLOAT(out + 0, x->mag_x);
    SETFLOAT(out + 1, x->mag_y);
    SETFLOAT(out + 2, x->mag_z);
    outlet_anything(x->o_out, gensym("mag"), 3, out);
}

/* List: [ax ay az gx gy gz] (accel g, gyro per witsensor turnrange). Mag via separate "mag" message. */
static void witmagic_list(t_witmagic *x, t_symbol *s, int argc, t_atom *argv) {
    (void)s;
    if (argc < 6) return;
    float ax = atom_getfloat(argv + 0);
    float ay = atom_getfloat(argv + 1);
    float az = atom_getfloat(argv + 2);
    float gx = atom_getfloat(argv + 3);
    float gy = atom_getfloat(argv + 4);
    float gz = atom_getfloat(argv + 5);
    witmagic_process(x, ax, ay, az, gx, gy, gz);
}

/* Accel: 3 floats (g). Store; no process until gyro arrives. */
static void witmagic_accel(t_witmagic *x, t_symbol *s, int argc, t_atom *argv) {
    (void)s;
    if (argc < 3) return;
    x->last_ax = atom_getfloat(argv + 0);
    x->last_ay = atom_getfloat(argv + 1);
    x->last_az = atom_getfloat(argv + 2);
    if (x->do_calibrate_next) {
        x->bias_ax = x->last_ax;
        x->bias_ay = x->last_ay;
        x->bias_az = x->last_az - 1.f;
        x->bias_valid = 1;
        x->do_calibrate_next = 0;
        post("witmagic: bias (offset) set to %.4g %.4g %.4g", x->bias_ax, x->bias_ay, x->bias_az);
    }
}

/* Mag: 3 floats (µT). Min-max calibrate if done. magcal 1/0: collect samples, compute offset+scale. */
static void witmagic_mag(t_witmagic *x, t_symbol *s, int argc, t_atom *argv) {
    (void)s;
    if (argc < 3) return;
    float mx = atom_getfloat(argv + 0);
    float my = atom_getfloat(argv + 1);
    float mz = atom_getfloat(argv + 2);
    if (x->mag_cal_collecting) {
        float bm = sqrtf(mx * mx + my * my + mz * mz);
        x->mag_mag_sum += bm;
        if (x->mag_cal_count == 0) {
            x->mag_cal_min_x = x->mag_cal_max_x = mx;
            x->mag_cal_min_y = x->mag_cal_max_y = my;
            x->mag_cal_min_z = x->mag_cal_max_z = mz;
        } else {
            if (mx < x->mag_cal_min_x) x->mag_cal_min_x = mx;
            if (mx > x->mag_cal_max_x) x->mag_cal_max_x = mx;
            if (my < x->mag_cal_min_y) x->mag_cal_min_y = my;
            if (my > x->mag_cal_max_y) x->mag_cal_max_y = my;
            if (mz < x->mag_cal_min_z) x->mag_cal_min_z = mz;
            if (mz > x->mag_cal_max_z) x->mag_cal_max_z = mz;
        }
        x->mag_cal_count++;
    } else {
        float raw_mx = mx, raw_my = my, raw_mz = mz;
        if (x->mag_cal_valid) {
            float sx = (x->mag_scale_x > 1e-6f) ? x->mag_scale_x : 1.f;
            float sy = (x->mag_scale_y > 1e-6f) ? x->mag_scale_y : 1.f;
            float sz = (x->mag_scale_z > 1e-6f) ? x->mag_scale_z : 1.f;
            mx = (mx - x->mag_offset_x) / sx;
            my = (my - x->mag_offset_y) / sy;
            mz = (mz - x->mag_offset_z) / sz;
        }
        /* EMA low-pass (after cal): out = a*new + (1-a)*prev; a=1 → no smoothing */
        {
            float a = x->mag_smooth_alpha;
            if (a < 1.f - 1e-6f) {
                if (x->mag_lp_valid) {
                    mx = a * mx + (1.f - a) * x->mag_lp_x;
                    my = a * my + (1.f - a) * x->mag_lp_y;
                    mz = a * mz + (1.f - a) * x->mag_lp_z;
                }
                x->mag_lp_x = mx;
                x->mag_lp_y = my;
                x->mag_lp_z = mz;
                x->mag_lp_valid = 1;
            } else {
                x->mag_lp_x = mx;
                x->mag_lp_y = my;
                x->mag_lp_z = mz;
                x->mag_lp_valid = 1;
            }
        }
        x->mag_x = mx;
        x->mag_y = my;
        x->mag_z = mz;
        x->mag_valid = 1;

        if (x->mag_cal_valid) {
            int outside = (raw_mx < x->mag_cal_min_x || raw_mx > x->mag_cal_max_x
                || raw_my < x->mag_cal_min_y || raw_my > x->mag_cal_max_y
                || raw_mz < x->mag_cal_min_z || raw_mz > x->mag_cal_max_z);
            if (outside != x->mag_outside_prev) {
                t_atom st;
                SETFLOAT(&st, outside ? 1.f : 0.f);
                outlet_anything(x->o_status, gensym("mag_recal"), 1, &st);
                x->mag_outside_prev = outside;
            }
        }
    }
}

static void witmagic_magcal(t_witmagic *x, t_floatarg f) {
    if (f != 0.f) {
        x->mag_cal_collecting = 1;
        x->mag_cal_count = 0;
        x->mag_mag_sum = 0.f;
        x->mag_cal_valid = 0;
        x->mag_lp_valid = 0;
        post("witmagic: magcal started — rotate sensor through all orientations, then magcal 0");
    } else {
        x->mag_cal_collecting = 0;
        if (x->mag_cal_count > 0) {
            x->mag_offset_x = 0.5f * (x->mag_cal_min_x + x->mag_cal_max_x);
            x->mag_offset_y = 0.5f * (x->mag_cal_min_y + x->mag_cal_max_y);
            x->mag_offset_z = 0.5f * (x->mag_cal_min_z + x->mag_cal_max_z);
            x->mag_scale_x = 0.5f * (x->mag_cal_max_x - x->mag_cal_min_x);
            x->mag_scale_y = 0.5f * (x->mag_cal_max_y - x->mag_cal_min_y);
            x->mag_scale_z = 0.5f * (x->mag_cal_max_z - x->mag_cal_min_z);
            if (x->mag_scale_x < 1e-6f) x->mag_scale_x = 1.f;
            if (x->mag_scale_y < 1e-6f) x->mag_scale_y = 1.f;
            if (x->mag_scale_z < 1e-6f) x->mag_scale_z = 1.f;
            x->mag_ref_mag = x->mag_mag_sum / (float)x->mag_cal_count;
            x->mag_outside_prev = 0;
            x->mag_cal_valid = 1;
            x->mag_lp_valid = 0;
            post("witmagic: mag min-max cal done — offset (%.1f %.1f %.1f) scale (%.1f %.1f %.1f) |B|_ref=%.1f µT (%d samples)",
                 x->mag_offset_x, x->mag_offset_y, x->mag_offset_z,
                 x->mag_scale_x, x->mag_scale_y, x->mag_scale_z,
                 x->mag_ref_mag, x->mag_cal_count);
        } else {
            post("witmagic: magcal stopped — no samples collected");
        }
    }
}

/* Gyro: 3 floats (deg/s). Process with last accel. */
static void witmagic_gyro(t_witmagic *x, t_symbol *s, int argc, t_atom *argv) {
    (void)s;
    if (argc < 3) return;
    float gx = atom_getfloat(argv + 0);
    float gy = atom_getfloat(argv + 1);
    float gz = atom_getfloat(argv + 2);
    witmagic_process(x, x->last_ax, x->last_ay, x->last_az, gx, gy, gz);
}

/* xyzero: when sensor is flat (Z up), set accel offset so pitch/roll → 0 */
static void witmagic_xyzero(t_witmagic *x) {
    x->vx = x->vy = x->vz = 0.f;
    x->do_calibrate_next = 1;
    post("witmagic: next sample will set accel offset (sensor flat, Z up)");
}

static void witmagic_rate(t_witmagic *x, t_floatarg f) {
    x->rate_hz = (f > 0.f && f <= 1000.f) ? f : 0.f;
    if (x->rate_hz == 0.f) x->first_sample = 1;  /* reinit wall-clock on next sample */
}

static void witmagic_fusiongain(t_witmagic *x, t_floatarg f) {
    if (f >= WITMAGIC_FUSIONGAIN_MIN && f <= WITMAGIC_FUSIONGAIN_MAX) x->fusiongain = f;
}

static void witmagic_magsmooth(t_witmagic *x, t_floatarg f) {
    float a = (float)f;
    x->mag_smooth_alpha = (a >= 1.f) ? 1.f : a;
    x->mag_lp_valid = 0;
}

static void witmagic_veldecay(t_witmagic *x, t_floatarg f) {
    if (f >= 0.5f && f <= 0.9999f) x->veldecay = f;
}

static void witmagic_anything(t_witmagic *x, t_symbol *s, int argc, t_atom *argv) {
    /* don't pass through — we output improved versions */
    if (s == gensym("angle") || s == gensym("speed") || s == gensym("disp") || s == gensym("quat"))
        return;
    outlet_anything(x->o_out, s, argc, argv);
}

static void witmagic_reset(t_witmagic *x) {
    x->qw = 1.f; x->qx = x->qy = x->qz = 0.f;
    x->vx = x->vy = x->vz = 0.f;
    x->yaw_offset = 0.f;
    x->first_sample = 1;
    x->last_time = 0.0;
    x->bias_ax = x->bias_ay = x->bias_az = 0.f;
    x->bias_valid = 0;
    x->do_calibrate_next = 0;
    /* Keep last mag + mag_valid: clearing forced IMU-only until next mag poll → bad yaw drift after reset. */
    x->mag_outside_prev = 0;
    x->mag_lp_valid = 0;
}

/* zzero: displayed yaw → 0 (subtract euler yaw); quaternion unchanged — quat outlet still absolute. */
static void witmagic_zzero(t_witmagic *x) {
    float yaw, pitch, roll;
    quat_to_euler(x->qw, x->qx, x->qy, x->qz, &yaw, &pitch, &roll);
    while (yaw > 180.f) yaw -= 360.f;
    while (yaw < -180.f) yaw += 360.f;
    x->yaw_offset = yaw;
    post("witmagic: yaw offset set to %.1f° (angle yaw 0; quat outlet unchanged)", yaw);
}

static void *witmagic_new(void) {
    t_witmagic *x = (t_witmagic *)pd_new(witmagic_class);
    x->qw = 1.f;
    x->qx = x->qy = x->qz = 0.f;
    x->fusiongain = WITMAGIC_FUSIONGAIN;
    x->veldecay = WITMAGIC_VELDECAY;
    x->bias_ax = x->bias_ay = x->bias_az = 0.f;
    x->bias_valid = 0;
    x->do_calibrate_next = 0;
    x->vx = x->vy = x->vz = 0.f;
    x->first_sample = 1;
    x->last_time = 0.0;
    x->rate_hz = 0.f;
    x->last_ax = x->last_ay = x->last_az = 0.f;
    x->mag_x = x->mag_y = x->mag_z = 0.f;
    x->mag_valid = 0;
    x->mag_offset_x = x->mag_offset_y = x->mag_offset_z = 0.f;
    x->mag_scale_x = x->mag_scale_y = x->mag_scale_z = 1.f;
    x->mag_cal_valid = 0;
    x->mag_cal_collecting = 0;
    x->mag_cal_count = 0;
    x->mag_mag_sum = 0.f;
    x->mag_ref_mag = 0.f;
    x->mag_outside_prev = 0;
    x->mag_smooth_alpha = WITMAG_MAG_SMOOTH;
    x->mag_lp_x = x->mag_lp_y = x->mag_lp_z = 0.f;
    x->mag_lp_valid = 0;
    x->yaw_offset = 0.f;
    x->turnrange = WITMAGIC_TURNRANGE_DEFAULT;

    x->o_out = outlet_new(&x->x_obj, &s_anything);
    x->o_status = outlet_new(&x->x_obj, &s_anything);
    return (void *)x;
}

void witmagic_setup(void) {
    witmagic_class = class_new(gensym("witmagic"),
        (t_newmethod)witmagic_new, 0, sizeof(t_witmagic), 0, 0);
    class_addlist(witmagic_class, witmagic_list);
    class_addmethod(witmagic_class, (t_method)witmagic_accel, gensym("accel"), A_GIMME, 0);
    class_addmethod(witmagic_class, (t_method)witmagic_gyro, gensym("gyro"), A_GIMME, 0);
    class_addmethod(witmagic_class, (t_method)witmagic_mag, gensym("mag"), A_GIMME, 0);
    class_addmethod(witmagic_class, (t_method)witmagic_rate, gensym("rate"), A_DEFFLOAT, 0);
    class_addmethod(witmagic_class, (t_method)witmagic_xyzero, gensym("xyzero"), 0);
    class_addmethod(witmagic_class, (t_method)witmagic_fusiongain, gensym("fusiongain"), A_DEFFLOAT, 0);
    class_addmethod(witmagic_class, (t_method)witmagic_magsmooth, gensym("magsmooth"), A_DEFFLOAT, 0);
    class_addmethod(witmagic_class, (t_method)witmagic_veldecay, gensym("veldecay"), A_DEFFLOAT, 0);
    class_addmethod(witmagic_class, (t_method)witmagic_turnrange, gensym("turnrange"), A_DEFFLOAT, 0);
    class_addmethod(witmagic_class, (t_method)witmagic_reset, gensym("reset"), 0);
    class_addmethod(witmagic_class, (t_method)witmagic_zzero, gensym("zzero"), 0);
    class_addmethod(witmagic_class, (t_method)witmagic_magcal, gensym("magcal"), A_DEFFLOAT, 0);
    class_addmethod(witmagic_class, (t_method)witmagic_anything, gensym("anything"), A_GIMME, 0);
}
