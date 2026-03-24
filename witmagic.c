/* witmagic.c
 * Pure Data external: IMU motion processor (accel + gyro [+ mag] -> cleaned movement/orientation)
 * Uses EKF AHRS (quaternion + gyro bias), gravity subtraction, velocity integration.
 *
 * Inlet: list [ax ay az gx gy gz], or accel/gyro (3 floats each). Mag via "mag" message.
 * Units: accel in g (1g = gravity), gyro in °/s, mag in µT (normalized internally). Frame: x right, y front, z up.
 * Mag at lower rate OK: last mag used until new one arrives. Use sensor-calibrated mag (no witmagic cal).
 * Messages: rate, xyzero, reset, zzero, ekf_gain, ekf_q, veldecay.
 *
 * ekf_gain (0.01–10) scales measurement trust; higher = trust accel/mag more.
 * ekf_q = process noise (gyro/bias uncertainty). Same acclin/speed pipeline as before.
 * rate 0 (default) = use wall-clock dt for gyro integration (recommended for BLE burst).
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
#define WITMAGIC_EKF_GAIN_DEF  2.0f  /* EKF measurement trust (higher = trust accel/mag more) */
#define WITMAGIC_EKF_GAIN_MIN  0.01f
#define WITMAGIC_EKF_GAIN_MAX  10.f
#define WITMAGIC_EKF_Q_DEF     0.01f /* process noise; higher = more correction from accel/mag */
#define WITMAGIC_VELDECAY_DEF  0.995f
static t_class *witmagic_class;

#define EKF_N 7   /* state: q0,q1,q2,q3, bx,by,bz */

typedef struct _witmagic {
    t_object x_obj;

    /* EKF state: quaternion q (sensor to world), gyro bias in rad/s */
    float qw, qx, qy, qz;
    float bias_gx, bias_gy, bias_gz;
    float P[EKF_N * EKF_N];   /* covariance 7x7 */
    float ekf_gain;
    float ekf_q;
    float veldecay;
    int ekf_initialized;       /* 0 until first accel init */

    /* Calibration: accel bias subtracted before processing */
    float bias_ax, bias_ay, bias_az;
    int bias_valid;

    /* Integration state */
    float vx, vy, vz;

    /* Timing */
    double last_time;
    int first_sample;
    int do_calibrate_next;
    float rate_hz;
    float last_ax, last_ay, last_az;
    float mag_x, mag_y, mag_z;
    int mag_valid;
    float yaw_offset;

    t_outlet *o_out;
} t_witmagic;

static inline float inv_sqrt(float x) {
    if (x <= 0.f) return 1.f;
    return 1.f / (float)sqrt((double)x);
}

/* Rotate vector v by quaternion q: v' = q * v * q^-1 (body to world) */
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

/* Rotate vector v from world to body: v_body = q* ⊗ v_world ⊗ q */
static void quat_rotate_inv(float qw, float qx, float qy, float qz,
                            float vx, float vy, float vz,
                            float *out_x, float *out_y, float *out_z) {
    quat_rotate_vector(qw, -qx, -qy, -qz, vx, vy, vz, out_x, out_y, out_z);
}

/* Invert 3x3 matrix M, result in Mi. Returns 0 on success. */
static int inv3x3(const float *M, float *Mi) {
    float det = M[0]*(M[4]*M[8]-M[5]*M[7]) - M[1]*(M[3]*M[8]-M[5]*M[6]) + M[2]*(M[3]*M[7]-M[4]*M[6]);
    if (fabs((double)det) < 1e-12) return -1;
    float idet = 1.f / det;
    Mi[0] = (M[4]*M[8]-M[5]*M[7]) * idet;
    Mi[1] = (M[2]*M[7]-M[1]*M[8]) * idet;
    Mi[2] = (M[1]*M[5]-M[2]*M[4]) * idet;
    Mi[3] = (M[5]*M[6]-M[3]*M[8]) * idet;
    Mi[4] = (M[0]*M[8]-M[2]*M[6]) * idet;
    Mi[5] = (M[2]*M[3]-M[0]*M[5]) * idet;
    Mi[6] = (M[3]*M[7]-M[4]*M[6]) * idet;
    Mi[7] = (M[1]*M[6]-M[0]*M[7]) * idet;
    Mi[8] = (M[0]*M[4]-M[1]*M[3]) * idet;
    return 0;
}

/* EKF prediction: integrate quaternion with corrected gyro, bias constant */
static void ekf_predict(t_witmagic *x, float gx, float gy, float gz, float dt) {
    float q0 = x->qw, q1 = x->qx, q2 = x->qy, q3 = x->qz;
    float wx = gx - x->bias_gx, wy = gy - x->bias_gy, wz = gz - x->bias_gz;

    /* Quaternion integration: q_dot = 0.5 * omega * q */
    float qd0 = 0.5f * (-q1*wx - q2*wy - q3*wz);
    float qd1 = 0.5f * ( q0*wx + q2*wz - q3*wy);
    float qd2 = 0.5f * ( q0*wy - q1*wz + q3*wx);
    float qd3 = 0.5f * ( q0*wz + q1*wy - q2*wx);

    q0 += qd0 * dt; q1 += qd1 * dt; q2 += qd2 * dt; q3 += qd3 * dt;
    float n = inv_sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 *= n; q1 *= n; q2 *= n; q3 *= n;
    if (q0 * x->qw + q1 * x->qx + q2 * x->qy + q3 * x->qz < 0.f) {
        q0 = -q0; q1 = -q1; q2 = -q2; q3 = -q3;
    }
    x->qw = q0; x->qx = q1; x->qy = q2; x->qz = q3;

    /* F matrix (simplified): identity + dt*A for quaternion part. P = F*P*F' + Q */
    /* Use first-order approx: F ≈ I + dt*df/dx. For bias: F is identity. */
    /* Process noise: quat ~ gyro ARW, bias ~ slow drift. Scale q_noise with sqrt(dt) for angle random walk. */
    float *P = x->P;
    float dt_safe = (dt > 0.f && dt <= 1.f) ? dt : 0.005f;
    float q_noise = x->ekf_q * (float)sqrt((double)dt_safe) * 2.f;  /* ~angle random walk */
    float b_noise = x->ekf_q * 0.02f;  /* allow bias to adapt when accel/mag disagree */
    P[0*7+0] += q_noise; P[1*7+1] += q_noise;
    P[2*7+2] += q_noise; P[3*7+3] += q_noise;
    P[4*7+4] += b_noise; P[5*7+5] += b_noise; P[6*7+6] += b_noise;
}

/* EKF update with 3D vector measurement. z = measured (normalized), h = predicted from state. */
static void ekf_update_vector(t_witmagic *x, const float *z, float bx, float by, float bz) {
    float q0 = x->qw, q1 = x->qx, q2 = x->qy, q3 = x->qz;
    float *P = x->P;
    const float eps = 1e-5f;
    float H[3 * EKF_N];  /* 3x7 */
    float h0, h1, h2;

    /* Predicted measurement: rotate reference [bx,by,bz] from world to body */
    quat_rotate_inv(q0, q1, q2, q3, bx, by, bz, &h0, &h1, &h2);

    /* Numerical Jacobian H = dh/dx. Perturb each state, recompute h. */
    for (int j = 0; j < EKF_N; j++) {
        float q0p = q0, q1p = q1, q2p = q2, q3p = q3;
        if (j == 0) q0p += eps; else if (j == 1) q1p += eps; else if (j == 2) q2p += eps; else if (j == 3) q3p += eps;
        float hp0, hp1, hp2;
        quat_rotate_inv(q0p, q1p, q2p, q3p, bx, by, bz, &hp0, &hp1, &hp2);
        H[0*7+j] = (hp0 - h0) / eps;
        H[1*7+j] = (hp1 - h1) / eps;
        H[2*7+j] = (hp2 - h2) / eps;
    }

    /* Innovation y = z - h */
    float y0 = z[0] - h0, y1 = z[1] - h1, y2 = z[2] - h2;

    /* S = H*P*H' + R. R = (1/ekf_gain) * I for measurement noise */
    float r = 0.1f / (x->ekf_gain > 0.01f ? x->ekf_gain : 0.01f);
    float HP[3*7];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 7; j++) {
            HP[i*7+j] = 0;
            for (int k = 0; k < 7; k++) HP[i*7+j] += H[i*7+k] * P[k*7+j];
        }
    float S[9];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) {
            S[i*3+j] = (i==j ? r : 0);
            for (int k = 0; k < 7; k++) S[i*3+j] += HP[i*7+k] * H[j*7+k];
        }

    float Si[9];
    if (inv3x3(S, Si) != 0) return;

    /* K = P*H'*inv(S) */
    float PHt[7*3];
    for (int i = 0; i < 7; i++)
        for (int j = 0; j < 3; j++) {
            PHt[i*3+j] = 0;
            for (int k = 0; k < 7; k++) PHt[i*3+j] += P[i*7+k] * H[j*7+k];
        }
    float K[7*3];
    for (int i = 0; i < 7; i++)
        for (int j = 0; j < 3; j++) {
            K[i*3+j] = 0;
            for (int k = 0; k < 3; k++) K[i*3+j] += PHt[i*3+k] * Si[k*3+j];
        }

    /* x = x + K*y */
    float dy0 = K[0*3+0]*y0 + K[0*3+1]*y1 + K[0*3+2]*y2;
    float dy1 = K[1*3+0]*y0 + K[1*3+1]*y1 + K[1*3+2]*y2;
    float dy2 = K[2*3+0]*y0 + K[2*3+1]*y1 + K[2*3+2]*y2;
    float dy3 = K[3*3+0]*y0 + K[3*3+1]*y1 + K[3*3+2]*y2;
    x->qw = q0 + dy0; x->qx = q1 + dy1; x->qy = q2 + dy2; x->qz = q3 + dy3;
    x->bias_gx += K[4*3+0]*y0 + K[4*3+1]*y1 + K[4*3+2]*y2;
    x->bias_gy += K[5*3+0]*y0 + K[5*3+1]*y1 + K[5*3+2]*y2;
    x->bias_gz += K[6*3+0]*y0 + K[6*3+1]*y1 + K[6*3+2]*y2;

    /* Renormalize quaternion */
    float n = inv_sqrt(x->qw*x->qw + x->qx*x->qx + x->qy*x->qy + x->qz*x->qz);
    x->qw *= n; x->qx *= n; x->qy *= n; x->qz *= n;

    /* P = (I - K*H)*P */
    float KH[7*7];
    for (int i = 0; i < 7; i++)
        for (int j = 0; j < 7; j++) {
            KH[i*7+j] = 0;
            for (int k = 0; k < 3; k++) KH[i*7+j] += K[i*3+k] * H[k*7+j];
        }
    float Pnew[49];
    for (int i = 0; i < 7; i++)
        for (int j = 0; j < 7; j++) {
            Pnew[i*7+j] = (i==j ? 1 : 0) - KH[i*7+j];
        }
    float Ptmp[49];
    for (int i = 0; i < 7; i++)
        for (int j = 0; j < 7; j++) {
            Ptmp[i*7+j] = 0;
            for (int k = 0; k < 7; k++) Ptmp[i*7+j] += Pnew[i*7+k] * P[k*7+j];
        }
    memcpy(P, Ptmp, sizeof(Ptmp));
}

/* EKF AHRS update: prediction + accel update + mag update (when valid) */
static void ekf_ahrs_update(t_witmagic *x, float gx, float gy, float gz,
                            float ax, float ay, float az, float dt) {
    if (dt <= 0.f) dt = 0.001f;

    /* First sample: init quaternion from accel when stationary (|accel| ≈ 1g) */
    if (!x->ekf_initialized && (ax != 0.f || ay != 0.f || az != 0.f)) {
        float acc_norm = (float)sqrt((double)(ax*ax + ay*ay + az*az));
        if (acc_norm > 0.5f && acc_norm < 1.5f) {
            float axn = ax / acc_norm, ayn = ay / acc_norm, azn = az / acc_norm;
            /* Accel in body = gravity direction. We want q s.t. q* ⊗ [0,0,1] ⊗ q = [axn,ayn,azn].
             * So [axn,ayn,azn] is "down" in body. One solution: q from identity to align z with accel. */
            float zx = 0.f, zy = 0.f, zz = 1.f;
            float dot = zx*axn + zy*ayn + zz*azn;
            if (dot < 0.9999f) {
                float rx = zy*azn - zz*ayn, ry = zz*axn - zx*azn, rz = zx*ayn - zy*axn;
                float rn = inv_sqrt(rx*rx + ry*ry + rz*rz);
                float angle = (float)acos((double)(dot < -1.f ? -1.f : (dot > 1.f ? 1.f : dot)));
                float ha = (float)sin((double)(angle * 0.5));
                x->qw = (float)cos((double)(angle * 0.5));
                x->qx = rx * rn * ha;
                x->qy = ry * rn * ha;
                x->qz = rz * rn * ha;
            }
        }
        x->ekf_initialized = 1;
    }

    ekf_predict(x, gx, gy, gz, dt);

    /* Accel update: gravity in world = [0,0,1] */
    if (ax != 0.f || ay != 0.f || az != 0.f) {
        float n = inv_sqrt(ax*ax + ay*ay + az*az);
        float z[3] = { ax*n, ay*n, az*n };
        ekf_update_vector(x, z, 0.f, 0.f, 1.f);
    }

    /* Mag update when valid */
    if (x->mag_valid) {
        float mx = x->mag_x, my = x->mag_y, mz = x->mag_z;
        if (mx != 0.f || my != 0.f || mz != 0.f) {
            float n = inv_sqrt(mx*mx + my*my + mz*mz);
            mx *= n; my *= n; mz *= n;
            /* Reference mag in world: project to horizontal like Madgwick */
            float hx, hy, hz;
            quat_rotate_vector(x->qw, x->qx, x->qy, x->qz, mx, my, mz, &hx, &hy, &hz);
            float bx = (float)sqrt((double)(hx*hx + hy*hy));
            float bz = hz;
            if (bx < 1e-6f) bx = 1e-6f;
            float z[3] = { mx, my, mz };
            ekf_update_vector(x, z, bx, 0.f, bz);
        }
    }
}

/* Quaternion to Euler (ZYX) in degrees */
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

static void witmagic_process(t_witmagic *x, float ax, float ay, float az,
                            float gx, float gy, float gz) {
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

    if (x->do_calibrate_next) {
        x->bias_ax = ax;
        x->bias_ay = ay;
        x->bias_az = az - 1.f;
        x->bias_valid = 1;
        x->do_calibrate_next = 0;
        post("witmagic: bias (offset) set to %.4g %.4g %.4g", x->bias_ax, x->bias_ay, x->bias_az);
    }
    if (x->bias_valid) {
        ax -= x->bias_ax;
        ay -= x->bias_ay;
        az -= x->bias_az;
    }

    gx *= DEG2RAD;
    gy *= DEG2RAD;
    gz *= DEG2RAD;

    ekf_ahrs_update(x, gx, gy, gz, ax, ay, az, dt);

    float aw_x, aw_y, aw_z;
    quat_rotate_vector(x->qw, x->qx, x->qy, x->qz, ax, ay, az, &aw_x, &aw_y, &aw_z);

    float ad_x = aw_x;
    float ad_y = aw_y;
    float ad_z = aw_z - 1.f;

    x->vx += ad_x * dt * G_TO_MM_S2;
    x->vy += ad_y * dt * G_TO_MM_S2;
    x->vz += ad_z * dt * G_TO_MM_S2;
    x->vx *= x->veldecay;
    x->vy *= x->veldecay;
    x->vz *= x->veldecay;

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
    SETFLOAT(out + 0, roll);
    SETFLOAT(out + 1, pitch);
    SETFLOAT(out + 2, yaw);
    outlet_anything(x->o_out, gensym("angle"), 3, out);
    SETFLOAT(out + 0, ad_x);
    SETFLOAT(out + 1, ad_y);
    SETFLOAT(out + 2, ad_z);
    outlet_anything(x->o_out, gensym("acclin"), 3, out);
    SETFLOAT(out + 0, x->vx);
    SETFLOAT(out + 1, x->vy);
    SETFLOAT(out + 2, x->vz);
    outlet_anything(x->o_out, gensym("speed"), 3, out);
    SETFLOAT(out + 0, x->mag_x);
    SETFLOAT(out + 1, x->mag_y);
    SETFLOAT(out + 2, x->mag_z);
    outlet_anything(x->o_out, gensym("mag"), 3, out);
}

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

static void witmagic_mag(t_witmagic *x, t_symbol *s, int argc, t_atom *argv) {
    (void)s;
    if (argc < 3) return;
    x->mag_x = atom_getfloat(argv + 0);
    x->mag_y = atom_getfloat(argv + 1);
    x->mag_z = atom_getfloat(argv + 2);
    x->mag_valid = 1;
}

static void witmagic_gyro(t_witmagic *x, t_symbol *s, int argc, t_atom *argv) {
    (void)s;
    if (argc < 3) return;
    float gx = atom_getfloat(argv + 0);
    float gy = atom_getfloat(argv + 1);
    float gz = atom_getfloat(argv + 2);
    witmagic_process(x, x->last_ax, x->last_ay, x->last_az, gx, gy, gz);
}

static void witmagic_xyzero(t_witmagic *x) {
    x->vx = x->vy = x->vz = 0.f;
    x->do_calibrate_next = 1;
    post("witmagic: next sample will set accel offset (sensor flat, Z up)");
}

static void witmagic_rate(t_witmagic *x, t_floatarg f) {
    x->rate_hz = (f > 0.f && f <= 1000.f) ? f : 0.f;
    if (x->rate_hz == 0.f) x->first_sample = 1;
}

static void witmagic_ekf_gain(t_witmagic *x, t_floatarg f) {
    if (f >= WITMAGIC_EKF_GAIN_MIN && f <= WITMAGIC_EKF_GAIN_MAX) x->ekf_gain = f;
}

static void witmagic_ekf_q(t_witmagic *x, t_floatarg f) {
    if (f > 0.f) x->ekf_q = f;
}

static void witmagic_veldecay(t_witmagic *x, t_floatarg f) {
    if (f >= 0.5f && f <= 0.9999f) x->veldecay = f;
}

static void witmagic_anything(t_witmagic *x, t_symbol *s, int argc, t_atom *argv) {
    if (s == gensym("angle") || s == gensym("speed") || s == gensym("disp") || s == gensym("quat"))
        return;
    outlet_anything(x->o_out, s, argc, argv);
}

static void witmagic_reset(t_witmagic *x) {
    x->qw = 1.f; x->qx = x->qy = x->qz = 0.f;
    x->bias_gx = x->bias_gy = x->bias_gz = 0.f;
    x->vx = x->vy = x->vz = 0.f;
    x->yaw_offset = 0.f;
    x->first_sample = 1;
    x->ekf_initialized = 0;
    /* Reset P to initial */
    for (int i = 0; i < EKF_N * EKF_N; i++) x->P[i] = 0.f;
    for (int i = 0; i < EKF_N; i++) x->P[i * EKF_N + i] = 0.01f;
}

static void witmagic_zzero(t_witmagic *x) {
    float yaw, pitch, roll;
    quat_to_euler(x->qw, x->qx, x->qy, x->qz, &yaw, &pitch, &roll);
    x->yaw_offset = yaw;
    post("witmagic: yaw offset set to %.1f° (current heading = 0)", yaw);
}

static void *witmagic_new(void) {
    t_witmagic *x = (t_witmagic *)pd_new(witmagic_class);
    x->qw = 1.f;
    x->qx = x->qy = x->qz = 0.f;
    x->bias_gx = x->bias_gy = x->bias_gz = 0.f;
    x->ekf_gain = WITMAGIC_EKF_GAIN_DEF;
    x->ekf_q = WITMAGIC_EKF_Q_DEF;
    x->veldecay = WITMAGIC_VELDECAY_DEF;
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
    x->yaw_offset = 0.f;
    x->ekf_initialized = 0;
    for (int i = 0; i < EKF_N * EKF_N; i++) x->P[i] = 0.f;
    for (int i = 0; i < EKF_N; i++) x->P[i * EKF_N + i] = 0.01f;

    x->o_out = outlet_new(&x->x_obj, &s_anything);
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
    class_addmethod(witmagic_class, (t_method)witmagic_ekf_gain, gensym("ekf_gain"), A_DEFFLOAT, 0);
    class_addmethod(witmagic_class, (t_method)witmagic_ekf_q, gensym("ekf_q"), A_DEFFLOAT, 0);
    class_addmethod(witmagic_class, (t_method)witmagic_veldecay, gensym("veldecay"), A_DEFFLOAT, 0);
    class_addmethod(witmagic_class, (t_method)witmagic_reset, gensym("reset"), 0);
    class_addmethod(witmagic_class, (t_method)witmagic_zzero, gensym("zzero"), 0);
    class_addmethod(witmagic_class, (t_method)witmagic_anything, gensym("anything"), A_GIMME, 0);
}
