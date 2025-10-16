/* pd-witsensor-ble.c
 * PureData external for WIT BWT901BLE5.0 sensor with BLE support
 * based on simpleBLE library
 * 
 * This is free and unencumbered software released into the public domain.
 * For more information, please refer to <https://unlicense.org>
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

// Platform-specific includes
#ifdef _WIN32
    #include <windows.h>
    #define usleep(x) Sleep((x)/1000)  // usleep takes microseconds, Sleep takes milliseconds
#else
    #include <unistd.h>
    #include <pthread.h>
#endif

#include "m_pd.h"

// BLE includes
#include "witsensor_ble_simpleble.h"

#define WITSENSOR_MAJOR_VERSION 0
#define WITSENSOR_MINOR_VERSION 2
#define WITSENSOR_BUGFIX_VERSION 0

#define MAX_DEVICES 20
#define BUFFER_SIZE 256
#define PACKET_SIZE 20

// WIT sensor UUIDs
#define WIT_SERVICE_UUID "0000ffe5-0000-1000-8000-00805f9a34fb"
#define WIT_CHAR_READ_UUID "0000ffe4-0000-1000-8000-00805f9a34fb"
#define WIT_CHAR_WRITE_UUID "0000ffe9-0000-1000-8000-00805f9a34fb"

typedef struct _witsensor {
    t_object x_obj;
    
    // BLE connection
    int is_connected;
    int is_scanning;
    char device_name[64];
    char device_address[32];
    
    // Data buffers
    unsigned char temp_bytes[PACKET_SIZE];
    int temp_bytes_count;
    unsigned char data_buffer[BUFFER_SIZE];
    
    // Sensor data
    float accel_x, accel_y, accel_z;
    float gyro_x, gyro_y, gyro_z;
    float angle_x, angle_y, angle_z;
    float quat_w, quat_x, quat_y, quat_z;
    // Optional streaming variants per output_mode
    float disp_x, disp_y, disp_z;
    float speed_x, speed_y, speed_z;
    unsigned short ts_lo, ts_hi;
    int use_disp_speed;          // 0: accel/gyro, 1: disp/speed
    int use_timestamp;   // 0: angle trio, 1: timestamp(+rz)

    // Threading (legacy - not currently used)
#ifndef _WIN32
    pthread_t scan_thread;
    pthread_t data_thread;
#endif
    int should_stop;
    
    // PureData outlets
    t_outlet *data_out;
    t_outlet *status_out;
    
    // Clock for polling
    t_clock *poll_clock;
    t_float poll_interval;
    t_symbol *poll_type;  // "quat", "mag", "battery", "temp", etc.
    
    // Tracked sensor state
    int axis_mode;       // 6 or 9
    int output_mode;     // AGPVSEL 0..3
    
    // BLE specific
    witsensor_ble_simpleble_t *ble_data;
    
    // Pd instance for pd_queue_mess
    t_pdinstance *pd_instance;
    // Autoconnect state: pending_target == NULL → none, "*" → any WIT, else exact match
    t_symbol *pending_target;
    // Dedupe device announcements per scan
    char **seen_ids;
    unsigned long seen_count;
    
} t_witsensor;

t_class *witsensor_class;

// Forward declarations
static void witsensor_scan_devices(t_witsensor *x);
static void witsensor_get_scan_results(t_witsensor *x);
static void witsensor_connect(t_witsensor *x, t_symbol *s, int argc, t_atom *argv);
static void witsensor_disconnect(t_witsensor *x);
static void witsensor_process_register_response(t_witsensor *x, unsigned char *data, int length);
static void witsensor_process_streaming_data(t_witsensor *x, unsigned char *data, int length);
static void witsensor_send_sensor_data(t_witsensor *x);
static void witsensor_send_quaternion_data(t_witsensor *x);
static void witsensor_poll_tick(t_witsensor *x);
static void witsensor_battery(t_witsensor *x);
static void witsensor_temp(t_witsensor *x);
static void witsensor_mag(t_witsensor *x);
static void witsensor_quat(t_witsensor *x);
static void witsensor_save(t_witsensor *x);
static void witsensor_restore(t_witsensor *x);
static void witsensor_read_version(t_witsensor *x);
static void witsensor_read_time(t_witsensor *x);
static void witsensor_reset(t_witsensor *x);
static void witsensor_setname(t_witsensor *x, t_symbol *s, int argc, t_atom *argv);
// pd_queue_mess marshaling
typedef struct _queued_output { 
    t_symbol *msg; 
    int argc; 
    t_atom argv[4]; 
} t_queued_output;

static void witsensor_pd_output_handler(t_pd *obj, void *data);
static void witsensor_ble_data_callback(void *user_data, unsigned char *data, int length);
void witsensor_pd_scan_complete_handler(t_pd *obj, void *data);
// New Pd-thread handlers for status output
typedef struct _queued_flag { int value; } t_queued_flag;
typedef struct _queued_device { char *tag; char *addr; char *id; } t_queued_device;
void witsensor_pd_scanning_handler(t_pd *obj, void *data);
void witsensor_pd_device_found_handler(t_pd *obj, void *data);
void witsensor_pd_connected_handler(t_pd *obj, void *data);

// BLE data callback function
static void witsensor_ble_data_callback(void *user_data, unsigned char *data, int length) {
    t_witsensor *x = (t_witsensor *)user_data;
    if (!x) return;
    if (length <= 0 || length > 64) return;
    
    // Process any register read response (0x71) immediately to avoid queue flooding
    if (data[0] == 0x55 && data[1] == 0x71 && length >= 6) {
        witsensor_process_register_response(x, data, length);
        return;
    }
    
    // Process streaming data (0x61) - parse on BLE thread, queue for Pd thread
    if (data[0] == 0x55 && data[1] == 0x61 && length >= 20) {
        witsensor_process_streaming_data(x, data, length);
        // Queue the output for immediate Pd thread processing
        t_queued_output *out = (t_queued_output *)malloc(sizeof(t_queued_output));
        if (out) {
            out->msg = gensym("streaming");
            out->argc = 0;
            pd_queue_mess(x->pd_instance, (t_pd *)x, out, witsensor_pd_output_handler);
        }
        return;
    }
    return;
}

// Process register read responses immediately on BLE thread (thread-safe for register reads)
static void witsensor_process_register_response(t_witsensor *x, unsigned char *data, int length) {
    if (!x || !data || length < 6) return;
    
    unsigned char start = data[2];
    if (start == 0x64) {
        // Battery centivolts in first word (little-endian)
        uint16_t vraw = (uint16_t)(data[4] | (data[5] << 8));
        float volts = (float)vraw / 100.0f;
        int pct = 0;
        if (vraw > 396) pct = 100;
        else if (vraw >= 393) pct = 90;
        else if (vraw >= 387) pct = 75;
        else if (vraw >= 382) pct = 60;
        else if (vraw >= 379) pct = 50;
        else if (vraw >= 377) pct = 40;
        else if (vraw >= 373) pct = 30;
        else if (vraw >= 370) pct = 20;
        else if (vraw >= 368) pct = 15;
        else if (vraw >= 350) pct = 10;
        else if (vraw >= 340) pct = 5;
        else pct = 0;
        
        // Queue the output for Pd thread (thread-safe)
        t_queued_output *out = (t_queued_output *)malloc(sizeof(t_queued_output));
        if (out) {
            out->msg = gensym("battery");
            out->argc = 2;
            SETFLOAT(&out->argv[0], volts);
            SETFLOAT(&out->argv[1], (t_float)pct);
            pd_queue_mess(x->pd_instance, (t_pd *)x, out, witsensor_pd_output_handler);
        }
        return;
    }
    if (start == 0x40 && length >= 6) {
        // Temperature: assume first word is in centi-degC
        uint16_t traw = (uint16_t)(data[4] | (data[5] << 8));
        float degC = (float)((int16_t)traw) / 100.0f;
        
        // Queue the output for Pd thread (thread-safe)
        t_queued_output *out = (t_queued_output *)malloc(sizeof(t_queued_output));
        if (out) {
            out->msg = gensym("temp");
            out->argc = 1;
            SETFLOAT(&out->argv[0], (t_float)degC);
            pd_queue_mess(x->pd_instance, (t_pd *)x, out, witsensor_pd_output_handler);
        }
        return;
    }
    if (start == 0x3A && length >= 10) {
        // Magnetic field: three words; convert to uT via /150
        int16_t mx = (int16_t)((data[5] << 8) | data[4]);
        int16_t my = (int16_t)((data[7] << 8) | data[6]);
        int16_t mz = (int16_t)((data[9] << 8) | data[8]);
        
        // Queue the output for Pd thread (thread-safe)
        t_queued_output *out = (t_queued_output *)malloc(sizeof(t_queued_output));
        if (out) {
            out->msg = gensym("mag");
            out->argc = 3;
            SETFLOAT(&out->argv[0], (t_float)((float)mx / 150.0f));
            SETFLOAT(&out->argv[1], (t_float)((float)my / 150.0f));
            SETFLOAT(&out->argv[2], (t_float)((float)mz / 150.0f));
            pd_queue_mess(x->pd_instance, (t_pd *)x, out, witsensor_pd_output_handler);
        }
        return;
    }
    if (start == 0x51 && length >= 12) {
        // Quaternion from register page starting at 0x51: four int16 words
        int16_t q0 = (int16_t)((data[5] << 8) | data[4]);
        int16_t q1 = (int16_t)((data[7] << 8) | data[6]);
        int16_t q2 = (int16_t)((data[9] << 8) | data[8]);
        int16_t q3 = (int16_t)((data[11] << 8) | data[10]);
        
        // Store in object (thread-safe for simple assignments)
        x->quat_w = (float)q0 / 32768.0f;
        x->quat_x = (float)q1 / 32768.0f;
        x->quat_y = (float)q2 / 32768.0f;
        x->quat_z = (float)q3 / 32768.0f;
        
        // Queue the output for Pd thread (thread-safe)
        t_queued_output *out = (t_queued_output *)malloc(sizeof(t_queued_output));
        if (out) {
            out->msg = gensym("quat");
            out->argc = 0;
            pd_queue_mess(x->pd_instance, (t_pd *)x, out, witsensor_pd_output_handler);
        }
        return;
    }
    if (start == 0x2E && length >= 6) {
        uint16_t v1 = (uint16_t)(data[4] | (data[5] << 8));
        t_queued_output *out = (t_queued_output *)malloc(sizeof(t_queued_output));
        if (out) {
            out->msg = gensym("version1");
            out->argc = 1;
            SETFLOAT(&out->argv[0], (t_float)v1);
            pd_queue_mess(x->pd_instance, (t_pd *)x, out, witsensor_pd_output_handler);
        }
        return;
    }
    if (start == 0x2F && length >= 6) {
        uint16_t v2 = (uint16_t)(data[4] | (data[5] << 8));
        t_queued_output *out = (t_queued_output *)malloc(sizeof(t_queued_output));
        if (out) {
            out->msg = gensym("version2");
            out->argc = 1;
            SETFLOAT(&out->argv[0], (t_float)v2);
            pd_queue_mess(x->pd_instance, (t_pd *)x, out, witsensor_pd_output_handler);
        }
        return;
    }
    if (start == 0x30 && length >= 6) {
        uint16_t yymm = (uint16_t)(data[4] | (data[5] << 8));
        t_queued_output *out = (t_queued_output *)malloc(sizeof(t_queued_output));
        if (out) {
            out->msg = gensym("time_yymm");
            out->argc = 1;
            SETFLOAT(&out->argv[0], (t_float)yymm);
            pd_queue_mess(x->pd_instance, (t_pd *)x, out, witsensor_pd_output_handler);
        }
        return;
    }
    if (start == 0x31 && length >= 6) {
        uint16_t ddh = (uint16_t)(data[4] | (data[5] << 8));
        t_queued_output *out = (t_queued_output *)malloc(sizeof(t_queued_output));
        if (out) {
            out->msg = gensym("time_ddh");
            out->argc = 1;
            SETFLOAT(&out->argv[0], (t_float)ddh);
            pd_queue_mess(x->pd_instance, (t_pd *)x, out, witsensor_pd_output_handler);
        }
        return;
    }
    if (start == 0x32 && length >= 6) {
        uint16_t mmss = (uint16_t)(data[4] | (data[5] << 8));
        t_queued_output *out = (t_queued_output *)malloc(sizeof(t_queued_output));
        if (out) {
            out->msg = gensym("time_mmss");
            out->argc = 1;
            SETFLOAT(&out->argv[0], (t_float)mmss);
            pd_queue_mess(x->pd_instance, (t_pd *)x, out, witsensor_pd_output_handler);
        }
        return;
    }
    if (start == 0x33 && length >= 6) {
        uint16_t ms = (uint16_t)(data[4] | (data[5] << 8));
        t_queued_output *out = (t_queued_output *)malloc(sizeof(t_queued_output));
        if (out) {
            out->msg = gensym("time_ms");
            out->argc = 1;
            SETFLOAT(&out->argv[0], (t_float)ms);
            pd_queue_mess(x->pd_instance, (t_pd *)x, out, witsensor_pd_output_handler);
        }
        return;
    }
}

// Pd-thread handler to print cached scan results
void witsensor_pd_scan_complete_handler(t_pd *obj, void *data) {
    (void)data;
    t_witsensor *x = (t_witsensor *)obj;
    if (!x || !x->ble_data) return;
    post("WITSensorBLE: Scan complete.");
    // Debug: print adapter info and callback-found count if available
    if (x->ble_data->adapter_id[0] || x->ble_data->adapter_addr[0]) {
        post("WITSensorBLE: Adapter %s [%s]", x->ble_data->adapter_id, x->ble_data->adapter_addr);
    }
    if (x->ble_data->scan_found_count > 0) {
        post("WITSensorBLE: Devices seen via callbacks: %d", x->ble_data->scan_found_count);
    }
    unsigned long n = x->ble_data->cached_count;
    if (n == 0) {
        post("WITSensorBLE: Found 0 devices. Ensure Bluetooth is on and devices are advertising.");
        return;
    }
    post("WITSensorBLE: Found %lu devices", n);
    for (unsigned long i = 0; i < n; i++) {
        const char *id = x->ble_data->cached_ids ? x->ble_data->cached_ids[i] : NULL;
        const char *addr = x->ble_data->cached_addrs ? x->ble_data->cached_addrs[i] : NULL;
        if (id && addr) post("WITSensorBLE: Found device: %s [%s]", id, addr);
    }
}

// Emit scanning status on Pd thread
void witsensor_pd_scanning_handler(t_pd *obj, void *data) {
    t_witsensor *x = (t_witsensor *)obj;
    t_queued_flag *q = (t_queued_flag *)data;
    if (x && q) {
        // Update internal scanning flag so autoconnect gating reflects actual state
        x->is_scanning = q->value;
        t_atom a; SETFLOAT(&a, (t_float)q->value);
        outlet_anything(x->status_out, gensym("scanning"), 1, &a);
    }
    if (q) free(q);
}

// Emit device record on Pd thread
void witsensor_pd_device_found_handler(t_pd *obj, void *data) {
    t_witsensor *x = (t_witsensor *)obj;
    t_queued_device *d = (t_queued_device *)data;
    if (x && d) {
        // Dedupe in current session
        if (d->id) {
            int seen = 0;
            for (unsigned long i = 0; i < x->seen_count; i++) {
                if (x->seen_ids && x->seen_ids[i] && strcmp(x->seen_ids[i], d->id) == 0) { seen = 1; break; }
            }
            if (!seen) {
                char **new_ids = (char**)realloc(x->seen_ids, (x->seen_count + 1) * sizeof(char*));
                if (new_ids) {
                    x->seen_ids = new_ids;
                    x->seen_ids[x->seen_count] = strdup(d->id);
                    x->seen_count++;
                }
            }
        }
        if (x->pending_target && !x->is_connected && x->is_scanning) {
            int should_connect = 0;
            const char *target = x->pending_target->s_name;
            if (target && target[0] && strcmp(target, "*") != 0) {
                if ((d->id && strcmp(d->id, target) == 0) || (d->addr && strcmp(d->addr, target) == 0)) {
                    should_connect = 1;
                }
            } else if (target && strcmp(target, "*") == 0) {
                if (d->tag && strcmp(d->tag, "wit") == 0) {
                    should_connect = 1;
                }
            }
            if (should_connect && d->id) {
                // Emit device status before autoconnect so UI sees the WIT
                t_atom da[3];
                SETSYMBOL(&da[0], gensym(d->tag ? d->tag : "other"));
                SETSYMBOL(&da[1], gensym(d->addr ? d->addr : ""));
                SETSYMBOL(&da[2], gensym(d->id));
                outlet_anything(x->status_out, gensym("device"), 3, da);
                // Emit autoconnecting notice
                t_atom ac[1]; SETSYMBOL(&ac[0], gensym(d->id));
                outlet_anything(x->status_out, gensym("autoconnecting"), 1, ac);
                // Reuse Pd-level connect (A_GIMME signature)
                t_atom a[1];
                SETSYMBOL(&a[0], gensym(d->id));
                witsensor_connect(x, &s_, 1, a);
                // Clear pending_target after initiating connect to avoid duplicate autoconnects
                x->pending_target = NULL;
                if (d->tag) free(d->tag);
                if (d->addr) free(d->addr);
                if (d->id) free(d->id);
                free(d);
                return;
            }
        }
        t_atom a[3];
        SETSYMBOL(&a[0], gensym(d->tag ? d->tag : "other"));
        SETSYMBOL(&a[1], gensym(d->addr ? d->addr : ""));
        SETSYMBOL(&a[2], gensym(d->id ? d->id : ""));
        outlet_anything(x->status_out, gensym("device"), 3, a);
    }
    if (d) {
        if (d->tag) free(d->tag);
        if (d->addr) free(d->addr);
        if (d->id) free(d->id);
        free(d);
    }
}


// Process streaming data directly on BLE thread (SAFE - no Pd calls)
static void witsensor_process_streaming_data(t_witsensor *x, unsigned char *data, int length) {
    if (!x || !data || length < 20) return;
    
    // Derive flags from output_mode bits
    int16_t i0 = (int16_t)((data[3] << 8) | data[2]);
    int16_t i1 = (int16_t)((data[5] << 8) | data[4]);
    int16_t i2 = (int16_t)((data[7] << 8) | data[6]);
    int16_t i3 = (int16_t)((data[9] << 8) | data[8]);
    int16_t i4 = (int16_t)((data[11] << 8) | data[10]);
    int16_t i5 = (int16_t)((data[13] << 8) | data[12]);
    int16_t i6 = (int16_t)((data[15] << 8) | data[14]);
    int16_t i7 = (int16_t)((data[17] << 8) | data[16]);
    int16_t i8 = (int16_t)((data[19] << 8) | data[18]);

    // First 12 bytes: either disp/speed or accel/gyro
    if (x->use_disp_speed) {
        // Displacement (mm) and speed (mm/s) are direct int16 units per vendor docs
        x->disp_x = (float)i0;
        x->disp_y = (float)i1;
        x->disp_z = (float)i2;
        x->speed_x = (float)i3;
        x->speed_y = (float)i4;
        x->speed_z = (float)i5;
    } else {
        x->accel_x = (float)i0 / 32768.0f * 16.0f;
        x->accel_y = (float)i1 / 32768.0f * 16.0f;
        x->accel_z = (float)i2 / 32768.0f * 16.0f;
        x->gyro_x = (float)i3 / 32768.0f * 2000.0f;
        x->gyro_y = (float)i4 / 32768.0f * 2000.0f;
        x->gyro_z = (float)i5 / 32768.0f * 2000.0f;
    }
    if (x->use_timestamp) {
        // Timestamp in ms: 32-bit little-endian composed from two int16 words
        x->ts_lo = (unsigned short)((uint16_t)i6);
        x->ts_hi = (unsigned short)((uint16_t)i7);
    } else {
        x->angle_x = (float)i6 / 32768.0f * 180.0f;
        x->angle_y = (float)i7 / 32768.0f * 180.0f;
    }
    x->angle_z = (float)i8 / 32768.0f * 180.0f;
}

// Send quaternion data as PureData messages
static void witsensor_send_quaternion_data(t_witsensor *x) {
    t_atom args[4];
    
    // Send quaternion data (4 floats: w, x, y, z)
    SETFLOAT(&args[0], x->quat_w);
    SETFLOAT(&args[1], x->quat_x);
    SETFLOAT(&args[2], x->quat_y);
    SETFLOAT(&args[3], x->quat_z);
    outlet_anything(x->data_out, gensym("quat"), 4, args);
}

// Send sensor data as PureData messages
static void witsensor_send_sensor_data(t_witsensor *x) {
    t_atom args[3];
    if (x->use_disp_speed) {
        SETFLOAT(&args[0], x->disp_x);
        SETFLOAT(&args[1], x->disp_y);
        SETFLOAT(&args[2], x->disp_z);
        outlet_anything(x->data_out, gensym("disp"), 3, args);
        SETFLOAT(&args[0], x->speed_x);
        SETFLOAT(&args[1], x->speed_y);
        SETFLOAT(&args[2], x->speed_z);
        outlet_anything(x->data_out, gensym("speed"), 3, args);
    } else {
        SETFLOAT(&args[0], x->accel_x);
        SETFLOAT(&args[1], x->accel_y);
        SETFLOAT(&args[2], x->accel_z);
        outlet_anything(x->data_out, gensym("accel"), 3, args);
        SETFLOAT(&args[0], x->gyro_x);
        SETFLOAT(&args[1], x->gyro_y);
        SETFLOAT(&args[2], x->gyro_z);
        outlet_anything(x->data_out, gensym("gyro"), 3, args);
    }
    if (x->use_timestamp) {
        SETFLOAT(&args[0], (t_float)x->ts_hi);
        SETFLOAT(&args[1], (t_float)x->ts_lo);
        outlet_anything(x->data_out, gensym("timestamp"), 2, args);
        SETFLOAT(&args[0], 0);
        SETFLOAT(&args[1], 0);
        SETFLOAT(&args[2], x->angle_z);
        outlet_anything(x->data_out, gensym("angle"), 3, args);
    } else {
        SETFLOAT(&args[0], x->angle_x);
        SETFLOAT(&args[1], x->angle_y);
        SETFLOAT(&args[2], x->angle_z);
        outlet_anything(x->data_out, gensym("angle"), 3, args);
    }
}

// Function for polling requests (battery/temp/mag/quat)
static void witsensor_poll_tick(t_witsensor *x) {
    if (!x || !x->ble_data) {
        post("witsensor: ERROR - Invalid object or BLE data");
        return;
    }
    
    // Check connection status
    x->is_connected = witsensor_ble_simpleble_is_connected(x->ble_data);
    x->is_scanning = witsensor_ble_simpleble_is_scanning(x->ble_data);

    // Only poll if connected and interval is set
    if (x->poll_interval > 0 && x->is_connected && x->poll_type) {
        // Call the appropriate single-shot function based on poll_type
        if (x->poll_type == gensym("quat")) {
            witsensor_quat(x);
        } else if (x->poll_type == gensym("mag")) {
            witsensor_mag(x);
        } else if (x->poll_type == gensym("battery")) {
            witsensor_battery(x);
        } else if (x->poll_type == gensym("temp")) {
            witsensor_temp(x);
        }
        
        // Schedule next poll
        clock_delay(x->poll_clock, x->poll_interval);
    } else if (x->poll_interval > 0 && !x->is_connected) {
        // Stop polling when disconnected
        post("witsensor: disconnected, stopping %s polling", x->poll_type ? x->poll_type->s_name : "unknown");
        x->poll_interval = 0;
        x->poll_type = NULL;
    }
}

// Handle queued output messages on Pd scheduler thread
static void witsensor_pd_output_handler(t_pd *obj, void *data) {
    if (!obj || !data) return;
    t_witsensor *x = (t_witsensor *)obj;
    t_queued_output *out = (t_queued_output *)data;
    
    if (out->msg == gensym("battery")) {
        outlet_anything(x->status_out, out->msg, out->argc, out->argv);
    } else if (out->msg == gensym("temp")) {
        outlet_anything(x->status_out, out->msg, out->argc, out->argv);
    } else if (out->msg == gensym("mag")) {
        outlet_anything(x->data_out, out->msg, out->argc, out->argv);
    } else if (out->msg == gensym("quat")) {
        witsensor_send_quaternion_data(x);
    } else if (out->msg == gensym("streaming")) {
        witsensor_send_sensor_data(x);
    } else if (out->msg == gensym("version1")) {
        outlet_anything(x->status_out, out->msg, out->argc, out->argv);
    } else if (out->msg == gensym("version2")) {
        outlet_anything(x->status_out, out->msg, out->argc, out->argv);
    } else if (out->msg == gensym("time_yymm")) {
        outlet_anything(x->status_out, out->msg, out->argc, out->argv);
    } else if (out->msg == gensym("time_ddh")) {
        outlet_anything(x->status_out, out->msg, out->argc, out->argv);
    } else if (out->msg == gensym("time_mmss")) {
        outlet_anything(x->status_out, out->msg, out->argc, out->argv);
    } else if (out->msg == gensym("time_ms")) {
        outlet_anything(x->status_out, out->msg, out->argc, out->argv);
    }
    
    free(out);
}

// Handle connection status changes on Pd scheduler thread
void witsensor_pd_connected_handler(t_pd *obj, void *data) {
    if (!obj || !data) return;
    t_witsensor *x = (t_witsensor *)obj;
    t_queued_flag *flag = (t_queued_flag *)data;
    
    x->is_connected = flag->value;
    
    t_atom a;
    SETFLOAT(&a, flag->value);
    outlet_anything(x->status_out, gensym("connected"), 1, &a);
    
    if (!flag->value) {
        // Device disconnected - stop polling
        if (x->poll_interval > 0) {
            post("witsensor: device disconnected, stopping %s polling", 
                 x->poll_type ? x->poll_type->s_name : "unknown");
            x->poll_interval = 0;
            x->poll_type = NULL;
        }
    }
    
    free(flag);
}

// Scan for WIT devices (continuous until stopped or connected)
static void witsensor_scan_devices(t_witsensor *x) {
    post("witsensor: scanning for BLE devices...");
    if (!x->ble_data) { post("witsensor: BLE not initialized"); return; }
    
    // Reset scan results list
    witsensor_ble_simpleble_clear_scan_results(x->ble_data);
    // Reset dedupe list
    if (x->seen_ids) {
        for (unsigned long i = 0; i < x->seen_count; i++) free(x->seen_ids[i]);
        free(x->seen_ids);
        x->seen_ids = NULL;
        x->seen_count = 0;
    }
    
    // Continuous scanning (no timeout needed)
    witsensor_ble_simpleble_start_scanning(x->ble_data);
}

// Get scan results
static void witsensor_get_scan_results(t_witsensor *x) {
    if (!x->ble_data) { post("witsensor: BLE not initialized"); return; }
    witsensor_ble_simpleble_get_scan_results(x->ble_data);
}

// Connect to device by name or address
static void witsensor_connect(t_witsensor *x, t_symbol *s, int argc, t_atom *argv) {
    (void)s;
    t_symbol *device_identifier = NULL;
    if (argc > 0 && argv[0].a_type == A_SYMBOL) device_identifier = argv[0].a_w.w_symbol;
    if (device_identifier && device_identifier->s_name) {
        strcpy(x->device_name, device_identifier->s_name);
    } else {
        x->device_name[0] = '\0';
    }
    
    post("witsensor: connecting to device: %s", x->device_name);
    
    // If already connected, do nothing to avoid surprising implicit disconnects
    if (x->is_connected) {
        post("witsensor: already connected; disconnect first to connect to a new device");
        x->pending_target = NULL; // cancel any pending autoconnect
        return;
    }

    if (x->ble_data) {
        int connected = 0;
        if (x->device_name[0]) {
            // Try immediate targeted connect
            connected = witsensor_ble_simpleble_connect(x->ble_data, x->device_name);
        } else {
            // No target specified: try current cached results for first WIT device
            if (x->ble_data->cached_ids && x->ble_data->cached_count > 0) {
                for (unsigned long i = 0; i < x->ble_data->cached_count; i++) {
                    const char *id = x->ble_data->cached_ids[i];
                    if (id && strstr(id, "WT") != NULL) {
                        connected = witsensor_ble_simpleble_connect(x->ble_data, id);
                        if (connected) break;
                    }
                }
            }
        }
        
        if (connected) {
            x->is_connected = 1;
            // Clear pending autoconnect since we achieved a connection
            x->pending_target = NULL;
            t_atom a; SETFLOAT(&a, 1);
            outlet_anything(x->status_out, gensym("connected"), 1, &a);
            // On-connect configuration: set desired streaming/output mode consistently for Pd usage
            unsigned char cmd_unlock[] = {0xFF, 0xAA, 0x69, 0x88, 0xB5};
            witsensor_ble_simpleble_write_data(x->ble_data, cmd_unlock, sizeof(cmd_unlock));
            usleep(50000);
            // Axis: 9-axis (reg 0x24, code 0x00)
            unsigned char cmd_axis9[] = {0xFF, 0xAA, 0x24, 0x00, 0x00};
            witsensor_ble_simpleble_write_data(x->ble_data, cmd_axis9, sizeof(cmd_axis9));
            x->axis_mode = 9;
            t_atom ax; SETFLOAT(&ax, 9);
            outlet_anything(x->status_out, gensym("axis"), 1, &ax);
            usleep(30000);
            // Output mode (AGPVSEL, reg 0x96): 0 = accel+gyro+angle
            unsigned char cmd_outmode0[] = {0xFF, 0xAA, 0x96, 0x00, 0x00};
            witsensor_ble_simpleble_write_data(x->ble_data, cmd_outmode0, sizeof(cmd_outmode0));
            x->output_mode = 0;
            t_atom om; SETFLOAT(&om, 0);
            outlet_anything(x->status_out, gensym("outputmode"), 1, &om);
            usleep(30000);
            // Default stream rate 50 Hz
            unsigned char cmd_rate[] = {0xFF, 0xAA, 0x03, 0x08, 0x00};
            witsensor_ble_simpleble_write_data(x->ble_data, cmd_rate, sizeof(cmd_rate));
            t_atom rate_args[2];
            SETFLOAT(&rate_args[0], 50.0f);
            SETFLOAT(&rate_args[1], 0x08);
            outlet_anything(x->status_out, gensym("rate"), 2, rate_args);
            usleep(30000);
             // bandwidth 256 Hz
            unsigned char cmd_bw[] = {0xFF, 0xAA, 0x1F, 0x00, 0x00};
            witsensor_ble_simpleble_write_data(x->ble_data, cmd_bw, sizeof(cmd_bw));
            t_atom bw; SETFLOAT(&bw, 256.0f);
            outlet_anything(x->status_out, gensym("bandwidth"), 1, &bw);
            x->poll_interval = 0;
        } else {
            post("witsensor: starting autoconnect...");
            x->pending_target = gensym(x->device_name[0] ? x->device_name : "*");
            if (!witsensor_ble_simpleble_is_scanning(x->ble_data)) {
                witsensor_ble_simpleble_start_scanning(x->ble_data);
            }
        }
        return;
    } else {
        post("witsensor: BLE not initialized");
    }
}

// Disconnect from device
static void witsensor_disconnect(t_witsensor *x) {
    if (!x->is_connected) {
        post("witsensor: no device connected");
        return;
    }
    
    if (x->ble_data) {
        // Cancel any pending autoconnect so subsequent 'results' won't reconnect implicitly
        x->pending_target = NULL;
        witsensor_ble_simpleble_disconnect(x->ble_data);
        x->should_stop = 1;
    }
}

// Set streaming rate (in Hz)
static void witsensor_set_rate(t_witsensor *x, t_float rate) {
    if (rate < 0.1f) rate = 0.1f;
    if (rate > 200.0f) rate = 200.0f;
    
    if (x->is_connected && x->ble_data) {
        // Unlock sensor first
        unsigned char cmd_unlock[] = {0xFF, 0xAA, 0x69, 0x88, 0xB5}; // Unlock command
        witsensor_ble_simpleble_write_data(x->ble_data, cmd_unlock, sizeof(cmd_unlock));
        
        unsigned char rate_code;
        if (rate <= 0.15f) rate_code = 0x01; // 0.1 Hz
        else if (rate <= 0.75f) rate_code = 0x02; // 0.5 Hz
        else if (rate <= 1.5f) rate_code = 0x03; // 1 Hz
        else if (rate <= 3.0f) rate_code = 0x04; // 2 Hz
        else if (rate <= 7.5f) rate_code = 0x05; // 5 Hz
        else if (rate <= 15.0f) rate_code = 0x06; // 10 Hz
        else if (rate <= 35.0f) rate_code = 0x07; // 20 Hz
        else if (rate <= 75.0f) rate_code = 0x08; // 50 Hz
        else if (rate <= 150.0f) rate_code = 0x09; // 100 Hz
        else rate_code = 0x0B; // 200Hz

        // Unlock and send rate immediately
        // Small spacing; sensor accepts back-to-back frames over BLE without blocking main thread
        unsigned char cmd_rate[] = {0xFF, 0xAA, 0x03, rate_code, 0x00};
        witsensor_ble_simpleble_write_data(x->ble_data, cmd_rate, sizeof(cmd_rate));

        t_atom args[2];
        SETFLOAT(&args[0], rate);
        SETFLOAT(&args[1], rate_code);
        outlet_anything(x->status_out, gensym("rate"), 2, args);
    } else {
        post("witsensor: not connected to device");
    }
}

// Set digital filter bandwidth (register 0x1F)
static void witsensor_set_bandwidth(t_witsensor *x, t_float hz) {
    if (!x->is_connected || !x->ble_data) { post("witsensor: not connected to device"); return; }
    unsigned char bw_code;
    if (hz >= 220.0f) bw_code = 0x00; // 256 Hz
    else if (hz >= 140.0f) bw_code = 0x01; // 188 Hz
    else if (hz >= 70.0f) bw_code = 0x02; // 98 Hz
    else if (hz >= 30.0f) bw_code = 0x03; // 42 Hz
    else if (hz >= 15.0f) bw_code = 0x04; // 20 Hz
    else if (hz >= 7.0f) bw_code = 0x05; // 10 Hz
    else bw_code = 0x06; // 5 Hz
    unsigned char cmd_unlock[] = {0xFF, 0xAA, 0x69, 0x88, 0xB5};
    witsensor_ble_simpleble_write_data(x->ble_data, cmd_unlock, sizeof(cmd_unlock));
    usleep(50000);
    unsigned char cmd_bw[] = {0xFF, 0xAA, 0x1F, bw_code, 0x00};
    witsensor_ble_simpleble_write_data(x->ble_data, cmd_bw, sizeof(cmd_bw));
    
    t_atom a; SETFLOAT(&a, hz);
    outlet_anything(x->status_out, gensym("bandwidth"), 1, &a);
}

// Battery request: FF AA 27 64 00
static void witsensor_battery(t_witsensor *x) {
    if (!x->is_connected || !x->ble_data) { post("witsensor: not connected to device"); return; }
    
    // Note: Battery requests may be unreliable with active streaming
    // Users should pause streaming (rate 0) before requesting battery data
    unsigned char cmd[] = {0xFF, 0xAA, 0x27, 0x64, 0x00};
    witsensor_ble_simpleble_write_data(x->ble_data, cmd, sizeof(cmd));
}

// Request temperature data: FF AA 27 40 00
static void witsensor_temp(t_witsensor *x) {
    if (!x->is_connected || !x->ble_data) { post("witsensor: not connected to device"); return; }
    
    // Note: Temperature requests may be unreliable with active streaming
    // Users should pause streaming (rate 0) before requesting temperature data
    unsigned char cmd[] = {0xFF, 0xAA, 0x27, 0x40, 0x00};
    witsensor_ble_simpleble_write_data(x->ble_data, cmd, sizeof(cmd));
}

// Request magnetic field data: FF AA 27 3A 00
static void witsensor_mag(t_witsensor *x) {
    if (!x->is_connected || !x->ble_data) { post("witsensor: not connected to device"); return; }
    
    // Note: Magnetic field requests may be unreliable with active streaming
    // Users should pause streaming (rate 0) before requesting magnetic field data
    unsigned char cmd[] = {0xFF, 0xAA, 0x27, 0x3A, 0x00};
    witsensor_ble_simpleble_write_data(x->ble_data, cmd, sizeof(cmd));
}

// Request quaternion data: FF AA 27 51 00
static void witsensor_quat(t_witsensor *x) {
    if (!x->is_connected || !x->ble_data) { post("witsensor: not connected to device"); return; }
    
    // Note: Quaternion requests may be unreliable with active streaming
    // Users should pause streaming (rate 0) before requesting quaternion data
    unsigned char cmd[] = {0xFF, 0xAA, 0x27, 0x51, 0x00};
    witsensor_ble_simpleble_write_data(x->ble_data, cmd, sizeof(cmd));
}

// Read firmware version registers 0x2E and 0x2F
static void witsensor_read_version(t_witsensor *x) {
    if (!x->is_connected || !x->ble_data) { post("witsensor: not connected to device"); return; }
    unsigned char cmd1[] = {0xFF, 0xAA, 0x27, 0x2E, 0x00};
    unsigned char cmd2[] = {0xFF, 0xAA, 0x27, 0x2F, 0x00};
    witsensor_ble_simpleble_write_data(x->ble_data, cmd1, sizeof(cmd1));
    usleep(60000);
    witsensor_ble_simpleble_write_data(x->ble_data, cmd2, sizeof(cmd2));
}

// Read device time registers 0x30..0x33
static void witsensor_read_time(t_witsensor *x) {
    if (!x->is_connected || !x->ble_data) { post("witsensor: not connected to device"); return; }
    unsigned char c30[] = {0xFF, 0xAA, 0x27, 0x30, 0x00};
    unsigned char c31[] = {0xFF, 0xAA, 0x27, 0x31, 0x00};
    unsigned char c32[] = {0xFF, 0xAA, 0x27, 0x32, 0x00};
    unsigned char c33[] = {0xFF, 0xAA, 0x27, 0x33, 0x00};
    witsensor_ble_simpleble_write_data(x->ble_data, c30, sizeof(c30));
    usleep(60000);
    witsensor_ble_simpleble_write_data(x->ble_data, c31, sizeof(c31));
    usleep(60000);
    witsensor_ble_simpleble_write_data(x->ble_data, c32, sizeof(c32));
    usleep(60000);
    witsensor_ble_simpleble_write_data(x->ble_data, c33, sizeof(c33));
}

// Clear cached scan results (Pd message: reset)
static void witsensor_reset(t_witsensor *x) {
    if (!x || !x->ble_data) { post("witsensor: BLE not initialized"); return; }
    // Stop scanning if active
    if (witsensor_ble_simpleble_is_scanning(x->ble_data)) {
        witsensor_ble_simpleble_stop_scanning(x->ble_data);
    }
    witsensor_ble_simpleble_clear_scan_results(x->ble_data);
    if (x->seen_ids) {
        for (unsigned long i = 0; i < x->seen_count; i++) free(x->seen_ids[i]);
        free(x->seen_ids);
        x->seen_ids = NULL;
        x->seen_count = 0;
    }
}

// Set angle reference (zero): FF AA 01 08 00
static void witsensor_xyzero(t_witsensor *x) {
    if (!x->is_connected || !x->ble_data) { post("witsensor: not connected to device"); return; }
    unsigned char cmd[] = {0xFF, 0xAA, 0x01, 0x08, 0x00};
    witsensor_ble_simpleble_write_data(x->ble_data, cmd, sizeof(cmd));
}


// Set installation orientation: FF AA 23 <0|1> 00
static void witsensor_set_orientation(t_witsensor *x, t_float f) {
    if (!x->is_connected || !x->ble_data) { post("witsensor: not connected to device"); return; }
    int orient = (int)f; if (orient < 0) orient = 0; if (orient > 1) orient = 1;
    unsigned char cmd[] = {0xFF, 0xAA, 0x23, (unsigned char)orient, 0x00};
    witsensor_ble_simpleble_write_data(x->ble_data, cmd, sizeof(cmd));
}

// Set output content (AGPVS): FF AA 96 <0..3> 00
static void witsensor_set_output_mode(t_witsensor *x, t_float f) {
    if (!x->is_connected || !x->ble_data) { post("witsensor: not connected to device"); return; }
    int mode = (int)f; if (mode < 0) mode = 0; if (mode > 3) mode = 3;
    unsigned char cmd[] = {0xFF, 0xAA, 0x96, (unsigned char)mode, 0x00};
    witsensor_ble_simpleble_write_data(x->ble_data, cmd, sizeof(cmd));
    x->output_mode = mode;
    t_atom a; SETFLOAT(&a, mode);
    outlet_anything(x->status_out, gensym("outputmode"), 1, &a);
    x->use_disp_speed = (mode & 1);
    x->use_timestamp = ((mode >> 1) & 1);
}

// Set baud rate: FF AA 04 <0..255> 00
static void witsensor_set_baud(t_witsensor *x, t_float f) {
    if (!x->is_connected || !x->ble_data) { post("witsensor: not connected to device"); return; }
    int baud = (int)f; if (baud < 0) baud = 0; if (baud > 255) baud = 255;
    post("witsensor: setting baud rate to %d", baud);
    unsigned char cmd[] = {0xFF, 0xAA, 0x04, (unsigned char)baud, 0x00};
    witsensor_ble_simpleble_write_data(x->ble_data, cmd, sizeof(cmd));
}

// Save configuration: FF AA 00 00 00
static void witsensor_save(t_witsensor *x) {
    if (!x->is_connected || !x->ble_data) { post("witsensor: not connected to device"); return; }
    post("witsensor: saving configuration");
    unsigned char cmd[] = {0xFF, 0xAA, 0x00, 0x00, 0x00};
    witsensor_ble_simpleble_write_data(x->ble_data, cmd, sizeof(cmd));
}

// Restore configuration: FF AA 00 01 00
static void witsensor_restore(t_witsensor *x) {
    if (!x->is_connected || !x->ble_data) { 
        post("witsensor: not connected to device"); 
        return; 
    }
    
    unsigned char cmd[] = {0xFF, 0xAA, 0x00, 0x01, 0x00};
    int result = witsensor_ble_simpleble_write_data(x->ble_data, cmd, sizeof(cmd));
    
    if (!result) {
        post("witsensor: failed to send restore command");
    } else {
        post("witsensor: restore command sent successfully");
    }
}

// Unified polling method: poll <type> <interval>
// Examples: poll quat 10, poll mag 5, poll battery 1, poll temp 2
static void witsensor_poll(t_witsensor *x, t_symbol *type, t_float interval) {
    if (!type) {
        post("witsensor: poll requires a type (quat, mag, battery, temp)");
        return;
    }
    
    // Validate type
    if (type != gensym("quat") && type != gensym("mag") && 
        type != gensym("battery") && type != gensym("temp")) {
        post("witsensor: poll type must be one of: quat, mag, battery, temp");
        return;
    }
    
    // Stop current polling if interval is 0 or negative
    if (interval <= 0) {
        x->poll_interval = 0;
        x->poll_type = NULL;
        // Stop the polling clock - streaming data has its own separate clock
        clock_unset(x->poll_clock);
        outlet_anything(x->status_out, gensym("poll"), 0, NULL);
        return;
    }
    
    // Cap at 50 Hz to avoid BLE congestion
    if (interval > 50.0f) interval = 50.0f;
    int period_ms = (int)(1000.0f / interval);
    if (period_ms < 1) period_ms = 1;
    
    x->poll_interval = period_ms;
    x->poll_type = type;
    
    if (x->is_connected) {
        clock_unset(x->poll_clock);
        clock_delay(x->poll_clock, x->poll_interval);
        t_atom args[3];
        SETSYMBOL(&args[0], type);
        SETFLOAT(&args[1], 1000.0f / x->poll_interval);
        SETFLOAT(&args[2], x->poll_interval);
        outlet_anything(x->status_out, gensym("poll"), 3, args);
    }
}

// Constructor
static void *witsensor_new(void) {
    t_witsensor *x = (t_witsensor *)pd_new(witsensor_class);
    
    // macOS: preflight CoreBluetooth authorization. If missing, fail creation cleanly.
    #ifdef __APPLE__
    if (!macos_bt_authorized_always()) {
        pd_error(x, "witsensor: Bluetooth permission not granted. Grant Pd in System Settings → Privacy & Security → Bluetooth");
        return NULL;
    }
    #endif
    
    x->data_out = outlet_new(&x->x_obj, &s_anything);
    x->status_out = outlet_new(&x->x_obj, &s_float);
    x->poll_clock = clock_new(x, (t_method)witsensor_poll_tick);
    
    x->is_connected = 0;
    x->is_scanning = 0;
    x->should_stop = 0;
    x->poll_interval = 0;
    x->poll_type = NULL;
    // Tracked state defaults
    x->axis_mode = 0;
    x->output_mode = -1;
    x->temp_bytes_count = 0;
    x->pd_instance = pd_this;
    x->pending_target = NULL;
    x->seen_ids = NULL;
    x->seen_count = 0;
    
    // Initialize BLE system with crash protection
    post("witsensor: initializing BLE system...");
    x->ble_data = witsensor_ble_simpleble_create();
    if (x->ble_data) {
        // Set up data callback
        x->ble_data->pd_obj = x;
        x->ble_data->data_callback = witsensor_ble_data_callback;
        x->ble_data->pd_instance = x->pd_instance;
        post("witsensor: BLE system initialized successfully");
    } else {
        pd_error(x, "witsensor: BLE system initialization failed");
    }
    
    // Initialize sensor data
    x->accel_x = x->accel_y = x->accel_z = 0.0f;
    x->gyro_x = x->gyro_y = x->gyro_z = 0.0f;
    x->angle_x = x->angle_y = x->angle_z = 0.0f;
    x->quat_w = x->quat_x = x->quat_y = x->quat_z = 0.0f;
    x->disp_x = x->disp_y = x->disp_z = 0.0f;
    x->speed_x = x->speed_y = x->speed_z = 0.0f;
    x->ts_lo = x->ts_hi = 0;
    x->use_disp_speed = 0;
    x->use_timestamp = 0;
    
    return (void *)x;
}

// Destructor
static void witsensor_free(t_witsensor *x) {
    // Stop scanning first to avoid callbacks firing after free
    if (x->ble_data && witsensor_ble_simpleble_is_scanning(x->ble_data)) {
        witsensor_ble_simpleble_stop_scanning(x->ble_data);
    }
    // Disconnect device
    witsensor_disconnect(x);
    pd_queue_cancel((t_pd *)x);
    if (x->ble_data) {
        witsensor_ble_simpleble_destroy(x->ble_data);
    }
    // Free dedupe list
    if (x->seen_ids) {
        for (unsigned long i = 0; i < x->seen_count; i++) free(x->seen_ids[i]);
        free(x->seen_ids);
        x->seen_ids = NULL;
        x->seen_count = 0;
    }
    clock_free(x->poll_clock);
}

// WIT sensor command functions

// Calibration: calibrate (default=accel), calibrate 1 (mag), calibrate 2 (mag complete)
static void witsensor_calibrate(t_witsensor *x) {
    if (!x->is_connected || !x->ble_data) { post("witsensor: not connected to device"); return; }
    
    // Unlock
    unsigned char cmd_unlock[] = {0xFF, 0xAA, 0x69, 0x88, 0xB5};
    witsensor_ble_simpleble_write_data(x->ble_data, cmd_unlock, sizeof(cmd_unlock));
    usleep(50000);
    post("witsensor: starting accelerometer calibration - keep sensor still");
    unsigned char cmd[] = {0xFF, 0xAA, 0x01, 0x01, 0x00};
    witsensor_ble_simpleble_write_data(x->ble_data, cmd, sizeof(cmd));
}


// Set algorithm: 6-axis or 9-axis (register 0x24)
static void witsensor_axis(t_witsensor *x, t_float axis_count) {
    if (!x->is_connected || !x->ble_data) { post("witsensor: not connected to device"); return; }
    unsigned char code = 0x01; // default 6-axis
    if (axis_count == 9) {
        code = 0x00; // 9-axis
    }
    unsigned char cmd_unlock[] = {0xFF, 0xAA, 0x69, 0x88, 0xB5};
    witsensor_ble_simpleble_write_data(x->ble_data, cmd_unlock, sizeof(cmd_unlock));
    usleep(50000);
    unsigned char cmd_algo[] = {0xFF, 0xAA, 0x24, code, 0x00};
    witsensor_ble_simpleble_write_data(x->ble_data, cmd_algo, sizeof(cmd_algo));
    x->axis_mode = (axis_count == 9 ? 9 : 6);
    t_atom a; SETFLOAT(&a, x->axis_mode);
    outlet_anything(x->status_out, gensym("axis"), 1, &a);
}

static void witsensor_magcal_start(t_witsensor *x) {
    if (!x->is_connected || !x->ble_data) { post("witsensor: not connected to device"); return; }
    unsigned char cmd_unlock[] = {0xFF, 0xAA, 0x69, 0x88, 0xB5};
    witsensor_ble_simpleble_write_data(x->ble_data, cmd_unlock, sizeof(cmd_unlock));
    usleep(50000);
    unsigned char cmd_start[] = {0xFF, 0xAA, 0x01, 0x07, 0x00};
    witsensor_ble_simpleble_write_data(x->ble_data, cmd_start, sizeof(cmd_start));
    t_atom a; SETSYMBOL(&a, gensym("start"));
    outlet_anything(x->status_out, gensym("magcal"), 1, &a);
}

static void witsensor_magcal_stop(t_witsensor *x) {
    if (!x->is_connected || !x->ble_data) { post("witsensor: not connected to device"); return; }
    unsigned char cmd_stop[] = {0xFF, 0xAA, 0x01, 0x00, 0x00};
    witsensor_ble_simpleble_write_data(x->ble_data, cmd_stop, sizeof(cmd_stop));
    t_atom a; SETSYMBOL(&a, gensym("stop"));
    outlet_anything(x->status_out, gensym("magcal"), 1, &a);
}

static void witsensor_zzero(t_witsensor *x) {
    if (!x->is_connected || !x->ble_data) { post("witsensor: not connected to device"); return; }
    unsigned char cmd_unlock[] = {0xFF, 0xAA, 0x69, 0x88, 0xB5};
    witsensor_ble_simpleble_write_data(x->ble_data, cmd_unlock, sizeof(cmd_unlock));
    usleep(50000);
    unsigned char cmd_algo6[] = {0xFF, 0xAA, 0x24, 0x01, 0x00};
    witsensor_ble_simpleble_write_data(x->ble_data, cmd_algo6, sizeof(cmd_algo6));
    usleep(50000);
    unsigned char cmd_zeroz[] = {0xFF, 0xAA, 0x01, 0x04, 0x00};
    witsensor_ble_simpleble_write_data(x->ble_data, cmd_zeroz, sizeof(cmd_zeroz));
    outlet_anything(x->status_out, gensym("zzero"), 0, NULL);
}

// Version info
static void witsensor_version(void) {
    post("witsensor v%d.%d.%d - WIT BWT901BLE5.0 sensor external for PureData", 
         WITSENSOR_MAJOR_VERSION, WITSENSOR_MINOR_VERSION, WITSENSOR_BUGFIX_VERSION);
}

// Setup function
#if defined(_WIN32)
__declspec(dllexport)
#else
__attribute__((visibility("default")))
#endif
void witsensor_setup(void) {
    witsensor_class = class_new(gensym("witsensor"),
                               (t_newmethod)witsensor_new,
                               (t_method)witsensor_free,
                               sizeof(t_witsensor),
                               CLASS_DEFAULT,
                               0);
    
    class_addmethod(witsensor_class, (t_method)witsensor_scan_devices, gensym("scan"), 0);
    class_addmethod(witsensor_class, (t_method)witsensor_get_scan_results, gensym("results"), 0);
    class_addmethod(witsensor_class, (t_method)witsensor_connect, gensym("connect"), A_GIMME, 0);
    class_addmethod(witsensor_class, (t_method)witsensor_disconnect, gensym("disconnect"), 0);
    class_addmethod(witsensor_class, (t_method)witsensor_poll, gensym("poll"), A_SYMBOL, A_DEFFLOAT, 0);
    class_addmethod(witsensor_class, (t_method)witsensor_set_rate, gensym("rate"), A_FLOAT, 0);
    class_addmethod(witsensor_class, (t_method)witsensor_set_bandwidth, gensym("bandwidth"), A_FLOAT, 0);
    class_addmethod(witsensor_class, (t_method)witsensor_axis, gensym("axis"), A_FLOAT, 0);
    class_addmethod(witsensor_class, (t_method)witsensor_calibrate, gensym("calibrate"), 0);
    class_addmethod(witsensor_class, (t_method)witsensor_magcal_start, gensym("magcal-start"), 0);
    class_addmethod(witsensor_class, (t_method)witsensor_magcal_stop, gensym("magcal-stop"), 0);
    class_addmethod(witsensor_class, (t_method)witsensor_xyzero, gensym("xyzero"), 0);
    class_addmethod(witsensor_class, (t_method)witsensor_zzero, gensym("zzero"), 0);
    // About/version of the external
    class_addmethod(witsensor_class, (t_method)witsensor_version, gensym("about"), 0);
    // Device queries
    class_addmethod(witsensor_class, (t_method)witsensor_read_version, gensym("version"), 0);
    class_addmethod(witsensor_class, (t_method)witsensor_read_time, gensym("time"), 0);
    class_addmethod(witsensor_class, (t_method)witsensor_battery, gensym("battery"), 0);
    class_addmethod(witsensor_class, (t_method)witsensor_temp, gensym("temp"), 0);
    class_addmethod(witsensor_class, (t_method)witsensor_mag, gensym("mag"), 0);
    class_addmethod(witsensor_class, (t_method)witsensor_quat, gensym("quat"), 0);
    class_addmethod(witsensor_class, (t_method)witsensor_set_orientation, gensym("orientation"), A_DEFFLOAT, 0);
    class_addmethod(witsensor_class, (t_method)witsensor_set_output_mode, gensym("outputmode"), A_DEFFLOAT, 0);
    class_addmethod(witsensor_class, (t_method)witsensor_save, gensym("save"), 0);
    class_addmethod(witsensor_class, (t_method)witsensor_restore, gensym("restore"), 0);
    class_addmethod(witsensor_class, (t_method)witsensor_set_baud, gensym("baud"), A_DEFFLOAT, 0);
    class_addmethod(witsensor_class, (t_method)witsensor_reset, gensym("reset"), 0);
    class_addmethod(witsensor_class, (t_method)witsensor_setname, gensym("setname"), A_GIMME, 0);
    
    witsensor_version();
}

// Set Bluetooth name via vendor ASCII command with selectable variant
// Usage: setname <editable_part> [variant]
//   variant 1: "WT %s \r\n" (space before and after name, CRLF) [default per docs]
//   variant 2: "WT %s\r\n"  (space, CRLF)
//   variant 3: "WT%s\r\n"   (no space, CRLF)
static void witsensor_setname(t_witsensor *x, t_symbol *s, int argc, t_atom *argv) {
    (void)s;
    if (!x || !x->ble_data || !x->is_connected) { post("witsensor: not connected to device"); return; }
    if (argc < 1 || argv[0].a_type != A_SYMBOL) { post("witsensor: setname requires a symbol argument"); return; }
    const char *input = argv[0].a_w.w_symbol ? argv[0].a_w.w_symbol->s_name : NULL;
    if (!input || !input[0]) { post("witsensor: setname requires non-empty name"); return; }
    
    int variant = 1;
    if (argc >= 2 && argv[1].a_type == A_FLOAT) {
        int v = (int)atom_getfloat(&argv[1]);
        if (v >= 1 && v <= 3) variant = v;
    }

    // Build full name: 'WT' + up to 14 non-space chars from editable; strip leading 'WT'
    const char *editable = input;
    if (editable[0] == 'W' && editable[1] == 'T') editable += 2;
    char full_name[20];
    size_t fn = 0;
    full_name[fn++] = 'W';
    full_name[fn++] = 'T';
    size_t editable_nonspace = 0;
    for (size_t i = 0; editable[i] != '\0' && fn < 16; i++) {
        char c = editable[i];
        if (c == ' ' || c == '\t' || c == '\n' || c == '\r') continue;
        editable_nonspace++;
        full_name[fn++] = c;
    }
    full_name[fn] = '\0';
    if (editable_nonspace > 14) {
        post("witsensor: setname truncated to 14 chars: %s", full_name);
    }
    
    post("witsensor: setname starting - variant %d, name: %s", variant, full_name);
    
    // Unlock
    unsigned char unlock[] = {0xFF, 0xAA, 0x69, 0x88, 0xB5};
    witsensor_ble_simpleble_write_data(x->ble_data, unlock, sizeof(unlock));
    usleep(100000);  // Increased delay
    
    // Build ASCII command per variant
    char cmd[64];
    int n = 0;
    if (variant == 1) {
        n = snprintf(cmd, sizeof(cmd), "WT %s \r\n", full_name);
    } else if (variant == 2) {
        n = snprintf(cmd, sizeof(cmd), "WT %s\r\n", full_name);
    } else {
        n = snprintf(cmd, sizeof(cmd), "WT%s\r\n", full_name);
    }
    
    if (n > 0) {
        // Debug: print the exact bytes being sent
        post("witsensor: sending ASCII command: '%s' (length: %d)", cmd, n);
        for (int i = 0; i < n; i++) {
            post("  byte[%d] = 0x%02X ('%c')", i, (unsigned char)cmd[i], (cmd[i] >= 32 && cmd[i] <= 126) ? cmd[i] : '.');
        }
        
        // Use write-request to ensure correct length and delivery semantics
        witsensor_ble_simpleble_write_request_raw(x->ble_data, (const unsigned char*)cmd, n);
        
        // Send save immediately - no delay to avoid device timeout/reboot
        usleep(10000);  // Minimal delay
        post("witsensor: sending save command immediately");
        unsigned char save[] = {0xFF, 0xAA, 0x00, 0x00, 0x00};
        witsensor_ble_simpleble_write_data(x->ble_data, save, sizeof(save));
    }
    
    post("witsensor: setname complete - expect disconnect/reboot");
}
