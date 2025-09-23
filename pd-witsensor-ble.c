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
#include <unistd.h>
#include <pthread.h>
#include <time.h>

#include "m_pd.h"

// BLE includes
#include "witsensor_ble_simpleble.h"

#define WITSENSOR_MAJOR_VERSION 0
#define WITSENSOR_MINOR_VERSION 1
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

    // Threading
    pthread_t scan_thread;
    pthread_t data_thread;
    int should_stop;
    
    // PureData outlets
    t_outlet *data_out;
    t_outlet *status_out;
    
    // Clock for polling
    t_clock *poll_clock;
    t_float poll_interval;
    
    // BLE specific
    witsensor_ble_simpleble_t *ble_data;
    
    // Pd instance for pd_queue_mess
    t_pdinstance *pd_instance;
    
} t_witsensor;

t_class *witsensor_class;

// Forward declarations
static void witsensor_scan_devices(t_witsensor *x, t_float timeout_sec);
static void witsensor_get_scan_results(t_witsensor *x);
static void witsensor_connect_device(t_witsensor *x, t_symbol *device_name);
static void witsensor_disconnect(t_witsensor *x);
static void witsensor_process_data(t_witsensor *x, unsigned char *data, int length);
static void witsensor_send_sensor_data(t_witsensor *x);
static void witsensor_send_quaternion_data(t_witsensor *x);
static void witsensor_poll_tick(t_witsensor *x);
// pd_queue_mess marshaling
typedef struct _queued_pkt { int length; unsigned char bytes[64]; } t_queued_pkt;
static void witsensor_pd_queued_handler(t_pd *obj, void *data);
static void witsensor_ble_data_callback(void *user_data, unsigned char *data, int length);
void witsensor_pd_scan_complete_handler(t_pd *obj, void *data);
// New Pd-thread handlers for status output
typedef struct _queued_flag { int value; } t_queued_flag;
typedef struct _queued_device { char *tag; char *addr; char *id; } t_queued_device;
void witsensor_pd_scanning_handler(t_pd *obj, void *data);
void witsensor_pd_device_found_handler(t_pd *obj, void *data);

// BLE data callback function
static void witsensor_ble_data_callback(void *user_data, unsigned char *data, int length) {
    t_witsensor *x = (t_witsensor *)user_data;
    if (!x) return;
    if (length <= 0 || length > 64) return;
    t_queued_pkt *pkt = (t_queued_pkt *)malloc(sizeof(t_queued_pkt));
    if (!pkt) return;
    pkt->length = length;
    memcpy(pkt->bytes, data, length);
    pd_queue_mess(x->pd_instance, (t_pd *)x, pkt, witsensor_pd_queued_handler);
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


// Process incoming sensor data
static void witsensor_process_data(t_witsensor *x, unsigned char *data, int length) {
    if (!x || !data || length < 1) {
        post("witsensor: ERROR - Invalid data processing parameters");
        return;
    }
    
    unsigned char packet_type = data[0];
    
    if (packet_type == 0x61 && length >= 37) {
        
        // Accelerometer, Gyroscope, and Angle data (9 floats + 1 byte type)
        float *values = (float*)(data + 1);
        
        // Update sensor data with bounds checking
        x->accel_x = values[0];
        x->accel_y = values[1];
        x->accel_z = values[2];
        x->gyro_x = values[3];
        x->gyro_y = values[4];
        x->gyro_z = values[5];
        x->angle_x = values[6];
        x->angle_y = values[7];
        x->angle_z = values[8];

        // Output to Pure Data outlets
        witsensor_send_sensor_data(x);
        
    } else if (packet_type == 0x71) {
        // Quaternion data (4 floats)
        if (length >= 17) {
            float *values = (float*)(data + 1);
            // Sanity check floats are finite
            if (values[0] == values[0] && values[1] == values[1]) {
                x->quat_w = values[0];
                x->quat_x = values[1];
                x->quat_y = values[2];
                x->quat_z = values[3];
                witsensor_send_quaternion_data(x);
                return;
            }
        }
    }
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
    
    // Accelerometer data (3 floats)
    SETFLOAT(&args[0], x->accel_x);
    SETFLOAT(&args[1], x->accel_y);
    SETFLOAT(&args[2], x->accel_z);
    outlet_anything(x->data_out, gensym("accel"), 3, args);
    
    // Gyroscope data (3 floats)
    SETFLOAT(&args[0], x->gyro_x);
    SETFLOAT(&args[1], x->gyro_y);
    SETFLOAT(&args[2], x->gyro_z);
    outlet_anything(x->data_out, gensym("gyro"), 3, args);
    
    // Angle data (3 floats)
    SETFLOAT(&args[0], x->angle_x);
    SETFLOAT(&args[1], x->angle_y);
    SETFLOAT(&args[2], x->angle_z);
    outlet_anything(x->data_out, gensym("angle"), 3, args);
}

static void witsensor_poll_tick(t_witsensor *x) {
    if (!x || !x->ble_data) {
        post("witsensor: ERROR - Invalid object or BLE data");
        return;
    }
    
    // Check connection status
    x->is_connected = witsensor_ble_simpleble_is_connected(x->ble_data);
    x->is_scanning = witsensor_ble_simpleble_is_scanning(x->ble_data);

    // Only poll if connected and interval is set
    if (x->poll_interval > 0 && x->is_connected) {
        // Request quaternion data only (per manual: FF AA 27 51 00)
        unsigned char cmd_quat[] = {0xFF, 0xAA, 0x27, 0x51, 0x00};
        (void)witsensor_ble_simpleble_write_data(x->ble_data, cmd_quat, sizeof(cmd_quat));
        
        // Schedule next poll
        clock_delay(x->poll_clock, x->poll_interval);
    } else if (x->poll_interval > 0 && !x->is_connected) {
        // Stop polling when disconnected
        post("witsensor: disconnected, stopping quaternion polling");
        x->poll_interval = 0;
    }
}

// Drain queued packets on Pd scheduler thread
// pd_queue_mess handler
static void witsensor_pd_queued_handler(t_pd *obj, void *data) {
    if (!obj || !data) return;
    t_witsensor *x = (t_witsensor *)obj;
    t_queued_pkt *pkt = (t_queued_pkt *)data;
    witsensor_process_data(x, pkt->bytes, pkt->length);
    free(pkt);
}

// Scan for WIT devices
static void witsensor_scan_devices(t_witsensor *x, t_float timeout_sec) {
    post("witsensor: scanning for BLE devices...");
    if (!x->ble_data) { post("witsensor: BLE not initialized"); return; }
    if (timeout_sec > 0) {
        int ms = (int)(timeout_sec * 1000.0f);
        if (ms < 100) ms = 100;
        x->ble_data->scan_timeout_ms = ms;
    }
    witsensor_ble_simpleble_start_scanning(x->ble_data);
}

// Get scan results
static void witsensor_get_scan_results(t_witsensor *x) {
    if (!x->ble_data) { post("witsensor: BLE not initialized"); return; }
    witsensor_ble_simpleble_get_scan_results(x->ble_data);
}

// Connect to device by name or address
static void witsensor_connect_device(t_witsensor *x, t_symbol *device_identifier) {
    if (device_identifier && device_identifier->s_name) {
        strcpy(x->device_name, device_identifier->s_name);
    }
    
    post("witsensor: connecting to device: %s", x->device_name);
    
    if (x->ble_data) {
        // Try connecting by address first, then by identifier as fallback
        int connected = 0;
        
        // First try: connect by address
        connected = witsensor_ble_simpleble_connect_by_address(x->ble_data, x->device_name);
        if (!connected) {
            // Second try: connect by identifier
            post("witsensor: address connection failed, trying identifier...");
            connected = witsensor_ble_simpleble_connect_by_identifier(x->ble_data, x->device_name);
        }
        
        if (connected) {
            x->is_connected = 1;
            t_atom a; SETFLOAT(&a, 1);
            outlet_anything(x->status_out, gensym("connected"), 1, &a);
            // Default to 50 Hz on connect and set bandwidth ~42 Hz, then enable quat polling 10 Hz
            unsigned char cmd_unlock[] = {0xFF, 0xAA, 0x69, 0x88, 0xB5};
            witsensor_ble_simpleble_write_data(x->ble_data, cmd_unlock, sizeof(cmd_unlock));
            usleep(50000);
            unsigned char cmd_rate[] = {0xFF, 0xAA, 0x03, 0x08, 0x00}; // 50 Hz
            witsensor_ble_simpleble_write_data(x->ble_data, cmd_rate, sizeof(cmd_rate));
            usleep(30000);
            unsigned char cmd_bw[] = {0xFF, 0xAA, 0x1F, 0x03, 0x00}; // bandwidth 42 Hz
            witsensor_ble_simpleble_write_data(x->ble_data, cmd_bw, sizeof(cmd_bw));
            unsigned char cmd_save[] = {0xFF, 0xAA, 0x00, 0x00, 0x00};
            witsensor_ble_simpleble_write_data(x->ble_data, cmd_save, sizeof(cmd_save));
            // Do not start quaternion polling by default
            x->poll_interval = 0;
        } else {
            post("witsensor: failed to connect to device: %s", x->device_name);
        }
        return;
    } else {
        post("witsensor: BLE not initialized");
    }
}

// Connect to device by address
static void witsensor_connect_by_address(t_witsensor *x, t_symbol *device_address) {
    if (device_address && device_address->s_name) {
        strcpy(x->device_address, device_address->s_name);
        post("witsensor: connecting to device by address: %s", x->device_address);
        
        if (x->ble_data) {
            witsensor_ble_simpleble_connect_by_address(x->ble_data, x->device_address);
        } else {
            post("witsensor: BLE not initialized");
        }
    }
}

// Disconnect from device
static void witsensor_disconnect(t_witsensor *x) {
    if (!x->is_connected) {
        post("witsensor: no device connected");
        return;
    }
    
    if (x->ble_data) {
        witsensor_ble_simpleble_disconnect(x->ble_data);
        x->is_connected = 0;
        x->should_stop = 1;
        t_atom a; SETFLOAT(&a, 0);
        outlet_anything(x->status_out, gensym("connected"), 1, &a);
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
        usleep(100000); // 100ms delay
        
        // Map Hz to manual codes: 0x01:0.1, 0x02:0.5, 0x03:1, 0x04:2, 0x05:5,
        // 0x06:10, 0x07:20, 0x08:50, 0x09:100, 0x0B:200
        unsigned char rate_code = 0x06; // default 10Hz
        if (rate <= 0.15f) rate_code = 0x01;
        else if (rate <= 0.75f) rate_code = 0x02;
        else if (rate <= 1.5f) rate_code = 0x03;
        else if (rate <= 3.0f) rate_code = 0x04;
        else if (rate <= 7.5f) rate_code = 0x05;
        else if (rate <= 15.0f) rate_code = 0x06;
        else if (rate <= 35.0f) rate_code = 0x07;
        else if (rate <= 75.0f) rate_code = 0x08;
        else if (rate <= 150.0f) rate_code = 0x09;
        else rate_code = 0x0B; // 200Hz

        unsigned char cmd_rate[] = {0xFF, 0xAA, 0x03, rate_code, 0x00};
        witsensor_ble_simpleble_write_data(x->ble_data, cmd_rate, sizeof(cmd_rate));
        
        // Save configuration
        unsigned char cmd_save[] = {0xFF, 0xAA, 0x00, 0x00, 0x00}; // Save command
        witsensor_ble_simpleble_write_data(x->ble_data, cmd_save, sizeof(cmd_save));
        
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
    // Map desired cutoff to closest supported code: 256,188,98,42,20,10,5 Hz
    unsigned char bw_code = 0x03; // 42 Hz default
    if (hz >= 220.0f) bw_code = 0x00;
    else if (hz >= 140.0f) bw_code = 0x01;
    else if (hz >= 70.0f) bw_code = 0x02;
    else if (hz >= 30.0f) bw_code = 0x03;
    else if (hz >= 15.0f) bw_code = 0x04;
    else if (hz >= 7.0f) bw_code = 0x05;
    else bw_code = 0x06;
    unsigned char cmd_unlock[] = {0xFF, 0xAA, 0x69, 0x88, 0xB5};
    witsensor_ble_simpleble_write_data(x->ble_data, cmd_unlock, sizeof(cmd_unlock));
    usleep(50000);
    unsigned char cmd_bw[] = {0xFF, 0xAA, 0x1F, bw_code, 0x00};
    witsensor_ble_simpleble_write_data(x->ble_data, cmd_bw, sizeof(cmd_bw));
    
    
    unsigned char cmd_save[] = {0xFF, 0xAA, 0x00, 0x00, 0x00};
    witsensor_ble_simpleble_write_data(x->ble_data, cmd_save, sizeof(cmd_save));
    t_atom a; SETFLOAT(&a, hz);
    outlet_anything(x->status_out, gensym("bandwidth"), 1, &a);
}

// Set polling interval (in milliseconds) - for backward compatibility
static void witsensor_set_pollquat(t_witsensor *x, t_float interval) {
    // Interpret argument as desired quaternion polling Hz (0 to disable)
    if (interval <= 0) {
        x->poll_interval = 0;
        clock_unset(x->poll_clock);
        outlet_anything(x->status_out, gensym("pollquat"), 0, NULL);
        return;
    }
    if (interval > 50.0f) interval = 50.0f; // cap at 50 Hz to avoid BLE congestion
    int period_ms = (int)(1000.0f / interval);
    if (period_ms < 1) period_ms = 1;
    x->poll_interval = period_ms;
    if (x->is_connected) {
        clock_unset(x->poll_clock);
        clock_delay(x->poll_clock, x->poll_interval);
        t_atom args[2];
        SETFLOAT(&args[0], 1000.0f / x->poll_interval);
        SETFLOAT(&args[1], x->poll_interval);
        outlet_anything(x->status_out, gensym("pollquat"), 2, args);
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
    x->temp_bytes_count = 0;
    x->pd_instance = pd_this;
    
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
    
    return (void *)x;
}

// Destructor
static void witsensor_free(t_witsensor *x) {
    witsensor_disconnect(x);
    pd_queue_cancel((t_pd *)x);
    if (x->ble_data) {
        witsensor_ble_simpleble_destroy(x->ble_data);
    }
    clock_free(x->poll_clock);
}

// WIT sensor command functions

static void witsensor_calibrate(t_witsensor *x) {
    if (!x->is_connected || !x->ble_data) { post("witsensor: not connected to device"); return; }
    
    post("witsensor: starting accelerometer calibration - keep sensor still");
    
    // Unlock
    unsigned char cmd_unlock[] = {0xFF, 0xAA, 0x69, 0x88, 0xB5};
    witsensor_ble_simpleble_write_data(x->ble_data, cmd_unlock, sizeof(cmd_unlock));
    usleep(50000);
    
    // Accelerometer calibration (keep still)
    unsigned char cmd_accal[] = {0xFF, 0xAA, 0x01, 0x01, 0x00};
    witsensor_ble_simpleble_write_data(x->ble_data, cmd_accal, sizeof(cmd_accal));
    
    // Save (sensor applies over ~10s)
    unsigned char cmd_save[] = {0xFF, 0xAA, 0x00, 0x00, 0x00};
    witsensor_ble_simpleble_write_data(x->ble_data, cmd_save, sizeof(cmd_save));
    
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
    
    
    unsigned char cmd_save[] = {0xFF, 0xAA, 0x00, 0x00, 0x00};
    witsensor_ble_simpleble_write_data(x->ble_data, cmd_save, sizeof(cmd_save));
    t_atom a; SETFLOAT(&a, axis_count);
    outlet_anything(x->status_out, gensym("axis"), 1, &a);
}

static void witsensor_magcal_start(t_witsensor *x) {
    if (!x->is_connected || !x->ble_data) { post("witsensor: not connected to device"); return; }
    unsigned char cmd_unlock[] = {0xFF, 0xAA, 0x69, 0x88, 0xB5};
    witsensor_ble_simpleble_write_data(x->ble_data, cmd_unlock, sizeof(cmd_unlock));
    usleep(50000);
    unsigned char cmd_start[] = {0xFF, 0xAA, 0x01, 0x07, 0x00};
    witsensor_ble_simpleble_write_data(x->ble_data, cmd_start, sizeof(cmd_start));
    outlet_anything(x->status_out, gensym("magcal"), 1, (t_atom[]){(t_atom){.a_type=A_SYMBOL,.a_w.w_symbol=gensym("start")}});
}

static void witsensor_magcal_stop(t_witsensor *x) {
    if (!x->is_connected || !x->ble_data) { post("witsensor: not connected to device"); return; }
    unsigned char cmd_stop[] = {0xFF, 0xAA, 0x01, 0x00, 0x00};
    witsensor_ble_simpleble_write_data(x->ble_data, cmd_stop, sizeof(cmd_stop));
    unsigned char cmd_save[] = {0xFF, 0xAA, 0x00, 0x00, 0x00};
    witsensor_ble_simpleble_write_data(x->ble_data, cmd_save, sizeof(cmd_save));
    outlet_anything(x->status_out, gensym("magcal"), 1, (t_atom[]){(t_atom){.a_type=A_SYMBOL,.a_w.w_symbol=gensym("stop")}});
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
    unsigned char cmd_save[] = {0xFF, 0xAA, 0x00, 0x00, 0x00};
    witsensor_ble_simpleble_write_data(x->ble_data, cmd_save, sizeof(cmd_save));
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
    
    class_addmethod(witsensor_class, (t_method)witsensor_scan_devices, gensym("scan"), A_DEFFLOAT, 0);
    class_addmethod(witsensor_class, (t_method)witsensor_get_scan_results, gensym("results"), 0);
    class_addmethod(witsensor_class, (t_method)witsensor_connect_device, gensym("connect"), A_SYMBOL, 0);
    class_addmethod(witsensor_class, (t_method)witsensor_connect_by_address, gensym("connect-addr"), A_SYMBOL, 0);
    class_addmethod(witsensor_class, (t_method)witsensor_disconnect, gensym("disconnect"), 0);
    class_addmethod(witsensor_class, (t_method)witsensor_set_pollquat, gensym("pollquat"), A_FLOAT, 0);
    class_addmethod(witsensor_class, (t_method)witsensor_set_rate, gensym("rate"), A_FLOAT, 0);
    class_addmethod(witsensor_class, (t_method)witsensor_set_bandwidth, gensym("bandwidth"), A_FLOAT, 0);
    class_addmethod(witsensor_class, (t_method)witsensor_calibrate, gensym("calibrate"), 0);
    class_addmethod(witsensor_class, (t_method)witsensor_axis, gensym("axis"), A_FLOAT, 0);
    class_addmethod(witsensor_class, (t_method)witsensor_magcal_start, gensym("magcal-start"), 0);
    class_addmethod(witsensor_class, (t_method)witsensor_magcal_stop, gensym("magcal-stop"), 0);
    // z-axis zero (requires 6-axis setting)
    class_addmethod(witsensor_class, (t_method)witsensor_zzero, gensym("zzero"), 0);
    class_addmethod(witsensor_class, (t_method)witsensor_version, gensym("version"), 0);
    
    witsensor_version();
}
