/* witsensor_ble_simpleble.c
 * Cross-platform BLE implementation using SimpleBLE
 * Supports macOS, Windows, and Linux
 * 
 * Copyright (c) 2025 pd-witsensor contributors
 * Licensed under Business Source License 1.1 (BUSL-1.1)
 * See LICENSE for details
 */

#include "m_pd.h"
#include "witsensor_ble_simpleble.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <time.h>
#include <pthread.h>
#ifdef __APPLE__
#import <CoreBluetooth/CoreBluetooth.h>
#endif

// SimpleBLE includes - use simplecble headers directly
#include <simplecble/simpleble.h>
#include "m_pd.h"
// Pd-thread handler to announce scan completion and print cached devices
extern void witsensor_pd_scan_complete_handler(t_pd *obj, void *data);
// Forward declarations for Pd-thread status handlers and payloads
typedef struct _queued_flag { int value; } t_queued_flag;
typedef struct _queued_device { char *tag; char *addr; char *id; } t_queued_device;
void witsensor_pd_scanning_handler(t_pd *obj, void *data);
void witsensor_pd_device_found_handler(t_pd *obj, void *data);

// Forward declaration for notification callback used in connect path
static void simpleble_on_data_received(simpleble_peripheral_t peripheral, simpleble_uuid_t service, simpleble_uuid_t characteristic, const uint8_t *data, size_t length, void *user_data);

// Helper function to get signed int16 (from Python SDK)
static int16_t get_signed_int16(uint16_t num) {
    if (num >= 32768) {
        return (int16_t)(num - 65536);
    }
    return (int16_t)num;
}

// WIT sensor service and characteristic UUIDs (from Python SDK)
#define WIT_SERVICE_UUID_STR "0000ffe5-0000-1000-8000-00805f9a34fb"
#define WIT_READ_CHARACTERISTIC_UUID_STR "0000ffe4-0000-1000-8000-00805f9a34fb"
#define WIT_WRITE_CHARACTERISTIC_UUID_STR "0000ffe9-0000-1000-8000-00805f9a34fb"

// Convert string UUIDs to SimpleBLE format
static simpleble_uuid_t WIT_SERVICE_UUID = {.value = WIT_SERVICE_UUID_STR};
/* unused: static simpleble_uuid_t WIT_READ_CHARACTERISTIC_UUID = {.value = WIT_READ_CHARACTERISTIC_UUID_STR}; */
static simpleble_uuid_t WIT_WRITE_CHARACTERISTIC_UUID = {.value = WIT_WRITE_CHARACTERISTIC_UUID_STR};

// Forward declarations

// Forward declare helpers used by macOS scan tasks
static void _clear_cached_results(witsensor_ble_simpleble_t *ble);
#ifdef __APPLE__
static void _enumerate_results_mac(witsensor_ble_simpleble_t *ble);
#endif

// Helper function to output scan results via status outlet
static void _output_scan_results(witsensor_ble_simpleble_t *ble_data) {
    if (!ble_data || !ble_data->adapter) return;
    
    size_t count = simpleble_adapter_scan_get_results_count(ble_data->adapter);
    for (size_t i = 0; i < count; i++) {
        simpleble_peripheral_t p = simpleble_adapter_scan_get_results_handle(ble_data->adapter, i);
        if (!p) continue;
        
        char *addr = simpleble_peripheral_address(p);
        char *id = simpleble_peripheral_identifier(p);
        
        if (id) {
            // Send via status outlet: device wit|other <addr> <id>
            const char *tag = (strstr(id, "WT") != NULL) ? "wit" : "other";
            if (ble_data->pd_instance && ble_data->pd_obj) {
                t_queued_device *d = (t_queued_device *)malloc(sizeof(t_queued_device));
                if (d) {
                    d->tag = strdup(tag);
                    d->addr = addr ? strdup(addr) : NULL;
                    d->id = strdup(id);
                    pd_queue_mess((t_pdinstance*)ble_data->pd_instance, (t_pd*)ble_data->pd_obj, d, witsensor_pd_device_found_handler);
                }
            }
        }
        
        if (addr) simpleble_free(addr);
        if (id) simpleble_free(id);
    }
}

// Watchdog thread to force-stop scan after timeout (platform-agnostic)
typedef struct { witsensor_ble_simpleble_t *ble; int ms; } _scan_stop_args;
static void* _scan_stop_thread(void *arg) {
    _scan_stop_args *a = (_scan_stop_args *)arg;
    if (!a || !a->ble) { if (a) free(a); return NULL; }
    int ms = a->ms + 200; // small slack
    struct timespec req = { ms / 1000, (ms % 1000) * 1000000L };
    nanosleep(&req, NULL);
    if (a->ble->is_scanning) {
        if (a->ble->adapter) simpleble_adapter_scan_stop(a->ble->adapter);
        a->ble->is_scanning = 0;
        post("WITSensorBLE: watchdog forced scan stop");
        // Don't send status message here - let the primary timer handle it
    }
    free(a);
    return NULL;
}

#ifdef __APPLE__
#include <dispatch/dispatch.h>
typedef void (*ble_main_fn)(void *);
static void ble_main_async(ble_main_fn fn, void *ctx) {
    dispatch_async_f(dispatch_get_main_queue(), ctx, fn);
}
static void ble_main_after_ms(uint64_t delay_ms, ble_main_fn fn, void *ctx) {
    dispatch_time_t when = dispatch_time(DISPATCH_TIME_NOW, (int64_t)delay_ms * 1000000LL);
    dispatch_after_f(when, dispatch_get_main_queue(), ctx, fn);
}

// Schedule scan stop on main queue after timeout
static void _scan_stop_task(void *v) {
    witsensor_ble_simpleble_t *ble = (witsensor_ble_simpleble_t *)v;
    if (!ble || !ble->adapter) return;
    simpleble_adapter_scan_stop(ble->adapter);
    ble->is_scanning = 0;
    if (ble->pd_instance && ble->pd_obj) {
        t_queued_flag *q = (t_queued_flag *)malloc(sizeof(t_queued_flag));
        if (q) { q->value = 0; pd_queue_mess((t_pdinstance*)ble->pd_instance, (t_pd*)ble->pd_obj, q, witsensor_pd_scanning_handler); }
    }
}
#endif

#ifdef __APPLE__
static void _enumerate_results_mac(witsensor_ble_simpleble_t *ble) {
    if (!ble || !ble->adapter) return;
    size_t results_count = simpleble_adapter_scan_get_results_count(ble->adapter);
    // Refresh cache
    for (unsigned long i = 0; i < ble->cached_count; i++) {
        if (ble->cached_ids) free(ble->cached_ids[i]);
        if (ble->cached_addrs) free(ble->cached_addrs[i]);
    }
    free(ble->cached_ids); ble->cached_ids = NULL;
    free(ble->cached_addrs); ble->cached_addrs = NULL;
    ble->cached_count = (unsigned long)results_count;
    if (results_count == 0) { return; }
    ble->cached_ids = (char**)calloc(results_count, sizeof(char*));
    ble->cached_addrs = (char**)calloc(results_count, sizeof(char*));
    for (size_t i = 0; i < results_count; i++) {
        simpleble_peripheral_t peripheral = simpleble_adapter_scan_get_results_handle(ble->adapter, i);
        if (!peripheral) continue;
        char *peripheral_address = simpleble_peripheral_address(peripheral);
        char *peripheral_identifier = simpleble_peripheral_identifier(peripheral);
        if (peripheral_identifier && peripheral_address) {
            ble->cached_ids[i] = strdup(peripheral_identifier);
            ble->cached_addrs[i] = strdup(peripheral_address);
        }
        if (peripheral_address) simpleble_free(peripheral_address);
        if (peripheral_identifier) simpleble_free(peripheral_identifier);
    }
}
#endif

// Delayed scan function to avoid GUI threading issues
static void witsensor_ble_simpleble_scan_delayed(witsensor_ble_simpleble_t *ble_data) {
    if (!ble_data) return;
    
    post("WITSensorBLE: Starting scan...");
    if (!witsensor_ble_simpleble_ensure_initialized(ble_data)) {
        post("WITSensorBLE: Failed to initialize BLE system for scan");
        return;
    }
    _clear_cached_results(ble_data);
    ble_data->is_scanning = 1;
    simpleble_adapter_scan_for(ble_data->adapter, 4000);
    ble_data->is_scanning = 0;
#ifdef __APPLE__
    _enumerate_results_mac(ble_data);
#endif
}

// Helpers to manage cached scan results (CoreBluetooth thread safe: no Pd calls)
static void _clear_cached_results(witsensor_ble_simpleble_t *ble) {
    if (!ble) return;
    ble->scan_found_count = 0;
    ble->adapter_id[0] = '\0';
    ble->adapter_addr[0] = '\0';
    if (ble->cached_ids) {
        for (unsigned long i = 0; i < ble->cached_count; i++) free(ble->cached_ids[i]);
        free(ble->cached_ids);
        ble->cached_ids = NULL;
    }
    if (ble->cached_addrs) {
        for (unsigned long i = 0; i < ble->cached_count; i++) free(ble->cached_addrs[i]);
        free(ble->cached_addrs);
        ble->cached_addrs = NULL;
    }
    ble->cached_count = 0;
}

static void _append_cached_result(witsensor_ble_simpleble_t *ble, const char *id, const char *addr) {
    if (!ble || !id || !addr) return;
    if (!ble->is_scanning) return; // avoid races after stop
    unsigned long n = ble->cached_count;
    char **new_ids = (char**)realloc(ble->cached_ids, (n+1) * sizeof(char*));
    char **new_addrs = (char**)realloc(ble->cached_addrs, (n+1) * sizeof(char*));
    if (!new_ids || !new_addrs) {
        // Allocation failed; keep old arrays
        return;
    }
    ble->cached_ids = new_ids;
    ble->cached_addrs = new_addrs;
    ble->cached_ids[n] = strdup(id);
    ble->cached_addrs[n] = strdup(addr);
    ble->cached_count = n + 1;
}

// SimpleBLE scan callbacks
static void simpleble_on_scan_start(simpleble_adapter_t adapter, void *user_data) {
    witsensor_ble_simpleble_t *ble_data = (witsensor_ble_simpleble_t *)user_data;
    if (!ble_data) return;
    _clear_cached_results(ble_data);
    // cache adapter id/addr for debug
    char *aid = simpleble_adapter_identifier(adapter);
    char *aad = simpleble_adapter_address(adapter);
    if (aid) { snprintf(ble_data->adapter_id, sizeof(ble_data->adapter_id), "%s", aid); simpleble_free(aid); }
    if (aad) { snprintf(ble_data->adapter_addr, sizeof(ble_data->adapter_addr), "%s", aad); simpleble_free(aad); }
}

static void simpleble_on_scan_stop(simpleble_adapter_t adapter, void *user_data) {
    (void)adapter;
    witsensor_ble_simpleble_t *ble_data = (witsensor_ble_simpleble_t *)user_data;
    if (!ble_data) return;
    // Emit scanning 0 via Pd thread
    if (ble_data->pd_instance && ble_data->pd_obj) {
        t_queued_flag *q = (t_queued_flag *)malloc(sizeof(t_queued_flag));
        if (q) { q->value = 0; pd_queue_mess((t_pdinstance*)ble_data->pd_instance, (t_pd*)ble_data->pd_obj, q, witsensor_pd_scanning_handler); }
    }
}

static void simpleble_on_scan_found(simpleble_adapter_t adapter, simpleble_peripheral_t peripheral, void *user_data) {
    (void)adapter;
    witsensor_ble_simpleble_t *ble_data = (witsensor_ble_simpleble_t *)user_data;
    if (!ble_data || !peripheral) return;
    simpleble_peripheral_t p = peripheral;
    char *addr = simpleble_peripheral_address(p);
    char *id = simpleble_peripheral_identifier(p);
    if (id && addr) {
        _append_cached_result(ble_data, id, addr);
        ble_data->scan_found_count++;
        // Emit device status via Pd thread: device wit|other <id>
        if (ble_data->pd_instance && ble_data->pd_obj) {
            t_queued_device *d = (t_queued_device *)malloc(sizeof(t_queued_device));
            if (d) {
                const char *tag = (strstr(id, "WT") != NULL) ? "wit" : "other";
                d->tag = strdup(tag);
                d->addr = addr ? strdup(addr) : NULL;
                d->id = id ? strdup(id) : NULL;
                pd_queue_mess((t_pdinstance*)ble_data->pd_instance, (t_pd*)ble_data->pd_obj, d, witsensor_pd_device_found_handler);
            }
        }
    }
    if (addr) simpleble_free(addr);
    if (id) simpleble_free(id);
}

static void simpleble_on_connected(void *user_data, void *peripheral) {
    (void)peripheral; (void)user_data;
    post("WITSensorBLE: Connected to device");
}

static void simpleble_on_disconnected(void *user_data, void *peripheral) {
    (void)peripheral; (void)user_data;
    post("WITSensorBLE: Disconnected from device");
}

static void simpleble_on_data_received(simpleble_peripheral_t peripheral, simpleble_uuid_t service, simpleble_uuid_t characteristic, const uint8_t *data, size_t length, void *user_data) {
    (void)peripheral; (void)service; (void)characteristic;
    witsensor_ble_simpleble_t *ble_data = (witsensor_ble_simpleble_t *)user_data;
    if (!ble_data) return;
    
        // Parse WIT sensor data
        if (length >= 20 && data[0] == 0x55) {
            unsigned char packet_type = data[1];

            switch (packet_type) {
                case 0x61: { // Accel, Gyro, Angles
                    // Accel
                    int16_t ax = get_signed_int16(data[3] << 8 | data[2]);
                    int16_t ay = get_signed_int16(data[5] << 8 | data[4]);
                    int16_t az = get_signed_int16(data[7] << 8 | data[6]);
                    float accel_x = (float)ax / 32768.0f * 16.0f; // g
                    float accel_y = (float)ay / 32768.0f * 16.0f; // g
                    float accel_z = (float)az / 32768.0f * 16.0f; // g

                    // Gyro (degrees/second @ 2000 dps full-scale)
                    int16_t gx = get_signed_int16(data[9] << 8 | data[8]);
                    int16_t gy = get_signed_int16(data[11] << 8 | data[10]);
                    int16_t gz = get_signed_int16(data[13] << 8 | data[12]);
                    float gyro_x = (float)gx / 32768.0f * 2000.0f;
                    float gyro_y = (float)gy / 32768.0f * 2000.0f;
                    float gyro_z = (float)gz / 32768.0f * 2000.0f;

                    // Angles (degrees)
                    int16_t ang_x = get_signed_int16(data[15] << 8 | data[14]);
                    int16_t ang_y = get_signed_int16(data[17] << 8 | data[16]);
                    int16_t ang_z = get_signed_int16(data[19] << 8 | data[18]);
                    float angle_x = (float)ang_x / 32768.0f * 180.0f;
                    float angle_y = (float)ang_y / 32768.0f * 180.0f;
                    float angle_z = (float)ang_z / 32768.0f * 180.0f;

                    if (ble_data->data_callback) {
                        unsigned char callback_data[37];
                        callback_data[0] = 0x61;
                        float *data_ptr = (float*)(callback_data + 1);
                        data_ptr[0] = accel_x; data_ptr[1] = accel_y; data_ptr[2] = accel_z;
                        data_ptr[3] = gyro_x;  data_ptr[4] = gyro_y;  data_ptr[5] = gyro_z;
                        data_ptr[6] = angle_x; data_ptr[7] = angle_y; data_ptr[8] = angle_z;
                        ble_data->data_callback(ble_data->pd_obj, callback_data, 37);
                    }
                    break;
                }

                case 0x71: { // Quaternion only
                    if (length < 12) break;
                    // Many firmwares send quaternion directly under 0x71 frame; interpret as q0..q3 next words
                    int16_t q0 = get_signed_int16(data[5] << 8 | data[4]);
                    int16_t q1 = get_signed_int16(data[7] << 8 | data[6]);
                    int16_t q2 = get_signed_int16(data[9] << 8 | data[8]);
                    int16_t q3 = get_signed_int16(data[11] << 8 | data[10]);
                    float quat_w = (float)q0 / 32768.0f;
                    float quat_x = (float)q1 / 32768.0f;
                    float quat_y = (float)q2 / 32768.0f;
                    float quat_z = (float)q3 / 32768.0f;

                    if (ble_data->data_callback) {
                        unsigned char callback_data[17];
                        callback_data[0] = 0x71;
                        float *data_ptr = (float*)(callback_data + 1);
                        data_ptr[0] = quat_w; data_ptr[1] = quat_x; data_ptr[2] = quat_y; data_ptr[3] = quat_z;
                        ble_data->data_callback(ble_data->pd_obj, callback_data, 17);
                    }
                    break;
                }

                default:
                    // Ignore other packet types for now
                    break;
            }
        }
    
    // Update performance monitoring
    ble_data->data_count++;
    ble_data->last_data_time = time(NULL);
}

// Create BLE data structure
witsensor_ble_simpleble_t *witsensor_ble_simpleble_create(void) {
    witsensor_ble_simpleble_t *ble_data = (witsensor_ble_simpleble_t *)calloc(1, sizeof(witsensor_ble_simpleble_t));
    if (!ble_data) {
        post("WITSensorBLE: Failed to allocate memory");
        return NULL;
    }
    
    // Initialize with lazy loading - don't call SimpleBLE functions yet
    ble_data->adapter = NULL;
    ble_data->peripheral = NULL;
    ble_data->is_scanning = 0;
    ble_data->is_connected = 0;
    ble_data->cached_ids = NULL;
    ble_data->cached_addrs = NULL;
    ble_data->cached_count = 0;
    
    ble_data->scan_timeout_ms = 6000;
    post("WITSensorBLE: BLE data structure created (lazy initialization)");
    return ble_data;
}

// Destroy BLE data structure - COMPLETELY CRASH-SAFE
void witsensor_ble_simpleble_destroy(witsensor_ble_simpleble_t *ble_data) {
    if (ble_data) {
        // Don't call SimpleBLE release functions - they might crash
        // Just free the memory
        if (ble_data->cached_ids) {
            for (unsigned long i = 0; i < ble_data->cached_count; i++) free(ble_data->cached_ids[i]);
            free(ble_data->cached_ids);
        }
        if (ble_data->cached_addrs) {
            for (unsigned long i = 0; i < ble_data->cached_count; i++) free(ble_data->cached_addrs[i]);
            free(ble_data->cached_addrs);
        }
        free(ble_data);
    }
}

// Ensure BLE is initialized (lazy initialization) - REAL SimpleBLE for console
int witsensor_ble_simpleble_ensure_initialized(witsensor_ble_simpleble_t *ble_data) {
    if (!ble_data) return 0;
    
    // If already initialized, return success
    if (ble_data->adapter) {
        return 1;
    }
    
    post("WITSensorBLE: Initializing SimpleBLE system...");
    
    // Skip initialization if permissions might be missing
    // This prevents crashes - the object will be created but non-functional
    // until permissions are granted and a BLE operation is attempted
    post("WITSensorBLE: BLE initialization deferred - will initialize on first BLE operation");
    return 1; // Return success but don't actually initialize
}

// Start scanning for devices - always GUI-safe (main thread)
void witsensor_ble_simpleble_start_scanning(witsensor_ble_simpleble_t *ble_data) {
    if (!ble_data) return;
    
    post("WITSensorBLE: Starting BLE scan ...");
    
    // Authorization already checked on object creation
    
    // Try to initialize BLE now
    if (!ble_data->adapter) {
        post("WITSensorBLE: Attempting BLE initialization on first scan...");
        
        // Try to get adapter count first
        size_t adapter_count = simpleble_adapter_get_count();
        if (adapter_count == 0) {
            post("WITSensorBLE: No BLE adapters found - check Bluetooth permissions in System Settings → Privacy & Security → Bluetooth");
            return;
        }
        
        // Get the first adapter
        ble_data->adapter = simpleble_adapter_get_handle(0);
        if (!ble_data->adapter) {
            post("WITSensorBLE: Failed to get adapter - check Bluetooth permissions in System Settings → Privacy & Security → Bluetooth");
            return;
        }
        
        // Set up callbacks
        simpleble_adapter_set_callback_on_scan_start(ble_data->adapter, simpleble_on_scan_start, ble_data);
        simpleble_adapter_set_callback_on_scan_stop(ble_data->adapter, simpleble_on_scan_stop, ble_data);
        simpleble_adapter_set_callback_on_scan_found(ble_data->adapter, simpleble_on_scan_found, ble_data);
        
        post("WITSensorBLE: BLE system initialized successfully");
    }
    _clear_cached_results(ble_data);
    ble_data->is_scanning = 1;
    // Emit scanning 1 via Pd thread
    if (ble_data->pd_instance && ble_data->pd_obj) {
        t_queued_flag *q = (t_queued_flag *)malloc(sizeof(t_queued_flag));
        if (q) { q->value = 1; pd_queue_mess((t_pdinstance*)ble_data->pd_instance, (t_pd*)ble_data->pd_obj, q, witsensor_pd_scanning_handler); }
    }
    // Non-blocking scan: start, and let 'results' query current list
    simpleble_err_t err = simpleble_adapter_scan_start(ble_data->adapter);
    if (err != SIMPLEBLE_SUCCESS) {
        ble_data->is_scanning = 0;
        post("WITSensorBLE: scan_start failed: %d", err);
        return;
    }
    // Compute effective timeout once
    int timeout_ms = ble_data->scan_timeout_ms;
    if (timeout_ms <= 0) {
        timeout_ms = 6000;
        post("WITSensorBLE: scan timeout not set, defaulting to %d ms", timeout_ms);
    }
    post("WITSensorBLE: auto-stopping scan in %d ms", timeout_ms);
#ifdef __APPLE__
    // Schedule auto-stop on main queue
    ble_main_after_ms((uint64_t)timeout_ms, _scan_stop_task, ble_data);
#endif
    // Schedule watchdog hard-stop using a detached thread (platform-agnostic)
    _scan_stop_args *args = (_scan_stop_args *)malloc(sizeof(_scan_stop_args));
    if (args) {
        args->ble = ble_data; args->ms = timeout_ms;
        pthread_t th; if (pthread_create(&th, NULL, _scan_stop_thread, args) == 0) pthread_detach(th);
    }
}

// Stop scanning for devices
void witsensor_ble_simpleble_stop_scanning(witsensor_ble_simpleble_t *ble_data) {
    if (!ble_data) return;
    
    post("WITSensorBLE: Stopping cross-platform scan...");
    
    // Ensure BLE is initialized
    if (!witsensor_ble_simpleble_ensure_initialized(ble_data)) {
        post("WITSensorBLE: Failed to initialize BLE system");
        return;
    }
    
    // Stop scanning
    simpleble_err_t err = simpleble_adapter_scan_stop(ble_data->adapter);
    if (err != SIMPLEBLE_SUCCESS) {
        post("WITSensorBLE: Failed to stop scan, error: %d", err);
    } else {
        post("WITSensorBLE: BLE scan stopped successfully");
    }
    
    ble_data->is_scanning = 0;
}

// Get scan results - GUI-safe
void witsensor_ble_simpleble_get_scan_results(witsensor_ble_simpleble_t *ble_data) {
    if (!ble_data) return;
    
    // Ensure BLE is initialized
    if (!witsensor_ble_simpleble_ensure_initialized(ble_data)) {
        post("WITSensorBLE: Failed to initialize BLE system");
        return;
    }
    
    // Use helper function to output results (works on all platforms)
    _output_scan_results(ble_data);
}

// Connect to a device by address
int witsensor_ble_simpleble_connect_by_address(witsensor_ble_simpleble_t *ble_data, const char *address) {
    if (!ble_data || !address) return 0;
    
    post("WITSensorBLE: Connecting to device: %s", address);
    
    // Ensure BLE is initialized
    if (!witsensor_ble_simpleble_ensure_initialized(ble_data)) {
        post("WITSensorBLE: Failed to initialize BLE system");
        return 0;
    }
    
    // Find the peripheral by address
    size_t peripheral_count = simpleble_adapter_scan_get_results_count(ble_data->adapter);
    for (size_t i = 0; i < peripheral_count; i++) {
        simpleble_peripheral_t peripheral = simpleble_adapter_scan_get_results_handle(ble_data->adapter, i);
        if (peripheral) {
            char *peripheral_address = simpleble_peripheral_address(peripheral);
            if (peripheral_address && strcmp(peripheral_address, address) == 0) {
                // Connect to this peripheral
                if (simpleble_peripheral_connect(peripheral) == SIMPLEBLE_SUCCESS) {
                    ble_data->peripheral = peripheral;
                    ble_data->is_connected = 1;
                    
                    // Set up data callback for WIT sensor
                    simpleble_uuid_t service_uuid = {.value = WIT_SERVICE_UUID_STR};
                    simpleble_uuid_t read_characteristic_uuid = {.value = WIT_READ_CHARACTERISTIC_UUID_STR};
                    simpleble_peripheral_notify(peripheral, service_uuid, read_characteristic_uuid, simpleble_on_data_received, ble_data);
                    
                    post("WITSensorBLE: Connected to device: %s", address);
                    simpleble_free(peripheral_address);
                    return 1;
                } else {
                    post("WITSensorBLE: Failed to connect to device: %s", address);
                }
            }
            if (peripheral_address) simpleble_free(peripheral_address);
        }
    }
    
    post("WITSensorBLE: Device not found: %s", address);
    return 0;
}

// Connect to a device by identifier (name/UUID shown by SimpleBLE)
int witsensor_ble_simpleble_connect_by_identifier(witsensor_ble_simpleble_t *ble_data, const char *identifier) {
    if (!ble_data || !identifier) return 0;

    post("WITSensorBLE: Connecting to identifier: %s", identifier);

#ifdef __APPLE__
    if (!witsensor_ble_simpleble_ensure_initialized(ble_data)) {
        post("WITSensorBLE: Failed to initialize BLE system");
        return 0;
    }
    size_t count = simpleble_adapter_scan_get_results_count(ble_data->adapter);
    for (size_t i = 0; i < count; i++) {
        simpleble_peripheral_t p = simpleble_adapter_scan_get_results_handle(ble_data->adapter, i);
        if (!p) continue;
        char *pid = simpleble_peripheral_identifier(p);
        if (pid && strcmp(pid, identifier) == 0) {
            post("WITSensorBLE: Attempting connect to %s", pid);
            if (simpleble_peripheral_connect(p) == SIMPLEBLE_SUCCESS) {
                ble_data->peripheral = p;
                ble_data->is_connected = 1;
                simpleble_uuid_t service_uuid = {.value = WIT_SERVICE_UUID_STR};
                simpleble_uuid_t read_characteristic_uuid = {.value = WIT_READ_CHARACTERISTIC_UUID_STR};
                simpleble_peripheral_notify(p, service_uuid, read_characteristic_uuid, simpleble_on_data_received, ble_data);
                post("WITSensorBLE: Connected to %s", pid);
                if (pid) simpleble_free(pid);
                return 1;
            } else {
                post("WITSensorBLE: Failed to connect to %s", pid);
            }
        }
        if (pid) simpleble_free(pid);
    }
    post("WITSensorBLE: Identifier not found: %s", identifier);
    return 0;
#else
    if (!witsensor_ble_simpleble_ensure_initialized(ble_data)) {
        post("WITSensorBLE: Failed to initialize BLE system");
        return 0;
    }
    size_t count = simpleble_adapter_scan_get_results_count(ble_data->adapter);
    for (size_t i = 0; i < count; i++) {
        simpleble_peripheral_t p = simpleble_adapter_scan_get_results_handle(ble_data->adapter, i);
        if (!p) continue;
        char *pid = simpleble_peripheral_identifier(p);
        if (pid && strcmp(pid, identifier) == 0) {
            if (simpleble_peripheral_connect(p) == SIMPLEBLE_SUCCESS) {
                ble_data->peripheral = p;
                ble_data->is_connected = 1;
                simpleble_uuid_t service_uuid = {.value = WIT_SERVICE_UUID_STR};
                simpleble_uuid_t read_characteristic_uuid = {.value = WIT_READ_CHARACTERISTIC_UUID_STR};
                simpleble_peripheral_notify(p, service_uuid, read_characteristic_uuid, simpleble_on_data_received, ble_data);
                if (pid) simpleble_free(pid);
                return 1;
            }
        }
        if (pid) simpleble_free(pid);
    }
    post("WITSensorBLE: Identifier not found: %s", identifier);
    return 0;
#endif
}

// Connect to a device - find and connect to first WIT sensor
int witsensor_ble_simpleble_connect(witsensor_ble_simpleble_t *ble_data) {
    if (!ble_data) return 0;
    
    post("WITSensorBLE: Looking for WIT sensors...");
    
#ifdef __APPLE__
    // Direct (synchronous) connect path
    if (!witsensor_ble_simpleble_ensure_initialized(ble_data)) {
        post("WITSensorBLE: Failed to initialize BLE system");
        return 0;
    }
    size_t results_count = simpleble_adapter_scan_get_results_count(ble_data->adapter);
    post("WITSensorBLE: Found %zu devices", results_count);
    for (size_t i = 0; i < results_count; i++) {
        simpleble_peripheral_t peripheral = simpleble_adapter_scan_get_results_handle(ble_data->adapter, i);
        if (!peripheral) continue;
        char *peripheral_address = simpleble_peripheral_address(peripheral);
        char *peripheral_identifier = simpleble_peripheral_identifier(peripheral);
        if (peripheral_identifier && strstr(peripheral_identifier, "WT") != NULL) {
            post("WITSensorBLE: Found WIT sensor: %s [%s]", peripheral_identifier, peripheral_address);
            simpleble_err_t connect_result = simpleble_peripheral_connect(peripheral);
            if (connect_result == SIMPLEBLE_SUCCESS) {
                ble_data->peripheral = peripheral;
                ble_data->is_connected = 1;
                simpleble_uuid_t service_uuid = {.value = WIT_SERVICE_UUID_STR};
                simpleble_uuid_t read_characteristic_uuid = {.value = WIT_READ_CHARACTERISTIC_UUID_STR};
                simpleble_peripheral_notify(peripheral, service_uuid, read_characteristic_uuid, simpleble_on_data_received, ble_data);
                if (peripheral_address) simpleble_free(peripheral_address);
                if (peripheral_identifier) simpleble_free(peripheral_identifier);
                return 1;
            }
        }
        if (peripheral_address) simpleble_free(peripheral_address);
        if (peripheral_identifier) simpleble_free(peripheral_identifier);
    }
    post("WITSensorBLE: No WIT sensors found");
    return 0;
#else
    // Non-macOS direct path
    if (!witsensor_ble_simpleble_ensure_initialized(ble_data)) {
        post("WITSensorBLE: Failed to initialize BLE system");
        return 0;
    }
    size_t results_count = simpleble_adapter_scan_get_results_count(ble_data->adapter);
    post("WITSensorBLE: Found %zu devices", results_count);
    for (size_t i = 0; i < results_count; i++) {
        simpleble_peripheral_t peripheral = simpleble_adapter_scan_get_results_handle(ble_data->adapter, i);
        if (!peripheral) continue;
        char *peripheral_address = simpleble_peripheral_address(peripheral);
        char *peripheral_identifier = simpleble_peripheral_identifier(peripheral);
        if (peripheral_identifier && strstr(peripheral_identifier, "WT") != NULL) {
            post("WITSensorBLE: Found WIT sensor: %s [%s]", peripheral_identifier, peripheral_address);
            simpleble_err_t connect_result = simpleble_peripheral_connect(peripheral);
            if (connect_result == SIMPLEBLE_SUCCESS) {
                ble_data->peripheral = peripheral;
                ble_data->is_connected = 1;
                simpleble_uuid_t service_uuid = {.value = WIT_SERVICE_UUID_STR};
                simpleble_uuid_t read_characteristic_uuid = {.value = WIT_READ_CHARACTERISTIC_UUID_STR};
                simpleble_peripheral_notify(peripheral, service_uuid, read_characteristic_uuid, simpleble_on_data_received, ble_data);
                if (peripheral_address) simpleble_free(peripheral_address);
                if (peripheral_identifier) simpleble_free(peripheral_identifier);
                return 1;
            }
        }
        if (peripheral_address) simpleble_free(peripheral_address);
        if (peripheral_identifier) simpleble_free(peripheral_identifier);
    }
    post("WITSensorBLE: No WIT sensors found");
    return 0;
#endif
}

// Disconnect from device
void witsensor_ble_simpleble_disconnect(witsensor_ble_simpleble_t *ble_data) {
    if (!ble_data) return;
    
    post("WITSensorBLE: Disconnecting from device...");
    
    if (ble_data->peripheral) {
        simpleble_peripheral_disconnect(ble_data->peripheral);
        simpleble_peripheral_release_handle(ble_data->peripheral);
        ble_data->peripheral = NULL;
    }
    
    ble_data->is_connected = 0;
    post("WITSensorBLE: Disconnected from device");
}

// Write data to device - send WIT sensor commands
int witsensor_ble_simpleble_write_data(witsensor_ble_simpleble_t *ble_data, const unsigned char *data, int length) {
    if (!ble_data || !data || length <= 0) return 0;
    
    if (!ble_data->is_connected || !ble_data->peripheral) {
        post("WITSensorBLE: Not connected to device");
        return 0;
    }
    
    // Debug: log outgoing commands (disabled)
    // post("WITSensorBLE: sending command: ");
    // for (int i = 0; i < length; i++) {
    //     post("0x%02X ", data[i]);
    // }
    // post("(length=%d)", length);
    
    // Send data to WIT sensor using write characteristic
    simpleble_peripheral_write_command(ble_data->peripheral, WIT_SERVICE_UUID, WIT_WRITE_CHARACTERISTIC_UUID, data, length);
    return 1;
}

// Check if connected
int witsensor_ble_simpleble_is_connected(witsensor_ble_simpleble_t *ble_data) {
    if (!ble_data) return 0;
    return ble_data->is_connected;
}

// Check if scanning
int witsensor_ble_simpleble_is_scanning(witsensor_ble_simpleble_t *ble_data) {
    if (!ble_data) return 0;
    return ble_data->is_scanning;
}

// Permission probe: short bounded scan and count devices
int witsensor_ble_simpleble_permcheck(witsensor_ble_simpleble_t *ble_data, int timeout_ms) {
    (void)timeout_ms;
    if (!ble_data) return -1;
    // Only perform a safe initialization check; DO NOT scan here to avoid crashes without TCC
    if (!witsensor_ble_simpleble_ensure_initialized(ble_data)) return -1;
    // Return 0 to indicate "unknown/likely missing permission until a scan is attempted"
    return 0;
}