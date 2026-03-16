/* witsensor_ble_simpleble.c
 * Cross-platform BLE implementation using SimpleBLE
 * Supports macOS, Windows, and Linux
 * 
 * Copyright (c) 2025 pd-witsensor contributors
 * Licensed under Business Source License 1.1 (BUSL-1.1)
 * See LICENSE for details
 */

#include "witsensor_ble_simpleble.h"
#include "m_pd.h"
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <time.h>

// Platform-specific includes
#ifndef _WIN32
    #include <pthread.h>
#endif

// SimpleBLE includes - use simplecble C API headers directly
// Note: in newer SimpleBLE, the C API lives in simplecble/simplecble.h
#include <simplecble/simplecble.h>
#include "m_pd.h"
// Pd-thread handler to announce scan completion and print cached devices
extern void witsensor_pd_scan_complete_handler(t_pd *obj, void *data);
// Forward declarations for Pd-thread status handlers and payloads
typedef struct _queued_flag { int value; } t_queued_flag;
typedef struct _queued_device { char *tag; char *addr; char *id; int from_live_scan; } t_queued_device;
void witsensor_pd_scanning_handler(t_pd *obj, void *data);
void witsensor_pd_device_found_handler(t_pd *obj, void *data);
void witsensor_pd_connected_handler(t_pd *obj, void *data);

// Forward declaration for notification callback used in connect path
static void simpleble_on_data_received(simpleble_peripheral_t peripheral, simpleble_uuid_t service, simpleble_uuid_t characteristic, const uint8_t *data, size_t length, void *user_data);

// WIT sensor service and characteristic UUIDs (from Python SDK)
#define WIT_SERVICE_UUID_STR "0000ffe5-0000-1000-8000-00805f9a34fb"
#define WIT_READ_CHARACTERISTIC_UUID_STR "0000ffe4-0000-1000-8000-00805f9a34fb"
#define WIT_WRITE_CHARACTERISTIC_UUID_STR "0000ffe9-0000-1000-8000-00805f9a34fb"

// Convert string UUIDs to SimpleBLE format
static simpleble_uuid_t WIT_SERVICE_UUID = {.value = WIT_SERVICE_UUID_STR};
/* unused: static simpleble_uuid_t WIT_READ_CHARACTERISTIC_UUID = {.value = WIT_READ_CHARACTERISTIC_UUID_STR}; */
static simpleble_uuid_t WIT_WRITE_CHARACTERISTIC_UUID = {.value = WIT_WRITE_CHARACTERISTIC_UUID_STR};
static simpleble_uuid_t WIT_READ_CHARACTERISTIC_UUID = {.value = WIT_READ_CHARACTERISTIC_UUID_STR};

/* Serial scan: only one instance can own the scan at a time (SimpleBLE has one callback slot). */
static witsensor_ble_simpleble_t *s_scan_owner = NULL;
static int s_adapter_scan_active = 0;  /* 1 while adapter is scanning (stays true when owner connects) */

// Output devices from adapter (single source of truth - no per-instance cache)
static void _output_scan_results(witsensor_ble_simpleble_t *ble_data) {
    if (!ble_data || !ble_data->adapter) return;
    size_t n = simpleble_adapter_scan_get_results_count(ble_data->adapter);
    for (size_t i = 0; i < n; i++) {
        simpleble_peripheral_t p = simpleble_adapter_scan_get_results_handle(ble_data->adapter, i);
        if (!p) continue;
        char *addr = simpleble_peripheral_address(p);
        char *id = simpleble_peripheral_identifier(p);
        if (id && ble_data->pd_instance && ble_data->pd_obj) {
            const char *tag = (strstr(id, "WT") != NULL) ? "wit" : "other";
            t_queued_device *d = (t_queued_device *)malloc(sizeof(t_queued_device));
            if (d) {
                d->tag = strdup(tag);
                d->addr = addr ? strdup(addr) : NULL;
                d->id = strdup(id);
                d->from_live_scan = 0;  /* from results dump */
                pd_queue_mess((t_pdinstance*)ble_data->pd_instance, (t_pd*)ble_data->pd_obj, d, witsensor_pd_device_found_handler);
            }
        }
        if (addr) simpleble_free(addr);
        if (id) simpleble_free(id);
        simpleble_peripheral_release_handle(p);
    }
}

// SimpleBLE scan callbacks
static void simpleble_on_scan_start(simpleble_adapter_t adapter, void *user_data) {
    witsensor_ble_simpleble_t *ble_data = (witsensor_ble_simpleble_t *)user_data;
    if (!ble_data) return;
    ble_data->scan_found_count = 0;
    ble_data->adapter_id[0] = '\0';
    ble_data->adapter_addr[0] = '\0';
    char *aid = simpleble_adapter_identifier(adapter);
    char *aad = simpleble_adapter_address(adapter);
    if (aid) { snprintf(ble_data->adapter_id, sizeof(ble_data->adapter_id), "%s", aid); simpleble_free(aid); }
    if (aad) { snprintf(ble_data->adapter_addr, sizeof(ble_data->adapter_addr), "%s", aad); simpleble_free(aad); }
}

static void simpleble_on_scan_stop(simpleble_adapter_t adapter, void *user_data) {
    (void)adapter;
    witsensor_ble_simpleble_t *ble_data = (witsensor_ble_simpleble_t *)user_data;
    if (!ble_data) return;
    if (s_scan_owner == ble_data) s_scan_owner = NULL;
    s_adapter_scan_active = 0;
    ble_data->is_scanning = 0;
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
    if (ble_data->is_connected) return;
    char *addr = simpleble_peripheral_address(peripheral);
    char *id = simpleble_peripheral_identifier(peripheral);
    if (id && ble_data->pd_instance && ble_data->pd_obj) {
        ble_data->scan_found_count++;
        const char *tag = (strstr(id, "WT") != NULL) ? "wit" : "other";
        t_queued_device *d = (t_queued_device *)malloc(sizeof(t_queued_device));
        if (d) {
            d->tag = strdup(tag);
            d->addr = addr ? strdup(addr) : NULL;
            d->id = strdup(id);
            d->from_live_scan = 1;  /* from scan_found; suppress if scan stopped */
            pd_queue_mess((t_pdinstance*)ble_data->pd_instance, (t_pd*)ble_data->pd_obj, d, witsensor_pd_device_found_handler);
        }
    }
    if (addr) simpleble_free(addr);
    if (id) simpleble_free(id);
}


static void simpleble_on_data_received(simpleble_peripheral_t peripheral, simpleble_uuid_t service, simpleble_uuid_t characteristic, const uint8_t *data, size_t length, void *user_data) {
    (void)peripheral; (void)service; (void)characteristic;
    witsensor_ble_simpleble_t *ble_data = (witsensor_ble_simpleble_t *)user_data;
    if (!ble_data) return;
    // Pass-through: forward full notification payload to Pd layer for stream parsing.
    if (ble_data->data_callback && data && length > 0) {
        ble_data->data_callback(ble_data->pd_obj, (unsigned char *)data, (int)length);
    }
    ble_data->data_count++;
    ble_data->last_data_time = time(NULL);
}

// Callback for disconnection events
static void simpleble_on_disconnected(simpleble_peripheral_t peripheral, void *user_data) {
    (void)peripheral;
    witsensor_ble_simpleble_t *ble_data = (witsensor_ble_simpleble_t *)user_data;
    if (!ble_data) return;
    
    post("WITSensorBLE: Device disconnected unexpectedly");
    ble_data->is_connected = 0;
    
    // Notify Pd layer about disconnection
    if (ble_data->pd_instance && ble_data->pd_obj) {
        t_queued_flag *q = (t_queued_flag *)malloc(sizeof(t_queued_flag));
        if (q) {
            q->value = 0; // 0 = disconnected
            pd_queue_mess((t_pdinstance*)ble_data->pd_instance, (t_pd*)ble_data->pd_obj, q, witsensor_pd_connected_handler);
        }
    }
}

// Create BLE data structure
witsensor_ble_simpleble_t *witsensor_ble_simpleble_create(void) {
    witsensor_ble_simpleble_t *ble_data = (witsensor_ble_simpleble_t *)calloc(1, sizeof(witsensor_ble_simpleble_t));
    if (!ble_data) {
        post("WITSensorBLE: Failed to allocate memory");
        return NULL;
    }
    
    ble_data->adapter = NULL;
    ble_data->peripheral = NULL;
    ble_data->is_scanning = 0;
    ble_data->is_connected = 0;
    return ble_data;
}

// Destroy BLE data structure - COMPLETELY CRASH-SAFE
void witsensor_ble_simpleble_destroy(witsensor_ble_simpleble_t *ble_data) {
    if (ble_data) {
        if (s_scan_owner == ble_data) s_scan_owner = NULL;
        free(ble_data);
    }
}

// Ensure BLE is initialized (adapter obtained when NULL, for connect/results)
int witsensor_ble_simpleble_ensure_initialized(witsensor_ble_simpleble_t *ble_data) {
    if (!ble_data) return 0;
    if (ble_data->adapter) return 1;

    size_t adapter_count = simpleble_adapter_get_count();
    if (adapter_count == 0) return 0;
    ble_data->adapter = simpleble_adapter_get_handle(0);
    if (!ble_data->adapter) return 0;

    simpleble_adapter_set_callback_on_scan_start(ble_data->adapter, simpleble_on_scan_start, ble_data);
    simpleble_adapter_set_callback_on_scan_stop(ble_data->adapter, simpleble_on_scan_stop, ble_data);
    simpleble_adapter_set_callback_on_scan_found(ble_data->adapter, simpleble_on_scan_found, ble_data);
    return 1;
}

// Start scanning for devices
void witsensor_ble_simpleble_start_scanning(witsensor_ble_simpleble_t *ble_data) {
    if (!ble_data) return;

    /* Serial scan: only one instance can own the scan (SimpleBLE has one callback slot).
     * Warn and return if another instance is already scanning. */
    if (s_scan_owner != NULL && s_scan_owner != ble_data) {
        post("witsensor: another instance is already scanning - stop it first, or use 'connect <device_id>' with a known ID");
        return;
    }
    
    post("WITSensorBLE: Starting BLE scan ...");
    
    // Authorization already checked on object creation
    
    // Try to initialize BLE now
    if (!ble_data->adapter) {
        post("WITSensorBLE: Attempting BLE initialization on first scan...");
        
        // Try to get adapter count first
        size_t adapter_count = simpleble_adapter_get_count();
        if (adapter_count == 0) {
            pd_error(ble_data->pd_obj, "WITSensorBLE: No BLE adapters found - check Bluetooth permissions in System Settings → Privacy & Security → Bluetooth");
            return;
        }
        
        // Get the first adapter
        ble_data->adapter = simpleble_adapter_get_handle(0);
        if (!ble_data->adapter) {
            pd_error(ble_data->pd_obj, "WITSensorBLE: Failed to get adapter - check Bluetooth permissions in System Settings → Privacy & Security → Bluetooth");
            return;
        }
        
        // Set up callbacks
        simpleble_adapter_set_callback_on_scan_start(ble_data->adapter, simpleble_on_scan_start, ble_data);
        simpleble_adapter_set_callback_on_scan_stop(ble_data->adapter, simpleble_on_scan_stop, ble_data);
        simpleble_adapter_set_callback_on_scan_found(ble_data->adapter, simpleble_on_scan_found, ble_data);
        
        post("WITSensorBLE: BLE adapter initialized successfully");
    }
    
    // Check if Bluetooth is enabled before attempting scan
    if (!simpleble_adapter_is_bluetooth_enabled()) {
        pd_error(ble_data->pd_obj, "WITSensorBLE: Bluetooth is not enabled - please enable Bluetooth and grant permissions");
        return;
    }
    
    /* Ensure scan_found callback points to this instance (SimpleBLE has one callback slot) */
    simpleble_adapter_set_callback_on_scan_found(ble_data->adapter, simpleble_on_scan_found, ble_data);
    
    s_scan_owner = ble_data;
    s_adapter_scan_active = 1;
    ble_data->is_scanning = 1;
    // Emit scanning 1 via Pd thread
    if (ble_data->pd_instance && ble_data->pd_obj) {
        t_queued_flag *q = (t_queued_flag *)malloc(sizeof(t_queued_flag));
        if (q) { q->value = 1; pd_queue_mess((t_pdinstance*)ble_data->pd_instance, (t_pd*)ble_data->pd_obj, q, witsensor_pd_scanning_handler); }
    }
    // Non-blocking scan: start, and let 'results' query current list
    simpleble_err_t err = simpleble_adapter_scan_start(ble_data->adapter);
    if (err != SIMPLEBLE_SUCCESS) {
        s_scan_owner = NULL;
        s_adapter_scan_active = 0;
        ble_data->is_scanning = 0;
        pd_error(ble_data->pd_obj, "WITSensorBLE: scan_start failed: %d", err);
        return;
    }
    // Continuous scanning - scan until manually stopped or connection succeeds
    post("WITSensorBLE: continuous scanning (no timeout)");
}

// Stop scanning for devices
void witsensor_ble_simpleble_stop_scanning(witsensor_ble_simpleble_t *ble_data) {
    if (!ble_data) return;
    /* Suppress device_found immediately; set before any async work */
    s_adapter_scan_active = 0;
    s_scan_owner = NULL;  /* any instance can stop; clear so next scan can start */
    
    post("WITSensorBLE: Stopping cross-platform scan...");
    
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

// Log scan results to post (for scan_complete)
void witsensor_ble_simpleble_log_scan_results(witsensor_ble_simpleble_t *ble_data) {
    if (!ble_data || !ble_data->adapter) return;
    size_t n = simpleble_adapter_scan_get_results_count(ble_data->adapter);
    if (n == 0) {
        post("WITSensorBLE: Found 0 devices. Ensure Bluetooth is on and devices are advertising.");
        return;
    }
    post("WITSensorBLE: Found %zu devices", n);
    for (size_t i = 0; i < n; i++) {
        simpleble_peripheral_t p = simpleble_adapter_scan_get_results_handle(ble_data->adapter, i);
        if (!p) continue;
        char *id = simpleble_peripheral_identifier(p);
        char *addr = simpleble_peripheral_address(p);
        if (id && addr) post("WITSensorBLE: Found device: %s [%s]", id, addr);
        if (id) simpleble_free(id);
        if (addr) simpleble_free(addr);
        simpleble_peripheral_release_handle(p);
    }
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

// Clear scan results - no-op (adapter holds results; clearing would affect all instances)
void witsensor_ble_simpleble_clear_scan_results(witsensor_ble_simpleble_t *ble_data) {
    (void)ble_data;
    post("witsensor: clear is a no-op (adapter holds scan results)");
}

// Get first free (not yet connected) WIT device ID from adapter (for no-arg connect)
int witsensor_ble_simpleble_get_first_wit_id(witsensor_ble_simpleble_t *ble_data, char *buf, size_t bufsize) {
    if (!ble_data || !buf || bufsize < 2) return 0;
    if (!witsensor_ble_simpleble_ensure_initialized(ble_data)) return 0;
    size_t n = simpleble_adapter_scan_get_results_count(ble_data->adapter);
    for (size_t i = 0; i < n; i++) {
        simpleble_peripheral_t p = simpleble_adapter_scan_get_results_handle(ble_data->adapter, i);
        if (!p) continue;
        char *id = simpleble_peripheral_identifier(p);
        if (id && strstr(id, "WT") != NULL) {
            bool already_connected = false;
            if (simpleble_peripheral_is_connected(p, &already_connected) == SIMPLEBLE_SUCCESS && already_connected) {
                if (id) simpleble_free(id);
                simpleble_peripheral_release_handle(p);
                continue;  /* skip, try next */
            }
            strncpy(buf, id, bufsize - 1);
            buf[bufsize - 1] = '\0';
            simpleble_free(id);
            simpleble_peripheral_release_handle(p);
            return 1;
        }
        if (id) simpleble_free(id);
        simpleble_peripheral_release_handle(p);
    }
    return 0;
}

// Returns 1 if device with given id is already connected (to any instance)
int witsensor_ble_simpleble_is_device_connected(witsensor_ble_simpleble_t *ble_data, const char *id) {
    if (!ble_data || !id || !ble_data->adapter) return 0;
    size_t n = simpleble_adapter_scan_get_results_count(ble_data->adapter);
    for (size_t i = 0; i < n; i++) {
        simpleble_peripheral_t p = simpleble_adapter_scan_get_results_handle(ble_data->adapter, i);
        if (!p) continue;
        char *pid = simpleble_peripheral_identifier(p);
        int match = (pid && strcmp(pid, id) == 0);
        if (pid) simpleble_free(pid);
        if (match) {
            bool connected = false;
            int ok = (simpleble_peripheral_is_connected(p, &connected) == SIMPLEBLE_SUCCESS && connected);
            simpleble_peripheral_release_handle(p);
            return ok ? 1 : 0;
        }
        simpleble_peripheral_release_handle(p);
    }
    return 0;
}

// Connect to a device by target string (address or identifier)
int witsensor_ble_simpleble_connect(witsensor_ble_simpleble_t *ble_data, const char *target) {
    if (!ble_data || !target) return 0;
    // Ensure BLE is initialized (gets adapter when NULL)
    if (!witsensor_ble_simpleble_ensure_initialized(ble_data)) {
        post("WITSensorBLE: Failed to initialize BLE system");
        return 0;
    }

    /* Use adapter's scan results (shared across instances). Allows connect with
     * explicit ID even when this instance didn't scan (e.g. another instance scanned). */
    size_t count = simpleble_adapter_scan_get_results_count(ble_data->adapter);
    for (size_t i = 0; i < count; i++) {
        simpleble_peripheral_t p = simpleble_adapter_scan_get_results_handle(ble_data->adapter, i);
        if (!p) continue;

        char *addr = simpleble_peripheral_address(p);
        char *id = simpleble_peripheral_identifier(p);

        int is_match = 0;
        if (addr && strcmp(addr, target) == 0) is_match = 1;
        if (!is_match && id && strcmp(id, target) == 0) is_match = 1;

        if (is_match) {
            bool already_connected = false;
            if (simpleble_peripheral_is_connected(p, &already_connected) == SIMPLEBLE_SUCCESS && already_connected) {
                if (ble_data->peripheral == p) {
                    /* We already own this connection */
                    if (addr) simpleble_free(addr);
                    if (id) simpleble_free(id);
                    return 1;
                }
                post("witsensor: %s already connected to another instance", target);
                if (addr) simpleble_free(addr);
                if (id) simpleble_free(id);
                simpleble_peripheral_release_handle(p);
                return 0;
            }
            if (simpleble_peripheral_connect(p) == SIMPLEBLE_SUCCESS) {
                ble_data->peripheral = p;
                ble_data->is_connected = 1;
                /* Keep adapter scan running so other instances with pending_target can connect */
                if (ble_data->is_scanning) ble_data->is_scanning = 0;

                simpleble_uuid_t service_uuid = {.value = WIT_SERVICE_UUID_STR};
                simpleble_uuid_t read_characteristic_uuid = {.value = WIT_READ_CHARACTERISTIC_UUID_STR};
                simpleble_peripheral_notify(p, service_uuid, read_characteristic_uuid, simpleble_on_data_received, ble_data);
                simpleble_peripheral_set_callback_on_disconnected(p, simpleble_on_disconnected, ble_data);
                if (addr) simpleble_free(addr);
                if (id) simpleble_free(id);
                return 1;
            } else {
                pd_error(ble_data->pd_obj, "WITSensorBLE: Failed to connect to %s", target);
                simpleble_peripheral_release_handle(p);
            }
        } else {
            simpleble_peripheral_release_handle(p);
        }

        if (addr) simpleble_free(addr);
        if (id) simpleble_free(id);
    }

    pd_error(ble_data->pd_obj, "WITSensorBLE: Device not found: %s", target);
    return 0;
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
        pd_error(ble_data->pd_obj, "WITSensorBLE: Not connected to device");
        return 0;
    }
        
    // Send data to WIT sensor using write characteristic
    simpleble_peripheral_write_command(ble_data->peripheral, WIT_SERVICE_UUID, WIT_WRITE_CHARACTERISTIC_UUID, data, length);
    return 1;
}

// Write via request (with response) to ensure delivery semantics for config/ASCII commands
int witsensor_ble_simpleble_write_request_raw(witsensor_ble_simpleble_t *ble_data, const unsigned char *data, int length) {
    if (!ble_data || !data || length <= 0) return 0;
    if (!ble_data->is_connected || !ble_data->peripheral) {
        pd_error(ble_data->pd_obj, "WITSensorBLE: Not connected to device");
        return 0;
    }
    simpleble_err_t err = simpleble_peripheral_write_request(
        ble_data->peripheral,
        WIT_SERVICE_UUID,
        WIT_WRITE_CHARACTERISTIC_UUID,
        data,
        (uint16_t)length);
    return (err == SIMPLEBLE_SUCCESS) ? 1 : 0;
}

int witsensor_ble_simpleble_set_notifications_enabled(witsensor_ble_simpleble_t *ble_data, int enabled) {
    if (!ble_data || !ble_data->peripheral) return 0;
    if (enabled) {
        simpleble_peripheral_notify(
            ble_data->peripheral,
            WIT_SERVICE_UUID,
            WIT_READ_CHARACTERISTIC_UUID,
            simpleble_on_data_received,
            ble_data);
    } else {
        simpleble_peripheral_unsubscribe(
            ble_data->peripheral,
            WIT_SERVICE_UUID,
            WIT_READ_CHARACTERISTIC_UUID);
    }
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

// Returns 1 if adapter scan is active (stays true when owner connects so others can autoconnect)
int witsensor_ble_simpleble_is_any_scanning(void) {
    return s_adapter_scan_active;
}

// Get connected device's BLE address
int witsensor_ble_simpleble_get_connected_address(witsensor_ble_simpleble_t *ble_data, char *buf, size_t bufsize) {
    if (!ble_data || !buf || bufsize < 2) return 0;
    if (!ble_data->is_connected || !ble_data->peripheral) return 0;
    char *addr = simpleble_peripheral_address(ble_data->peripheral);
    if (!addr) return 0;
    snprintf(buf, bufsize, "%s", addr);
    simpleble_free(addr);
    return 1;
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