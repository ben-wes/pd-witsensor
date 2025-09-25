/* witsensor_ble_simpleble.h
 * Cross-platform BLE implementation using SimpleBLE
 * Supports macOS, Windows, and Linux
 * 
 * This is free and unencumbered software released into the public domain.
 * For more information, please refer to <https://unlicense.org>
 */

#ifndef WITSENSOR_BLE_SIMPLEBLE_H
#define WITSENSOR_BLE_SIMPLEBLE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// SimpleBLE handle types (forward declarations)
typedef void* simpleble_adapter_t;
typedef void* simpleble_peripheral_t;

// BLE data structure
typedef struct witsensor_ble_simpleble_t {
    void *pd_obj; // Pointer to the parent Pure Data object
    void (*data_callback)(void *user_data, unsigned char *data, int length); // Callback for received data

    simpleble_adapter_t adapter;
    simpleble_peripheral_t peripheral;
    int is_scanning;
    int is_connected;

    // Performance monitoring
    uint64_t data_count;
    uint64_t last_data_time;

    // Pd instance pointer for pd_queue_mess marshaling
    void *pd_instance;

    // Cached scan results for GUI-thread-safe enumeration
    char **cached_ids;
    char **cached_addrs;
    unsigned long cached_count;

    // Debug/state
    int scan_found_count;
    char adapter_id[128];
    char adapter_addr[64];
} witsensor_ble_simpleble_t;

// Cross-platform BLE interface functions
witsensor_ble_simpleble_t* witsensor_ble_simpleble_create(void);
void witsensor_ble_simpleble_destroy(witsensor_ble_simpleble_t *ble_data);
void witsensor_ble_simpleble_start_scanning(witsensor_ble_simpleble_t *ble_data);
void witsensor_ble_simpleble_stop_scanning(witsensor_ble_simpleble_t *ble_data);
void witsensor_ble_simpleble_get_scan_results(witsensor_ble_simpleble_t *ble_data);
void witsensor_ble_simpleble_clear_scan_results(witsensor_ble_simpleble_t *ble_data);
int witsensor_ble_simpleble_connect(witsensor_ble_simpleble_t *ble_data);
int witsensor_ble_simpleble_connect_by_address(witsensor_ble_simpleble_t *ble_data, const char *address);
int witsensor_ble_simpleble_connect_by_identifier(witsensor_ble_simpleble_t *ble_data, const char *identifier);
void witsensor_ble_simpleble_disconnect(witsensor_ble_simpleble_t *ble_data);
int witsensor_ble_simpleble_write_data(witsensor_ble_simpleble_t *ble_data, const unsigned char *data, int length);
int witsensor_ble_simpleble_is_connected(witsensor_ble_simpleble_t *ble_data);
int witsensor_ble_simpleble_is_scanning(witsensor_ble_simpleble_t *ble_data);
// Permission probe: returns number of devices found after a short bounded scan,
// or -1 on initialization failure
int witsensor_ble_simpleble_ensure_initialized(witsensor_ble_simpleble_t *ble_data);
// macOS authorization preflight (bridged via Objective-C)
int macos_bt_authorized_always(void);

// (Pd handler declarations are kept in implementation to avoid header dependency)

#ifdef __cplusplus
}
#endif

#endif // WITSENSOR_BLE_SIMPLEBLE_H