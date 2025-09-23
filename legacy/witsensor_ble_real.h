/* witsensor_ble_real.h
 * Header file for real WIT sensor BLE communication
 * Optimized for minimal latency
 */

#ifndef WITSENSOR_BLE_REAL_H
#define WITSENSOR_BLE_REAL_H

#ifdef __cplusplus
extern "C" {
#endif

// Forward declaration
typedef struct witsensor_ble witsensor_ble_t;

// BLE interface functions
witsensor_ble_t* witsensor_ble_create(void *user_data, void (*data_callback)(void *user_data, unsigned char *data, int length));
void witsensor_ble_destroy(witsensor_ble_t *ble_data);
void witsensor_ble_start_scanning(witsensor_ble_t *ble_data);
void witsensor_ble_stop_scanning(witsensor_ble_t *ble_data);
void witsensor_ble_connect(witsensor_ble_t *ble_data, const char *device_name);
void witsensor_ble_connect_by_address(witsensor_ble_t *ble_data, const char *device_address);
void witsensor_ble_disconnect(witsensor_ble_t *ble_data);
void witsensor_ble_write_data(witsensor_ble_t *ble_data, unsigned char *data, int length);
int witsensor_ble_is_connected(witsensor_ble_t *ble_data);
int witsensor_ble_is_scanning(witsensor_ble_t *ble_data);

// Performance monitoring
uint64_t witsensor_ble_get_data_count(witsensor_ble_t *ble_data);
uint64_t witsensor_ble_get_last_data_time(witsensor_ble_t *ble_data);

// Device info getters
const char* witsensor_ble_get_device_name(witsensor_ble_t *ble_data);
const char* witsensor_ble_get_device_address(witsensor_ble_t *ble_data);

#ifdef __cplusplus
}
#endif

#endif // WITSENSOR_BLE_REAL_H
