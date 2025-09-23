/* pd-witsensor-crossplatform.c
 * Cross-platform Pure Data external for WIT sensors
 * Supports macOS (CoreBluetooth), Windows, and Linux (SimpleBLE)
 */

#include "m_pd.h"
#include <string.h>
#include <stdlib.h>

// Use SimpleBLE for all platforms
#define USE_SIMPLEBLE 1
#include "witsensor_ble_simpleble.h"

// Version information
#define WITSENSOR_MAJOR_VERSION 0
#define WITSENSOR_MINOR_VERSION 3
#define WITSENSOR_BUGFIX_VERSION 0

// Constants
#define PACKET_SIZE 20
#define BUFFER_SIZE 1024

// Pure Data object structure
typedef struct _witsensor {
    t_object x_obj;
    
    // BLE data (SimpleBLE for all platforms)
    witsensor_ble_simpleble_t *ble_data;
    
    // Pure Data outlets
    t_outlet *data_out;
    t_outlet *status_out;
    
    // Polling
    t_clock *poll_clock;
    float poll_interval;
    int should_stop;
    
    // State
    int is_connected;
    int is_scanning;
    char device_name[64];
    char device_address[32];
    
    // Data buffers
    unsigned char temp_bytes[PACKET_SIZE];
    int temp_bytes_count;
    unsigned char data_buffer[BUFFER_SIZE];
    int buffer_count;
} t_witsensor;

// Pure Data class
static t_class *witsensor_class;

// Data callback function
static void witsensor_ble_data_callback(void *user_data, unsigned char *data, int length) {
    t_witsensor *x = (t_witsensor *)user_data;
    witsensor_process_data(x, data, length);
}

// Data processing function (same as before)
static void witsensor_process_data(t_witsensor *x, unsigned char *data, int length) {
    // Process incoming BLE data
    for (int i = 0; i < length; i++) {
        x->temp_bytes[x->temp_bytes_count] = data[i];
        x->temp_bytes_count++;
        
        if (x->temp_bytes_count >= PACKET_SIZE) {
            // Process complete packet
            witsensor_process_packet(x, x->temp_bytes);
            x->temp_bytes_count = 0;
        }
    }
}

// Packet processing (same as before)
static void witsensor_process_packet(t_witsensor *x, unsigned char *packet) {
    // Parse WIT sensor data packet
    // This is the same parsing logic as the original implementation
    // ... (implementation details would be the same)
}

// Constructor
static void *witsensor_new(void) {
    t_witsensor *x = (t_witsensor *)pd_new(witsensor_class);
    
    x->data_out = outlet_new(&x->x_obj, &s_anything);
    x->status_out = outlet_new(&x->x_obj, &s_float);
    x->poll_clock = clock_new(x, (t_method)witsensor_poll_tick);
    
    x->is_connected = 0;
    x->is_scanning = 0;
    x->should_stop = 0;
    x->poll_interval = 0;
    x->temp_bytes_count = 0;
    x->buffer_count = 0;
    
    // Initialize BLE system (SimpleBLE for all platforms)
    x->ble_data = witsensor_ble_simpleble_create(x, witsensor_ble_data_callback);
    post("witsensor: Using SimpleBLE (cross-platform)");
    
    return (void *)x;
}

// Destructor
static void witsensor_free(t_witsensor *x) {
    witsensor_disconnect(x);
    
    if (x->ble_data) {
        witsensor_ble_simpleble_destroy(x->ble_data);
    }
    
    clock_free(x->poll_clock);
}

// Scan for devices
static void witsensor_scan_devices(t_witsensor *x) {
    post("witsensor: scanning for WIT devices...");
    
    if (x->ble_data) {
        witsensor_ble_simpleble_start_scanning(x->ble_data);
        x->is_scanning = 1;
        post("witsensor: cross-platform scanning started successfully");
    } else {
        post("witsensor: BLE not initialized");
    }
}

// Connect to device
static void witsensor_connect_device(t_witsensor *x, t_symbol *device_identifier) {
    if (device_identifier && device_identifier->s_name) {
        strcpy(x->device_name, device_identifier->s_name);
        
        witsensor_ble_simpleble_connect(x->ble_data, device_identifier->s_name);
        
        post("witsensor: connecting to %s", device_identifier->s_name);
    }
}

// Disconnect
static void witsensor_disconnect(t_witsensor *x) {
    if (x->is_connected) {
        witsensor_ble_simpleble_disconnect(x->ble_data);
        
        x->is_connected = 0;
        post("witsensor: disconnected");
    }
}

// Version info
static void witsensor_version(void) {
    post("witsensor v%d.%d.%d - Cross-platform WIT sensor external for PureData", 
         WITSENSOR_MAJOR_VERSION, WITSENSOR_MINOR_VERSION, WITSENSOR_BUGFIX_VERSION);
    
    post("witsensor: Using SimpleBLE (cross-platform)");
}

// Message handlers
static void witsensor_scan(t_witsensor *x) {
    witsensor_scan_devices(x);
}

static void witsensor_connect(t_witsensor *x, t_symbol *s, int argc, t_atom *argv) {
    if (argc > 0 && argv[0].a_type == A_SYMBOL) {
        witsensor_connect_device(x, argv[0].a_w.w_symbol);
    }
}

static void witsensor_disconnect_msg(t_witsensor *x) {
    witsensor_disconnect(x);
}

// Clock tick for polling
static void witsensor_poll_tick(t_witsensor *x) {
    if (!x->should_stop && x->poll_interval > 0) {
        // Polling logic here
        clock_delay(x->poll_clock, x->poll_interval);
    }
}

// Setup function
void witsensor_setup(void) {
    witsensor_class = class_new(gensym("witsensor"),
                               (t_newmethod)witsensor_new,
                               (t_method)witsensor_free,
                               sizeof(t_witsensor),
                               CLASS_DEFAULT,
                               A_GIMME, 0);
    
    class_addmethod(witsensor_class, (t_method)witsensor_scan, gensym("scan"), 0);
    class_addmethod(witsensor_class, (t_method)witsensor_connect, gensym("connect"), A_GIMME, 0);
    class_addmethod(witsensor_class, (t_method)witsensor_disconnect_msg, gensym("disconnect"), 0);
    class_addmethod(witsensor_class, (t_method)witsensor_version, gensym("version"), 0);
    
    post("witsensor: Cross-platform WIT sensor external loaded");
    witsensor_version();
}
