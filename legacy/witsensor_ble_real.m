/* witsensor_ble_real.m
 * Real CoreBluetooth implementation for minimal latency
 * Optimized for real-time sensor data streaming
 */

#import <Foundation/Foundation.h>
#import <CoreBluetooth/CoreBluetooth.h>
#import <mach/mach_time.h>
#include "m_pd.h"

// C interface for the PureData external
typedef struct {
    void *central_manager;
    void *peripheral;
    void *service;
    void *characteristic_read;
    void *characteristic_write;
    int is_connected;
    int is_scanning;
    char device_name[64];
    char device_address[32];
    void (*data_callback)(void *user_data, unsigned char *data, int length);
    void *user_data;
    // Performance tracking
    uint64_t last_data_time;
    uint64_t data_count;
} witsensor_ble_t;

// WIT sensor UUIDs
static NSString *WIT_SERVICE_UUID = @"0000ffe5-0000-1000-8000-00805f9a34fb";
static NSString *WIT_CHAR_READ_UUID = @"0000ffe4-0000-1000-8000-00805f9a34fb";
static NSString *WIT_CHAR_WRITE_UUID = @"0000ffe9-0000-1000-8000-00805f9a34fb";

@interface WITSensorBLEReal : NSObject <CBCentralManagerDelegate, CBPeripheralDelegate>
@property (nonatomic, strong) CBCentralManager *centralManager;
@property (nonatomic, strong) CBPeripheral *peripheral;
@property (nonatomic, strong) CBService *service;
@property (nonatomic, strong) CBCharacteristic *characteristicRead;
@property (nonatomic, strong) CBCharacteristic *characteristicWrite;
@property (nonatomic, assign) witsensor_ble_t *ble_data;
@property (nonatomic, strong) NSTimer *dataTimer;
@property (nonatomic, assign) BOOL isDataStreaming;
@property (nonatomic, assign) BOOL shouldStartScanning;
@end

@implementation WITSensorBLEReal

- (instancetype)initWithData:(witsensor_ble_t *)data {
    self = [super init];
    if (self) {
        self.ble_data = data;
        // Use high priority queue for minimal latency
        dispatch_queue_t bleQueue = dispatch_queue_create("com.witsensor.ble", DISPATCH_QUEUE_SERIAL);
        self.centralManager = [[CBCentralManager alloc] initWithDelegate:self queue:bleQueue];
        self.isDataStreaming = NO;
        self.shouldStartScanning = NO;
    }
    return self;
}

- (void)startScanning {
    post("WITSensorBLE: startScanning called, central manager state: %ld", (long)self.centralManager.state);
    if (self.centralManager.state == CBManagerStatePoweredOn) {
        self.ble_data->is_scanning = 1;
        // Scan with high priority for minimal discovery latency
        NSDictionary *options = @{
            CBCentralManagerScanOptionAllowDuplicatesKey: @YES
        };
        [self.centralManager scanForPeripheralsWithServices:nil options:options];
        post("WITSensorBLE: Started scanning for devices");
    } else {
        post("WITSensorBLE: Bluetooth not ready, state: %ld. Will start scanning when ready.", (long)self.centralManager.state);
        self.shouldStartScanning = YES;
    }
}

- (void)stopScanning {
    [self.centralManager stopScan];
    self.ble_data->is_scanning = 0;
    post("WITSensorBLE: Stopped scanning");
}

- (void)connectToPeripheral:(CBPeripheral *)peripheral {
    self.peripheral = peripheral;
    self.peripheral.delegate = self;
    [self.centralManager connectPeripheral:peripheral options:nil];
    // NSLog removed
}

- (void)connectByAddress:(NSString *)address {
    // Find peripheral by address
    NSArray *peripherals = [self.centralManager retrievePeripheralsWithIdentifiers:@[[[NSUUID alloc] initWithUUIDString:address]]];
    if (peripherals.count > 0) {
        [self connectToPeripheral:peripherals[0]];
    } else {
        // NSLog removed
    }
}

- (void)disconnect {
    if (self.peripheral) {
        [self.centralManager cancelPeripheralConnection:self.peripheral];
        [self stopDataStreaming];
        self.peripheral = nil;
        self.service = nil;
        self.characteristicRead = nil;
        self.characteristicWrite = nil;
        self.ble_data->is_connected = 0;
        // NSLog removed
    }
}

- (void)writeData:(NSData *)data {
    if (self.characteristicWrite && self.peripheral) {
        [self.peripheral writeValue:data forCharacteristic:self.characteristicWrite type:CBCharacteristicWriteWithResponse];
    }
}

- (void)startDataStreaming {
    if (self.characteristicRead && self.peripheral && !self.isDataStreaming) {
        [self.peripheral setNotifyValue:YES forCharacteristic:self.characteristicRead];
        self.isDataStreaming = YES;
        // NSLog removed
    }
}

- (void)stopDataStreaming {
    if (self.characteristicRead && self.peripheral && self.isDataStreaming) {
        [self.peripheral setNotifyValue:NO forCharacteristic:self.characteristicRead];
        self.isDataStreaming = NO;
        // NSLog removed
    }
}

#pragma mark - CBCentralManagerDelegate

- (void)centralManagerDidUpdateState:(CBCentralManager *)central {
    switch (central.state) {
        case CBManagerStatePoweredOn:
            post("WITSensorBLE: Bluetooth is on");
            if (self.shouldStartScanning) {
                post("WITSensorBLE: Bluetooth is now ready, starting scan...");
                [self startScanning];
                self.shouldStartScanning = NO;
            }
            break;
        case CBManagerStatePoweredOff:
            post("WITSensorBLE: Bluetooth is off");
            break;
        case CBManagerStateUnauthorized:
            post("WITSensorBLE: Bluetooth unauthorized");
            break;
        case CBManagerStateUnsupported:
            post("WITSensorBLE: Bluetooth unsupported");
            break;
        default:
            post("WITSensorBLE: Bluetooth state unknown: %ld", (long)central.state);
            break;
    }
}

- (void)centralManager:(CBCentralManager *)central didDiscoverPeripheral:(CBPeripheral *)peripheral advertisementData:(NSDictionary *)advertisementData RSSI:(NSNumber *)RSSI {
    NSString *peripheralName = peripheral.name;
    if (peripheralName) {
        post("WITSensorBLE: Found device: %s", [peripheralName UTF8String]);
        
        // Check if it's a WIT sensor
        if ([peripheralName containsString:@"WT"]) {
            post("WITSensorBLE: WIT sensor found: %s", [peripheralName UTF8String]);
            // Store device info for WIT sensors
            strncpy(self.ble_data->device_name, [peripheralName UTF8String], sizeof(self.ble_data->device_name) - 1);
            strncpy(self.ble_data->device_address, [peripheral.identifier.UUIDString UTF8String], sizeof(self.ble_data->device_address) - 1);
        }
    } else {
        post("WITSensorBLE: Found device: Unknown (No name)");
    }
}

- (void)centralManager:(CBCentralManager *)central didConnectPeripheral:(CBPeripheral *)peripheral {
    post("WITSensorBLE: Connected to device: %s", [peripheral.name UTF8String]);
    self.ble_data->is_connected = 1;
    [peripheral discoverServices:@[[CBUUID UUIDWithString:WIT_SERVICE_UUID]]];
}

- (void)centralManager:(CBCentralManager *)central didFailToConnectPeripheral:(CBPeripheral *)peripheral error:(NSError *)error {
    // NSLog removed
    self.ble_data->is_connected = 0;
}

- (void)centralManager:(CBCentralManager *)central didDisconnectPeripheral:(CBPeripheral *)peripheral error:(NSError *)error {
    // NSLog removed
    self.ble_data->is_connected = 0;
    [self stopDataStreaming];
}

#pragma mark - CBPeripheralDelegate

- (void)peripheral:(CBPeripheral *)peripheral didDiscoverServices:(NSError *)error {
    if (error) {
        // NSLog removed
        return;
    }
    
    for (CBService *service in peripheral.services) {
        if ([service.UUID.UUIDString isEqualToString:WIT_SERVICE_UUID]) {
            // NSLog removed
            self.service = service;
            [peripheral discoverCharacteristics:@[
                [CBUUID UUIDWithString:WIT_CHAR_READ_UUID],
                [CBUUID UUIDWithString:WIT_CHAR_WRITE_UUID]
            ] forService:service];
            break;
        }
    }
}

- (void)peripheral:(CBPeripheral *)peripheral didDiscoverCharacteristicsForService:(CBService *)service error:(NSError *)error {
    if (error) {
        // NSLog removed
        return;
    }
    
    for (CBCharacteristic *characteristic in service.characteristics) {
        if ([characteristic.UUID.UUIDString isEqualToString:WIT_CHAR_READ_UUID]) {
            // NSLog removed
            self.characteristicRead = characteristic;
            [self startDataStreaming];
        } else if ([characteristic.UUID.UUIDString isEqualToString:WIT_CHAR_WRITE_UUID]) {
            // NSLog removed
            self.characteristicWrite = characteristic;
        }
    }
}

- (void)peripheral:(CBPeripheral *)peripheral didUpdateValueForCharacteristic:(CBCharacteristic *)characteristic error:(NSError *)error {
    if (error) {
        // NSLog removed
        return;
    }
    
    if (characteristic == self.characteristicRead && characteristic.value) {
        // High-priority data forwarding for minimal latency
        NSData *data = characteristic.value;
        
        // Performance tracking
        self.ble_data->data_count++;
        self.ble_data->last_data_time = mach_absolute_time();
        
        // Forward data immediately to PureData external
        if (self.ble_data->data_callback) {
            self.ble_data->data_callback(self.ble_data->user_data, (unsigned char *)data.bytes, (int)data.length);
        }
    }
}

- (void)peripheral:(CBPeripheral *)peripheral didWriteValueForCharacteristic:(CBCharacteristic *)characteristic error:(NSError *)error {
    if (error) {
        // NSLog removed
    } else {
        // NSLog removed
    }
}

@end



// C interface functions with minimal latency
witsensor_ble_t* witsensor_ble_create(void *user_data, void (*data_callback)(void *user_data, unsigned char *data, int length)) {
    witsensor_ble_t *ble_data = malloc(sizeof(witsensor_ble_t));
    memset(ble_data, 0, sizeof(witsensor_ble_t));
    ble_data->user_data = user_data;
    ble_data->data_callback = data_callback;
    
    // Initialize BLE lazily to prevent crashes during external loading
    ble_data->central_manager = NULL;
    
    return ble_data;
}

void witsensor_ble_destroy(witsensor_ble_t *ble_data) {
    if (ble_data) {
        // Clean up BLE resources
        if (ble_data->central_manager) {
            WITSensorBLEReal *ble_wrapper = (__bridge WITSensorBLEReal *)ble_data->central_manager;
            [ble_wrapper disconnect];
        }
        free(ble_data);
    }
}

void witsensor_ble_start_scanning(witsensor_ble_t *ble_data) {
    // Set scanning flag
    ble_data->is_scanning = 1;
    
    // Initialize BLE only when actually needed for scanning
    if (ble_data->central_manager == NULL) {
        post("WITSensorBLE: Initializing BLE for scanning...");
        // Use dispatch_async to ensure BLE initialization happens on main thread
        // This is the most reliable approach that works consistently
        dispatch_async(dispatch_get_main_queue(), ^{
            @try {
                WITSensorBLEReal *ble_wrapper = [[WITSensorBLEReal alloc] initWithData:ble_data];
                ble_data->central_manager = (__bridge void *)ble_wrapper;
                post("WITSensorBLE: BLE initialized for scanning");
                
                // Now start scanning
                [ble_wrapper startScanning];
            } @catch (NSException *exception) {
                post("WITSensorBLE: BLE initialization failed: %s", [exception.reason UTF8String]);
            }
        });
    } else {
        // BLE already initialized, just start scanning
        WITSensorBLEReal *ble_wrapper = (__bridge WITSensorBLEReal *)ble_data->central_manager;
        @try {
            [ble_wrapper startScanning];
        } @catch (NSException *exception) {
            post("WITSensorBLE: Scanning failed: %s", [exception.reason UTF8String]);
        }
    }
}

void witsensor_ble_stop_scanning(witsensor_ble_t *ble_data) {
    // Clear scanning flag
    ble_data->is_scanning = 0;
    
    if (ble_data->central_manager) {
        WITSensorBLEReal *ble_wrapper = (__bridge WITSensorBLEReal *)ble_data->central_manager;
        @try {
            [ble_wrapper stopScanning];
        } @catch (NSException *exception) {
            post("WITSensorBLE: Stop scanning failed: %s", [exception.reason UTF8String]);
        }
    } else {
        post("WITSensorBLE: BLE not initialized, nothing to stop");
    }
}

void witsensor_ble_connect(witsensor_ble_t *ble_data, const char *device_name) {
    if (ble_data->central_manager) {
        WITSensorBLEReal *ble_wrapper = (__bridge WITSensorBLEReal *)ble_data->central_manager;
        // Implementation for connecting by name
    }
}

void witsensor_ble_connect_by_address(witsensor_ble_t *ble_data, const char *device_address) {
    if (ble_data->central_manager) {
        WITSensorBLEReal *ble_wrapper = (__bridge WITSensorBLEReal *)ble_data->central_manager;
        NSString *address = [NSString stringWithUTF8String:device_address];
        [ble_wrapper connectByAddress:address];
    }
}

void witsensor_ble_disconnect(witsensor_ble_t *ble_data) {
    if (ble_data->central_manager) {
        WITSensorBLEReal *ble_wrapper = (__bridge WITSensorBLEReal *)ble_data->central_manager;
        [ble_wrapper disconnect];
    }
}

void witsensor_ble_write_data(witsensor_ble_t *ble_data, unsigned char *data, int length) {
    if (ble_data->central_manager) {
        WITSensorBLEReal *ble_wrapper = (__bridge WITSensorBLEReal *)ble_data->central_manager;
        NSData *ns_data = [NSData dataWithBytes:data length:length];
        [ble_wrapper writeData:ns_data];
    }
}

int witsensor_ble_is_connected(witsensor_ble_t *ble_data) {
    return ble_data->is_connected;
}

int witsensor_ble_is_scanning(witsensor_ble_t *ble_data) {
    return ble_data->is_scanning;
}

// Performance monitoring
uint64_t witsensor_ble_get_data_count(witsensor_ble_t *ble_data) {
    return ble_data->data_count;
}

uint64_t witsensor_ble_get_last_data_time(witsensor_ble_t *ble_data) {
    return ble_data->last_data_time;
}

// Device info getters
const char* witsensor_ble_get_device_name(witsensor_ble_t *ble_data) {
    return ble_data->device_name;
}

const char* witsensor_ble_get_device_address(witsensor_ble_t *ble_data) {
    return ble_data->device_address;
}
