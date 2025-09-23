#import <Foundation/Foundation.h>
#import <CoreBluetooth/CoreBluetooth.h>

int macos_bt_authorized_always(void) {
    if (@available(macOS 10.15, *)) {
        CBManagerAuthorization auth = CBCentralManager.authorization;
        return auth == CBManagerAuthorizationAllowedAlways ? 1 : 0;
    }
    return 1;
}
