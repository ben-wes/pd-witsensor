# library name
lib.name = witsensor

# Pure Data path (use newer version)
PD_PATH = /Applications/Pd-0.56-1.app/Contents/Resources/src
# Allow overriding via environment; default to PD_PATH
PDINCLUDEDIR ?= $(PD_PATH)

# source files - CoreBluetooth implementation
witsensor.class.sources = pd-witsensor-ble.c witsensor_ble_simpleble.c macos_bt_auth.m

# include directories (use submodule SimpleBLE C API)
cflags = -I. -I./SimpleBLE/simplecble/include -I./SimpleBLE/simpleble/include -I./SimpleBLE/simplecble/build/simpleble/export
# no extra C++ flags; we stay in C

# libraries
ldlibs = -lpthread

# platform-specific settings - SimpleBLE for all platforms
define forDarwin
	# Link against static libs built by SimpleBLE (no runtime dylib needed)
	ldlibs += -L./SimpleBLE/simplecble/build-static/lib -Wl,-force_load,./SimpleBLE/simplecble/build-static/lib/libsimplecble.a -Wl,-force_load,./SimpleBLE/simplecble/build-static/lib/libsimpleble.a -framework CoreBluetooth -framework Foundation
endef


define forLinux
	ldlibs += -L./SimpleBLE/simplecble/build -Wl,--whole-archive -lsimplecble -lsimpleble -Wl,--no-whole-archive
endef

define forWindows
	ldlibs += -L./SimpleBLE/simplecble/build -Wl,--whole-archive -lsimplecble -lsimpleble -Wl,--no-whole-archive
endef

# data files
datafiles = \
	README.md \
	witsensor-help.pd \
	PERFORMANCE.md \
	${empty}

# include pd-lib-builder
PDLIBBUILDER_DIR=./pd-lib-builder
include $(PDLIBBUILDER_DIR)/Makefile.pdlibbuilder

# Build Objective-C helper explicitly (place AFTER include so 'all' stays default)
macos_bt_auth.pd_darwin.o: macos_bt_auth.m
	cc -DPD -I "$(PDINCLUDEDIR)" -Wall -Wextra -O3 -arch arm64 -mmacosx-version-min=10.6 -c macos_bt_auth.m -o macos_bt_auth.pd_darwin.o

# SimpleBLE dependencies (build static for macOS and shared for Linux/Windows)
SIMPLEBLE_DIR=SimpleBLE/simplecble
SIMPLEBLE_STATIC_DIR=$(SIMPLEBLE_DIR)/build-static
SIMPLEBLE_SHARED_DIR=$(SIMPLEBLE_DIR)/build

SIMPLEBLE_STATIC_LIBS=$(SIMPLEBLE_STATIC_DIR)/lib/libsimplecble.a $(SIMPLEBLE_STATIC_DIR)/lib/libsimpleble.a
SIMPLEBLE_SHARED_LIBS=$(SIMPLEBLE_SHARED_DIR)/lib/libsimplecble.a $(SIMPLEBLE_SHARED_DIR)/lib/libsimpleble.a

# Ensure the macOS external links against locally built static libs
witsensor.pd_darwin: $(SIMPLEBLE_STATIC_LIBS)

# Ensure Linux/Windows externals depend on built SimpleBLE (shared build tree)
witsensor.pd_linux: $(SIMPLEBLE_SHARED_LIBS)
witsensor.dll: $(SIMPLEBLE_SHARED_LIBS)

.PHONY: deps
deps: $(SIMPLEBLE_STATIC_LIBS) $(SIMPLEBLE_SHARED_LIBS)

$(SIMPLEBLE_STATIC_LIBS):
	git submodule update --init --recursive
	cd $(SIMPLEBLE_DIR) && cmake -S . -B build-static -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=OFF
	$(MAKE) -C $(SIMPLEBLE_STATIC_DIR) -j$(shell sysctl -n hw.ncpu 2>/dev/null || nproc)

$(SIMPLEBLE_SHARED_LIBS):
	git submodule update --init --recursive
	cd $(SIMPLEBLE_DIR) && cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=ON
	$(MAKE) -C $(SIMPLEBLE_SHARED_DIR) -j$(shell sysctl -n hw.ncpu 2>/dev/null || nproc)
