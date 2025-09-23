# library name
lib.name = witsensor

# Pure Data path (use newer version)
PD_PATH = /Applications/Pd-0.56-1.app/Contents/Resources/src
# Allow overriding via environment; default to PD_PATH
PDINCLUDEDIR ?= $(PD_PATH)

# source files
witsensor.class.sources = pd-witsensor-ble.c witsensor_ble_simpleble.c

# include directories (use submodule SimpleBLE C API)
# Add export include paths for both static (macOS) and shared (Linux) builds
cflags = -I. -I./SimpleBLE/simplecble/include -I./SimpleBLE/simpleble/include -I./SimpleBLE/simplecble/build-static/simpleble/export -I./SimpleBLE/simplecble/build/simpleble/export
# no extra C++ flags; we stay in C

# libraries
ldlibs = -lpthread

# platform-specific settings - SimpleBLE for all platforms
define forDarwin
	# Link against static libs built by SimpleBLE (no runtime dylib needed)
	ldlibs += -L./SimpleBLE/simplecble/build-static/lib -Wl,-force_load,./SimpleBLE/simplecble/build-static/lib/libsimplecble.a -Wl,-force_load,./SimpleBLE/simplecble/build-static/lib/libsimpleble.a -framework CoreBluetooth -framework Foundation
	# Include Objective-C helper for all macOS builds (needed for Bluetooth permissions)
	witsensor.class.sources += macos_bt_auth.m
endef


define forLinux
	# Link against shared libs produced under build/lib on Linux
	ldlibs += -L./SimpleBLE/simplecble/build/lib -lsimplecble -lsimpleble
endef

define forWindows
	ldlibs += -L./SimpleBLE/simplecble/build -Wl,--whole-archive -lsimplecble -lsimpleble -Wl,--no-whole-archive
endef

# data files
datafiles = \
	README.md \
	witsensor-help.pd \
	${empty}

# include pd-lib-builder
PDLIBBUILDER_DIR=./pd-lib-builder
include $(PDLIBBUILDER_DIR)/Makefile.pdlibbuilder

# Build Objective-C helper for all architectures (place AFTER include so 'all' stays default)
macos_bt_auth.d_amd64.o: macos_bt_auth.m
	cc -DPD -I "$(PDINCLUDEDIR)" -Wall -Wextra -O3 -arch x86_64 -mmacosx-version-min=10.6 -c macos_bt_auth.m -o macos_bt_auth.d_amd64.o

macos_bt_auth.d_arm64.o: macos_bt_auth.m
	cc -DPD -I "$(PDINCLUDEDIR)" -Wall -Wextra -O3 -arch arm64 -mmacosx-version-min=10.6 -c macos_bt_auth.m -o macos_bt_auth.d_arm64.o

# Build rules for shared library extensions
macos_bt_auth.darwin-amd64-64.so.o: macos_bt_auth.m
	cc -DPD -I "$(PDINCLUDEDIR)" -Wall -Wextra -O3 -arch x86_64 -mmacosx-version-min=10.6 -c macos_bt_auth.m -o macos_bt_auth.darwin-amd64-64.so.o

macos_bt_auth.darwin-arm64-64.so.o: macos_bt_auth.m
	cc -DPD -I "$(PDINCLUDEDIR)" -Wall -Wextra -O3 -arch arm64 -mmacosx-version-min=10.6 -c macos_bt_auth.m -o macos_bt_auth.darwin-arm64-64.so.o

# Build rule for generic pd_darwin extension (local builds)
macos_bt_auth.pd_darwin.o: macos_bt_auth.m
	cc -DPD -I "$(PDINCLUDEDIR)" -Wall -Wextra -O3 -arch arm64 -mmacosx-version-min=10.6 -c macos_bt_auth.m -o macos_bt_auth.pd_darwin.o

# SimpleBLE dependencies (build static for macOS and shared for Linux)
SIMPLEBLE_DIR=SimpleBLE/simplecble
SIMPLEBLE_STATIC_DIR=$(SIMPLEBLE_DIR)/build-static
SIMPLEBLE_SHARED_DIR=$(SIMPLEBLE_DIR)/build

SIMPLEBLE_STATIC_LIBS=$(SIMPLEBLE_STATIC_DIR)/lib/libsimplecble.a $(SIMPLEBLE_STATIC_DIR)/lib/libsimpleble.a
SIMPLEBLE_SHARED_LIBS=$(SIMPLEBLE_SHARED_DIR)/lib/libsimplecble.a $(SIMPLEBLE_SHARED_DIR)/lib/libsimpleble.a

# Ensure the macOS external links against locally built static libs
witsensor.pd_darwin: $(SIMPLEBLE_STATIC_LIBS)

# Ensure Linux externals depend on built SimpleBLE (shared build tree)
witsensor.pd_linux: $(SIMPLEBLE_SHARED_LIBS)

# Platform detection for deps target
UNAME_S := $(shell uname -s)

.PHONY: deps
ifeq ($(UNAME_S),Darwin)
deps: $(SIMPLEBLE_STATIC_LIBS)
else ifeq ($(UNAME_S),Linux)
deps: $(SIMPLEBLE_SHARED_LIBS)
else
deps:
	@echo "No deps to build for this platform."
endif

$(SIMPLEBLE_STATIC_LIBS):
	git submodule update --init --recursive
	cd $(SIMPLEBLE_DIR) && cmake -S . -B build-static -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=OFF
	$(MAKE) -C $(SIMPLEBLE_STATIC_DIR) -j$(shell sysctl -n hw.ncpu 2>/dev/null || nproc)

$(SIMPLEBLE_SHARED_LIBS):
	git submodule update --init --recursive
	cd $(SIMPLEBLE_DIR) && cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=ON
	$(MAKE) -C $(SIMPLEBLE_SHARED_DIR) -j$(shell sysctl -n hw.ncpu 2>/dev/null || nproc)
