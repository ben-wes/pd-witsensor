# library name
lib.name = witsensor

# Helper variables for Makefile string manipulation
space := $(subst ,, )

# Pure Data path (use newer version)
PD_PATH = /Applications/Pd-0.56-1.app/Contents/Resources/src
# Allow overriding via environment; default to PD_PATH
PDINCLUDEDIR ?= $(PD_PATH)

# source files
witsensor.class.sources = pd-witsensor-ble.c witsensor_ble_simpleble.c macos_bt_auth.m

# include directories (use submodule SimpleBLE C API)
# Add export include paths for both static (macOS) and shared (Linux) builds
cflags = -I. -I./SimpleBLE/simplecble/include -I./SimpleBLE/simpleble/include -I./SimpleBLE/simplecble/build-static/simpleble/export -I./SimpleBLE/simplecble/build/simpleble/export

# libraries
ldlibs = -lpthread

# platform-specific settings - SimpleBLE for all platforms
define forDarwin
	# Link against static libs built by SimpleBLE (no runtime dylib needed)
	ldlibs += -L./SimpleBLE/simplecble/build-static/lib -Wl,-force_load,./SimpleBLE/simplecble/build-static/lib/libsimplecble.a -Wl,-force_load,./SimpleBLE/simplecble/build-static/lib/libsimpleble.a -framework CoreBluetooth -framework Foundation
endef

define forLinux
	# Link against shared libs produced under build/lib on Linux
	ldlibs += -L./SimpleBLE/simplecble/build/lib -lsimplecble -lsimpleble
endef

define forWindows
	# Link against static libs built by SimpleBLE (similar to macOS)
	ldlibs += -L./SimpleBLE/simplecble/build-windows/lib -Wl,--whole-archive,./SimpleBLE/simplecble/build-windows/lib/libsimplecble.a,./SimpleBLE/simplecble/build-windows/lib/libsimpleble.a,--no-whole-archive -lws2_32 -liphlpapi -lole32 -lsetupapi
endef

# data files
datafiles = \
	README.md \
	witsensor-help.pd \
	${empty}

# include pd-lib-builder
PDLIBBUILDER_DIR=./pd-lib-builder
include $(PDLIBBUILDER_DIR)/Makefile.pdlibbuilder

# SimpleBLE dependencies (build static for macOS and shared for Linux)
SIMPLEBLE_DIR=SimpleBLE/simplecble
SIMPLEBLE_STATIC_DIR=$(SIMPLEBLE_DIR)/build-static
SIMPLEBLE_SHARED_DIR=$(SIMPLEBLE_DIR)/build

SIMPLEBLE_STATIC_LIBS=$(SIMPLEBLE_STATIC_DIR)/lib/libsimplecble.a $(SIMPLEBLE_STATIC_DIR)/lib/libsimpleble.a
SIMPLEBLE_SHARED_LIBS=$(SIMPLEBLE_SHARED_DIR)/lib/libsimplecble.a $(SIMPLEBLE_SHARED_DIR)/lib/libsimpleble.a
SIMPLEBLE_WINDOWS_LIBS=$(SIMPLEBLE_DIR)/build-windows/lib/libsimplecble.a $(SIMPLEBLE_DIR)/build-windows/lib/libsimpleble.a

# Ensure the macOS external links against locally built static libs
witsensor.pd_darwin: $(SIMPLEBLE_STATIC_LIBS)

# Ensure Linux externals depend on built SimpleBLE (shared build tree)
witsensor.pd_linux: $(SIMPLEBLE_SHARED_LIBS)

# Ensure Windows externals depend on built SimpleBLE (static build tree)
%.dll: $(SIMPLEBLE_WINDOWS_LIBS)

# Platform detection for deps target
UNAME_S := $(shell uname -s)

.PHONY: deps
ifeq ($(UNAME_S),Darwin)
deps: $(SIMPLEBLE_STATIC_LIBS)
else ifeq ($(UNAME_S),Linux)
deps: $(SIMPLEBLE_SHARED_LIBS)
else ifeq ($(OS),Windows_NT)
deps: $(SIMPLEBLE_WINDOWS_LIBS)
else
deps:
	@echo "No deps to build for this platform."
endif

$(SIMPLEBLE_STATIC_LIBS):
	git submodule update --init --recursive
	# Build SimpleBLE for the target architecture(s). Convert space-separated arch list to semicolon-separated for CMake.
	cd $(SIMPLEBLE_DIR) && cmake -S . -B build-static -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=OFF $(if $(arch),-DCMAKE_OSX_ARCHITECTURES="$(subst $(space),;,$(arch))",)
	$(MAKE) -C $(SIMPLEBLE_STATIC_DIR) -j$(shell sysctl -n hw.ncpu 2>/dev/null || nproc)

$(SIMPLEBLE_SHARED_LIBS):
	git submodule update --init --recursive
	cd $(SIMPLEBLE_DIR) && cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=ON
	$(MAKE) -C $(SIMPLEBLE_SHARED_DIR) -j$(shell sysctl -n hw.ncpu 2>/dev/null || nproc)

$(SIMPLEBLE_WINDOWS_LIBS):
	git submodule update --init --recursive
	cd $(SIMPLEBLE_DIR) && cmake -S . -B build-windows -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=OFF -G "MSYS Makefiles"
	$(MAKE) -C $(SIMPLEBLE_DIR)/build-windows -j$(shell nproc 2>/dev/null || echo 4)
