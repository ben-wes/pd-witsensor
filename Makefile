# library name
lib.name = witsensor

# Default macOS minimum versions for local builds (use these names — not MACOSX_DEPLOYMENT_*,
# which Xcode often exports empty and would break ?= defaults).
# Override: MACOSX_DEPLOYMENT_TARGET, or WITSENSOR_MACOSX_INTEL_MIN / WITSENSOR_MACOSX_ARM64_MIN.
# Apple Silicon binaries cannot target macOS 10.x. Intel: SimpleBLE uses CoreBluetooth
# authorization + NSDate.now (macOS 10.15+); 10.14 is not consistent with that code.
WITSENSOR_MACOSX_INTEL_MIN := 10.15
WITSENSOR_MACOSX_ARM64_MIN := 11.0

# Helper variables for Makefile string manipulation
space := $(subst ,, )

# source files
witsensor.class.sources = witsensor.c witsensor_ble_simpleble.c
witmagic.class.sources = witmagic.c
butter3~.class.sources = butter3~.c

# include directories (use submodule SimpleBLE C API)
# Add export include paths for both static (macOS) and shared (Linux) builds.
# Newer SimpleBLE generates export headers under <build>/export, not <build>/simpleble/export.
cflags = -I. -I./SimpleBLE/simplecble/include -I./SimpleBLE/simpleble/include -I./SimpleBLE/simplecble/build-static/export -I./SimpleBLE/simplecble/build/export

# libraries
ldlibs = -lpthread -lm

# platform-specific settings - SimpleBLE for all platforms
# pd-lib-builder: any -mmacosx-version-min= in cflags becomes version.flag (link + compile).
define forDarwin
cflags += -mmacosx-version-min=$(MACOSX_DEPLOYMENT_TARGET)
ldlibs += -L./SimpleBLE/simplecble/build-static/lib -Wl,-force_load,./SimpleBLE/simplecble/build-static/lib/libsimplecble.a -Wl,-force_load,./SimpleBLE/simplecble/build-static/lib/libsimpleble.a -framework CoreBluetooth -framework Foundation
witsensor.class.sources += macos_bt_auth.m
endef

define forLinux
	# Link against shared libs produced under build/lib on Linux
	ldlibs += -L./SimpleBLE/simplecble/build/lib -lsimplecble -lsimpleble
endef

define forWindows
	# Add Windows-specific include path for SimpleBLE
	cflags += -I./SimpleBLE/simplecble/build-windows/export
	# Link against MinGW-built SimpleBLE libraries
	ldlibs += -L./SimpleBLE/simplecble/build-windows/lib -lsimpleble -lws2_32 -liphlpapi -lole32 -lsetupapi
endef

# data files
datafiles = \
	README.md \
	witsensor-help.pd \
	witmagic-help.pd \
	butter3~-help.pd \
	${empty}

# Set before pd-lib-builder reads cflags; ifeq inside $(eval forDarwin) is unreliable here.
ifeq ($(shell uname -s),Darwin)
ifeq ($(strip $(MACOSX_DEPLOYMENT_TARGET)),)
MACOSX_DEPLOYMENT_TARGET := $(if $(filter arm64,$(arch)),$(WITSENSOR_MACOSX_ARM64_MIN),$(if $(filter x86_64,$(arch)),$(WITSENSOR_MACOSX_INTEL_MIN),$(if $(filter arm64,$(shell uname -m)),$(WITSENSOR_MACOSX_ARM64_MIN),$(WITSENSOR_MACOSX_INTEL_MIN))))
endif
export MACOSX_DEPLOYMENT_TARGET
endif

# include pd-lib-builder
PDLIBBUILDER_DIR=./pd-lib-builder
include $(PDLIBBUILDER_DIR)/Makefile.pdlibbuilder

# Build Objective-C helper for all architectures (place AFTER include so 'all' stays default)
macos_bt_auth.d_amd64.o: macos_bt_auth.m
	cc -DPD -I "$(PDINCLUDEDIR)" -Wall -Wextra -O3 -arch x86_64 -mmacosx-version-min=$(MACOSX_DEPLOYMENT_TARGET) -c macos_bt_auth.m -o macos_bt_auth.d_amd64.o

macos_bt_auth.d_arm64.o: macos_bt_auth.m
	cc -DPD -I "$(PDINCLUDEDIR)" -Wall -Wextra -O3 -arch arm64 -mmacosx-version-min=$(MACOSX_DEPLOYMENT_TARGET) -c macos_bt_auth.m -o macos_bt_auth.d_arm64.o

# Build rules for shared library extensions
macos_bt_auth.darwin-amd64-64.so.o: macos_bt_auth.m
	cc -DPD -I "$(PDINCLUDEDIR)" -Wall -Wextra -O3 -arch x86_64 -mmacosx-version-min=$(MACOSX_DEPLOYMENT_TARGET) -c macos_bt_auth.m -o macos_bt_auth.darwin-amd64-64.so.o

macos_bt_auth.darwin-arm64-64.so.o: macos_bt_auth.m
	cc -DPD -I "$(PDINCLUDEDIR)" -Wall -Wextra -O3 -arch arm64 -mmacosx-version-min=$(MACOSX_DEPLOYMENT_TARGET) -c macos_bt_auth.m -o macos_bt_auth.darwin-arm64-64.so.o

macos_bt_auth.darwin-amd64-32.so.o: macos_bt_auth.m
	cc -DPD -I "$(PDINCLUDEDIR)" -Wall -Wextra -O3 -arch x86_64 -mmacosx-version-min=$(MACOSX_DEPLOYMENT_TARGET) -c macos_bt_auth.m -o macos_bt_auth.darwin-amd64-32.so.o

macos_bt_auth.darwin-arm64-32.so.o: macos_bt_auth.m
	cc -DPD -I "$(PDINCLUDEDIR)" -Wall -Wextra -O3 -arch arm64 -mmacosx-version-min=$(MACOSX_DEPLOYMENT_TARGET) -c macos_bt_auth.m -o macos_bt_auth.darwin-arm64-32.so.o

# Build rule for generic pd_darwin extension (local builds)
macos_bt_auth.pd_darwin.o: macos_bt_auth.m
	cc -DPD -I "$(PDINCLUDEDIR)" -Wall -Wextra -O3 -mmacosx-version-min=$(MACOSX_DEPLOYMENT_TARGET) -c macos_bt_auth.m -o macos_bt_auth.pd_darwin.o

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
	# Build SimpleBLE for the target architecture(s). Convert space-separated arch list to semicolon-separated for CMake.
	cd $(SIMPLEBLE_DIR) && cmake -S . -B build-static -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=OFF -DCMAKE_OSX_DEPLOYMENT_TARGET="$(MACOSX_DEPLOYMENT_TARGET)" $(if $(arch),-DCMAKE_OSX_ARCHITECTURES="$(subst $(space),;,$(arch))",)
	$(MAKE) -C $(SIMPLEBLE_STATIC_DIR) -j$(shell sysctl -n hw.ncpu 2>/dev/null || nproc)

$(SIMPLEBLE_SHARED_LIBS):
	cd $(SIMPLEBLE_DIR) && cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=ON
	$(MAKE) -C $(SIMPLEBLE_SHARED_DIR) -j$(shell sysctl -n hw.ncpu 2>/dev/null || nproc)

$(SIMPLEBLE_WINDOWS_LIBS):
	cd $(SIMPLEBLE_DIR) && cmake -S . -B build-windows -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=OFF -G "MSYS Makefiles"
	$(MAKE) -C $(SIMPLEBLE_DIR)/build-windows -j$(shell nproc 2>/dev/null || echo 4)
