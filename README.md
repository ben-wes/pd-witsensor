## [witsensor] Pure Data External

Pure Data external for WIT BWT901 BLE sensors. macOS build uses SimpleBLE (static)

### Status

| Platform | Backend | Status |
| --- | --- | --- |
| macOS | SimpleBLE (+ small Obj‑C auth helper) | tested |
| Linux | SimpleBLE | experimental |
| Windows | SimpleBLE | not yet available |

### Clone

```bash
# Recommended: clone with submodules
git clone --recurse-submodules https://github.com/ben-wes/pd-witsensor.git
cd pd-witsensor

# If you already cloned without submodules
git submodule update --init --recursive
```

### Build (all platforms)

```bash
# Build SimpleBLE and the external
make deps && make
```

Note: On macOS we link SimpleBLE statically; on Linux we use the shared build tree. The same commands above handle both.

### Usage (Pd)

```pd
[scan(
[connect <name-or-mac>(
[disconnect(
|
[witsensor]
```

### Project notes

- Implementation: `witsensor_ble_simpleble.c` (C, SimpleBLE). Small `macos_bt_auth.m` helper for Bluetooth permission/auth prompts.
- Build system: `Makefile` integrates SimpleBLE builds (`make deps`).

### License

- **This project**: Unlicense (public domain). Free for any use. See `LICENSE`.
- **SimpleBLE**: Business Source License 1.1 (BUSL‑1.1). Non‑commercial use permitted; commercial use requires a SimpleBLE commercial license. Each version converts to GPL‑3 after four years. See `SimpleBLE/LICENSE.md`.
- **WIT sensors**: This project uses the sensor's communication protocol (not proprietary SDK code).
