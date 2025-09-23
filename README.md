## WIT Sensor Pure Data External

Pure Data external for WIT BWT901 BLE sensors. macOS build uses SimpleBLE (static) today; broader cross‑platform support is in progress.

### Status

| Platform | Backend | Status |
| --- | --- | --- |
| macOS | SimpleBLE (+ small Obj‑C auth helper) | Working (active dev) |
| Linux | SimpleBLE | Experimental (builds, untested) |
| Windows | SimpleBLE | Experimental (builds, untested) |

### Clone

```bash
# Recommended: clone with submodules
git clone --recurse-submodules https://github.com/<your-org>/pd-witsensor.git
cd pd-witsensor

# If you already cloned without submodules
git submodule update --init --recursive
```

### Build (all platforms)

```bash
# Build SimpleBLE and the external
make deps && make
```

Note: On macOS we link SimpleBLE statically; on Linux/Windows we use the shared build tree. The same commands above handle both.

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
- Goals: unify Linux/Windows builds via the same SimpleBLE path.

### License

- **This project**: Business Source License 1.1 (BUSL‑1.1). Non‑commercial use permitted; commercial use requires a license. Each version converts to GPL‑3 after four years. See `LICENSE`.
- **SimpleBLE**: Business Source License 1.1 (BUSL‑1.1). Non‑commercial use permitted; commercial use requires a SimpleBLE commercial license. Each version converts to GPL‑3 after four years. See `SimpleBLE/LICENSE.md`.
- **WIT sensors**: This project uses the sensor's communication protocol (not proprietary SDK code).
