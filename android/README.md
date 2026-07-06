# CO2 Sensor Android App

This is a standalone Android app for the BLE sensor described in `../SENSOR.md`.
It does not share source or build configuration with the embedded Rust project.

## Build

Open this `android/` directory in Android Studio and run the `app` target, or
build from this directory with:

```sh
gradle :app:assembleDebug
```

The current source uses only Android platform APIs. No Compose, AppCompat, or
Kotlin dependencies are required.

## BLE behavior

- Scans for service UUID `b22d7f14-9361-4309-932e-ffbdefed97fe`.
- Subscribes to CO2 characteristic `c7d0c8a8-db04-4199-869d-5f80091f2036`.
- Subscribes to temperature characteristic `c7d0c8a8-db04-4199-869d-5f80091f2037`.
- Decodes both notification payloads as little-endian 16-bit values.
