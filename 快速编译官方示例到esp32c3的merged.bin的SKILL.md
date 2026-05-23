---
name: "esp-idf-build"
description: "Build ESP-IDF projects for ESP32 family chips using GitHub Actions CI. Invoke when user wants to compile ESP32/ESP32C3 firmware, set up ESP-IDF CI, or troubleshoot ESP-IDF compilation issues."
---

# ESP-IDF Build Skill

This skill helps you build ESP-IDF projects for Espressif ESP32 family chips (ESP32, ESP32C3, ESP32S2, ESP32S3, etc.) using GitHub Actions CI.

## When to Invoke

- User wants to compile ESP32 firmware via GitHub Actions
- User needs to set up ESP-IDF CI/CD pipeline
- User encounters ESP-IDF compilation errors
- User wants to create merged.bin for direct flashing

## Core Knowledge

### 1. esp-idf-ci-action Architecture

**CRITICAL**: `espressif/esp-idf-ci-action@v1` runs in a **Docker container**, not the GitHub runner host.

```
┌─────────────────────────────────────────────────────────┐
│ GitHub Runner (ubuntu-latest)                           │
│  ┌───────────────────────────────────────────────────┐  │
│  │ Docker Container (espressif/idf:v5.4.4)           │  │
│  │  - /opt/esp/idf/          (ESP-IDF installation)  │  │
│  │  - /opt/esp/python_env/   (Python virtual env)    │  │
│  │  - /app/                  (mounted project)       │  │
│  │                                                    │  │
│  │  Build happens HERE, not on host!                 │  │
│  └───────────────────────────────────────────────────┘  │
│                                                         │
│  Subsequent steps run on HOST, cannot access container  │
└─────────────────────────────────────────────────────────┘
```

**Implication**: All build operations must complete within the action's container. Use `command` parameter for multi-step operations.

### 2. Correct CI Configuration Pattern

```yaml
- name: Build and Merge BIN
  uses: espressif/esp-idf-ci-action@v1
  with:
    esp_idf_version: v5.4.4      # Match your code's API version
    target: esp32c3               # Target chip
    path: ./your-project          # Project directory
    command: >                    # Override default "idf.py build"
      bash -c "idf.py build &&
      esptool.py --chip esp32c3 merge_bin
      -o build/firmware_merged.bin
      --flash_mode dio --flash_freq 80m --flash_size 4MB
      0x0 build/bootloader/bootloader.bin
      0x8000 build/partition_table/partition-table.bin
      0x10000 build/your_app.bin"
```

### 3. Common Pitfalls & Solutions

| Problem | Cause | Solution |
|---------|-------|----------|
| `undefined reference to X` | ESP-IDF version mismatch | Check API version, upgrade `esp_idf_version` |
| `/opt/esp/idf/export.sh: No such file` | Running in host, not container | Use `command` parameter, not separate step |
| `python: No module named esptool` | Wrong Python environment | All operations must be in container |
| Wrong bin filename | CMakeLists.txt `project()` name differs | Check `project(X)` in CMakeLists.txt, bin is `X.bin` |
| CI path not found | Matrix path doesn't match directory | Verify `path` matches actual project folder |

### 4. ESP-IDF Version Compatibility

| ESP-IDF Version | Key API Changes |
|-----------------|-----------------|
| v5.2.x | Stable, widely used |
| v5.3.x | BLE 5.0 features |
| v5.4.x | `ESP_BLE_GAP_CONN_ITVL_MS`, `ESP_BLE_ADV_NAME_LEN_MAX`, `esp_ble_gattc_enh_open` |
| v6.0.x | Major API changes |

**Rule**: Check example code's API usage against ESP-IDF release notes.

### 5. merge_bin Address Mapping

| Offset | Component |
|--------|-----------|
| `0x0` | bootloader.bin |
| `0x8000` | partition-table.bin |
| `0x10000` | app.bin |

**Flash Parameters**:
- `--flash_mode`: dio (common), qio, dout, qout
- `--flash_freq`: 80m, 40m, 26m, 20m
- `--flash_size`: 4MB, 2MB, 8MB, 16MB

## CI Template

### Minimal Template

```yaml
name: Build ESP32C3
on:
  push:
    branches: [main]
  workflow_dispatch:

jobs:
  build:
    runs-on: ubuntu-latest
    strategy:
      matrix:
        project:
          - {name: myapp, path: ./myapp, bin_name: myapp}
    steps:
    - uses: actions/checkout@v4
    - uses: espressif/esp-idf-ci-action@v1
      with:
        esp_idf_version: v5.4.4
        target: esp32c3
        path: ${{ matrix.project.path }}
        command: >
          bash -c "idf.py build &&
          esptool.py --chip esp32c3 merge_bin
          -o build/${{ matrix.project.name }}_merged.bin
          --flash_mode dio --flash_freq 80m --flash_size 4MB
          0x0 build/bootloader/bootloader.bin
          0x8000 build/partition_table/partition-table.bin
          0x10000 build/${{ matrix.project.bin_name }}.bin"
    - uses: actions/upload-artifact@v4
      with:
        name: ${{ matrix.project.name }}_merged.bin
        path: ${{ matrix.project.path }}/build/${{ matrix.project.name }}_merged.bin
```

### Multi-Project Template

```yaml
strategy:
  matrix:
    project:
      - {name: gatt_client, path: ./gatt_client, bin_name: gatt_client_demo}
      - {name: gatt_server, path: ./gatt_server, bin_name: gatt_server_demos}
```

## Pre-Build Checklist

Before setting up CI, verify:

- [ ] ESP-IDF version matches example code's API requirements
- [ ] `path` in matrix matches actual project directory
- [ ] `bin_name` matches `project()` name in CMakeLists.txt
- [ ] Target chip is correct (esp32, esp32c3, esp32s2, esp32s3)
- [ ] Flash parameters match your hardware (mode, freq, size)

## Troubleshooting Guide

### Compilation Errors

1. **Undefined symbol / implicit declaration**
   - Check ESP-IDF version compatibility
   - Search symbol in ESP-IDF release notes
   - Upgrade `esp_idf_version` if needed

2. **Component not found**
   - Add to `CMakeLists.txt`: `REQUIRES component_name`
   - Or use `PRIV_REQUIRES` for private dependencies

### CI/CD Errors

1. **Path not found**
   - Verify directory structure
   - Check `path` in matrix configuration

2. **Docker/container issues**
   - All operations must be in `command` parameter
   - Cannot access `/opt/esp/` from host steps

3. **Artifact not uploaded**
   - Check path in `upload-artifact` step
   - Verify file was created in `command` step

## Useful Commands

```bash
# Check bin file info
esptool.py --chip esp32c3 image_info firmware_merged.bin

# Flash directly
esptool.py --chip esp32c3 -p COM3 write_flash 0x0 firmware_merged.bin

# Build locally
idf.py build
idf.py -p COM3 flash
```

## References

- [ESP-IDF Documentation](https://docs.espressif.com/projects/esp-idf/)
- [esp-idf-ci-action GitHub](https://github.com/espressif/esp-idf-ci-action)
- [Docker Hub: espressif/idf](https://hub.docker.com/r/espressif/idf)
- [ESP-IDF Release Notes](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/versions.html)
