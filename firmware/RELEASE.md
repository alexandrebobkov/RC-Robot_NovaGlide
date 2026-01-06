# RC-Robot NovaGlide Firmware

### Hrdware Requirements

- Espressif microcontroller ESP32-C3

## Flashing the Firmware

### 1. Create a project
``` sh
idf.py create-project ESP-IDF_NovaGlide
```

### 2. Navigate into your project folder
``` sh
cd ESP-IDF_NovaGlide
```

### 3. Set target microprocessor
``` sh
idf.py set-target esp32c3
```

### Copy all source files and CMake files for the project

### Add dependencies:
``` sh 
idf.py add-dependency esp-idf-lib/ina219
```
``` sh
idf.py add-dependency esp-idf-lib/i2cdev
```
``` sh
idf.py add-dependency esp-idf-lib/ultrasonic
```

### Build the project

``` sh idf.py build ```

### Flash the firmware

``` sh idf.py -p PORT flash ```

## Flash the firmware from build directory

``` sh
python -m esptool --chip esp32c3 -b 460800 --before default_reset --after hard_reset write_flash "@flash_args"
```

### Flash the firmware from the repository firmware release directory
``` sh esptool [--chip] [--port] [--baud] [--before] [--after] [--flash_mode] [--flash_size] [--flash_freq] [bootloader.bin] [partition.bin] [formware.bin] ```
``` sh
python -m esptool --chip esp32c3 -b 460800 --before default_reset --after hard_reset write_flash --flash_mode dio --flash_size 2MB --flash_freq 80m 0x0 firmware/release/bootloader.bin 0x8000 firmware/release/partition-table.bin 0x10000 firmware/release/ESP-IDF_NovaGlide.bin

```
