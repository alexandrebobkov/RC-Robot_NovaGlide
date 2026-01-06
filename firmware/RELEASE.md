# RC-Robot NovaGlide Firmware

### Hrdware Requirements

This project is designed and optimized for the Espressif microcontroller ESP32-C3

## Creating the Project

### 1. Create a project

Generates a new ESP-IDF project folder with the standard structure (main/, CMakeLists, etc.) so you have a clean base to copy the source code.

``` sh
idf.py create-project ESP-IDF_NovaGlide
```

### 2. Navigate into your project folder

You must be inside the project directory before running any ESP-IDF commands.

``` sh
cd ESP-IDF_NovaGlide
```

### 3. Set target microprocessor

This configures the toolchain, compiler flags, and build system specifically for the ESP32-C3 architecture.

``` sh
idf.py set-target esp32c3
```

### 4. Copy all source files and CMake files for the project

Move your NovaGlide firmware (main/, subsystems/, CMakeLists.txt, etc.) into this project folder so the build system can compile your actual code.

### 5. Add dependencies

Adds the INA219 sensor driver from the ESP Component Registry and updates your project's dependency file.

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

``` sh
idf.py build
```

### Flash the firmware

``` sh
idf.py -p PORT flash
```

## Flash the firmware from build directory

``` sh
python -m esptool --chip esp32c3 -b 460800 --before default_reset --after hard_reset write_flash "@flash_args"
```

### Flash the firmware from the repository firmware release directory
``` sh
esptool [--chip] [--port] [--baud] [--before] [--after] [--flash_mode] [--flash_size] [--flash_freq] [bootloader.bin] [partition.bin] [formware.bin]
```

``` sh
python -m esptool --chip esp32c3 -b 460800 --before default_reset --after hard_reset write_flash 
--flash_mode dio --flash_size 2MB --flash_freq 80m 
0x0 firmware/release/bootloader.bin 
0x8000 firmware/release/partition-table.bin 
0x10000 firmware/release/ESP-IDF_NovaGlide.bin

```
