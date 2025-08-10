# Codebase Notes

These notes are intended to assist in understanding how the `esp-drone` codebase is structured and how it functions. It is based on Commit `527ee2e` on [GitHub](https://github.com/espressif/esp-drone) and assumes you are using ESP-IDF v5.0 installed via the VS Code ESP-IDF extension. It also assumes you are using an ESP32-S3 for your drone.

## Getting Started

To start exploring and modifying the `esp-drone` codebase, follow this guide.

### Configuring VS Code

First configure VS Code for the `esp-drone` project by following these steps:

1. Clone the project from [GitHub](https://github.com/espressif/esp-drone) and open the project folder in VS Code.
1. Install the following extensions in VS Code:
    - C/C++
    - C/C++ Extension Pack
    - C/C++ Themes
    - CMake Tools
    - ESP-IDF
1. In VS Code, use `ctrl+shift+p`, search `ESP-IDF: Configure ESP-IDF Extension`, and follow the Express setup mode to install ESP-IDF v5.0. 
1. Once installation is complete, follow the VS Code prompt to generate `compile_commands.json`.
1. Once `.\build\compile_commands.json` has been generated, point the C/C++ Intellisense to the compile commands by using `ctrl+shift+p` and searching `C/C++: Select Intellisense Configuration...`, selecting the newly-generated file from the presented options. 

### Testing Build

Then do a test build of the `esp-drone` project by following these steps:

1. Set the device target by using `ctrl+shift+p` and searching `ESP-IDF: Set Target`, selecting `esp32s3` from the list of options, and then `ESP32-S3 chip (via builtin USB-JTAG)`. 
1. Build the project by using `ctrl+shift+p` and searching `ESP-IDF: Build Your Project`. 


## Reset / Power-Up

This outlines what happens when the `esp-drone` project has been flashed to an ESP32-S3 and then resets or powers up. 

### ROM Bootloader

The ESP32-S3 has an immutable bootloader that has been burned into ROM, which runs after the ESP32-S3 has been powered up or reset (including the reset that occurs after flashing your code). 

**What it does:**
- Initialises the clocks, the CPU, and a minimal amount of RAM so that basic start up code can be executed.
- Configures the SPI (serial peripheral interface) for flash memory so that flashed code can be accessed.
- Reads the partition table from a fixed flash address so that it can locate the second-stage bootloader.
- Loads the second-stage bootloader from flash into RAM and jumps to it to begin executing it.

### Second-Stage Bootloader

When you build and flash a project using ESP-IDF, the second-stage bootloader will be generated for you at runtime when you build the project before flashing it to your device. The ROM bootloader will load it into RAM and jump to it after powering up or resetting.

**What it does:**
- Reads the flash's partition table to locate the available app partitions.
- Chooses which app partition to boot (your app is usually located in the default `factory` partition).
- Finishes initialising RAM (including external PSRAM if you have it) so it can all be used by your app.
- Further initialises the clocks to perform a full boot.
- Configures and starts the MMU (memory management unit), which maps sections of your app code's instructions and constants to flash memory addresses. The instructions and constants are loaded into CPU cache in small chunks, and if the next instruction is in the cache then it will execute immediately. If not, the MMU will load it on demand from flash into cache for subsequent execution (called execute-in-place, or XIP).
- Jumps to your app's "reset vector", which is the very first instruction in your app.

### App Startup Code

Part of the build process involves including some startup code (part of the ESP-IDF and ESP's FreeRTOS port) that links the boot process to your app code. This is usually your app's "reset vector" and runs on CPU0 after the second-stage bootloader completes. 

**What it does on CPU0:**
- `esp\v5.0\esp-idf\components\esp_system\port\cpu_start.c` contains the ESP-IDF entrypoint function `call_start_cpu0()` that acts as the app's "reset vector". It does a bunch of low-level bring-up and then calls the `SYS_STARTUP_FN()` macro. This ultimately resolves to calling `start_cpu0_default()` on CPU0.
- `esp\v5.0\esp-idf\components\esp_system\startup.c` contains `start_cpu0_default()`, which does more initialisation and calls the FreeRTOS function `esp_startup_start_app()`.
- `esp\v5.0\esp-idf\components\freertos\FreeRTOS-Kernel\portable\xtensa\port.c` contains `esp_startup_start_app()`, which calls `esp_startup_start_app_common()` and then starts the FreeRTOS scheduler on CPU0 (which also starts the FreeRTOS tick timer). 
- `esp\v5.0\esp-idf\components\freertos\FreeRTOS-Kernel\portable\port_common.c` contains `esp_startup_start_app_common()`, which initialises cross-core interrupts on CPU0 and creates the `main` task in the `Ready` state from the `main_task()` function.
- The running scheduler on CPU0 sees the `main` task in its list of ready tasks and begins executing its `main_task()` function. 
- `esp\v5.0\esp-idf\components\freertos\FreeRTOS-Kernel\portable\port_common.c` contains the `main_task()` function, which waits for the second core to finish initialising (if there is one), frees the RAM of FreeRTOS startup stacks, and then calls the `app_main()` function (in your project's directory). Usually, the `app_main()` function (indirectly) creates your app's tasks that run periodically via the scheduler and then returns.
- If `app_main()` has this "bootstrap-and-return" pattern and returns after completion, the `main_task()` function will delete the `main` task from the scheduler's list of tasks, freeing its stack in RAM.

**What it does on CPU1:**
- If there is a second core (like on the ESP32-S3), then the `call_start_cpu0()` function also sets the boot address of CPU1 to the `call_start_cpu1()` function via `start_other_core()` and resets CPU1, which causes CPU1 to start executing `call_start_cpu1()`. 
- Like `call_start_cpu0()`, the `call_start_cpu1()` function also does a bunch of low-level bring-up and then calls the `SYS_STARTUP_FN()` macro. This ultimately resolves to calling `start_cpu_other_cores_default()` on CPU1.
- `esp\v5.0\esp-idf\components\esp_system\startup.c` contains `start_cpu_other_cores_default()`, which does a bunch of initialisation and calls `esp_startup_start_app_other_cores()`.
- `esp\v5.0\esp-idf\components\freertos\FreeRTOS-Kernel\portable\xtensa\port.c`  contains `esp_startup_start_app_other_cores()`, which waits for the FreeRTOS scheduler to start on CPU0, initialises cross-core interrupts on CPU1, and starts the FreeRTOS scheduler on CPU1 (note that the ESP32-S3 does not have symmetric multi-processing, or SMP, and so each CPU runs its own scheduler and interrupts keep the cores synchronised instead of a global scheduler). 

