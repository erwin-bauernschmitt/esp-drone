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


## ESP Drone App Code

### Platform Initialisation

- After initialising non-volatile storage (NVS) flash memory for wifi communications, `app_main()` calls `platformInit()`. 
- `esp-drone\components\platform\platform.c` contains the `platformInit()` function, which is seemingly just scaffolding for supporting different hardware platforms. Currently, it defaults to the `EP20` device type, which causes the `active_config` instance of the `platformConfig_t` struct to be set to a default. The variables in that default are read by subsequent initialisation functions, and the default struct looks like this:

```
static const platformConfig_t *active_config;

active_config -> (platformConfig_t){
  .deviceType = "EP20",
  .deviceTypeName = "ESPlane 2.0 ",
  .sensorImplementation = SensorImplementation_mpu6050_HMC5883L_MS5611,
  .physicalLayoutAntennasAreClose = false,
  .motorMap = motorMapDefaultBrushed,
};
```

- Here, `SensorImplementation_mpu6050_HMC5883L_MS5611` is just an enum value, but `esp-drone\components\drivers\general\motors\motors_def_cf2.c` ultimately defines `motorMapDefaultBrushed` as follows: 

```
static const MotorPerifDef CONN_M1 = { .drvType = BRUSHED };
static const MotorPerifDef CONN_M2 = { .drvType = BRUSHED };
static const MotorPerifDef CONN_M3 = { .drvType = BRUSHED };
static const MotorPerifDef CONN_M4 = { .drvType = BRUSHED };

const MotorPerifDef *motorMapDefaultBrushed[NBR_OF_MOTORS] = {
    &CONN_M1, &CONN_M2, &CONN_M3, &CONN_M4
};
```

### The SYSTEM Task

- After `app_main()` calls `platformInit()`, it then calls `systemLaunch()`. 
- `esp-drone\components\core\crazyflie\modules\src\system.c` contains the `systemLaunch()` function, which contains the `STATIC_MEM_TASK_CREATE()` macro. This resolves to calling the `xTaskCreateStatic()` function to create the `SYSTEM` task using the stack that was initialised in memory by the file scope line `STATIC_MEM_TASK_ALLOC(systemTask, SYSTEM_TASK_STACKSIZE);` in `system.c`. The `SYSTEM` task runs the `systemTask()` function, which is also defined in `system.c`.
- After creating the `SYSTEM` task, the `systemLaunch()` function returns, which causes the `app_main()` function to return, which causes the `main_task()` function from ESP-IDF's FreeRTOS port to delete the `main` task that FreeRTOS created to launch the app code (i.e., the task that ran the `main_task()` function). 
- At this point, the app startup code provided by ESP-IDF has fully completed and the ESP Drone app code is entirely managing the FreeRTOS tasks. 
- The `SYSTEM` task is then the only ready task available and so the `systemTask()` function begins executing, starting with a bunch of initialisation.  

#### LED Initialisation

- First, `systemTask()` calls `ledInit()`, which is defined in `esp-drone\components\drivers\general\led\led_esp32.c`.
- `ledInit()` initialises the GPIO for three LEDs (Blue, Red, and Green) by first instantiating a `gpio_config_t` struct and then passing it to the `gpio_config()` function from ESP-IDF. 
- Then `systemTask()` calls `ledSet(CHG_LED, 1)`, which is aslo defined in `esp-drone\components\drivers\general\led\led_esp32.c`.
- `ledSet(CHG_LED, 1)` causes the Red LED to be turned on, indicating that the ESP Drone app code is starting up. 

#### Wifi Initialisation

- Then `systemTask()` calls `wifiInit()`, which is defined in `esp-drone\components\drivers\general\wifi\wifi_esp32.c`.
- `wifi_esp32.c` globally defines several values, including the following:

```
#define UDP_SERVER_PORT         2390  // UDP socket listens here
#define UDP_SERVER_BUFSIZE      64  // 64B for temp Rx buffer
static char WIFI_SSID[32] = "";
static char WIFI_PWD[64] = CONFIG_WIFI_PASSWORD;  // = 12345678
static uint8_t WIFI_CH = CONFIG_WIFI_CHANNEL;  // = 6
#define WIFI_MAX_STA_CONN CONFIG_WIFI_MAX_STA_CONN  // = 3
```

- And `wifi_esp32.h` also defines: 

```
// 32 bytes is enough for CRTP packets (30+1) + checksum 1
#define WIFI_RX_TX_PACKET_SIZE   (32)

/* Structure used for in/out data via USB */
typedef struct
{
  uint8_t size;
  uint8_t data[WIFI_RX_TX_PACKET_SIZE];
} UDPPacket;
```

- `wifiInit()` first initialises two queues of length 16*33B called `udpDataRx` and `udpDataTx` using the FreeRTOS function `xQueueCreate()`. These are essentially thread-safe FIFO buffers in heap memory that can hold 16 `UDPPacket` structs.
- `wifiInit()` then calls `espnow_storage_init()`, which essentially ensures NVS flash was correctly initialised for wifi communication by `app_main()`. If erasing and reflashing isn't working to configure NVS, it will flag that the ESP32 should select the next app partition for the next reboot if it exists (i.e., boot the application firmware from the `ota_1` app slot instead of the `factory` one). 
- `wifiInit()` then initialises an empty pointer called `ap_netif` that will store the address of an `esp_netif_t` network interface object, which will be opaquely created and managed by ESP-IDF.
- `wifiInit()` then calls the `esp_netif_init()` function from ESP-IDF, which initialises the LwIP TCP/IP stack. This indirectly creates the LwIP core task, which handles all of the internet protocol aspects of wifi communication such as UDP/TCP, IP, ARP/ND, DHCP, DNS. It also prepares the LwIP core task to connect with an `esp_netif` network interface. 
- `wifiInit()` then calls `esp_event_loop_create_default()` to create the default event loop, which includes creating the `sys_evt` task and a corresponding queue. This task accepts network events from various parts of the wifi stack and serialises them so the events can run their callbacks safely. 
- `wifiInit()` then calls the `esp_netif_create_default_wifi_ap()` function, which loads a default configuration for a SoftAP `esp_netif_t` network interface object, uses that configuration to create an `esp_netif_t` network interface object (which also creates the actual `esp_netif` interface), connects the LwIP and `esp_wifi` layers together with the `esp_netif` bridge, and registers default handlers with the default event loop (i.e., tells the `sys_evt` task what function it should run for each of the default event types that can be posted to the default event loop's queue). This function also returns a pointer to the newly created `esp_netif_t` network interface object, which `wifiInit()` assigns to `ap_netif` pointer variable. 
- `wifiInit()` then loads a default wifi driver configuration (shown below) and uses it to call the `esp_wifi_init()` function, which ultimately allocates resources for the `esp_wifi` driver (including Rx/Tx buffers and the wifi NVS structure) and starts the `wifi` task. 

```
wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT() = { .osi_funcs = &g_wifi_osi_funcs, .wpa_crypto_funcs = g_wifi_default_wpa_crypto_funcs, .static_rx_buf_num = 10, .dynamic_rx_buf_num = 32, .tx_buf_type = 1, .static_tx_buf_num = 0, .dynamic_tx_buf_num = 32, .cache_tx_buf_num = 0, .csi_enable = 0, .ampdu_rx_enable = 1, .ampdu_tx_enable = 1, .amsdu_tx_enable = 0, .nvs_enable = 1, .nano_enable = 0, .rx_ba_win = 6, .wifi_task_core_id = 0, .beacon_max_len = 752, .mgmt_sbuf_num = 32, .feature_caps = g_wifi_feature_caps, .sta_disconnected_pm = 0, .magic = 0x1F2F3F4F}
```
- `wifiInit()` then registers another handler with the default event loop using `esp_event_handler_instance_register()`, which essentially just logs when a station(device) joins or leaves the ESP32's SoftAP. 
- `wifiInit()` then gets the ESP32's AP MAC address with `esp_wifi_get_mac()` and uses it to create an SSID (i.e., the name of its wifi network) with `sprintf()` in the form `ESP-DRONE_00112233445566`.
- `wifiInit()` then defines a default configuration for the wifi radio that expands to:
```
wifi_config_t wifi_config = {
  .ap = {
    .channel = 6,  // sets 2.4GHz wifi channel
    .max_connection = 3,  // sets max number of clients
    .authmode = WIFI_AUTH_WPA_WPA2_PSK,  // requires password
  },
};
```
- `wifiInit()` then saves the SSID and password (default is `12345678`) with `memcpy()`, tells the `esp_wifi` driver to operate a SoftAP with `esp_wifi_set_mode()`, passes the `wifi_config` to the `esp_wifi` with `esp_wifi_set_config()`, and starts the SoftAP radio with `esp_wifi_start()`.
- `wifiInit()` then disables the second wifi channel with `esp_wifi_set_channel()` and loads a default configuration for ESP-NOW into `espnow_config` (shown below). 

```
espnow_config_t espnow_config = ESPNOW_INIT_CONFIG_DEFAULT() = { .pmk = "ESP_NOW", .forward_enable = 1, .forward_switch_channel = 0, .sec_enable = 0, .reserved1 = 0, .qsize = 32, .send_retry_num = 10, .send_max_timeout = ( ( TickType_t ) ( ( ( TickType_t ) ( 3000 ) * ( TickType_t ) 1000 ) / ( TickType_t ) 1000U ) ), .receive_enable = { .ack = 1, .forward = 1, .group = 1, .provisoning = 0, .control_bind = 0, .control_data = 0, .ota_status = 0, .ota_data = 0, .debug_log = 0, .debug_command = 0, .data = 0, .sec_status = 0, .sec = 0, .sec_data = 0, .reserved2 = 0, }, }
```

- `wifiInit()` then uses that configuration with `espnow_init()` to register a handler with the default event loop with `esp_event_handler_instance_register()` (just logs when a station joins or leaves the ESP32's SoftAP), register send/recv callback functions with the `esp_wifi` driver, and create the `espnow_main` task with `xTaskCreate()` using the `espnow_main_task()` function.

- `wifiInit()` then registers a handler with the default event loop that logs when another device using ESP-NOW connects to the drone as an ESP-NOW controller using `esp_event_handler_register()`, and then opens a "bind window" for 30 seconds (during which controllers can connect and be remembered as trusted controllers).  
- `wifiInit()` then registers the `espnow_ctrl_data_cb()` callback function that the ESP-NOW srecv callback function can call when it detects an ESP-NOW controller has sent a CONTROL_DATA frame. The `espnow_ctrl_data_cb()` callback function formats the parsed values from the CONTROL_DATA frame into a message and adds that message to the same `udpDataRx` queue that the UDP receiver uses (so the flight controller can read all control commands from one stream). 
- `wifiInit()` then defines a static IP (shown below), uses `esp_netif_dhcps_stop()` to stop the DHCP server that was automatically started by `esp_wifi_start()`, updates the IP info in the `esp_netif_t` network interface object that `ap_netif` points to with the new static IP using `esp_netif_set_ip_info()`, and then restarts the DHCP server with `esp_netif_dhcps_start()`.
```
esp_netif_ip_info_t ip_info = {
    .ip.addr = ipaddr_addr("192.168.43.42"),
    .netmask.addr = ipaddr_addr("255.255.255.0"),
    .gw.addr      = ipaddr_addr("192.168.43.42"),
};
```
- `wifiInit()` then uses `udp_server_create()` to tell LwIP that it needs to create a socket (which includes a queue and some other state), return the socket's handle to the `sock` variable, and bind the socket to `port 2390`. Subsequently, when LwIP receives a UDP packet destined for `port 2390`, it will post the processed UDP frame to the socket's queue. When `wifiInit()` eventually creates the `UDP_RX` task, it will loop the ESP-IDF `recvfrom()` function with the `sock` handle to ask LwIP for the next UDP frame to process. LwIP will then check the socket's queue and return the next UDP frame if there is one. If the socket's queue is empty, LwIP will block the `UDP_RX` task, and the unblocking condition is LwIP adding a new UDP frame to the socket's queue. 

- Finally, `wifiInit()` creates two tasks called `UDP_RX` and `UDP_TX` using the FreeRTOS function `xTaskCreate()`.

- The `UDP_RX` task executs the `udp_server_rx_task()` function, which is defined in `wifi_esp32.c`.
- The `udp_server_rx_task()` function loops calling the ESP-IDF `recvfrom()` function to ask LwIP for the next UDP frame to process using the socket handle `sock`. If it's length is between 0B and 32B inclusive, it computes a checksum for the message using `calculate_cksum()` and compares it to the one in the UDP packet, adding the data frame to the `udpDataRx` queue with `xQueueSend()` if the checksums match. If the socket's queue is empty, LwIP will block the `UDP_RX` task, and the unblocking condition is LwIP adding a new UDP frame to the socket's queue. 

- The `UDP_TX` task executs the `udp_server_tx_task()` function, which is defined in `wifi_esp32.c`.
- The `udp_server_tx_task()` function loops calling the FreeRTOS `xQueueReceive()` function to read the next UDP frame the app code wants to send from the `udpDataTx` queue. If the queue is empty, this will cause the `UDP_TX` task to block until a UDP frame is added to the `udpDataTx` queue. Once a UDP fame is ready, it will compute a checksum using `calculate_cksum()` and append it to the frame. It then uses the ESP-IDF `sendto()` function along with the `sock` handle to hand the UDP frame to LwIP, which will send the UDP frame to the last IP address that transmitted to `port 2390` via `esp_netif` and `esp_wifi`.  

#### Wifi Link Initialisation

- `systemTask()` then calls `systemInit()`, also defined in `system.c`. 
- The first initialisation function in `systemInit()` is `wifiLinkInit()` (defined in `wifilink.c`), which creates the `crtpPacketDelivery` queue and creates the `WIFILINK` task (with `PRIORITY=2`) that runs the `wifilinkTask()` function. 
- The `wifilinkTask()` function bridges the `udpDataRx` queue with the rest of the application logic by forwarding complete CRTP messages from the `udpDataRx` queue to the `crtpPacketDelivery` queue, reformatting any ESP-NOW messages in the `udpDataRx` queue into full CRTP messages before forwarding them. 
- The `wifilinkTask()` function retrieves messages from the `udpDataRx` queue with `wifiGetDataBlocking()`, which is defined in `wifi_esp32.c` and uses `xQueueReceive()` to block the `WIFILINK` task until a message can be received from the queue. 
- Note that `commInit()` in `comm.c` uses `crtpSetLink(wifilinkGetLink())` to point the CRTP link struct to the `wifilink.c` functions, enabling the `CRTP-TX` and `CRTP-RX` tasks to send and receive CRTP packets over UDP via the `wifilink.c` functions 

#### Task Dump Initialisation

- `systemInit()` then calls `sysLoadInit()` (defined in `sysload.c`), which creates and starts the 5-second auto-reloading `sysLoadMonitorTimer` FreeRTOS software timer. 
- Every 5 seconds, the `sysLoadMonitorTimer` will call the `timerHandler()` callback function, which logs the task dump in the terminal if `triggerDump=1` and there are less than 32 tasks (which is true initially, since `triggerDump` is preset and then `timerHandler()` resets `triggerDump=0` after completing the first task dump after startup).

#### Debug Initialisation
- `systemInit()` then calls `debugInit()`, which currently does nothing because `DEBUG_PRINT_ON_SEGGER_RTT` is not defined.

#### CRTP Initialisation
- `systemInit()` then calls `crtpInit()` (defined in `crtp.c`), which creates the `txQueue` queue and the `CRTP-TX` and `CRTP-RX` tasks (with `PRIORITY=2`) that run the `crtpTxTask()` and `crtpRxTask()` functions respectively. 
- Note that `commInit()` in `comm.c` uses `crtpSetLink(wifilinkGetLink())` to point the CRTP link struct to the `wifilink.c` functions, enabling the `CRTP-TX` and `CRTP-RX` tasks to send and receive CRTP packets over UDP via the `wifilink.c` functions 
- `crtpTxTask()` is defined in `crtp.c` and essentially uses function pointers to run `wifilinkSendPacket()` (defined in `wifilink.c`), which makes sure the CRTP packet is not too long, blinks the blue LED, and then uses `wifiSendData()` (defined in `wifi_esp32.c`) to add the packet to the `udpDataTx` queue with `xQueueSend()`. This runs in a loop until `txQueue` is empty, after which the `CRTP-TX` task blocks until another packet is added to `txQueue`. 
- `crtpRxTask()` is defined in `crtp.` and essentially uses function pointers to run `wifilinkReceiveCRTPPacket()` (defined in `wifilink.c`), which receives a CRTP packet from the `crtpPacketDelivery` queue with `xQueueReceive()` (blocking for up to 100ms if the queue is empty) and blinks the green LED. The `crtpRxTask()` function then adds the CRTP packet to the queue specified within the CRTP packet (see below for list of `CRTPPort` options).
  ```
  typedef enum {
    CRTP_PORT_CONSOLE          = 0x00,
    CRTP_PORT_PARAM            = 0x02,
    CRTP_PORT_SETPOINT         = 0x03,
    CRTP_PORT_MEM              = 0x04,
    CRTP_PORT_LOG              = 0x05,
    CRTP_PORT_LOCALIZATION     = 0x06,
    CRTP_PORT_SETPOINT_GENERIC = 0x07,
    CRTP_PORT_SETPOINT_HL      = 0x08,
    CRTP_PORT_PLATFORM         = 0x0D,
    CRTP_PORT_LINK             = 0x0F,
  } CRTPPort;
  ```
- Note that `crtp.c` contains a function called `crtpInitTaskQueue()`, which can be called by other tasks to create a queue that is attached to one of the `CRTPPort` types, which will then be registered in the `queues[]` array that is private to `crtp.c`. Then if the `CRTP-RX` task receives a packet addressed to that port, it will be put into that task's queue. 

#### Console Initialisation
- `systemInit()` then calls `consoleInit()` (defined in `console.c`), which initialises an empty CRTP packet configured to be transmitted to a `CRTP_PORT_CONSOLE` port, creates a binary semaphore for sending one message at a time, and initialises the `messageSendingIsPending` flag as `false`. Effectively, this process allows the drone to send CRTP packets to the ground station's console.

#### Configuration Initialisation
- `systemInit()` then calls `configblockInit()`, which is made for reading a radio communication configuration from EEPROM over I2C, but since neither ESP-Drone or Dell's Angel has an external EEPROM, it will load the following `configblockDefault` configuration into RAM:
```
static configblock_t configblockDefault =
{
    .magic = MAGIC,  // 0x43427830
    .version = VERSION,  // 1
    .radioChannel = RADIO_CHANNEL,  // 80
    .radioSpeed = RADIO_DATARATE,  // 2
    .calibPitch = 0.0,
    .calibRoll = 0.0,
    .radioAddress_upper = ((uint64_t)RADIO_ADDRESS >> 32),  // 0xE7E7E7E7E7ULL
    .radioAddress_lower = (RADIO_ADDRESS & 0xFFFFFFFFULL),
};
``` 
- However, since the CrazyRadio is not being used, this is probably redundant.

#### Worker Initialisation
- `systemInit()` then calls `workerInit()` (defined in `worker.c`), which just creates the `workerQueue` FreeRTOS queue. 

#### ADC Initialisation
- `systemInit()` then calls `adcInit()` (defined in `adc_esp32.c`), which sets the ADC1 bit resolution to 12 bits (fixed default for ESP32-S3 anyway), sets the ADC1 attenuation factor to `attn=3` or 11 dB so that the ADC can measure pin voltages over the full 0-3.3V range, and then attempts to calibrate ADC1 with that particular attenuation factor to correct for ADC inaccuracies due to process variations.

#### LED Sequence Initialisation
- `systemInit()` then calls `ledseqInit()` (defined in `ledseq.c`), which redundantly calls `ledInit()` again, registers all of the predefined system LED sequences with a priority implied by their registration order, initialises an array of state flags as all `0` (one for each sequence), creates an array of one-shot FreeRTOS software timers (one for each sequence), creates the `ledseqMutex` mutex for changing LED sequences, creates the `ledseqCmdQueue` queue for processing LED sequences, and then finally creates the `LEDSEQCMD` task (with `PRIORITY=1`), which runs the `lesdeqCmdTask()` function.
- The `lesdeqCmdTask()` function is defined in `ledseq.c` and it runs in a loop, blocking until an LED sequence command is put in `ledseqCmdQueue` via `ledseqRun()` or `ledseqStop()`. When either a run or stop command is received from the queue, it immediately starts that sequence, which is then handled asynchronously by the previously initialised software timers. 

#### Power Management Initialisation
- `systemInit()` then calls `pmInit()` (defined in `pm_esplane.c`), which configures battery voltage measurement to occur with a 2x multiplier (to correct for the voltage divider used on the PCB), initialises both the min and max battery voltages as 3.7V, and creates the `PWRMGNT` task (with `PRIORITY=1`) that runs the `pmTask()` function. 
- The `pmTask()` function is defined in `pm_esplane.c` and it runs a 100 ms loop that samples VBAT, tracks how long VBAT has been below low/critical thresholds, decides charging/charged/normal/low-power using charger flags and those timers, then drives LEDs/sounds, toggles flight permission, and can auto-shutdown on sustained critical voltage or long inactivity (although the auto-shutdown has not been implemented).

#### Buzzer Initialisation
- Finally, `systemInit()` calls `buzzerInit()` (defined in `buzzer.c`), which calls `buzzDeckInit()` (defined in `buzzdeck.c`) since `CONFIG_BUZZER_ON=1` in `sdkconfig.h` for some reason (potentially redundant since no buzz deck is being used).
- `buzzDeckInit()` calls `piezzoInit()` (defined in `piezzo.c`), which configures a timer with the ESP32's LED Control peripheral to set the tone of the buzzer, configures a channel with the LED Control peripheral to be able to drive the buzzer pin with the timer, and exposes a control interface so that higher level code can play sounds or patterns.

####
- `systemTask()` then moves on from `systemInit()` to call `commInit()` (defined in `comm.c`), which...

## ESP32-Pi UART Communication 

### What data is being sent?

- The ROS 2 Orb SLAM application running on the ground station requires IMU data fusion to improve the accuracy of its Orb localisaiton.
- Hence, the IMU data (along with the timestamp for when it was created) needs to be transmitted to the Raspberry Pi Zero 2 W over UART so that it can be timestamped with the same timebase as the Pi Camera video feed and transmitted over wifi to the ground station.
- Since the ESP32 does several stages of IMU data processing, there are three options for what data can be transmitted to the Pi over UART:
  1. The raw IMU data straight from the MPU6050 after being read by the `SENSORS` task.
  1. The IMU data after it has been processed by the `SENSORS` task.
  1. The Kalman-filtered IMU data, or state estimation of the current accel/gyro values.

#### Raw data
- This data is read by `sensorsTask()` --> `i2cdevReadReg8()` in `sensors_mpu6050_hm5883L_ms5611.c`.
- It is stored in `buffer`, which is currently defined as a static global variable in `sensors_mpu6050_hm5883L_ms5611.c`, meaning it exists for the full program duration but is only visible to the functions defined in `sensors_mpu6050_hm5883L_ms5611.c`. 
- The structure of `buffer` is a `uint8_t` array of length 28, although the IMU only uses the first 14 elements, with the remaining 14 elements being reserved in case a magnetometer and barometer are being read over I2C as well.
- This raw data is not immediately useful to ROS since it hasn't been axes-corrected, scale-corrected, bias-corrected, or filtered.

#### Processed data
- This data is created by `sensorsTask()` --> `processAccGyroMeasurements()` in `sensors_mpu6050_hm5883L_ms5611.c`.
- It processes the data in `buffer` and then updates the data in `sensorData.acc` and `sensorData.gyro` with the new values. 
- `sensorData` is currently defined as a static global variable in `sensors_mpu6050_hm5883L_ms5611.c`, meaning it exists for the full program duration but is only visible to the functions defined in `sensors_mpu6050_hm5883L_ms5611.c`. 
- Then `sensorsTask()` uses the new `sensorData` values to overwrite the contents of two queues with handles `accelerometerDataQueue` and `gyroDataQueue` by calling `xQueueOverwrite()`. 
- Given these handles, any FreeRTOS task (such as a UART task) can receive or peek these data queues to obtain the latest processed IMU data.

!!! I think the Kalman task consumes these Queue values by receiving from them, so a UART task might not be able to peek these queue values. Might need to read directly from sensorData by requesting a pointer. !!!

#### Kalman-filtered data


#### Current plan



### IMU Data

- `stabilizerInit()` in `stabilizer.c` creates the `STABILIZER` task (with `PRIORITY=7`) which runs the `stabilizerTask()` function. 
- `stabilizerTask()` in `stabilizer.c` has a main loop, and the first function call in the loop is `sensorsWaitDataReady()`, which blocks the task until data is ready, causing the loop to run at a frequency of 1kHz or 500Hz if the kalman filter is being used.
- `sensorsWaitDataReady()` in `sensors.c` points to `sensorsMpu6050Hmc5883lMs5611WaitDataReady()` in `sensors_mpu6050_hm5883L_ms5611.c`, which takes the `dataReady` semaphore, blocking until it becomes available. `sensorsTask()` also in `sensors_mpu6050_hm5883L_ms5611.c` is what gives the `dataReady` semaphore after it has read and processed the IMU data.
- `sensorsTaskInit()` in `sensors_mpu6050_hm5883L_ms5611.c` creates the `SENSORS` task (with `PRIORITY=6`), which calls the `sensorsTask()` function.
- `sensorsTask()` in `sensors_mpu6050_hm5883L_ms5611.c` does the following:
  1. Takes the `sensorsDataReady` semaphore when it becomes available (given by `sensors_inta_isr_handler()` when the MPU6050 raises the INT pin or GPIO12).
  1. Saves the `imuIntTimestamp` timestamp to `sensorData.interruptTimestamp`, measured in microseconds since startup (`uint64_t` created by `sensors_inta_isr_handler()` when the MPU6050 raises the INT pin or GPIO12).
  1. Reads the MPU6050's data buffer into a sensor data buffer (and magnetometer data, if enabled) with `i2cdevReadReg8()`.
  1. Processes the data with `processAccGyroMeasurements()` in `sensors_mpu6050_hm5883L_ms5611.c` by correcting for the MPU6050's mounting direction as it reads the buffer data into the `accelRaw` and `gyroRaw` structs, compensating the gyro measurements for any bias that has been detected by `processGyroBias()`, scaling the accel and gyro raw data based on the selected user-programmable scales, compensating the accel measurements with the misalignment-correcting trim values set by the user, and then low-pass filtering the accel and gyro data to suppress noise.
  1. Value-copies the processed accel and gyro data to `accelerometerDataQueue` and `gyroDataQueue` using `xQueueOverwrite()`, so the latest values are always available in the queue.
  1. Gives the `dataReady` semaphore to unblock the `STABILIZER` task. 

- `compressState()` in `stabilizer.c` flattens the `state` struct of type `state_t` into `stateCompressed` and also converts units.

- Before being "compressed", `state` looks like this:
  
  ```
  {
    "state": {
      "position": {
        "x": "32-bit float measured in m",
        "y": "32-bit float measured in m",
        "z": "32-bit float measured in m"
      },
      "velocity": {
        "x": "32-bit float measured in m s^-1",
        "y": "32-bit float measured in m s^-1",
        "z": "32-bit float measured in m s^-1"
      },
      "acc": {
        "x": "32-bit float measured in m s^-2",
        "y": "32-bit float measured in m s^-2",
        "z": "32-bit float measured in m s^-2"
      },
      "attitudeQuaternion": {
        "x": "32-bit float component of a normalised quaternion",
        "y": "32-bit float component of a normalised quaternion",
        "z": "32-bit float component of a normalised quaternion",
        "w": "32-bit float component of a normalised quaternion"
      },
      "gyro": {
        "x": "32-bit float measured in deg s^-2",
        "y": "32-bit float measured in deg s^-2",
        "z": "32-bit float measured in deg s^-2"
      }
    }
  }
  ```
- After being "compressed", `stateCompressed` looks like this: 
  ```
  {
    "stateCompressed": {
      "x": "int16_t measured in mm",
      "y": "int16_t measured in mm",
      "z": "int16_t measured in mm",
      "vx": "int16_t measured in mm s^-1", 
      "vy": "int16_t measured in mm s^-1",
      "vz": "int16_t measured in mm s^-1",
      "ax": "int16_t measured in mm s^-2",
      "ay": "int16_t measured in mm s^-2",
      "az": "int16_t measured in mm s^-2",
      "quat": "int32_T compressed quaternion",
      "rateRoll": "int16_t measured in millirad s^-2",
      "ratePitch": "int16_t measured in millirad s^-2",
      "rateYaw": "int16_t measured in millirad s^-2"
    }
  }
  ```
- The `quatcompress()` function in `quatcompress.h` is used by `compressState()` to (lossily) compress the 128-bit quaternion to a single 32-bit value by:
  1. Identifying the quaternion component with the largest magnitude and forcing it to be positive by inverting the quaternion's components (since this saves the need to transmit or store its sign bit - the receiver knows it is always positive).
  1. Discarding the largest quaternion (since it can be reconstructed from the others using the normalised length condition).
  1. Quantising each of the remaining three components to a 9-bit magnitude plus a sign bit (10 + 10 + 10 = 30 bits).
  1. Indicating which of the four quaternion components was removed due to being the largest (either x/y/z/w - requires 2 bits).
  1. Packaging the data into a single `int32_t` value.  



### How Does the ESP32-S3 Handle UART?

- It has three dedicated UART controllers (UART0, UART1, UART2).
- When a FreeRTOS task wants to transmit over UART, it will:
  1. Format its data into a flat string of bytes (uint8_t).
  1. Load them into a 128-byte UART TX FIFO buffer.
- The UART hardware controller will then handle the following: 
  1. The UART hardware controller will format each byte into a message frame.
  1. The default message frame configuration is `8N1`, meaning 8 data bits, no parity bits, 1 stop bit (so 10 bits total, including the compulsory start bit). 
  1. The UART hardware controller has a baud rate generator, which sets the bit/second transmission rate.
  1. Each message frame is transferred from the FIFO buffer into a shift register by the UART hardware controller.
  1. The shift register then sends out each bit in the message at the baud rate. 
- The UART hardware controller can be configured to raise an interrupt once the FIFO buffer has been cleared to a certain count (so the MCU can top it up), as well as when the FIFO buffer has been cleared.
- Since the FIFO buffer is 128 bytes long, configuring UART to use DMA does not really provide much benefit unless more than 128 bytes are being sent at a time.