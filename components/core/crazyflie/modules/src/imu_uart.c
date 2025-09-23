#include <stdbool.h>
#include <string.h>

#include "config.h"
#include "wifilink.h"
#include "wifi_esp32.h"
#include "crtp.h"
#include "configblock.h"
#include "ledseq.h"
#include "pm_esplane.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "queuemonitor.h"
#include "semphr.h"
#include "stm32_legacy.h"

#define DEBUG_MODULE "WIFILINK"
#include "debug_cf.h"
#include "static_mem.h"

// Create initialisation flag to be set to true when imuuartInit() completes
static bool isInit = false;

// Preallocate static memory for the IMU_UART task
STATIC_MEM_TASK_ALLOC(imuuartTask, IMU_UART_TASK_STACKSIZE);

/**
 * @brief FreeRTOS task that packages IMU data and transmits over UART.
 *
 * The task runs indefinitely. It reads the latest IMU samples, builds a frame,
 * and writes to the UART driver. Designed to be created by imuuartInit().
 *
 * @param[in] param Optional context pointer (unused; passed NULL).
 */
static void imuuartTask(void *param)
{
    while(0){
        // TODO
    }
}

/**
 * @brief Initialize the IMU_UART module and start its FreeRTOS task.
 *
 * Safe to call multiple times; subsequent calls are no-ops once initialised.
 * Creates the task that will package IMU data and send it over UART.
 *
 * @note Must be called after the sensor subsystem is initialized since
 *       the task reads from the accelerometer/gyroscope queues.
 */
void imuuartInit(void)
{
    // Return if imuuartInit() has already been called
    if (isInit) {
        return;
    }

    // Create the IMU_UART task (with PRIORITY=2), which runs the imuuartTask() function
    STATIC_MEM_TASK_CREATE(imuuartTask, imuuartTask, IMU_UART_TASK_NAME,NULL, IMU_UART_TASK_PRI);

    // Set the isInit flag to true, since the UART initialisation is complete
    isInit = true;
}


/**
 * @brief Check if the IMU→UART module has initialised successfully.
 *
 * Safe to call multiple times; subsequent calls are no-ops once initialized.
 * Creates the task that will package IMU data and send it over UART.
 *
 */
bool imuuartTest(void)
{
    // Return the current value of the isInit flag
    return isInit;
}