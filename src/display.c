/* Standard includes. */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

/* Kernel includes. */
#include <FreeRTOS.h> 
#include <portmacro.h>
#include <projdefs.h>
#include <semphr.h>
#include <task.h>

/* Display includes. */
#include <grlib.h>
#include <widget.h>
#include <canvas.h>

#include <Kentec320x240x16_ssd2119_spi.h>
#include <touch.h>

// Prototypes of Functions


/**
 * @defgroup display Display Subsystem
 * @{
 */

// Global Variables

/**
 * @defgroup InteruptHandlers Interupt Handlers for Display
 * @{
 */

/** @} */

/**
 * @defgroup tasks Tasks for the subsystem
 * Has the majority of the logic for the subsystem
 * @{
 */

/**
 * Creates the Tasks
 * This also has the required setup functions
 *
 * @todo
 * @return 0 for no errors or 1 for an error
 */
int CreateDisplayTask(void) {
    return 0;
}


/** @} */

/**
 * @defgroup HardwareSetup Hardware Setup Functions
 * @{
 */

/** @} */
/** @} */
