/**************************************************************************************************
*  Filename:       i2cOptDriver.c
*  By:             Jesse Haviland
*  Created:        1 February 2019
*  Revised:        23 March 2019
*  Revision:       2.0
*
*  Description:    i2c Driver for use with opt3001.c and the TI OP3001 Optical Sensor
*************************************************************************************************/

// ----------------------- Includes -----------------------
#include "i2cOptDriver.h"
#include "inc/hw_memmap.h"
#include "driverlib/i2c.h"
#include "utils/uartstdio.h"
#include "driverlib/sysctl.h"
#include "FreeRTOS.h"
#include "semphr.h"

extern SemaphoreHandle_t xI2CSemaphore;
extern SemaphoreHandle_t xI2CBusSemaphore;

// Interrupt for I2C 2
void I2C2IntHandler(void){
    BaseType_t TaskhasWoken = pdFALSE;
    UARTprintf("Interrupt Called:");
    I2CMasterIntClear(I2C2_BASE);
    if (!I2CMasterBusy(I2C2_BASE)){
        UARTprintf(" Master Busy");
        xSemaphoreGiveFromISR(xI2CSemaphore, &TaskhasWoken);
    }
    if (!I2CMasterBusBusy(I2C2_BASE)){
        UARTprintf(" Bus Busy");
        xSemaphoreGiveFromISR(xI2CBusSemaphore, &TaskhasWoken);
    }
    UARTprintf("\n");
    portYIELD_FROM_ISR(TaskhasWoken);
    
}

/*
 * Sets slave address to ui8Addr
 * Puts ui8Reg followed by two data bytes in *data and transfers
 * over i2c
 */
bool writeI2C(uint8_t ui8Addr, uint8_t ui8Reg, uint8_t *data)
{
    
    // Load device slave address
    I2CMasterSlaveAddrSet(I2C2_BASE, ui8Addr, false);
    UARTprintf("the slave has been set\n");
    // Place the character to be sent in the data register
    I2CMasterDataPut(I2C2_BASE, ui8Reg);
    UARTprintf("the master has been set\n");
    I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    UARTprintf("the control has been set\n");
    if (!I2CMasterBusy(I2C2_BASE)){
        xSemaphoreTake(xI2CSemaphore, portMAX_DELAY);
    }

    // Send Data
    I2CMasterDataPut(I2C2_BASE, data[0]);
    I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
    if (!I2CMasterBusy(I2C2_BASE)){
        xSemaphoreTake(xI2CSemaphore, portMAX_DELAY);
    }
    I2CMasterDataPut(I2C2_BASE, data[1]);
    I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
    if (!I2CMasterBusBusy(I2C2_BASE)){
        xSemaphoreTake(xI2CBusSemaphore, portMAX_DELAY);
    }
    return true;
}



/*
 * Sets slave address to ui8Addr
 * Writes ui8Reg over i2c to specify register being read from
 * Reads three bytes from i2c slave. The third is redundant but
 * helps to flush the i2c register
 * Stores first two received bytes into *data
 */
bool readI2C(uint8_t ui8Addr, uint8_t ui8Reg, uint8_t *data)
{
    // Load device slave address and change I2C to write
    I2CMasterSlaveAddrSet(I2C2_BASE, ui8Addr, false);

    // Place the character to be sent in the data register
    I2CMasterDataPut(I2C2_BASE, ui8Reg);
    I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_SINGLE_SEND);
    if (!I2CMasterBusy(I2C2_BASE)){
        xSemaphoreTake(xI2CSemaphore, portMAX_DELAY);
    }
    // Load device slave address and change I2C to read
    I2CMasterSlaveAddrSet(I2C2_BASE, ui8Addr, true);

    // Read two bytes from I2C
    I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
    if (!I2CMasterBusy(I2C2_BASE)){
        xSemaphoreTake(xI2CSemaphore, portMAX_DELAY);
    }
    data[0] = I2CMasterDataGet(I2C2_BASE);

    I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
    if (!I2CMasterBusy(I2C2_BASE)){
        xSemaphoreTake(xI2CSemaphore, portMAX_DELAY);
    }
    data[1] = I2CMasterDataGet(I2C2_BASE);

    return true;
}


