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
void I2C0IntHandler(void) {
    // UARTprintf("isr called\n");
    BaseType_t woken = pdFALSE;
    uint32_t status = I2CMasterIntStatusEx(I2C0_BASE, true);

    // signal on DATA or STOP only
    if (status & I2C_MASTER_INT_DATA) {
        xSemaphoreGiveFromISR(xI2CSemaphore, &woken);
    }
    if (status & I2C_MASTER_INT_STOP) {
        xSemaphoreGiveFromISR(xI2CSemaphore, &woken);
    }

    // clear whatever caused the interrupt
    I2CMasterIntClearEx(I2C0_BASE, status);
    portYIELD_FROM_ISR(woken);
}



bool writeI2C(uint8_t ui8Addr, uint8_t ui8Reg, uint8_t *data)
{   
    // UARTprintf("write called\n");

    xSemaphoreTake(xI2CSemaphore,portMAX_DELAY);
    // Load device slave address
    I2CMasterSlaveAddrSet(I2C0_BASE, ui8Addr, false);

    // Place the character to be sent in the data register
    I2CMasterDataPut(I2C0_BASE, ui8Reg);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    xSemaphoreTake(xI2CSemaphore,portMAX_DELAY);

    // Send Data
    I2CMasterDataPut(I2C0_BASE, data[0]);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);

    xSemaphoreTake(xI2CSemaphore,portMAX_DELAY);

    I2CMasterDataPut(I2C0_BASE, data[1]);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);

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
    // UARTprintf("read called\n");

    // Load device slave address and change I2C to write
    I2CMasterSlaveAddrSet(I2C0_BASE, ui8Addr, false);

    // Place the character to be sent in the data register
    I2CMasterDataPut(I2C0_BASE, ui8Reg);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_SEND);
    xSemaphoreTake(xI2CSemaphore,portMAX_DELAY);

    // Load device slave address and change I2C to read
    I2CMasterSlaveAddrSet(I2C0_BASE, ui8Addr, true);

    // Read two bytes from I2C
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
    xSemaphoreTake(xI2CSemaphore,portMAX_DELAY);
    data[0] = I2CMasterDataGet(I2C0_BASE);

    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
    xSemaphoreTake(xI2CSemaphore,portMAX_DELAY);
    data[1] = I2CMasterDataGet(I2C0_BASE);

    return true;
}




