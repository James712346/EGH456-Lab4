/*
 * led_task
 *
 * Copyright (C) 2022 Texas Instruments Incorporated
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* Hardware includes. */
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/i2c.h"
#include "utils/uartstdio.h"

#include "drivers/opt3001.h"

#define SYSTICKS_PER_SECOND     1
#define SYSTICK_PERIOD_MS       (1000 / SYSTICKS_PER_SECOND)

/* Button configuration */
#define BUTTON_GPIO_PERIPH      SYSCTL_PERIPH_GPIOF
#define BUTTON_GPIO_BASE        GPIO_PORTF_BASE
#define BUTTON_PIN              GPIO_PIN_4

volatile uint32_t g_ui32SysClock;
static volatile bool readingEnabled = false;

volatile bool sw1 = false;
volatile bool sw2 = false;

static void SensorInitTask(void *pvParameters);
static void ReadSensor(void *pvParameters);
void vCreateLEDTask(void);

//*****************************************************************************
// Configure the UART and its pins. This must be called before UARTprintf().
//*****************************************************************************
void ConfigureUART(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTStdioConfig(0, 9600, g_ui32SysClock);
    SysCtlDelay(g_ui32SysClock);
}

//*****************************************************************************
void vCreateLEDTask(void)
{
    g_ui32SysClock = SysCtlClockFreqSet(SYSCTL_XTAL_25MHZ |
                                        SYSCTL_OSC_MAIN   |
                                        SYSCTL_USE_PLL    |
                                        SYSCTL_CFG_VCO_480,
                                        120000000);

    ConfigureUART();
    UARTprintf("OPT001 Example\n");

    // Initialize I2C
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);
    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);
    I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), false);
    I2CMasterIntEnable(I2C0_BASE);
    I2CMasterIntEnableEx(I2C0_BASE,
        I2C_MASTER_INT_DATA |
        I2C_MASTER_INT_STOP);
    IntEnable(INT_I2C0);
    IntMasterEnable();

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOJ)) {}
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPION)) {}
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF)) {}

    GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    GPIOPadConfigSet(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1,
                     GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4);

    // Configure push-button on PF4
    SysCtlPeripheralEnable(BUTTON_GPIO_PERIPH);
    GPIOPinTypeGPIOInput(BUTTON_GPIO_BASE, BUTTON_PIN);
    GPIOPadConfigSet(BUTTON_GPIO_BASE, BUTTON_PIN,
                     GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);


    xTaskCreate(ReadSensor,     "Read", configMINIMAL_STACK_SIZE,
                NULL, tskIDLE_PRIORITY + 1, NULL);
}

//*****************************************************************************
static void SensorInitTask(void *pvParameters)
{
    
    vTaskDelete(NULL);
}

//*****************************************************************************
static void ReadSensor(void *pvParameters)
{
    bool success;
    uint16_t rawData;
    float convertedLux;
    uint8_t sw_state1;
    uint8_t sw_state2;
    bool on = false;
    sensorOpt3001Init();
    while (!sensorOpt3001Test()) {
        SysCtlDelay(g_ui32SysClock);
    }
    UARTprintf("All tests passed");

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(100));
        
        bool button0 = !(GPIOPinRead(GPIO_PORTJ_BASE, GPIO_PIN_0) & GPIO_PIN_0);
        bool button1 = !(GPIOPinRead(GPIO_PORTJ_BASE, GPIO_PIN_1) & GPIO_PIN_1);
    
        if (button0 || button1) 
        {
            if(on ==false)
            {
                on = true;
            }
            else{
                on = false;
            }

        }   
        if(on) {
            uint16_t raw;
            if (sensorOpt3001Read(&raw)) {
                float lux;
                sensorOpt3001Convert(raw, &lux);
                UARTprintf("Lux: %5d\n", (int)lux);
            }
            while (!(GPIOPinRead(GPIO_PORTJ_BASE, GPIO_PIN_0) & GPIO_PIN_0)
                    || !(GPIOPinRead(GPIO_PORTJ_BASE, GPIO_PIN_1) & GPIO_PIN_1)) {
                vTaskDelay(pdMS_TO_TICKS(10));
            }
        }
    }
}
