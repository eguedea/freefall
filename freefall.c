/*
 * freefall.c
 *
 *  Created on: Feb 11, 2018
 *      Author: eric_
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK64F12.h"
#include "fsl_debug_console.h"
#include "fsl_port.h"
#include "fsl_i2c.h"
#include "freefall.h"
#include "math.h"
#include "fsl_pit.h"

volatile bool g_MasterCompletionFlag = false;
volatile static int i = 0;
uint8_t buffer[6];

static void i2c_release_bus_delay(void)
{
    uint32_t i = 0;
    for (i = 0; i < 100; i++)
    {
        __NOP();
    }
}

void i2c_ReleaseBus()
{
	uint8_t i = 0;
	gpio_pin_config_t pin_config;
	port_pin_config_t i2c_pin_config =
	{ 0 };

	/* Config pin mux as gpio */
	i2c_pin_config.pullSelect = kPORT_PullUp;
	i2c_pin_config.mux = kPORT_MuxAsGpio;

	pin_config.pinDirection = kGPIO_DigitalOutput;
	pin_config.outputLogic = 1U;
	CLOCK_EnableClock(kCLOCK_PortE);
	PORT_SetPinConfig(PORTE, 24, &i2c_pin_config);
	PORT_SetPinConfig(PORTE, 25, &i2c_pin_config);

	GPIO_PinInit(GPIOE, 24, &pin_config);
	GPIO_PinInit(GPIOE, 25, &pin_config);

	GPIO_PinWrite(GPIOE, 25, 0U);
	i2c_release_bus_delay();

	for (i = 0; i < 9; i++)
	{
		GPIO_PinWrite(GPIOE, 24, 0U);
		i2c_release_bus_delay();

		GPIO_PinWrite(GPIOE, 25, 1U);
		i2c_release_bus_delay();

		GPIO_PinWrite(GPIOE, 24, 1U);
		i2c_release_bus_delay();
		i2c_release_bus_delay();
	}

	GPIO_PinWrite(GPIOE, 24, 0U);
	i2c_release_bus_delay();

	GPIO_PinWrite(GPIOE, 25, 0U);
	i2c_release_bus_delay();

	GPIO_PinWrite(GPIOE, 24, 1U);
	i2c_release_bus_delay();

	GPIO_PinWrite(GPIOE, 25, 1U);
	i2c_release_bus_delay();
}

static void i2c_master_callback(I2C_Type *base, i2c_master_handle_t *handle,
        status_t status, void * userData)
{

	if (status == kStatus_Success)
	{
		g_MasterCompletionFlag = true;
	}
}
void startI2C()
{

	CLOCK_EnableClock(kCLOCK_PortE);
	CLOCK_EnableClock(kCLOCK_I2c0);

	port_pin_config_t config_i2c =
	{ kPORT_PullDisable, kPORT_SlowSlewRate, kPORT_PassiveFilterDisable,
	        kPORT_OpenDrainDisable, kPORT_LowDriveStrength, kPORT_MuxAlt5,
	        kPORT_UnlockRegister, };

	PORT_SetPinConfig(PORTE, 24, &config_i2c);
	PORT_SetPinConfig(PORTE, 25, &config_i2c);

	i2c_master_config_t masterConfig;
	I2C_MasterGetDefaultConfig(&masterConfig);
	masterConfig.baudRate_Bps = 100000;
	I2C_MasterInit(I2C0, &masterConfig, CLOCK_GetFreq(kCLOCK_BusClk));

	i2c_master_handle_t g_m_handle;
	I2C_MasterTransferCreateHandle(I2C0, &g_m_handle, i2c_master_callback,
	NULL);

	i2c_master_transfer_t masterXfer;

	uint8_t data_buffer = 0x01;

#if 0
	masterXfer.slaveAddress = 0x1D;
	masterXfer.direction = kI2C_Write;
	masterXfer.subaddress = 0;
	masterXfer.subaddressSize = 0;
	masterXfer.data = &data_buffer;
	masterXfer.dataSize = 1;
	masterXfer.flags = kI2C_TransferNoStopFlag;

	I2C_MasterTransferNonBlocking(I2C0, &g_m_handle,
			&masterXfer);
	while (!g_MasterCompletionFlag)
	{}
	g_MasterCompletionFlag = false;

	uint8_t read_data;

	masterXfer.slaveAddress = 0x1D;
	masterXfer.direction = kI2C_Read;
	masterXfer.subaddress = 0;
	masterXfer.subaddressSize = 0;
	masterXfer.data = &read_data;
	masterXfer.dataSize = 1;
	masterXfer.flags = kI2C_TransferRepeatedStartFlag;

	I2C_MasterTransferNonBlocking(I2C0, &g_m_handle,
			&masterXfer);
	while (!g_MasterCompletionFlag)
	{}
	g_MasterCompletionFlag = false;
#else

	masterXfer.slaveAddress = 0x1D;
	masterXfer.direction = kI2C_Write;
	masterXfer.subaddress = 0x2A;
	masterXfer.subaddressSize = 1;
	masterXfer.data = &data_buffer;
	masterXfer.dataSize = 1;
	masterXfer.flags = kI2C_TransferDefaultFlag;

	I2C_MasterTransferNonBlocking(I2C0, &g_m_handle, &masterXfer);
	while (!g_MasterCompletionFlag)
	{
	}
	g_MasterCompletionFlag = false;

#endif

}
void IMUcapture(accelerometer_t *rawdata)
{
	i2c_master_transfer_t masterXfer;
	i2c_master_handle_t g_m_handle;

	masterXfer.slaveAddress = 0x1D;
	masterXfer.direction = kI2C_Read;
	masterXfer.subaddress = 0x01;
	masterXfer.subaddressSize = 1;
	masterXfer.data = buffer;
	masterXfer.dataSize = 6;
	masterXfer.flags = kI2C_TransferDefaultFlag;

	I2C_MasterTransferNonBlocking(I2C0, &g_m_handle, &masterXfer);
	while (!g_MasterCompletionFlag)
	{
	}
	g_MasterCompletionFlag = false;

	rawdata->x = buffer[0] << 8 | buffer[1];
	rawdata->y = buffer[2] << 8 | buffer[3];
	rawdata->z = buffer[4] << 8 | buffer[5];

}
void ledRoutine()
{
	if(1 == PIT_GetStatusFlags(PIT, kPIT_Chnl_0)){
		GPIO_TogglePinsOutput(GPIOB,22);
	}
}
void sequenceEnabler()
{
	//Led Configuration
	CLOCK_EnableClock(kCLOCK_PortB);
	port_pin_config_t config_led =
		{ kPORT_PullDisable, kPORT_SlowSlewRate, kPORT_PassiveFilterDisable,
				kPORT_OpenDrainDisable, kPORT_LowDriveStrength, kPORT_MuxAsGpio,
				kPORT_UnlockRegister,
		};

	PORT_SetPinConfig(PORTB, 22, &config_led);
	gpio_pin_config_t led_config_gpio =
		{ kGPIO_DigitalOutput, 1 };
	GPIO_PinInit(GPIOB, 22, &led_config_gpio);

	//Pit Configuration
	pit_config_t config_pit0;
	PIT_GetDefaultConfig(&config_pit0);
	PIT_Init (PIT, &config_pit0);
	PIT_EnableInterrupts (PIT, kPIT_Chnl_0, kPIT_TimerInterruptEnable);
	PIT_GetEnabledInterrupts(PIT, kPIT_Chnl_0);
	PIT_SetTimerPeriod (PIT, kPIT_Chnl_0, 7500000);



}
void freefallCalculation(int16_t axis_x, int16_t axis_y, int16_t axis_z)
{
	int16_t acceleration = sqrt((axis_x*axis_x)+(axis_y*axis_y)+(axis_z*axis_z));

	if((acceleration<200)&&(acceleration>=0))
	{
		ledRoutine();
	}
}


