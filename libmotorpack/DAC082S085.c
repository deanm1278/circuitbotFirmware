/*
 * DAC082S085.c
 *
 *  Created on: May 1, 2016
 *      Author: Dean Miller
 */
#include <stdint.h>
#include <stdbool.h>

#include "inc/hw_memmap.h"

#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/ssi.h"

#include "DAC082S085.h"

void DAC082S085_init(struct DAC082S085 *dac, uint32_t SSI_BASE, uint32_t SS_BASE, uint32_t SS_PIN){
	//todo: add compatibility with other SSI BASE values

	dac->SSI_BASE 	= SSI_BASE;
	dac->SS_BASE	= SS_BASE;
	dac->SS_PIN		= SS_PIN;

	GPIOPinTypeGPIOOutput(dac->SS_BASE, dac->SS_PIN);
	GPIOPadConfigSet(dac->SS_BASE, dac->SS_PIN, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD_WPU);
	GPIOPinWrite(dac->SS_BASE, dac->SS_PIN, dac->SS_PIN);

	if(SSI_BASE == SSI2_BASE){
		// // Enable the SSI2 peripheral //
		SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI2);
		// // Wait for the SSI2 module to be ready. //
		while(!SysCtlPeripheralReady(SYSCTL_PERIPH_SSI2)) { }

		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

		GPIOPinConfigure(GPIO_PB4_SSI2CLK);
		//GPIOPinConfigure(GPIO_PB5_SSI2FSS);
		GPIOPinConfigure(GPIO_PB7_SSI2TX);

		GPIOPinTypeSSI(GPIO_PORTB_BASE,GPIO_PIN_4|GPIO_PIN_7);

		GPIOPadConfigSet(GPIO_PORTB_BASE, GPIO_PIN_4|GPIO_PIN_7, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD_WPU);

		// // Configure the SSI. //
		SSIConfigSetExpClk(SSI2_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, DAC_SCLK_SPEED, DAC_BITS_PER_TXN);
		// // Enable the SSI module. //
		SSIEnable(SSI2_BASE);
	}
}

void DAC082S085_set_value(struct DAC082S085 *dac, uint8_t DAC_CHANNEL, uint8_t val){
	if(DAC_CHANNEL == DAC082S085_DAC0 || DAC_CHANNEL == DAC082S085_DAC1){

		uint16_t txbuf = ((DAC_CHANNEL << 12) | ((val >> 1) << 4)) & (0xFFFF);
		GPIOPinWrite(dac->SS_BASE, dac->SS_PIN, 0x00);
		SSIDataPut(dac->SSI_BASE, txbuf);
		while(SSIBusy(dac->SSI_BASE));
		GPIOPinWrite(dac->SS_BASE, dac->SS_PIN, dac->SS_PIN);
		if(DAC_CHANNEL == DAC082S085_DAC0) dac->AVAL = val;
		else dac->BVAL = val;
	}
}

uint8_t DAC082S085_get_value(struct DAC082S085 *dac, uint8_t DAC_CHANNEL){
	if(DAC_CHANNEL == DAC082S085_DAC0) return dac->AVAL;
	else if(DAC_CHANNEL == DAC082S085_DAC0) return dac->BVAL;
	else return 0;
}

