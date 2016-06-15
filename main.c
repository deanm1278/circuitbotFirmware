/*
 * main.c
 */
#include <stdint.h>
#include <stdbool.h>

#include "inc/hw_memmap.h"
#include "inc/hw_ssi.h"
#include "inc/hw_ints.h"

#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/ssi.h"

#include "libmotorpack/libmotorpack.h"

#define OP_BEGIN_TRANSMISSION 0x01
#define INTERP_PERIOD 		1000
#define NUM_AXIS			3
#define DATARATE			4000000

#define STEPS_PER_REV 		200
#define SEC_PER_MIN 		60

#define X_AXIS				0
#define Y_AXIS				1
#define Z_AXIS				2

#define RDY_BASE			GPIO_PORTB_BASE
#define RDY_PIN				GPIO_PIN_3

uint32_t receiveData[] = {0, 0, 0};
int 	 receiveSigns[] = {0, 0, 0};
volatile uint32_t dataReady = 0;
volatile uint32_t running = 0;

extern struct motor_pack mp;

void ConfigureTimer(void){

	// Enable the timer peripheral
	//
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);

	//
	// Configure the 32-bit periodic timer.
	//
	TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
	TimerLoadSet(TIMER0_BASE, TIMER_A, SysCtlClockGet()/INTERP_PERIOD);

	//
	// Setup the interrupts for the timer timeout.
	//
	IntEnable(INT_TIMER0A);
	TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

	//
	// Enable the timers.
	//
	TimerEnable(TIMER0_BASE, TIMER_A);
}

void SSI0IntHandler(void){
	if(running){
		SSIIntClear(SSI0_BASE, SSI_RXFF);
		GPIOPinWrite(RDY_BASE, RDY_PIN, 0x00);

		int i = 0;
		for(i = 0; i < NUM_AXIS; i++){
			//wait until a value is in the RX FIFO if there isn't one.
			uint32_t newVal;
			SSIDataGet(SSI0_BASE, &newVal);

			//get sign bit
			uint8_t sign = (newVal >> 15) & 1;
			//clear sign bit
			newVal &= ~(1 << 15);

			receiveSigns[i] = (sign ? 1 : -1);
			receiveData[i] = newVal; //num steps to speed
		}
		dataReady = 1;
	}
	else{
		//check for the begin command. if received start listening for data
		uint32_t newVal;
		while(SSIDataGetNonBlocking(SSI0_BASE, &newVal)){
			if(newVal == OP_BEGIN_TRANSMISSION){
				GPIOPinWrite(RDY_BASE, RDY_PIN, RDY_PIN);

				//get the motors going
				motor_pack_enable_motors();


				running = true;
			}
		}

	}
}

void ConfigureSPI(void){
	SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

	GPIOPinConfigure(GPIO_PA2_SSI0CLK);
	GPIOPinConfigure(GPIO_PA3_SSI0FSS);
	GPIOPinConfigure(GPIO_PA4_SSI0RX);
	GPIOPinTypeSSI(GPIO_PORTA_BASE,GPIO_PIN_4|GPIO_PIN_3|GPIO_PIN_2);

	SSIConfigSetExpClk(SSI0_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0,SSI_MODE_SLAVE,DATARATE,16);

	SSIIntRegister(SSI0_BASE, SSI0IntHandler);
	SSIIntEnable(SSI0_BASE, SSI_RXFF);

	SSIEnable(SSI0_BASE);
}

void
Timer0IntHandler(void){
	//
	// Clear the timer interrupt.
	//
	TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
	if(dataReady){
		motor_pack_set_stepper_dir(MP_STEPPER_1, receiveSigns[X_AXIS]);
		motor_pack_set_stepper_dir(MP_STEPPER_2, receiveSigns[Y_AXIS]);
		motor_pack_set_stepper_dir(MP_STEPPER_3, receiveSigns[Z_AXIS]);

		if (receiveData[X_AXIS] == 0)
			motor_pack_stepper_idle(MP_STEPPER_1);
		else
			motor_pack_set_stepping_rate(MP_STEPPER_1, receiveData[X_AXIS] * INTERP_PERIOD);
		if (receiveData[Y_AXIS] == 0)
			motor_pack_stepper_idle(MP_STEPPER_2);
		else
			motor_pack_set_stepping_rate(MP_STEPPER_2, receiveData[Y_AXIS] * INTERP_PERIOD);
		if (receiveData[Z_AXIS] == 0)
			motor_pack_stepper_idle(MP_STEPPER_3);
		else
			motor_pack_set_stepping_rate(MP_STEPPER_3, receiveData[Z_AXIS] * INTERP_PERIOD);

		GPIOPinWrite(RDY_BASE, RDY_PIN, RDY_PIN);
		dataReady = 0;
	}
	else{
		motor_pack_stepper_idle(MP_STEPPER_1 | MP_STEPPER_2 | MP_STEPPER_3);
	}
}

int main(void) {
	//
	// Enable lazy stacking for interrupt handlers.  This allows floating-point
	// instructions to be used within interrupt handlers, but at the expense of
	// extra stack usage.
	//
	FPULazyStackingEnable();
	FPUEnable();

	//
	// Set the clocking to run directly from the crystal.
	// Clock to 80MHZ
	//
	SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |
					   SYSCTL_OSC_MAIN);

	IntMasterEnable();
	
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	GPIOPinTypeGPIOOutput(RDY_BASE, RDY_PIN);

	//assert ready
	GPIOPinWrite(RDY_BASE, RDY_PIN, 0x00);

	//initialize the motor pack
	motor_pack_init();

	//initialize the steppers
	motor_pack_init_stepper(MP_STEPPER_1 | MP_STEPPER_2 | MP_STEPPER_3);

	//start position update timer
	ConfigureTimer();

	motor_pack_stepper_start(MP_STEPPER_1 | MP_STEPPER_2 | MP_STEPPER_3);

	//delay for stability
	SysCtlDelay(1000000);

	//setup communication with master and let it know we're ready!
	ConfigureSPI();

	while(1);

	return 0;
}
