/*
 * libmotorpack.c
 *
 *  Created on: May 1, 2016
 *      Author: Dean Miller
 */
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"

#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"

#include "libmotorpack.h"
#include "stepper.h"
#include "microstep_values.h"

struct motor_pack mp;

//stepper interrupt handlers
void
TIMER2AIntHandler(void){
	//
	// Clear the timer interrupt.
	//
	TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);

	if(TimerLoadGet(TIMER2_BASE, TIMER_A) == 0){
		stepper_idle(&mp.step1);
	}
	else{
		uint32_t val = microstep_values[mp.step1.t_val];
		if(mp.step1.dir == 1){
			if(val & MS_APH)	GPIOPinWrite(mp.step1.APH_BASE, mp.step1.APH_PIN, mp.step1.APH_PIN);
			else	GPIOPinWrite(mp.step1.APH_BASE, mp.step1.APH_PIN, 0x00);

			if(val & MS_BPH)	GPIOPinWrite(mp.step1.BPH_BASE, mp.step1.BPH_PIN, mp.step1.BPH_PIN);
			else	GPIOPinWrite(mp.step1.BPH_BASE, mp.step1.BPH_PIN, 0x00);

			DAC082S085_set_value(&mp.step1.dac, DAC082S085_DAC0, (val & MS_AVREF) >> 8);
			DAC082S085_set_value(&mp.step1.dac, DAC082S085_DAC1, (val & MS_BVREF));
		}
		else if(mp.step1.dir == -1){
			if(val & MS_APH)	GPIOPinWrite(mp.step1.BPH_BASE, mp.step1.BPH_PIN, mp.step1.BPH_PIN);
			else	GPIOPinWrite(mp.step1.BPH_BASE, mp.step1.BPH_PIN, 0x00);

			if(val & MS_BPH)	GPIOPinWrite(mp.step1.APH_BASE, mp.step1.APH_PIN, mp.step1.APH_PIN);
			else	GPIOPinWrite(mp.step1.APH_BASE, mp.step1.APH_PIN, 0x00);

			DAC082S085_set_value(&mp.step1.dac, DAC082S085_DAC1, (val & MS_AVREF) >> 8);
			DAC082S085_set_value(&mp.step1.dac, DAC082S085_DAC0, (val & MS_BVREF));
		}

		mp.step1.t_val += 256 / (256 >> mp.step1.div);
		mp.step1.fires_this_move++;
		if(mp.step1.t_val >= 1023) mp.step1.t_val = 0;
	}
}

void
TIMER3AIntHandler(void){
	//
	// Clear the timer interrupt.
	//
	TimerIntClear(TIMER3_BASE, TIMER_TIMA_TIMEOUT);
	if(TimerLoadGet(TIMER3_BASE, TIMER_A) == 0){
			stepper_idle(&mp.step2);
	}
	else{
		uint32_t val = microstep_values[mp.step2.t_val];
		if(mp.step2.dir == 1){
			if(val & MS_APH)	GPIOPinWrite(mp.step2.APH_BASE, mp.step2.APH_PIN, mp.step2.APH_PIN);
			else	GPIOPinWrite(mp.step2.APH_BASE, mp.step2.APH_PIN, 0x00);

			if(val & MS_BPH)	GPIOPinWrite(mp.step2.BPH_BASE, mp.step2.BPH_PIN, mp.step2.BPH_PIN);
			else	GPIOPinWrite(mp.step2.BPH_BASE, mp.step2.BPH_PIN, 0x00);

			DAC082S085_set_value(&mp.step2.dac, DAC082S085_DAC0, (val & MS_AVREF) >> 8);
			DAC082S085_set_value(&mp.step2.dac, DAC082S085_DAC1, (val & MS_BVREF));
		}
		else if(mp.step2.dir == -1){
			if(val & MS_APH)	GPIOPinWrite(mp.step2.BPH_BASE, mp.step2.BPH_PIN, mp.step2.BPH_PIN);
			else	GPIOPinWrite(mp.step2.BPH_BASE, mp.step2.BPH_PIN, 0x00);

			if(val & MS_BPH)	GPIOPinWrite(mp.step2.APH_BASE, mp.step2.APH_PIN, mp.step2.APH_PIN);
			else	GPIOPinWrite(mp.step2.APH_BASE, mp.step2.APH_PIN, 0x00);

			DAC082S085_set_value(&mp.step2.dac, DAC082S085_DAC1, (val & MS_AVREF) >> 8);
			DAC082S085_set_value(&mp.step2.dac, DAC082S085_DAC0, (val & MS_BVREF));
		}

		mp.step2.t_val += 256 / (256 >> mp.step2.div);
		mp.step2.fires_this_move++;
		if(mp.step2.t_val >= 1023) mp.step2.t_val = 0;
	}
}

void
TIMER4AIntHandler(void){
	//
	// Clear the timer interrupt.
	//
	TimerIntClear(TIMER4_BASE, TIMER_TIMA_TIMEOUT);
	if(TimerLoadGet(TIMER4_BASE, TIMER_A) == 0){
		stepper_idle(&mp.step3);
	}
	else{
		uint32_t val = microstep_values[mp.step3.t_val];

		if(mp.step3.dir == 1){
			if(val & MS_APH)	GPIOPinWrite(mp.step3.APH_BASE, mp.step3.APH_PIN, mp.step3.APH_PIN);
			else	GPIOPinWrite(mp.step3.APH_BASE, mp.step3.APH_PIN, 0x00);

			if(val & MS_BPH)	GPIOPinWrite(mp.step3.BPH_BASE, mp.step3.BPH_PIN, mp.step3.BPH_PIN);
			else	GPIOPinWrite(mp.step3.BPH_BASE, mp.step3.BPH_PIN, 0x00);

			DAC082S085_set_value(&mp.step3.dac, DAC082S085_DAC0, (val & MS_AVREF) >> 8);
			DAC082S085_set_value(&mp.step3.dac, DAC082S085_DAC1, (val & MS_BVREF));
		}
		else if(mp.step3.dir == -1){
			if(val & MS_APH)	GPIOPinWrite(mp.step3.BPH_BASE, mp.step3.BPH_PIN, mp.step3.BPH_PIN);
			else	GPIOPinWrite(mp.step3.BPH_BASE, mp.step3.BPH_PIN, 0x00);

			if(val & MS_BPH)	GPIOPinWrite(mp.step3.APH_BASE, mp.step3.APH_PIN, mp.step3.APH_PIN);
			else	GPIOPinWrite(mp.step3.APH_BASE, mp.step3.APH_PIN, 0x00);

			DAC082S085_set_value(&mp.step3.dac, DAC082S085_DAC1, (val & MS_AVREF) >> 8);
			DAC082S085_set_value(&mp.step3.dac, DAC082S085_DAC0, (val & MS_BVREF));
		}

		mp.step3.t_val += 256 / (256 >> mp.step3.div);
		mp.step3.fires_this_move++;
		if(mp.step3.t_val >= 1023) mp.step3.t_val = 0;
	}
}

void motor_pack_init(){
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_4); //enable / disable for motors
}

void motor_pack_enable_motors(){
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_PIN_4);
}

void motor_pack_disable_motors(){
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, 0);
}

void motor_pack_init_stepper(uint32_t num){
	if(num & MP_STEPPER_1){
		// Enable the timer
		//
		SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
		TimerConfigure(TIMER2_BASE, TIMER_CFG_PERIODIC);

		IntEnable(INT_TIMER2A);
		TimerIntRegister(TIMER2_BASE, TIMER_A, TIMER2AIntHandler);
		TimerIntEnable(TIMER2_BASE, TIMER_TIMA_TIMEOUT);

		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

		mp.step1.APH_BASE 	= GPIO_PORTD_BASE;
		mp.step1.APH_PIN	= GPIO_PIN_0;
		mp.step1.AEN_BASE	= GPIO_PORTD_BASE;
		mp.step1.AEN_PIN	= GPIO_PIN_1;
		mp.step1.BPH_BASE	= GPIO_PORTD_BASE;
		mp.step1.BPH_PIN	= GPIO_PIN_2;
		mp.step1.BEN_BASE	= GPIO_PORTD_BASE;
		mp.step1.BEN_PIN	= GPIO_PIN_3;
		mp.step1.TIMER_BASE	= TIMER2_BASE;
		mp.step1.TIMER		= TIMER_A;
		mp.step1.SS_BASE	= GPIO_PORTD_BASE;
		mp.step1.SS_PIN		= GPIO_PIN_6;
		stepper_init(&mp.step1);
	}
	if(num & MP_STEPPER_2){
		// Enable the timer
		//
		SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER3);
		TimerConfigure(TIMER3_BASE, TIMER_CFG_PERIODIC);

		IntEnable(INT_TIMER3A);
		TimerIntRegister(TIMER3_BASE, TIMER_A, TIMER3AIntHandler);
		TimerIntEnable(TIMER3_BASE, TIMER_TIMA_TIMEOUT);

		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

		mp.step2.APH_BASE 	= GPIO_PORTC_BASE;
		mp.step2.APH_PIN	= GPIO_PIN_7;
		mp.step2.AEN_BASE	= GPIO_PORTC_BASE;
		mp.step2.AEN_PIN	= GPIO_PIN_6;
		mp.step2.BPH_BASE	= GPIO_PORTC_BASE;
		mp.step2.BPH_PIN	= GPIO_PIN_5;
		mp.step2.BEN_BASE	= GPIO_PORTC_BASE;
		mp.step2.BEN_PIN	= GPIO_PIN_4;
		mp.step2.TIMER_BASE	= TIMER3_BASE;
		mp.step2.TIMER		= TIMER_A;
		mp.step2.SS_BASE	= GPIO_PORTE_BASE;
		mp.step2.SS_PIN		= GPIO_PIN_4;
		stepper_init(&mp.step2);
	}
	if(num & MP_STEPPER_3){
		// Enable the timer
		//
		SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER4);
		TimerConfigure(TIMER4_BASE, TIMER_CFG_PERIODIC);

		IntEnable(INT_TIMER4A);
		TimerIntRegister(TIMER4_BASE, TIMER_A, TIMER4AIntHandler);
		TimerIntEnable(TIMER4_BASE, TIMER_TIMA_TIMEOUT);

		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
		//Unlock GPIOD7 - Like PF0 its used for NMI - Without this step it doesn't work
		HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY; //In Tiva include this is the same as "_DD" in older versions (0x4C4F434B)
		HWREG(GPIO_PORTD_BASE + GPIO_O_CR) |= 0x80;
		HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = 0;

		mp.step3.APH_BASE 	= GPIO_PORTE_BASE;
		mp.step3.APH_PIN	= GPIO_PIN_2;
		mp.step3.AEN_BASE	= GPIO_PORTE_BASE;
		mp.step3.AEN_PIN	= GPIO_PIN_1;
		mp.step3.BPH_BASE	= GPIO_PORTE_BASE;
		mp.step3.BPH_PIN	= GPIO_PIN_0;
		mp.step3.BEN_BASE	= GPIO_PORTD_BASE;
		mp.step3.BEN_PIN	= GPIO_PIN_7;
		mp.step3.TIMER_BASE	= TIMER4_BASE;
		mp.step3.TIMER		= TIMER_A;
		mp.step3.SS_BASE	= GPIO_PORTE_BASE;
		mp.step3.SS_PIN		= GPIO_PIN_5;
		stepper_init(&mp.step3);

	}
}

void motor_pack_set_stepper_div(uint32_t num, uint32_t div){
	if(num & MP_STEPPER_1){
		mp.step1.div = div;
	}
	if(num & MP_STEPPER_2){
		mp.step2.div = div;
	}
	if(num & MP_STEPPER_3){
		mp.step2.div = div;
	}
}

void motor_pack_set_stepping_rate(uint32_t num, uint32_t stepping_rate){
	if(num & MP_STEPPER_1){
		stepper_set_stepping_rate(&mp.step1, stepping_rate);
	}
	if(num & MP_STEPPER_2){
		stepper_set_stepping_rate(&mp.step2, stepping_rate);
	}
	if(num & MP_STEPPER_3){
		stepper_set_stepping_rate(&mp.step3, stepping_rate);
	}
}

void motor_pack_move_steps(uint32_t num, uint32_t steps){
	if(num & MP_STEPPER_1){
		stepper_move_steps(&mp.step1, steps);
	}
	else if(num & MP_STEPPER_2){
		stepper_move_steps(&mp.step2, steps);
	}
	else if(num & MP_STEPPER_3){
		stepper_move_steps(&mp.step3, steps);
	}
}

void motor_pack_set_stepper_dir(uint32_t num, int dir){
	if(num & MP_STEPPER_1){
		stepper_set_dir(&mp.step1, dir);
	}
	if(num & MP_STEPPER_2){
		stepper_set_dir(&mp.step2, dir);
	}
	if(num & MP_STEPPER_3){
		stepper_set_dir(&mp.step3, dir);
	}
}

void motor_pack_stepper_start(uint32_t num){
	if(num & MP_STEPPER_1){
		stepper_start(&mp.step1);
	}
	if(num & MP_STEPPER_2){
		stepper_start(&mp.step2);
	}
	if(num & MP_STEPPER_3){
		stepper_start(&mp.step3);
	}
}

void motor_pack_stepper_stop(uint32_t num){
	if(num & MP_STEPPER_1){
		stepper_stop(&mp.step1);
	}
	if(num & MP_STEPPER_2){
		stepper_stop(&mp.step2);
	}
	if(num & MP_STEPPER_3){
		stepper_stop(&mp.step3);
	}
}

void motor_pack_stepper_idle(uint32_t num){
	if(num & MP_STEPPER_1){
		stepper_idle(&mp.step1);
	}
	if(num & MP_STEPPER_2){
		stepper_idle(&mp.step2);
	}
	if(num & MP_STEPPER_3){
		stepper_idle(&mp.step3);
	}
}
