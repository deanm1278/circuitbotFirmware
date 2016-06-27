/*
 * stepper.c
 *
 *  Created on: May 1, 2016
 *      Author: Dean Miller
 */

#include <stdint.h>
#include <stdbool.h>

#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"

#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"

#include "stepper.h"
#include "DAC082S085.h"

void stepper_init(struct stepper *s){
	DAC082S085_init(&s->dac, SSI2_BASE, s->SS_BASE, s->SS_PIN);
	s->stepping_rate = 0;
	s->div = 2;

	//set up other necessary GPIO
	GPIOPinTypeGPIOOutput(s->AEN_BASE, s->AEN_PIN);
	GPIOPinTypeGPIOOutput(s->APH_BASE, s->APH_PIN);
	GPIOPinTypeGPIOOutput(s->BEN_BASE, s->BEN_PIN);
	GPIOPinTypeGPIOOutput(s->BPH_BASE, s->BPH_PIN);
}

void stepper_idle(struct stepper *s){
	if(s->stepping_rate > 0){
		stepper_set_stepping_rate(s, 0);
	}
}

void stepper_disable(struct stepper *s){
	GPIOPinWrite(s->AEN_BASE, s->AEN_PIN, 0);
	GPIOPinWrite(s->BEN_BASE, s->BEN_PIN, 0);
}

void stepper_enable(struct stepper *s){
	GPIOPinWrite(s->AEN_BASE, s->AEN_PIN, s->AEN_PIN);
	GPIOPinWrite(s->BEN_BASE, s->BEN_PIN, s->BEN_PIN);
}

void stepper_stop(struct stepper *s){
	TimerDisable(s->TIMER_BASE, s->TIMER);
	stepper_disable(s);
	s->t_val = 0;
}

void stepper_start(struct stepper *s){
	s->fires_this_move = 0;
	//
	// Enable the timer.
	//
	TimerEnable(s->TIMER_BASE, s->TIMER);
	stepper_enable(s);
}

void stepper_set_dir(struct stepper *s, int dir){
	s->dir = dir;
}

void stepper_set_stepping_rate(struct stepper *s, uint32_t stepping_rate){
	TimerLoadSet(s->TIMER_BASE, s->TIMER, SysCtlClockGet()/(stepping_rate >> s->div));
	s->stepping_rate = stepping_rate >> s->div;
}

void stepper_move_steps(struct stepper *s, uint32_t steps){
	stepper_start(s);
	while(s->fires_this_move < steps);
	stepper_stop(s);
}
