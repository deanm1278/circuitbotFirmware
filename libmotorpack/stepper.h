/*
 * stepper.h
 *
 *  Created on: May 1, 2016
 *      Author: Dean Miller
 */

#ifndef STEPPER_H_
#define STEPPER_H_

#include <stdint.h>
#include <stdbool.h>

#include "DAC082S085.h"

#define	MS_APH		0x20000
#define MS_BPH		0x10000
#define MS_AVREF	0x0FF00
#define MS_BVREF	0x000FF

struct stepper {
	uint32_t 			stepping_rate;	//steps per second
	uint32_t 			div;			//microsteps per step
	uint32_t			fires_this_move;
	uint32_t			t_val;
	int					dir;
	uint32_t			AEN_BASE;
	uint32_t			AEN_PIN;
	uint32_t			APH_BASE;
	uint32_t			APH_PIN;
	uint32_t			BPH_BASE;
	uint32_t			BPH_PIN;
	uint32_t			BEN_BASE;
	uint32_t			BEN_PIN;
	uint32_t			TIMER_BASE;
	uint32_t			TIMER;
	uint32_t			SS_BASE;
	uint32_t			SS_PIN;
	struct DAC082S085 	dac;
};


void stepper_init(struct stepper *s);
void stepper_disable(struct stepper *s);
void stepper_idle(struct stepper *s);
void stepper_enable(struct stepper *s);
void stepper_stop(struct stepper *s);
void stepper_start(struct stepper *s);
void stepper_set_stepping_rate(struct stepper *s, uint32_t stepping_rate);
void stepper_set_dir(struct stepper *s, int dir);
void stepper_move_steps(struct stepper *s, uint32_t steps);

#endif /* STEPPER_H_ */
