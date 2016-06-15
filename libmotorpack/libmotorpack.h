/*
 * libmotorpack.h
 *
 *  Created on: May 1, 2016
 *      Author: Dean Miller
 */

#ifndef LIBMOTORPACK_H_
#define LIBMOTORPACK_H_

#include "stepper.h"

#define MP_STEPPER_1	0x01
#define MP_STEPPER_2	0x02
#define MP_STEPPER_3	0x04

struct motor_pack{
	struct stepper	step1;
	struct stepper 	step2;
	struct stepper 	step3;
};

void motor_pack_init();

void motor_pack_init_stepper(uint32_t num);
void motor_pack_set_stepper_div(uint32_t num, uint32_t div);
void motor_pack_set_stepping_rate(uint32_t num, uint32_t stepping_rate);
void motor_pack_set_stepper_dir(uint32_t num, int dir);
void motor_pack_move_steps(uint32_t num, uint32_t steps);
void motor_pack_stepper_idle(uint32_t num);

void motor_pack_stepper_start(uint32_t num);
void motor_pack_stepper_stop(uint32_t num);

void motor_pack_enable_motors();
void motor_pack_disable_motors();

#endif /* LIBMOTORPACK_H_ */
