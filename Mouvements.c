/*
 * Mouvements.c
 *
 *  Created on: 16 avr. 2020
 *      Author: Salma Chatagny
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <motors.h>
#include <Mouvements.h>
#include <chprintf.h>

#define NSTEP_ONE_TURN      1000 // number of step for 1 turn of the motor
#define WHEEL_PERIMETER     13.f // [cm]
#define MIN_DISPLACEMENT	(WHEEL_PERIMETER/NSTEP_ONE_TURN)
#define PI					3.141592f
#define FULL_TURN			360
#define DISTANCE_WHEELS		5.5f
#define CIRCLE_PERIMETER	(PI*DISTANCE_WHEELS)
#define NSTEP_ONE_CIRCLE	(CIRCLE_PERIMETER/MIN_DISPLACEMENT)

static ROBOT robot_situation;

//prototypes des fonctions

//Déclaration des fonctions
/***************************INTERNAL FUNCTIONS************************************/

/*************************END INTERNAL FUNCTIONS**********************************/


/****************************PUBLIC FUNCTIONS*************************************/
void turn(int angle)
{
	int nb_steps_to_do;
	nb_steps_to_do = (int) (angle*NSTEP_ONE_CIRCLE/FULL_TURN);
	chprintf((BaseSequentialStream *)&SD3, "nb_steps_to_do = %d \n", nb_steps_to_do);
	if (angle>=0)
	{
		right_motor_set_pos(0);
		right_motor_set_speed(LOW_SPEED);
		left_motor_set_speed(-LOW_SPEED);
		while(right_motor_get_pos() < nb_steps_to_do);
	}
	else
	{
		left_motor_set_pos(0);
		right_motor_set_speed(-LOW_SPEED);
		left_motor_set_speed(LOW_SPEED);
		while(left_motor_get_pos() < -nb_steps_to_do);
	}
//
//	while (right_motor_get_pos > 0 && left_motor_get_pos > 0);
//	{
//		__asm__ volatile ("nop");
//	}
}

void go_slow(void)
{
	right_motor_set_speed(LOW_SPEED);
	left_motor_set_speed(LOW_SPEED);
}

void go_fast(void)
{
	right_motor_set_speed(HIGH_SPEED);
	left_motor_set_speed(HIGH_SPEED);
}
