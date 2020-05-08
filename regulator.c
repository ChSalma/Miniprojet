#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>
#include <stdbool.h>

#include <main.h>
#include <Mouvements.h>
#include <regulator.h>
#include <constantes.h>

#define KP 1.1
#define KP_FW 0.9
#define KD 0.5
#define KD_FW 0.3
#define MAX_DIFF 30
#define MAX_DERIV 100
#define THRESHOLD_45_DEG 300//140
#define OFFSET 0 //car les capteurs gauche et droite ne donne pas exactement les mêmes valeurs pour une même distance à un obstacle

static float last_difference = 0;

void regulator_difference(int front_right_45deg_value, int front_left_45deg_value)
{
    int16_t right_speed, left_speed;
    int16_t difference=0;
    int16_t derivate=0;

	if ((front_right_45deg_value>THRESHOLD_45_DEG)&&(front_left_45deg_value>THRESHOLD_45_DEG))
	{
		difference = front_right_45deg_value-front_left_45deg_value-OFFSET;
		if(difference > MAX_DIFF)
			difference = MAX_DIFF;
		if(difference < -MAX_DIFF)
			difference = -MAX_DIFF;

		derivate = difference-last_difference;

		right_speed = get_right_speed()+ KP*difference + KD*derivate;
		left_speed = get_left_speed()-KP*difference-KD*derivate;

		set_speed(right_speed, left_speed);
		last_difference = difference;
	}
}

void regulator_follow_wall(int reference_value, int current_value, int sensor_id)
{
    int16_t right_speed, left_speed;
    int16_t difference=0,derivate;

    difference=current_value-reference_value;

	if (sensor_id==RIGHT_SENS)
		difference = -difference+OFFSET;
	if(difference > MAX_DIFF)
		difference = MAX_DIFF;
	if(difference < -MAX_DIFF)
		difference = -MAX_DIFF;
	derivate = difference-last_difference;
	right_speed = get_right_speed() + KP_FW*difference + KD_FW*derivate;
	left_speed = get_left_speed()-KP_FW*difference - KD_FW*derivate;
	set_speed(right_speed, left_speed);
	last_difference = difference;
}
