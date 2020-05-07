/*
 * process_sensor.c
 *
 *  Created on: 4 avr. 2020
 *      Author: Salma Chatagny
 */

#include "ch.h"
#include "hal.h"
#include <stdbool.h>
#include <sensors/proximity.h>
#include <Mouvements.h>
#include <process_sensor.h>
#include <constantes.h>
#include <maze_mapping.h>
#include <chprintf.h>
#include <regulator.h>

#include <main.h>

#define ROBOT_DIAMETER 75.f
#define COEFF 0.4f
#define ANTICIPATION_OFFSET 300

#define BUFFER_SIZE 3

enum{FREE_WAY_DETECTED, WALL_DETECTED};

static bool leaving_corridor;
static bool sensors_values[PROXIMITY_NB_CHANNELS];
static int reference_distance;
static int data_sensors[PROXIMITY_NB_CHANNELS*BUFFER_SIZE];
static uint8_t buffer_state;

static THD_WORKING_AREA(waProcessMeasure,256);
/*INTERNAL FUNCTIONS*/

void do_follow_wall_regulation(int sensor_id)
{
	if (leaving_corridor)
		reference_distance=1.1*data_sensors[sensor_id + buffer_state*PROXIMITY_NB_CHANNELS];

	regulator_follow_wall(reference_distance, data_sensors[sensor_id + buffer_state*PROXIMITY_NB_CHANNELS], sensor_id);
}
void process_sensors_values(void)
{
	uint8_t i, j;
	for(i = 0; i < PROXIMITY_NB_CHANNELS; i++)
	{
		int calibrated_prox = 0;
		data_sensors[i + buffer_state*PROXIMITY_NB_CHANNELS] = get_calibrated_prox(i);
		for(j = 0; j < BUFFER_SIZE; j++)
		{
			calibrated_prox += data_sensors[i + j*PROXIMITY_NB_CHANNELS];
		}
		calibrated_prox = (int)(calibrated_prox/BUFFER_SIZE);
		//chprintf((BaseSequentialStream *) &SD3, "id = %d , calibrated = %d\n",i, calibrated_prox);
		if(i==FRONT_RIGHT || i==FRONT_LEFT)
		{
			//si on sait qu'on est de toute façon dans un virage ou carrefour on peut tester plus rapidemeent le capteur avant

			if(((sensors_values[RIGHT_SENS]==FREE_WAY_DETECTED||sensors_values[LEFT_SENS]==FREE_WAY_DETECTED))&&
				(calibrated_prox > (FREE_WAY_FRONT-ANTICIPATION_OFFSET)))
				sensors_values[i]= WALL_DETECTED;
			else if (calibrated_prox > FREE_WAY_FRONT)
				sensors_values[i]= WALL_DETECTED;
			else
			{
				sensors_values[i]= FREE_WAY_DETECTED;
			}
		}
		else if ((calibrated_prox < FREE_WAY_LEFT) || (calibrated_prox < FREE_WAY_RIGHT))
		{
			if((i == RIGHT_SENS) && (calibrated_prox > FREE_WAY_RIGHT))
				sensors_values[i]= WALL_DETECTED;
			else if((i == LEFT_SENS) && (calibrated_prox > FREE_WAY_LEFT))
				sensors_values[i]= WALL_DETECTED;
			else
			{
				sensors_values[i]= FREE_WAY_DETECTED;
			}
		}
		else
		{
			sensors_values[i]= WALL_DETECTED;
		}
	}

	sensors_values[BACK_RIGHT]= FREE_WAY_DETECTED; //Those sensors won't be used at all
	sensors_values[BACK_LEFT]= FREE_WAY_DETECTED;

	if((sensors_values[FRONT_RIGHT] == WALL_DETECTED) && (sensors_values[FRONT_LEFT] == WALL_DETECTED))
		sensors_values[FRONT_RIGHT] = WALL_DETECTED;
	else
	{
		sensors_values[FRONT_RIGHT] = FREE_WAY_DETECTED;
	}
}

/*THREAD*/
static THD_FUNCTION(ProcessMeasure, arg){

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    while(1){
    	uint8_t next_order;
    	/*DETECTION DE L'ENVIRONNEMENT*/
    	process_sensors_values();

    	/*GO TO THE MIDDLE OF AREA*/

    	//if the robot is entering a crossroad or a corner he goes in the middle of the unit//
    	//to be well positioned for next move and for characterization of its environment.//

		if(leaving_corridor&&(sensors_values[RIGHT_SENS]== FREE_WAY_DETECTED||
								sensors_values[LEFT_SENS]== FREE_WAY_DETECTED))
		{
			go_for_distance(COEFF*ROBOT_DIAMETER);
			stop();
			leaving_corridor = false;
		}

		/*ORDER AQUISITION*/

		next_order = maze_mapping_next_move(sensors_values[FRONT_RIGHT], sensors_values[RIGHT_SENS], sensors_values[LEFT_SENS]);
		switch (next_order)
		{
		case (KEEP_GOING):
			go_fast();
			leaving_corridor = true;
			break;
		case(GO_RIGHT):
			turn(TURN_RIGHT);
			go_fast();
			break;
		case(GO_FORWARD):
			go_fast();
			break;
		case(GO_LEFT):
			turn(TURN_LEFT);
			go_fast();
			break;
		case(U_TURN):
			turn(HALF_TURN);
			go_fast();
			break;
		case(DONT_MOVE):
			stop();
			break;
		default:
			break;
		}

		/*REGULATION*/
		if(maze_mapping_mode_is_selected())
		{
			if ((sensors_values[LEFT_SENS]==WALL_DETECTED)&&(sensors_values[RIGHT_SENS]==WALL_DETECTED))
				regulator_difference(get_calibrated_prox(FRONT_RIGHT_45DEG), get_calibrated_prox(FRONT_LEFT_45DEG));
			else
			{
				if (sensors_values[LEFT_SENS]==WALL_DETECTED)
					do_follow_wall_regulation(LEFT_SENS);
				else if (sensors_values[RIGHT_SENS]==WALL_DETECTED)
					do_follow_wall_regulation(RIGHT_SENS);
			}

		}

		buffer_state ++;
		if (buffer_state >= BUFFER_SIZE)
		{
			buffer_state = 0;
		}
    	chThdSleepMilliseconds(25);
    }
}


/*PUBLIC FONCTIONS*/
void process_sensors_start(void){
	chThdCreateStatic(waProcessMeasure, sizeof(waProcessMeasure), NORMALPRIO+1, ProcessMeasure, NULL);
}
