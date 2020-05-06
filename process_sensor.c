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

#include <main.h>

#define ROBOT_DIAMETER 7.5f
#define COEFF 0.5f
#define MAZE_UNIT 13
#define AMBIENT_LIGHT_DIFF_THRESHOLD 50
#define ANTICIPATION_OFFSET 300

#define BUFFER_SIZE 3

enum{FREE_WAY_DETECTED, WALL_DETECTED};

static bool leaving_corridor;
static bool sensors_values[PROXIMITY_NB_CHANNELS];
static int previous_ambient_light_value;
static int data_sensors[PROXIMITY_NB_CHANNELS*BUFFER_SIZE];
static uint8_t buffer_state;
static THD_WORKING_AREA(waProcessMeasure,256);
//INTERNAL FUNCTIONS

static THD_FUNCTION(ProcessMeasure, arg){

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    while(1){
        //starts getting informations
    	uint8_t i, j, next_order;
    	int current_ambient_light_value = get_ambient_light(RIGHT_SENS);
    	//version avec moyenne mobile//
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
    			//si on sait qu'on est de toute fa�on dans un virage ou carrefour on peut tester plus rapidemeent le capteur avant

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

		//condition qui permet d'�liminer les valeurs aberrantes li�es � un chgt de luminosit� brusque
		if(!((current_ambient_light_value - previous_ambient_light_value) > AMBIENT_LIGHT_DIFF_THRESHOLD || (current_ambient_light_value - previous_ambient_light_value) < -AMBIENT_LIGHT_DIFF_THRESHOLD))
		{
			//lorsqu'il entre dans un nouveau carrefour ou un virage le robot avance jusqu'au centre de la case et s'arr�te en attendant le prochain ordre
			if(leaving_corridor&&(sensors_values[RIGHT_SENS]== FREE_WAY_DETECTED||
									sensors_values[LEFT_SENS]== FREE_WAY_DETECTED))
			{
				go_for_distance(COEFF*ROBOT_DIAMETER);
				stop();
				leaving_corridor = false;

			}
			//appelle maze_mapping pour conna�tre le prochain mvt � effectuer
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
		}
		buffer_state ++;
		if (buffer_state >= BUFFER_SIZE)
		{
			buffer_state = 0;
		}
		previous_ambient_light_value = current_ambient_light_value;
    	chThdSleepMilliseconds(25);
    }
}

void process_sensors_start(void){
	chThdCreateStatic(waProcessMeasure, sizeof(waProcessMeasure), NORMALPRIO+1, ProcessMeasure, NULL);
}
