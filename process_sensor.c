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
#define COEFF 0.6f
#define MAZE_UNIT 13
#define AMBIENT_LIGHT_DIFF_THRESHOLD 20
#define ANTICIPATION_OFFSET 300

#define BUFFER_SIZE 3

enum{FREE_WAY_DETECTED, WALL_DETECTED};

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
    		chprintf((BaseSequentialStream *) &SD3, "id = %d , calibrated = %d\n",i, calibrated_prox);
    		if(i==FRONT_RIGHT || i==FRONT_LEFT)
    		{
    			//si on sait qu'on est de toute façon dans un virage ou carrefour on peut tester plus rapidemeent le capteur avant

    			if(((sensors_values[RIGHT_SENS]==FREE_WAY_DETECTED||sensors_values[LEFT_SENS]==FREE_WAY_DETECTED))&&
    				(calibrated_prox > (FREE_WAY_FRONT-ANTICIPATION_OFFSET)))
    				sensors_values[i]= WALL_DETECTED;
    			else if (calibrated_prox > FREE_WAY_FRONT)
    				sensors_values[i]= WALL_DETECTED;
    			else
    				sensors_values[i]= FREE_WAY_DETECTED;
    		}
    		else if ((calibrated_prox < FREE_WAY_LEFT) || (calibrated_prox < FREE_WAY_RIGHT))
			{
				if((i == RIGHT_SENS) && (calibrated_prox > FREE_WAY_RIGHT))
					sensors_values[i]= WALL_DETECTED;
				else if((i == LEFT_SENS) && (calibrated_prox > FREE_WAY_LEFT))
					sensors_values[i]= WALL_DETECTED;
				else
					sensors_values[i]= FREE_WAY_DETECTED;
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
		if(!((current_ambient_light_value - previous_ambient_light_value) > AMBIENT_LIGHT_DIFF_THRESHOLD || (current_ambient_light_value - previous_ambient_light_value) < -AMBIENT_LIGHT_DIFF_THRESHOLD))
		{
			next_order = maze_mapping_next_move(sensors_values[FRONT_RIGHT], sensors_values[RIGHT_SENS], sensors_values[LEFT_SENS]);
			switch (next_order) //il faut penser à comment faire l'enclenchement initial du robot: est-ce qu'on appelle une autre fonction?
			{
			case (KEEP_GOING):
				go_slow();
				break;
			case(GO_RIGHT):
				go_for_distance(COEFF*ROBOT_DIAMETER); //pour éviter que le robot tourne en ayant seulement dépasser la moitié de la jonction
				turn(TURN_RIGHT);
				go_slow();
				break;
			case(GO_FORWARD):
				go_slow();
				break;
			case(GO_LEFT):
				go_for_distance(COEFF*ROBOT_DIAMETER);
				turn(TURN_LEFT);
				go_slow();
				break;
			case(U_TURN):
				turn(HALF_TURN);
				go_slow();
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
    	//version avec valeur unique et sécurité variation luminosité brusque//
//    	if (!((current_ambient_light_value - previous_ambient_light_value) > AMBIENT_LIGHT_DIFF_THRESHOLD || (current_ambient_light_value - previous_ambient_light_value) < -AMBIENT_LIGHT_DIFF_THRESHOLD))
//    	{
//			for(i = 0; i < PROXIMITY_NB_CHANNELS; i++)
//			{
//
//				calibrated_prox = get_calibrated_prox(i);
//				if ((calibrated_prox < FREE_WAY_LEFT) || (calibrated_prox < FREE_WAY_RIGHT) ||(calibrated_prox < FREE_WAY_FRONT))
//				{
//					if((i == RIGHT_SENS) && (calibrated_prox > FREE_WAY_RIGHT))
//						sensors_values[i]= WALL_DETECTED;
//					else if((i == LEFT_SENS) && (calibrated_prox > FREE_WAY_LEFT))
//						sensors_values[i]= WALL_DETECTED;
//					else if(((i == FRONT_RIGHT) || (i==FRONT_LEFT)) && (calibrated_prox > FREE_WAY_FRONT))
//						sensors_values[i]= WALL_DETECTED;
//					else
//						sensors_values[i]= FREE_WAY_DETECTED;
//				}
//				else
//					sensors_values[i]= WALL_DETECTED;
//			}
//
//			sensors_values[BACK_RIGHT]= FREE_WAY_DETECTED; //Those sensors won't be used at all
//			sensors_values[BACK_LEFT]= FREE_WAY_DETECTED;
//
//			if((sensors_values[FRONT_RIGHT] == WALL_DETECTED) && (sensors_values[FRONT_LEFT] == WALL_DETECTED))//a voir s'il faut plutôt une condition avec un & plutôt que ou
//				sensors_values[FRONT_RIGHT] = WALL_DETECTED;
//			else
//			{
//				sensors_values[FRONT_RIGHT] = FREE_WAY_DETECTED;
//			}
//			next_order = maze_mapping_next_move(sensors_values[FRONT_RIGHT], sensors_values[RIGHT_SENS], sensors_values[LEFT_SENS]);
//			switch (next_order) //il faut penser à comment faire l'enclenchement initial du robot: est-ce qu'on appelle une autre fonction?
//			{
//			case (KEEP_GOING):
//	//			chprintf((BaseSequentialStream *)&SD3, "keep going\n");
//				go_slow();
//				break;
//			case(GO_RIGHT): //ajouter sécurité
//	//			chprintf((BaseSequentialStream *)&SD3, "going right\n");
//				go_for_distance(COEFF*ROBOT_DIAMETER); //pour éviter que le robot tourne en ayant seulement dépasser la moitié de la jonction
//				turn(TURN_RIGHT);
//				go_slow();
//				break;
//			case(GO_FORWARD): //ajouter sécurité
//	//			chprintf((BaseSequentialStream *)&SD3, "going forward\n");
//				go_slow();
//				break;
//			case(GO_LEFT): //ajouter sécurité
//	//			chprintf((BaseSequentialStream *)&SD3, "going left\n");
//				go_for_distance(COEFF*ROBOT_DIAMETER);
//				turn(TURN_LEFT);
//				go_slow();
//				break;
//			case(U_TURN):
//	//			chprintf((BaseSequentialStream *)&SD3, "oups a deadend\n");
//				turn(HALF_TURN);
//				go_slow();
//				break;
//			case(DONT_MOVE):
//				stop();
//				break;
//			default:
//				break;
//			}
//    	}
    	//previous_ambient_light_value = current_ambient_light_value;
    	chThdSleepMilliseconds(25);
    }
}

void process_sensors_start(void){
	chThdCreateStatic(waProcessMeasure, sizeof(waProcessMeasure), NORMALPRIO+1, ProcessMeasure, NULL);
}
