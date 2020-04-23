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
#include <maze_mapping.h>
//#include <math.h>
//#include <usbcfg.h>
//#include <chprintf.h>

#include <main.h>

#define FREE_WAY 20//un chemin est considéré comme telle si l'on a une valeur inférieure à celle-ci
#define OBSTACLE 800 //un mur est considéré comme trop proche lorsque qu'on atteint cette valeur

enum{FRONT_RIGHT, FRONT_RIGHT_45DEG, RIGHT_SENS, BACK_RIGHT, BACK_LEFT, LEFT_SENS, FRONT_LEFT_45DEG, FRONT_LEFT};
enum{FREE_WAY_DETECTED, WALL_DETECTED, OBSTACLE_DETECTED};

static uint8_t sensors_values[PROXIMITY_NB_CHANNELS];
static bool obstacle_detected;
static THD_WORKING_AREA(waProcessMeasure,256);
static THD_FUNCTION(ProcessMeasure, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    while(1){
        //starts getting informations
    	uint8_t i, next_order;
    	for(i = 0; i < PROXIMITY_NB_CHANNELS; i++)
    	{
    		if (get_calibrated_prox(i) > OBSTACLE)
    		{
    			sensors_values[i] = OBSTACLE_DETECTED;
    			obstacle_detected=TRUE;
    			i=PROXIMITY_NB_CHANNELS; //Permet de sortir de la boucle for
    		}
    		else if (get_calibrated_prox(i) < FREE_WAY)
    			sensors_values[i]= FREE_WAY_DETECTED;
    		else
    			sensors_values[i]= WALL_DETECTED;
    	}

    	sensors_values[BACK_RIGHT]= FREE_WAY_DETECTED; //Those sensors won't be used at all
    	sensors_values[BACK_LEFT]= FREE_WAY_DETECTED;

    	if (!obstacle_detected)
    	{
    		if((sensors_values[FRONT_RIGHT]==WALL_DETECTED) || (sensors_values[FRONT_LEFT]==WALL_DETECTED))
    			sensors_values[FRONT_RIGHT]=WALL_DETECTED;
    		else
    			sensors_values[FRONT_RIGHT]=FREE_WAY_DETECTED;

    		next_order=maze_mapping_next_move((bool) sensors_values[FRONT_RIGHT], (bool) sensors_values[RIGHT_SENS], (bool) sensors_values[LEFT_SENS]);
    	}
    	else; //gestion d'obstacle

    	obstacle_detected = false;
    	chThdSleepMilliseconds(1000);
		//waits to get the informations
		//signals informations are ready
			//chBSemSignal(&image_ready_sem);
    }
}

void process_sensors_start(void){
	chThdCreateStatic(waProcessMeasure, sizeof(waProcessMeasure), NORMALPRIO, ProcessMeasure, NULL);
}
