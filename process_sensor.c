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
//#include <math.h>
//#include <usbcfg.h>
//#include <chprintf.h>

#include <main.h>

#define FREE_WAY 20//un chemin est consid�r� comme telle si l'on a une valeur inf�rieure � celle-ci
#define OBSTACLE 800 //un mur est consid�r� comme trop proche lorsque qu'on atteint cette valeur
enum{FREE_WAY_DETECTED, WALL_DETECTED, OBSTACLE_DETECTED};

static uint8_t sensors_values[PROXIMITY_NB_CHANNELS];
//static uint8_t cases_table[][6]=//0:devant_droite, 1:
//{{
static bool obstacle_detected;
static THD_WORKING_AREA(waProcessMeasure,256);
static THD_FUNCTION(ProcessMeasure, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    while(1){
        //starts getting informations
    	uint8_t i;
    	for(i = 0; i < PROXIMITY_NB_CHANNELS; i++)
    	{
    		if (get_calibrated_prox(i) > OBSTACLE)
    			sensors_values[i] = OBSTACLE_DETECTED;
    		else if (get_calibrated_prox(i) < FREE_WAY)
				sensors_values[i]= FREE_WAY_DETECTED;
    		else
    			sensors_values[i]= WALL_DETECTED;
    	}

    		if (sensors_values[i] > OBSTACLE)
    		{
    			//mettre � jour variable globale obstacle
    			obstacle_detected = true;
    			if (i < (PROXIMITY_NB_CHANNELS/2)) //Obstacle d�tect� � droite
    			{
    				turn(TURN_LEFT);
    				go_slow();
    			}
    			else
    			{
    				turn(TURN_RIGHT); //Obstacle d�tect� � gauche
    				go_slow();
    			}
    		}
    		else if (!obstacle_detected)
    		{
    			go_fast();
    		}
    	}
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
