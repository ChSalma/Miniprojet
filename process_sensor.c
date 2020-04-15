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
#include <motors.h>
//#include <math.h>
//#include <usbcfg.h>
//#include <chprintf.h>

#include <main.h>

#define FREE_WAY 20//un chemin est considéré comme telle si l'on a une valeur inférieure à celle-ci
#define OBSTACLE 800 //un mur est considéré comme trop proche lorsque qu'on atteint cette valeur

static unsigned int sensors_values[PROXIMITY_NB_CHANNELS];
static bool obstacle_detected;
static THD_WORKING_AREA(waProcessMeasure,256);
static THD_FUNCTION(ProcessMeasure, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    while(1){
        //starts getting informations
    	int i;
    	for(i = 0; i < PROXIMITY_NB_CHANNELS; i++)
    	{
    		sensors_values[i] = get_calibrated_prox(i);
    		if (sensors_values[i] > OBSTACLE)
    		{
    			//mettre à jour variable globale obstacle
    			obstacle_detected = true;
    			right_motor_set_speed(-300);
    			left_motor_set_speed(-300);
    		}
    		else if (!obstacle_detected)
    		{
    			right_motor_set_speed(300);
    			left_motor_set_speed(300);
    		}
    	}
    	chThdSleepMilliseconds(1000);
    	obstacle_detected = false;
		//waits to get the informations
		//signals informations are ready
			//chBSemSignal(&image_ready_sem);
    }
}

void process_sensors_start(void){
	chThdCreateStatic(waProcessMeasure, sizeof(waProcessMeasure), NORMALPRIO, ProcessMeasure, NULL);
}
