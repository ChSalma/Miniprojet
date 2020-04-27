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
#include <chprintf.h>

#include <main.h>

#define FREE_WAY 20//un chemin est considéré comme tel si l'on a une valeur inférieure à celle-ci
#define OBSTACLE 800 //un mur est considéré comme trop proche lorsque qu'on atteint cette valeur
#define ROBOT_DIAMETER 7.5f

enum{FRONT_RIGHT, FRONT_RIGHT_45DEG, RIGHT_SENS, BACK_RIGHT, BACK_LEFT, LEFT_SENS, FRONT_LEFT_45DEG, FRONT_LEFT};
enum{FREE_WAY_DETECTED, WALL_DETECTED, OBSTACLE_DETECTED};

static uint8_t sensors_values[PROXIMITY_NB_CHANNELS];
static bool obstacle_detected;
static THD_WORKING_AREA(waProcessMeasure,256);

//INTERNAL FUNCTIONS

static THD_FUNCTION(ProcessMeasure, arg){

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    while(1){
        //starts getting informations
    	uint8_t i, next_order;
    	int calibrated_prox;
    	for(i = 0; i < PROXIMITY_NB_CHANNELS; i++) //créer une variable pour checker les capteurs une fois sur deux si aquisstion de donées trop rapide
    	{

        	calibrated_prox = get_calibrated_prox(i);
//        	chprintf((BaseSequentialStream *)&SD3, "calibrated_prox %d = ", i);
//        	chprintf((BaseSequentialStream *)&SD3, "%d \n", calibrated_prox);
    		if (calibrated_prox > OBSTACLE)
    		{
    			sensors_values[i] = OBSTACLE_DETECTED;
    			//je vais faire en sorte que les capteurs avant considèrent un mur quand ils sont au niveau de l'obstacle...
    			//car la déteection est trop mauvaise
    			if ((i != 0) && (i != 7))
    			{
        			obstacle_detected=TRUE;
        			i=PROXIMITY_NB_CHANNELS; //Permet de sortir de la boucle for
    			}

    		}
    		else if (calibrated_prox < FREE_WAY)
    			sensors_values[i]= FREE_WAY_DETECTED;
    		else
    			sensors_values[i]= WALL_DETECTED;
    	}

    	sensors_values[BACK_RIGHT]= FREE_WAY_DETECTED; //Those sensors won't be used at all
    	sensors_values[BACK_LEFT]= FREE_WAY_DETECTED;

    	if (!obstacle_detected)
    	{
    		//si les capteurs avant détecte un mur ou un obstacle ils sont set comme mur détecté
    		if((sensors_values[FRONT_RIGHT] >= WALL_DETECTED) || (sensors_values[FRONT_LEFT] >= WALL_DETECTED))//a voir s'il faut plutôt une condition avec un & plutôt que ou
    			sensors_values[FRONT_RIGHT] = WALL_DETECTED;
    		else
    			sensors_values[FRONT_RIGHT]=FREE_WAY_DETECTED; //ligne inutile puisqu'on ne rentre là-dedans que si front-right est déjà free-way

    		next_order = maze_mapping_next_move((bool) sensors_values[FRONT_RIGHT], (bool) sensors_values[RIGHT_SENS], (bool) sensors_values[LEFT_SENS]);
    		switch (next_order) //il faut penser à comment faire l'enclenchement initial du robot: est-ce qu'on appelle une autre fonction?
    		{
    		case (KEEP_GOING):
                chprintf((BaseSequentialStream *)&SD3, "keep going\n");
    			go_slow();
    			break;
    		case(GO_RIGHT): //ajouter sécurité
				chprintf((BaseSequentialStream *)&SD3, "going right\n");
				go_for_distance(ROBOT_DIAMETER/2); //pour éviter que le robot tourne en ayant seulement dépasser la moitié de la jonction
    			turn(TURN_RIGHT);
				go_slow();
    			break;
    		case(GO_FORWARD): //ajouter sécurité
				chprintf((BaseSequentialStream *)&SD3, "going forward\n");
    			go_slow();
    			break;
    		case(GO_LEFT): //ajouter sécurité
				chprintf((BaseSequentialStream *)&SD3, "going left\n");
				go_for_distance(ROBOT_DIAMETER/2);
    			turn(TURN_LEFT);
				go_slow();
    			break;
    		case(U_TURN):
				chprintf((BaseSequentialStream *)&SD3, "oups a deadend\n");
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
    	else //gestion d'obstacle
    	{
    		stop(); //gestion d'obstacle manuelle
    	}

    	obstacle_detected = false;
    	chThdSleepMilliseconds(500);
		//waits to get the informations
		//signals informations are ready
			//chBSemSignal(&image_ready_sem);
    }
}

void process_sensors_start(void){
	chThdCreateStatic(waProcessMeasure, sizeof(waProcessMeasure), NORMALPRIO, ProcessMeasure, NULL);
}
