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

#define OBSTACLE 800 //un mur est consid�r� comme trop proche lorsque qu'on atteint cette valeur
#define ROBOT_DIAMETER 7.5f
#define MAZE_UNIT 13

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
    	for(i = 0; i < PROXIMITY_NB_CHANNELS; i++) //cr�er une variable pour checker les capteurs une fois sur deux si aquisstion de don�es trop rapide
    	{

        	calibrated_prox = get_calibrated_prox(i);
//        	chprintf((BaseSequentialStream *)&SD3, "calibrated_prox %d = ", i);
//        	chprintf((BaseSequentialStream *)&SD3, "%d \n", calibrated_prox);
    		if (calibrated_prox > OBSTACLE)
    		{
    			sensors_values[i] = OBSTACLE_DETECTED;
    			//je vais faire en sorte que les capteurs avant consid�rent un mur quand ils sont au niveau de l'obstacle...
    			//car la detection est trop mauvaise
    			if ((i != FRONT_RIGHT) && (i != FRONT_LEFT)) //on ne fait pas la diff�rence entre un mur et un ostacle quand c'est devant
    			{
        			obstacle_detected=TRUE;
        			i=PROXIMITY_NB_CHANNELS; //Permet de sortir de la boucle for
    			}

    		}
    		else if (calibrated_prox < FREE_WAY_LEFT || calibrated_prox < FREE_WAY_RIGHT ||calibrated_prox < FREE_WAY_FRONT)
    		{
    			if((i == RIGHT_SENS) && (calibrated_prox > FREE_WAY_RIGHT))
    				sensors_values[i]= WALL_DETECTED;
    			if((i == LEFT_SENS) && (calibrated_prox > FREE_WAY_LEFT))
    				sensors_values[i]= WALL_DETECTED;
    			if(((i == FRONT_RIGHT) || (i==FRONT_LEFT)) && (calibrated_prox > FREE_WAY_FRONT))
    				sensors_values[i]= WALL_DETECTED;
    			else
    				sensors_values[i]= FREE_WAY_DETECTED;
    		}
    		else
    			sensors_values[i]= WALL_DETECTED;
    	}

    	sensors_values[BACK_RIGHT]= FREE_WAY_DETECTED; //Those sensors won't be used at all
    	sensors_values[BACK_LEFT]= FREE_WAY_DETECTED;

    	if (!obstacle_detected)
    	{
    		//si les capteurs avant d�tecte un mur ou un obstacle ils sont set comme mur d�tect�
    		if((sensors_values[FRONT_RIGHT] >= WALL_DETECTED) && (sensors_values[FRONT_LEFT] >= WALL_DETECTED))//a voir s'il faut plut�t une condition avec un & plut�t que ou
    			sensors_values[FRONT_RIGHT] = WALL_DETECTED;
    		else
    			sensors_values[FRONT_RIGHT]=FREE_WAY_DETECTED; //ligne inutile puisqu'on ne rentre l�-dedans que si front-right est d�j� free-way

    		next_order = maze_mapping_next_move((bool) sensors_values[FRONT_RIGHT], (bool) sensors_values[RIGHT_SENS], (bool) sensors_values[LEFT_SENS]);
    		switch (next_order) //il faut penser � comment faire l'enclenchement initial du robot: est-ce qu'on appelle une autre fonction?
    		{
    		case (KEEP_GOING):
                chprintf((BaseSequentialStream *)&SD3, "keep going\n");
    			go_slow();
    			break;
    		case(GO_RIGHT): //ajouter s�curit�
				chprintf((BaseSequentialStream *)&SD3, "going right\n");
				go_for_distance(3*ROBOT_DIAMETER/5); //pour �viter que le robot tourne en ayant seulement d�passer la moiti� de la jonction
    			turn(TURN_RIGHT);
//    			go_for_distance(MAZE_UNIT/2);
				go_slow();
    			break;
    		case(GO_FORWARD): //ajouter s�curit�
				chprintf((BaseSequentialStream *)&SD3, "going forward\n");
//    			go_for_distance(MAZE_UNIT/2);
    			go_slow();
    			break;
    		case(GO_LEFT): //ajouter s�curit�
				chprintf((BaseSequentialStream *)&SD3, "going left\n");
				go_for_distance(3*ROBOT_DIAMETER/5);
    			turn(TURN_LEFT);
//    			go_for_distance(MAZE_UNIT/2);
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
//    		stop(); //gestion d'obstacle manuelle
    	}

    	obstacle_detected = false;
    	chThdSleepMilliseconds(30); //� 100 fonctionne bien mais ne d�tecte pas les "portes"
		//waits to get the informations
		//signals informations are ready
			//chBSemSignal(&image_ready_sem);
    }
}

void process_sensors_start(void){
	chThdCreateStatic(waProcessMeasure, sizeof(waProcessMeasure), NORMALPRIO+1, ProcessMeasure, NULL);
}
