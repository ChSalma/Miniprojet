/*
 * maze_mapping.c
 *
 *  Created on: 16 avr. 2020
 *      Author: afgou
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <maze_mapping.h>
#include <constantes.h>
#include "ch.h"
#include <chprintf.h>

#include <main.h>

#define MAX_MAP_SIZE 50
#define RESET 0

enum {NO_SELECTED_PATH, RIGHT, FORWARD, LEFT, ALL_PATHS_CHECKED};
enum {CORRIDOR=2, DEADEND};

//prototypes de fonctions
uint8_t maze_mapping_corridor_gestion(bool, bool);
uint8_t maze_mapping_memorise_crossroad(bool);
uint8_t maze_mapping_next_step_to_goal(void);

//variables globales
static uint8_t map[MAX_MAP_SIZE];
static int8_t current_crossroad=RESET, robot_position=RESET;
static bool crossroad_already_saved=false, switch_to_discover_mode=false, uturn_to_do=true;
static uint8_t mode = NO_MODE_SELECTED;

//Déclarations des fonctions
uint8_t maze_mapping_corridor_gestion(bool right_status, bool left_status)
{
	if (right_status && left_status)
        return KEEP_GOING;

    if (!right_status)
        return GO_RIGHT;
    else
        return GO_LEFT;
}

uint8_t maze_mapping_memorise_crossroad(bool right_status)
{
    if (!crossroad_already_saved)
    {
        uint8_t order;

        robot_position=current_crossroad;

        if (!right_status)
        {
            map[current_crossroad]+=RIGHT;
            order=GO_RIGHT;
        }
        else
        {
            map[current_crossroad]+=FORWARD;
            order=GO_FORWARD;
        }

        if (map[current_crossroad]!=ALL_PATHS_CHECKED)
        {
            if (current_crossroad<MAX_MAP_SIZE)
                current_crossroad++;//si cette valeur dépasse MAX_MAP_SIZE, le labyrithe est trop difficile pour le programme.
            else
            	chSysHalt("Le labyrinthe est trop difficile pour le programme");
        }
        else
        {
            if (current_crossroad>RESET)
            {
                map[current_crossroad]=NO_SELECTED_PATH; //Efface la case du tableau pour une nouvelle écriture -> oublie le carrefour actuel car c'est une deadend
                current_crossroad--;//si cette valeur passe en dessous de zéro, cela implique que le labyrinthe n'a pas de sortie.
            }
            else
            	chSysHalt("Le labyrinthe n'a pas d'issue!"); //Musique de déception
        }

        crossroad_already_saved=true;
        return order;
    }
    else
    {
    	return KEEP_GOING;
    }
}

uint8_t maze_mapping_next_move(bool forward_status, bool right_status, bool left_status)
{
	mode = DISCOVER ; // à supprimer lorsque les micros seront opérationnels
    if (mode==NO_MODE_SELECTED)
        return DONT_MOVE;

    uint8_t crossroad_form=forward_status+right_status+left_status;

    switch (crossroad_form)
    {
        case DEADEND:
            crossroad_already_saved=false;
            if (current_crossroad>RESET)
                current_crossroad--;
            uturn_to_do=false;
            return U_TURN;

        case CORRIDOR:
            crossroad_already_saved=false;
            return maze_mapping_corridor_gestion(right_status, left_status);

        default:
        	uturn_to_do=true;
        	break;
    }

    switch (mode)
    {
        case DISCOVER:
            return maze_mapping_memorise_crossroad(right_status);

        default:
            return maze_mapping_next_step_to_goal();
    }
}

uint8_t maze_mapping_next_step_to_goal(void)
{
    if (!crossroad_already_saved)
    {
        uint8_t order;

        if (mode==RETURN_HOME)
        {
            if (robot_position>=RESET)
            {
                order=ALL_PATHS_CHECKED-map[robot_position]; //Lors du chemin de retour, un virage à droite correspond à un virage à gauche et vice versa
                robot_position--;
            }
            else
            {
                robot_position++;
                order=U_TURN;
                mode=NO_MODE_SELECTED;
            }
        }
        else
        {
            if (robot_position<=current_crossroad)
            {
                order=map[robot_position];
                robot_position++;
            }
            else
            {
                robot_position--;
                if (switch_to_discover_mode)
                {
                    order=KEEP_GOING;
                    mode=DISCOVER;
                    switch_to_discover_mode=false;
                }
                else
                {
                    order=DONT_MOVE;
                    mode=NO_MODE_SELECTED;
                }
            }
        }
        crossroad_already_saved=true;
        return order;
    }
    else
        return KEEP_GOING;
}

bool maze_mapping_uturn_after_selecting_mode(uint8_t mode_selected)
{
	bool do_a_uturn;

	if (mode_selected!=mode)
    {
    	if (((mode==RETURN_HOME)||(mode_selected==RETURN_HOME))&&uturn_to_do)
    		do_a_uturn=true;
    	else
    		do_a_uturn=false;

    	if ((mode_selected==DISCOVER)&&(robot_position!=current_crossroad))
    	{
    		mode=GO_FURTHEST_POINT_KNOWN;
    		switch_to_discover_mode=true;
    	}
    	else
    		mode=mode_selected;
    }
    else
    	do_a_uturn=false;

    return do_a_uturn;
}
