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
#include <leds.h>
#include <Mouvements.h>

#include <main.h>

#define MAX_MAP_SIZE 50
#define RESET 0
#define DEADEND_VERIFIED 5//avant c'était 8

enum {NO_SELECTED_PATH, RIGHT, FORWARD, LEFT, ALL_PATHS_CHECKED, BEGINNING, END};
enum {CORRIDOR=2, DEADEND};
enum {TURN_OFF, TURN_ON};

//prototypes de fonctions
uint8_t maze_mapping_corridor_gestion(bool, bool);
uint8_t maze_mapping_furthest_point_reached(void);
uint8_t maze_mapping_memorise_crossroad(bool);
uint8_t maze_mapping_next_step_to_goal(void);
uint8_t maze_mapping_multi_check_for_deadend(void);
void maze_mapping_set_rgb_leds(uint8_t, uint8_t, uint8_t);
void maze_mapping_update_red_leds(unsigned int, unsigned int, unsigned int, unsigned int);
void maze_mapping_update_rgb_leds(void);
void maze_mapping_victory_dance(void);

//variables globales
static uint8_t map[MAX_MAP_SIZE], multi_check_deadend=RESET;
static int8_t current_crossroad=RESET, robot_position=RESET;
static bool crossroad_already_saved=false, switch_to_discover_mode=false, uturn_to_do=true, furthest_point_reached=false;
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

uint8_t maze_mapping_furthest_point_reached(void)
{
	furthest_point_reached=false;

	if (switch_to_discover_mode)
	{
	    mode=DISCOVER;
	    maze_mapping_update_rgb_leds();
	    switch_to_discover_mode=false;
	    return KEEP_GOING;
	}
	else
	{
	    mode=NO_MODE_SELECTED;
	    maze_mapping_update_rgb_leds();
	    return DONT_MOVE;
	}
}

uint8_t maze_mapping_memorise_crossroad(bool right_status)
{
    if (!crossroad_already_saved)
    {
        robot_position=current_crossroad;

        if (current_crossroad!=RESET)
        {
            if (map[current_crossroad]!=END)
            {
				uint8_t order;

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
					map[current_crossroad]=NO_SELECTED_PATH; //Efface la case du tableau pour une nouvelle écriture -> oublie le carrefour actuel car c'est une deadend
					current_crossroad--;//si cette valeur passe en dessous de zéro, cela implique que le labyrinthe n'a pas de sortie.
				}

				crossroad_already_saved=true;
				return order;
            }
            else
            {
            	current_crossroad++;
            	mode=NO_MODE_SELECTED;
            	maze_mapping_victory_dance();
            	return U_TURN;
            }
        }
        else
        {
        	map[current_crossroad]=BEGINNING;
        	current_crossroad++;
        	crossroad_already_saved=true;
        	return GO_FORWARD;
        }
    }
    else
    {
    	return KEEP_GOING;
    }
}

uint8_t maze_mapping_next_move(bool forward_status, bool right_status, bool left_status)
{
	//mode = DISCOVER ; // à supprimer lorsque les micros seront opérationnels
    if (mode==NO_MODE_SELECTED)
        return DONT_MOVE;

    uint8_t crossroad_form=forward_status+right_status+left_status;

    switch (crossroad_form)
    {
        case DEADEND:
            crossroad_already_saved=false;
            if (furthest_point_reached)
            	return maze_mapping_furthest_point_reached();
            else
            	return maze_mapping_multi_check_for_deadend();

        case CORRIDOR:
            crossroad_already_saved=false;
            multi_check_deadend=RESET;
            if (furthest_point_reached)
            	return maze_mapping_furthest_point_reached();
            else
            	return maze_mapping_corridor_gestion(right_status, left_status);

        default:
        	multi_check_deadend=RESET;
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
                if (map[robot_position]!=END)
                	order=ALL_PATHS_CHECKED-map[robot_position]; //Lors du chemin de retour, un virage à droite correspond à un virage à gauche et vice versa
                else
                	order=GO_FORWARD;
                robot_position--;
            }
            else
            {
                robot_position++;
                order=U_TURN;
                mode=NO_MODE_SELECTED;
                maze_mapping_victory_dance();
            }
        }
        else
        {
        	if (current_crossroad>RESET+1)
        	{
				if (map[robot_position]!=END)
				{
					order=map[robot_position];

					if (robot_position<(current_crossroad-1))
						robot_position++;
					else
						furthest_point_reached=true;
				}
				else
				{
					order=U_TURN;
					mode=NO_MODE_SELECTED;
					maze_mapping_victory_dance();
				}
        	}
        	else
        	{
        		furthest_point_reached=true;
        		order=GO_FORWARD;
        	}
        }
        maze_mapping_update_rgb_leds();
        crossroad_already_saved=true;
        return order;
    }
    else
        return KEEP_GOING;
}

bool maze_mapping_uturn_after_selecting_mode(uint8_t mode_selected)
{
	bool do_a_uturn;

	if ((map[robot_position]==END)&&(mode_selected!=RETURN_HOME))
		return false;

	if ((map[robot_position]==BEGINNING)&&(mode_selected==RETURN_HOME))
		return false;

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

    	maze_mapping_update_rgb_leds();
    }
    else
    {
    	do_a_uturn=false;
    }

    return do_a_uturn;
}

bool maze_mapping_mode_is_selected(void)
{
	if (mode==NO_MODE_SELECTED)
		return false;
	else
		return true;
}

uint8_t maze_mapping_multi_check_for_deadend(void)
{
	if (multi_check_deadend<DEADEND_VERIFIED)
	{
		multi_check_deadend++;
		return KEEP_GOING;
	}
	else
	{
		multi_check_deadend=RESET;
		if (current_crossroad>RESET)
			current_crossroad--;
		uturn_to_do=false;
		return U_TURN;
	}
}

void maze_mapping_process_end_of_maze(void)
{
	map[current_crossroad]=END;
}

void maze_mapping_set_rgb_leds(uint8_t red_intensity, uint8_t green_intensity, uint8_t blue_intensity)
{
	int led_id;

	for (led_id=0; led_id<=LED8; led_id++)
		set_rgb_led(led_id, red_intensity, green_intensity, blue_intensity);
}

void maze_mapping_update_red_leds(unsigned int led1_val, unsigned int led3_val, unsigned int led5_val, unsigned int led7_val)
{
	set_led(LED1, led1_val);
	set_led(LED3, led3_val);
	set_led(LED5, led5_val);
	set_led(LED7, led7_val);
}

void maze_mapping_update_rgb_leds(void)
{
	switch(mode)
	{
		case NO_MODE_SELECTED:
			maze_mapping_set_rgb_leds(RGB_MAX_INTENSITY, RGB_MAX_INTENSITY, RGB_MAX_INTENSITY);
			break;

		case DISCOVER:
			maze_mapping_set_rgb_leds(TURN_OFF, RGB_MAX_INTENSITY, TURN_OFF);
			break;

		case RETURN_HOME:
			maze_mapping_set_rgb_leds(RGB_MAX_INTENSITY, TURN_OFF, TURN_OFF);
			break;

		case GO_FURTHEST_POINT_KNOWN:
			maze_mapping_set_rgb_leds(TURN_OFF, TURN_OFF, RGB_MAX_INTENSITY);
			break;

		default:
			maze_mapping_set_rgb_leds(TURN_OFF, TURN_OFF, TURN_OFF);
			break;
	}
}

void maze_mapping_victory_dance(void)
{
	clear_leds();
	maze_mapping_update_red_leds(TURN_ON, TURN_OFF, TURN_ON, TURN_OFF);
	turn(-HALF_TURN);
	maze_mapping_update_red_leds(TURN_OFF, TURN_ON, TURN_OFF, TURN_ON);
	turn(-HALF_TURN);
	maze_mapping_update_red_leds(TURN_ON, TURN_OFF, TURN_ON, TURN_OFF);
	turn(HALF_TURN);
	maze_mapping_update_red_leds(TURN_ON, TURN_ON, TURN_ON, TURN_ON);
	turn(HALF_TURN);
	maze_mapping_update_red_leds(TURN_OFF, TURN_OFF, TURN_OFF, TURN_OFF);
	maze_mapping_update_rgb_leds();
}
