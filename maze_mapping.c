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
#include <leds.h>
#include <Mouvements.h>
#include <main.h>

#define MAX_MAP_SIZE 50
#define RESET 0
#define CASE_VERIFIED 5

enum {NO_SELECTED_PATH, RIGHT, FORWARD, LEFT, ALL_PATHS_CHECKED, BEGINNING, END};
enum {CROSSROAD_4, CROSSROAD_3, CORRIDOR, DEADEND, NO_CASE};
enum {TURN_OFF, TURN_ON};

//prototypes de fonctions
uint8_t maze_mapping_corridor_gestion(bool, bool);
uint8_t maze_mapping_crossroad_gestion(bool, uint8_t);
uint8_t maze_mapping_deadend_gestion(void);
uint8_t maze_mapping_furthest_point_reached(void);
uint8_t maze_mapping_memorise_crossroad(bool);
uint8_t maze_mapping_next_step_to_goal(void);
bool maze_mapping_multi_check_case(uint8_t new_case);
void maze_mapping_set_rgb_leds(uint8_t, uint8_t, uint8_t);
void maze_mapping_update_red_leds(unsigned int, unsigned int, unsigned int, unsigned int);
void maze_mapping_update_rgb_leds(void);
void maze_mapping_victory_dance(void);

//variables globales
static uint8_t map[MAX_MAP_SIZE], multi_check=RESET, last_case=NO_CASE;
static int8_t current_crossroad=RESET, robot_position=RESET;
static bool memorise_crossroad=true, switch_to_discover_mode=false, uturn_to_do=true, do_a_uturn=false, furthest_point_reached=false;
static uint8_t mode = NO_MODE_SELECTED;

//Déclarations des fonctions
uint8_t maze_mapping_corridor_gestion(bool right_status, bool left_status)
{
	if (right_status && left_status)
	{
		memorise_crossroad=true;
        return KEEP_GOING;
	}

	if(maze_mapping_multi_check_case(CORRIDOR))
	{
		//après avoir fait un quart de tour, le robot perçoit son environnement comme un carrefour à 3 issues
		//afin d'éviter de mémoriser ce carrefour "virtuel", on active la sécurité
		memorise_crossroad=false;

		if (!right_status)
			return GO_RIGHT;
		else
			return GO_LEFT;
	}
	else
		return GO_FORWARD;
}

uint8_t maze_mapping_crossroad_gestion(bool right_status, uint8_t crossroad_form)
{
	if (memorise_crossroad)
	{
		if(maze_mapping_multi_check_case(crossroad_form))
		{
			uturn_to_do=true;
			switch (mode)
			{
				case DISCOVER:
					return maze_mapping_memorise_crossroad(right_status);

				default:
					return maze_mapping_next_step_to_goal();
			}
		}
		else
			return GO_FORWARD;
	}
	else
		return GO_FORWARD;
}

uint8_t maze_mapping_deadend_gestion(void)
{
	if (!maze_mapping_multi_check_case(DEADEND))
		return KEEP_GOING;
	else
	{
		if (current_crossroad>RESET)
			current_crossroad--;
		uturn_to_do=false;
		return U_TURN;
	}
}

uint8_t maze_mapping_furthest_point_reached(void)
{
	furthest_point_reached=false;
	memorise_crossroad=true;
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

			memorise_crossroad=false;
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
        memorise_crossroad=false;
        return GO_FORWARD;
    }
}

uint8_t maze_mapping_next_move(bool forward_status, bool right_status, bool left_status)
{
    if (mode==NO_MODE_SELECTED)
        return DONT_MOVE;

    uint8_t crossroad_form=forward_status+right_status+left_status;

    switch (crossroad_form)
    {
        case DEADEND:
            memorise_crossroad=true;
            if (furthest_point_reached)
            	return maze_mapping_furthest_point_reached();
            else
            	return maze_mapping_deadend_gestion();

        case CORRIDOR:
            if (furthest_point_reached)
            	return maze_mapping_furthest_point_reached();
            else
            	return maze_mapping_corridor_gestion(right_status, left_status);

        default:
        	return maze_mapping_crossroad_gestion(right_status, crossroad_form);
    }

}

uint8_t maze_mapping_next_step_to_goal(void)
{
	uint8_t order;

    if (mode==RETURN_HOME)
    {
    	if (robot_position>RESET)
        {
    		if (map[robot_position]!=END)
    			order=ALL_PATHS_CHECKED-map[robot_position]; //Lors du chemin de retour, un virage à droite correspond à un virage à gauche et vice versa
            else
                order=GO_FORWARD;
            robot_position--;
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
    	if (current_crossroad>RESET+1)
        {
    		if (map[robot_position]!=END)
			{
    			if (robot_position!=RESET)
    				order=map[robot_position];
    			else
    			{
    				order=GO_FORWARD;
    			}

				if (robot_position<(current_crossroad-1))
					robot_position++;
				else
					furthest_point_reached=true;
			}
			else
			{
				order=U_TURN;
				uturn_to_do=false;
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
    memorise_crossroad=false;
    return order;
}

bool maze_mapping_do_a_uturn(void)
{
	if (do_a_uturn)
	{
		do_a_uturn=false;
		return true;
	}
	else
		return false;
}

bool maze_mapping_mode_is_selected(void)
{
	if (mode==NO_MODE_SELECTED)
		return false;
	else
		return true;
}

bool maze_mapping_multi_check_case(uint8_t new_case)
{
	if(new_case==last_case)
	{
		if (multi_check<CASE_VERIFIED)
		{
			multi_check++;
			return false;
		}
		else
		{
			multi_check=RESET;
			return true;
		}
	}
	else
	{
		last_case=new_case;
		multi_check=RESET;
		return false;
	}
}

void maze_mapping_process_end_of_maze(void)
{
	map[current_crossroad]=END;
}

void maze_mapping_select_mode(uint8_t mode_selected)
{
	bool ignore_order=false;

	if ((map[robot_position]==END)&&(mode_selected!=RETURN_HOME))
		ignore_order=true;

	if ((map[robot_position]==BEGINNING)&&(mode_selected==RETURN_HOME))
		ignore_order=true;

	if ((!ignore_order)&&(mode_selected!=mode))
    {
		if (((mode==RETURN_HOME)||(mode_selected==RETURN_HOME))&&uturn_to_do)
			do_a_uturn=true;

    	if ((mode_selected==DISCOVER)&&(robot_position!=current_crossroad))
    	{
    		mode=GO_FURTHEST_POINT_KNOWN;
    		switch_to_discover_mode=true;
    	}
    	else
    		mode=mode_selected;

    	maze_mapping_update_rgb_leds();
    }
}

void maze_mapping_set_rgb_leds(uint8_t red_intensity, uint8_t green_intensity, uint8_t blue_intensity)
{
	int led_id;

	for (led_id=LED2; led_id<=LED8; led_id++)
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
	turn(-HALF_TURN, ULTRA_HIGH_SPEED);
	maze_mapping_update_red_leds(TURN_OFF, TURN_ON, TURN_OFF, TURN_ON);
	turn(-HALF_TURN, ULTRA_HIGH_SPEED);
	maze_mapping_update_red_leds(TURN_ON, TURN_OFF, TURN_ON, TURN_OFF);
	turn(HALF_TURN, ULTRA_HIGH_SPEED);
	maze_mapping_update_red_leds(TURN_ON, TURN_ON, TURN_ON, TURN_ON);
	turn(HALF_TURN, ULTRA_HIGH_SPEED);
	maze_mapping_update_red_leds(TURN_OFF, TURN_OFF, TURN_OFF, TURN_OFF);
	maze_mapping_update_rgb_leds();
}
