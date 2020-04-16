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

#define DELTA_POS_MM 100.0

enum{NEW_CR=-1, KNOWN_CR};

//prototypes de fonctions
int maze_mapping_test_known_cr(CROSSROAD*, int, float, float);


//déclaration des fonctions
int maze_mapping_test_known_cr(CROSSROAD *cr_list, int size_cr_list, float robot_pos_x, float robot_pos_y)
{
	int  cr_id;

	for (cr_id = size_cr_list-1; cr_id >= 0; cr_id--)
	{
		if ((robot_pos_x < cr_list[cr_id].cr_pos_x + DELTA_POS_MM) &&
			(robot_pos_x > cr_list[cr_id].cr_pos_x - DELTA_POS_MM) &&
			(robot_pos_y < cr_list[cr_id].cr_pos_y + DELTA_POS_MM) &&
			(robot_pos_y > cr_list[cr_id].cr_pos_y - DELTA_POS_MM))
			return cr_id;
	}
	return NEW_CR;
}

