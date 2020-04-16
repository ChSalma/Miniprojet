/*
 * maze_mapping.h
 *
 *  Created on: 16 avr. 2020
 *      Author: afgou
 */

#ifndef MAZE_MAPPING_H_
#define MAZE_MAPPING_H_

#include <stdint.h>

typedef struct Crossroad
{
	uint8_t chosen_path;
	float cr_pos_x, cr_pos_y;
} CROSSROAD;



#endif /* MAZE_MAPPING_H_ */
