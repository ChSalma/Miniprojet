/*
 * maze_mapping.h
 *
 *  Created on: 16 avr. 2020
 *      Author: afgou
 */

#ifndef MAZE_MAPPING_H_
#define MAZE_MAPPING_H_

uint8_t maze_mapping_next_move(bool, bool, bool);
bool maze_mapping_uturn_after_selecting_mode(uint8_t);
bool maze_mapping_mode_is_selected(void);

#endif /* MAZE_MAPPING_H_ */
