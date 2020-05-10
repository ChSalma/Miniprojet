/*
 * maze_mapping.h
 *
 *  Created on: 16 avr. 2020
 *      Author: Aur�lien Goumaz
 */

#ifndef MAZE_MAPPING_H_
#define MAZE_MAPPING_H_

uint8_t maze_mapping_next_move(bool, bool, bool);
bool maze_mapping_do_a_uturn(void);
bool maze_mapping_mode_is_selected(void);
void maze_mapping_process_end_of_maze(void);
void maze_mapping_select_mode(uint8_t);

#endif /* MAZE_MAPPING_H_ */
