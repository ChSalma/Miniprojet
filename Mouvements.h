/*
 * Mouvements.h
 *
 *  Created on: 16 avr. 2020
 *      Author: Salma Chatagny
 */

#ifndef MOUVEMENTS_H_
#define MOUVEMENTS_H_

void turn(int angle); //angle [degrees]
void go_slow(void);
void go_fast(void);
void set_speed (int right_motor_new_speed, int left_motor_new_speed);
void stop(void); //possibilité de donner un temps en entrée?
void go_for_distance(int distance); //distance [cm]
int get_right_speed(void);
int get_left_speed(void);

#endif /* MOUVEMENTS_H_ */
