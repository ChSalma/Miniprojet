/*
 * Mouvements.h
 *
 *  Created on: 16 avr. 2020
 *      Author: Salma Chatagny
 */

#ifndef MOUVEMENTS_H_
#define MOUVEMENTS_H_

#define LOW_SPEED 200
#define HIGH_SPEED 600

#define TURN_RIGHT	-90 //[degrees]
#define TURN_LEFT	90 //[degrees]
#define HALF_TURN	180 //[degrees]
#define ADJUST_RIGHT -10 //[degrees]
#define ADJUST_LEFT 10 //[degrees]

void turn(int angle); //angle [degrees]
void go_slow(void);
void go_fast(void);
void stop(void); //possibilité de donner un temps en entrée?
void go_for_distance(int distance); //distance [cm]

#endif /* MOUVEMENTS_H_ */
