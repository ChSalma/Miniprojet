/*
 * Mouvements.h
 *
 *  Created on: 16 avr. 2020
 *      Author: Salma Chatagny
 */

#ifndef MOUVEMENTS_H_
#define MOUVEMENTS_H_

#define LOW_SPEED 300
#define HIGH_SPEED 600

#define TURN_RIGHT	-90
#define TURN_LEFT	90
#define HALF_TURN	180
#define ADJUST_RIGHT -10
#define ADJUST_LEFT 10

typedef struct robot
{
	float pos_x, pos_y, angle;
} ROBOT;

void turn(int angle);
void go_slow(void);
void go_fast(void);

#endif /* MOUVEMENTS_H_ */
