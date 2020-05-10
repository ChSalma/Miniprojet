/*
 * constantes.h
 *
 *  Created on: 1 mai 2020
 *      Author: Salma Chatagny
 */

#ifndef CONSTANTES_H_
#define CONSTANTES_H_

//mouvements
#define LOW_SPEED 200
#define HIGH_SPEED 500
#define ULTRA_HIGH_SPEED 1000

#define FULL_TURN	360
#define TURN_RIGHT	-90 //[degrees]
#define TURN_LEFT	90 //[degrees]
#define HALF_TURN	180 //[degrees]
#define ADJUST_RIGHT -10 //[degrees]
#define ADJUST_LEFT 10 //[degrees]

//proximity_sensors
#define FREE_WAY_FRONT 350//un chemin est considéré comme tel si l'on a une valeur inférieure à celle-ci
#define FREE_WAY_RIGHT 250
#define FREE_WAY_LEFT 350
enum{FRONT_RIGHT, FRONT_RIGHT_45DEG, RIGHT_SENS, BACK_RIGHT, BACK_LEFT, LEFT_SENS, FRONT_LEFT_45DEG, FRONT_LEFT};

//maze_mapping
enum {NO_MODE_SELECTED, DISCOVER, RETURN_HOME, GO_FURTHEST_POINT_KNOWN};
enum {KEEP_GOING, GO_RIGHT, GO_FORWARD, GO_LEFT, U_TURN, DONT_MOVE};

#endif /* CONSTANTES_H_ */
