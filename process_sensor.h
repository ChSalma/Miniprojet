/*
 * process_sensor.h
 *
 *  Created on: 4 avr. 2020
 *      Author: Salma Chatagny
 */

#ifndef PROCESS_SENSOR_H_
#define PROCESS_SENSOR_H_

#define FREE_WAY_RIGHT 10
#define FREE_WAY_LEFT 30

enum{FRONT_RIGHT, FRONT_RIGHT_45DEG, RIGHT_SENS, BACK_RIGHT, BACK_LEFT, LEFT_SENS, FRONT_LEFT_45DEG, FRONT_LEFT};

//Starts the proximity sensor thread
void process_sensors_start(void);

#endif /* PROCESS_SENSOR_H_ */
