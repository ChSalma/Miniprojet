#include "ch.h"
#include "hal.h"
#include <stdbool.h>

#include <main.h>
#include <regulator.h>
#include <constantes.h>
#include <mouvements.h>

#define KP 1.1 //Régulation PD entre deux murs
#define KD 0.5
#define KP_FW 0.9//Régulation PD par rapport à un seul mur
#define KD_FW 0.3

#define MAX_DIFF 30
#define THRESHOLD_45_DEG 250
//Pour corriger le fait que RIGHT_SENS donne des valeurs
//plus élevées que LEFT_SENS pour une distance donnée
//on définit un offset
#define OFFSET 20

static float last_difference = 0;

void regulator_difference(int front_right_45deg_value, int front_left_45deg_value, int front_right_value, int front_left_value)
{
    /*Cette fonction est appelée uniquement quand le robot est entre deux murs.
     * On utilise les capteurs avant 45deg autant que possible car ils permettent une meilleure anticipation de
     * l'environnment. Par contre, il est imperatif de ne plus les utiliser lorsqu'ils ne captent plus les murs
     * entourant le robot car le robot risquerait de dévier de sa trajectoire. C'est pourquoi, on repasse à
     * une régulation avec les capteurs latéraux dans ce cas là.*/

    int16_t right_speed, left_speed;
    int16_t difference, derivate;
	if ((front_right_45deg_value>THRESHOLD_45_DEG)&&(front_left_45deg_value>THRESHOLD_45_DEG))
	{
		difference = front_right_45deg_value-front_left_45deg_value;
	}
	else
		difference = front_right_value-front_left_value-OFFSET;
	/*Pour éviter des corrections angulaires trop fortes, qui pourrait faire dévier le robot
	 * de sa trajectoire principale, on définit un maximum.*/
	if(difference > MAX_DIFF)
		difference = MAX_DIFF;
	if(difference < -MAX_DIFF)
		difference = -MAX_DIFF;

	derivate = difference-last_difference;

    //REGULATION PD//
	right_speed = get_right_speed()+ KP*difference + KD*derivate;
	left_speed = get_left_speed()-KP*difference-KD*derivate;

	set_speed(right_speed, left_speed);
	last_difference = difference;
}

void regulator_follow_wall(int reference_value, int current_value, int sensor_id)
{
    int16_t right_speed, left_speed;
    int16_t difference, derivate;

    difference=reference_value-current_value;

	if (sensor_id==RIGHT_SENS)
		difference = -difference+OFFSET;

	if(difference > MAX_DIFF)
		difference = MAX_DIFF;
	if(difference < -MAX_DIFF)
		difference = -MAX_DIFF;

	derivate = difference-last_difference;

	//REGULATION PD//
	right_speed = get_right_speed() + KP_FW*difference + KD_FW*derivate;
	left_speed = get_left_speed()-KP_FW*difference - KD_FW*derivate;

	set_speed(right_speed, left_speed);
	last_difference = difference;
}
