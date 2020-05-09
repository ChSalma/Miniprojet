#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <regulator.h>
#include <constantes.h>
#include <mouvements.h>

#define KP 1.4 //Régulation PD entre deux murs
#define KD 0.7
#define KP_FW 1 //Régulation PD par rapport à un seul mur
#define KD_FW 1

#define MAX_DIFF 30
#define THRESHOLD_45_DEG 250
//Pour corriger le fait que RIGHT_SENS donne des valeurs
//plus élevées que LEFT_SENS pour une distance donnée
//on définit un offset
#define OFFSET 20

enum{DIFFERENCE, FOLLOW_WALL};

static float last_difference = 0;

//Fonction interne
void regulator_pd(int16_t difference, uint8_t regulation_type)
{
	int16_t right_speed, left_speed, derivate;

	/*Pour éviter des corrections angulaires trop fortes, qui pourrait faire dévier le robot
	 * de sa trajectoire principale, on définit un maximum.*/
	if(difference > MAX_DIFF)
		difference = MAX_DIFF;
	if(difference < -MAX_DIFF)
		difference = -MAX_DIFF;

	derivate = difference-last_difference;

	//REGULATION PD//
	if (regulation_type==DIFFERENCE)
	{
		right_speed = get_right_speed() + KP*difference + KD*derivate; 	//Cette méthode consomme moins de mémoire que
		left_speed = get_left_speed() - KP*difference - KD*derivate;	//de créer deux variables float que l'on passe
	}																	//comme arguments (p_coeff, d_coeff) pour les
	else																//bons KP et KD
	{
		right_speed = get_right_speed() + KP_FW*difference  + KD_FW*derivate;
		left_speed = get_left_speed() - KP_FW*difference  - KD_FW*derivate;
	}

	set_speed(right_speed, left_speed);
	last_difference = difference;
}

//Fonctions publiques
void regulator_difference(int front_right_45deg_value, int front_left_45deg_value, int front_right_value, int front_left_value)
{
    /*Cette fonction est appelée uniquement quand le robot est entre deux murs.
     * On utilise les capteurs avant 45deg autant que possible car ils permettent une meilleure anticipation de
     * l'environnment. Par contre, il est imperatif de ne plus les utiliser lorsqu'ils ne captent plus les murs
     * entourant le robot car le robot risquerait de dévier de sa trajectoire. C'est pourquoi, on repasse à
     * une régulation avec les capteurs latéraux dans ce cas là.*/

    int16_t difference;

    if ((front_right_45deg_value>THRESHOLD_45_DEG)&&(front_left_45deg_value>THRESHOLD_45_DEG))
	{
		difference = front_right_45deg_value-front_left_45deg_value;
	}
	else
		difference = front_right_value-front_left_value-OFFSET;

    regulator_pd(difference, DIFFERENCE);
}

void regulator_follow_wall(int reference_value, int current_value, int sensor_id)
{
    int16_t difference;

    difference=reference_value-current_value;

	if (sensor_id==FRONT_RIGHT_45DEG)
		difference = -difference+OFFSET;

	regulator_pd(difference, FOLLOW_WALL);
}
