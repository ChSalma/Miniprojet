#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>
#include <stdbool.h>

#include <main.h>
#include <Mouvements.h>
#include <regulator.h>
#include <constantes.h>
#include <sensors/proximity.h>
#include <maze_mapping.h>

#define KP 1
#define KD 0.6
#define MAX_DIFF 30
#define MAX_DERIV 100
#define THRESHOLD_45_DEG 100
#define THRESHOLD_RIGHT 100
#define THRESHOLD_LEFT 100
#define OFFSET 20 //car les capteurs gauche et droite ne donne pas exactement les mêmes valeurs pour une même distance à un obstacle
static THD_WORKING_AREA(waRegulator, 256);
static THD_FUNCTION(Regulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;


    int16_t right_speed, left_speed;
    int16_t difference=0;
    float derivate=0;
    static float last_difference = 0;

    while(1)
    {
    	if(maze_mapping_mode_is_selected())
    	{
    	    if ((get_calibrated_prox(LEFT_SENS)>THRESHOLD_LEFT)&&
    	    	(get_calibrated_prox(RIGHT_SENS)>THRESHOLD_RIGHT)&&
				(get_calibrated_prox(FRONT_RIGHT_45DEG)>THRESHOLD_45_DEG)&&
				(get_calibrated_prox(FRONT_LEFT_45DEG)>THRESHOLD_45_DEG))
    	    {
    	    	difference = get_calibrated_prox(FRONT_RIGHT_45DEG)-get_calibrated_prox( FRONT_LEFT_45DEG)-OFFSET; //-OFFSET
    	    	if(difference > MAX_DIFF)
    	    		difference = MAX_DIFF;
    	    	if(difference < -MAX_DIFF)
    	    		difference = -MAX_DIFF;
    	    	//we set a maximum and a minimum for the sum to avoid an uncontrolled growth
    	    	derivate = difference-last_difference;
    	    	if(derivate > MAX_DERIV){
    	    		derivate = MAX_DERIV;
    	    	}else if(derivate < -MAX_DERIV){
    	    		derivate = -MAX_DERIV;
    	    	}
    			//d'abord remettre la même vitesse aux deucc moteurs puis corrigé pour éviter explosion de correction
    	    	right_speed = (get_right_speed() + get_left_speed())/2; //moyenne de la vitesse
    	    	left_speed = right_speed-KP*difference+KD*derivate;
    	    	right_speed += KP*difference - KD*derivate;

    	    	set_speed(right_speed, left_speed);
    	    	last_difference = difference;
    	    }
    	}

        //50Hz
        chThdSleepMilliseconds(20);
    }
}

void regulator_start(void){
	chThdCreateStatic(waRegulator, sizeof(waRegulator), NORMALPRIO, Regulator, NULL);
}
