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
#define MAX_DIFF 30

static THD_WORKING_AREA(waRegulator, 256);
static THD_FUNCTION(Regulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;


    int16_t right_speed, left_speed;
    int16_t difference=0;

    while(1)
    {
    	if(maze_mapping_mode_is_selected())
    	{
    	    if ((get_calibrated_prox(LEFT_SENS)>FREE_WAY_LEFT)&&(get_calibrated_prox(RIGHT_SENS)>FREE_WAY_RIGHT))
    	    {
    	    	difference = get_calibrated_prox(RIGHT_SENS)-get_calibrated_prox(LEFT_SENS); //-OFFSET
    	    	if(difference > MAX_DIFF)
    	    		difference = MAX_DIFF;
    	    	if(difference < -MAX_DIFF)
    	    		difference = -MAX_DIFF;
    			//d'abord remettre la m�me vitesse aux deucc moteurs puis corrig� pour �viter explosion de correction
    	    	right_speed = (get_right_speed() + get_left_speed())/2; //moyenne de la vitesse
    	    	left_speed = right_speed-KP*difference;
    	    	right_speed += KP*difference;

    	    	set_speed(right_speed, left_speed);
    	    }
    	}

        //50Hz
        chThdSleepMilliseconds(20);
    }
}

void regulator_start(void){
	chThdCreateStatic(waRegulator, sizeof(waRegulator), NORMALPRIO, Regulator, NULL);
}