#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <main.h>
#include <msgbus/messagebus.h>
#include <motors.h>
#include <sensors/proximity.h>
#include <process_sensor.h>
#include <chprintf.h>

//define pour la conversion en mm
#define DIV_FACTOR_DIST 16
#define OFFSET_DIST 54

//Je suis pas s�re de ces lignes, copier du main de chibi--> m'a permis d'�viter l'erreur de thd dans proximity.c
//D�claration du bus
messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

static void serial_start(void)
{
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.
}

int main(void)
{

    halInit();
    chSysInit();
    mpu_init();

    //Init du bus
    messagebus_init(&bus, &bus_lock, &bus_condvar);
    //starts the serial communication
    serial_start();

    //Init des capteurs de proximit�
    proximity_start();
    calibrate_ir();

	//Init les moteurs
	motors_init();
	right_motor_set_speed(600);
	left_motor_set_speed(600);

	//start the threads for the pi regulator and the proximity sensors process
//	pi_regulator_start();
//	process_sensors_start();

	//init de test values
	int test_value[2];
	int i;

    /* Infinite loop. */
    while (1) {
//    	//test des valeurs re�ues par proximity
//    	for(i = 0; i < PROXIMITY_NB_CHANNELS; i++)
//    	{
//			test_value[0] = get_prox(i);
//			test_value[1] = get_calibrated_prox(i);
//			chprintf((BaseSequentialStream *)&SD3, "IR%d \n", i+1);
//			chprintf((BaseSequentialStream *)&SD3, "prox = %d \n", test_value[0]);
//			chprintf((BaseSequentialStream *)&SD3, "calibrated_prox = %d \n", test_value[1]);
//    	}//conclusion distance[mm] = -0.0625*prox + 54 = -x/16 +54 ??
//    	//test de calculs de distances
//    	for(i = 0; i < PROXIMITY_NB_CHANNELS; i++)
//    	{
//    		test_value[0] = get_prox(i);
//    		test_value[1] = (-test_value[0])/DIV_FACTOR_DIST + OFFSET_DIST;
//    		chprintf((BaseSequentialStream *)&SD3, "IR%d \n", i+1);
//    		chprintf((BaseSequentialStream *)&SD3, "dist[mm] = %d \n", test_value[1]);
//    	}
    	//waits 1 second
        chThdSleepMilliseconds(1000);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
