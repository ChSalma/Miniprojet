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

#define IR0 0

//Je suis pas sûre de ces lignes, copier du main de chibi--> m'a permis d'éviter l'erreur de thd dans proximity.c
messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

int main(void)
{

    halInit();
    chSysInit();
    mpu_init();

    //Init des capteurs de proximité
//	  proximity_start();
//    calibrate_ir();

	//Init les moteurs
	motors_init();
	right_motor_set_speed(600);
	left_motor_set_speed(600);

	//start the threads for the pi regulator and the proximity sensors process
//	pi_regulator_start();
//	process_sensors_start();

	//init de test values
	int test_value[2];

    /* Infinite loop. */
    while (1) {
    	//test des valeurs reçues par proximity
//    	test_value[0] = get_prox(IR0);
//    	test_value[1] = get_calibrated_prox(IR0);
//    	chprintf((BaseSequentialStream *)&SD3, "prox = %d \n", test_value[0]);
//    	chprintf((BaseSequentialStream *)&SD3, "calibrated_prox = %d \n", test_value[1]);

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
