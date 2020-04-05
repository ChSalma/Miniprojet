#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <main.h>
//#include "proximity.h"
#include <process_sensor.h>


int main(void)
{

    halInit();
    chSysInit();
    mpu_init();

    //Init des capteurs de proximité
    proximity_start();
    calibrate_ir();

	//Init les moteurs
	motors_init();

	//start the threads for the pi regulator and the proximity sensors process
//	pi_regulator_start();
	process_sensors_start();


    /* Infinite loop. */
    while (1) {
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
