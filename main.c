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
#include <audio/microphone.h>
#include <audio_processing.h>
#include <spi_comm.h>

//Déclaration du bus
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

    /*Init du bus*/
    messagebus_init(&bus, &bus_lock, &bus_condvar);

    /*starts the serial communication*/
    serial_start();

    /*Init des capteurs de proximité*/
    proximity_start();
	/*Init des moteurs*/
	motors_init();
	/*start thread to process proximity sensors data*/
	process_sensors_start();

	/*wait 3s before calibration*/
    chThdSleepMilliseconds(3000);
    calibrate_ir();
	/*start the microphones thread*/
	mic_start(&processAudioData);
	spi_comm_start();


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
