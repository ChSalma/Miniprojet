#include "ch.h"
#include "hal.h"
#include <main.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <maze_mapping.h>
#include <constantes.h>
#include <Mouvements.h>
#include <audio/microphone.h>
#include <audio_processing.h>
#include <fft.h>
#include <arm_math.h>

//2 times FFT_SIZE because these arrays contain complex numbers (real + imaginary)
static float micFront_cmplx_input[2 * FFT_SIZE];
//Arrays containing the computed magnitude of the complex numbers
static float micFront_output[FFT_SIZE];

#define MIN_VALUE_THRESHOLD	10000 
#define MAX_COUNT_INDEX 4

#define MIN_FREQ			10	//we don't analyze before this index to not use resources for nothing
#define FREQ_DISCOVER		16	//250Hz
#define FREQ_RETURN_HOME	19	//296Hz
#define FREQ_GO_FPK			23	//359HZ
#define FREQ_NO_MODE		26	//406Hz
#define MAX_FREQ			30	//we don't analyze after this index to not use resources for nothing

#define FREQ_DISCOVER_L		(FREQ_DISCOVER-1)
#define FREQ_DISCOVER_H		(FREQ_DISCOVER+1)
#define FREQ_RETURN_HOME_L	(RETURN_HOME-1)
#define FREQ_RETURN_HOME_H	(RETURN_HOME+1)
#define FREQ_GO_FPK_L		(FREQ_GO_FPK-1)
#define FREQ_GO_FPK_H		(FREQ_GO_FPK+1)
#define FREQ_NO_MODE_L		(FREQ_NO_MODE-1)
#define FREQ_NO_MODE_H		(FREQ_NO_MODE+1)

static uint8_t count_index=0;
static uint16_t last_index=0;

void sound_remote(float* data)
{
	float max_norm = MIN_VALUE_THRESHOLD;
	int16_t max_norm_index = -1; 
	bool do_a_uturn;

	//search for the highest peak
	for(uint16_t i = MIN_FREQ ; i <= MAX_FREQ ; i++)
	{
		if(data[i] > max_norm){
			max_norm = data[i];
			max_norm_index = i;
		}
	}

	if (max_norm_index != last_index)
	{
		last_index=max_norm_index;
		count_index=0;
	}
	else
		count_index++;

	if (count_index == MAX_COUNT_INDEX)
	{
		//RETURN_HOME
		if(max_norm_index >= FREQ_RETURN_HOME_L && max_norm_index <= FREQ_RETURN_HOME_H)
		{
			do_a_uturn=maze_mapping_uturn_after_selecting_mode(RETURN_HOME);
		}
		//GO_FPK
		else if(max_norm_index >= FREQ_GO_FPK_L && max_norm_index <= FREQ_GO_FPK_H)
		{
			do_a_uturn=maze_mapping_uturn_after_selecting_mode(GO_FURTHEST_POINT_KNOWN);
		}
		//DISCOVER
		else if(max_norm_index >= FREQ_DISCOVER_L && max_norm_index <= FREQ_DISCOVER_H)
		{
			do_a_uturn=maze_mapping_uturn_after_selecting_mode(DISCOVER);
		}
		//NO_MODE
		else if(max_norm_index >= FREQ_NO_MODE_L && max_norm_index <= FREQ_NO_MODE_H)
		{
			do_a_uturn=maze_mapping_uturn_after_selecting_mode(NO_MODE_SELECTED);
		}
		else
			do_a_uturn=false;

		if(do_a_uturn)
			turn(HALF_TURN);
	}
}

void processAudioData(int16_t *data, uint16_t num_samples)
{
	static uint16_t nb_samples = 0;

	//loop to fill the buffers
	for(uint16_t i = 0 ; i < num_samples ; i+=4)
	{
		micFront_cmplx_input[nb_samples] = (float)data[i + MIC_FRONT];
		nb_samples++;

		micFront_cmplx_input[nb_samples] = 0;
		nb_samples++;

		//stop when buffer is full
		if(nb_samples >= (2 * FFT_SIZE))
		{
			break;
		}
	}

	if(nb_samples >= (2 * FFT_SIZE))
	{
		doFFT_optimized(FFT_SIZE, micFront_cmplx_input);
		arm_cmplx_mag_f32(micFront_cmplx_input, micFront_output, FFT_SIZE);
		nb_samples = 0;
		sound_remote(micFront_output);
	}
}
