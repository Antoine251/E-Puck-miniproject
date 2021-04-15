#include "ch.h"
#include "hal.h"
#include <main.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <motors.h>
#include <audio/microphone.h>
#include <audio_processing.h>
#include <communications.h>
#include <fft.h>
#include <arm_math.h>

//semaphore
static BSEMAPHORE_DECL(sendToComputer_sem, TRUE);

//2 times FFT_SIZE because these arrays contain complex numbers (real + imaginary)
static float micLeft_cmplx_input[2 * FFT_SIZE];
static float micRight_cmplx_input[2 * FFT_SIZE];
static float micFront_cmplx_input[2 * FFT_SIZE];
static float micBack_cmplx_input[2 * FFT_SIZE];
//Arrays containing the computed magnitude of the complex numbers
static float micLeft_output[FFT_SIZE];
static float micRight_output[FFT_SIZE];
static float micFront_output[FFT_SIZE];
static float micBack_output[FFT_SIZE];

/*
*	Callback called when the demodulation of the four microphones is done.
*	We get 160 samples per mic every 10ms (16kHz)
*	
*	params :
*	int16_t *data			Buffer containing 4 times 160 samples. the samples are sorted by micro
*							so we have [micRight1, micLeft1, micBack1, micFront1, micRight2, etc...]
*	uint16_t num_samples	Tells how many data we get in total (should always be 640)
*/
void processAudioData(int16_t *data, uint16_t num_samples){

	/*
	*
	*	We get 160 samples per mic every 10ms
	*	So we fill the samples buffers to reach
	*	1024 samples, then we compute the FFTs.
	*
	*/

	static uint16_t compteur = 0;
	uint8_t sem_ready = 0;
	static uint8_t must_send = 0;

	for(uint16_t i = 0; i < num_samples; i += 4) {
		micRight_cmplx_input[compteur] = data[i];
		micRight_cmplx_input[compteur+1] = 0;
		micLeft_cmplx_input[compteur] = data[i+1];
		micLeft_cmplx_input[compteur+1] = 0;
		micBack_cmplx_input[compteur] = data[i+2];
		micBack_cmplx_input[compteur+1] = 0;
		micFront_cmplx_input[compteur] = data[i+3];
		micFront_cmplx_input[compteur+1] = 0;

		if (compteur == 2 * FFT_SIZE){
			doFFT_optimized(FFT_SIZE, micLeft_cmplx_input);
			arm_cmplx_mag_f32(micLeft_cmplx_input, micLeft_output, FFT_SIZE);
			compteur = 0;
			sem_ready = 1;
		} else {
			compteur += 2;
		}
	}

	// activate the semaphore
	if (sem_ready && (must_send == 8)) {
		chBSemSignal(&sendToComputer_sem);
		//chprintf((BaseSequentialStream *)&SDU1, "test \n");
		must_send = 0;
		move_motor();
	} else {
		if (sem_ready) {
			must_send++;
		}
	}
	//chprintf((BaseSequentialStream *)&SDU1, ".");
	sem_ready = 0;

}


void wait_send_to_computer(void){
	chBSemWait(&sendToComputer_sem);
}

float* get_audio_buffer_ptr(BUFFER_NAME_t name){
	if(name == LEFT_CMPLX_INPUT){
		return micLeft_cmplx_input;
	}
	else if (name == RIGHT_CMPLX_INPUT){
		return micRight_cmplx_input;
	}
	else if (name == FRONT_CMPLX_INPUT){
		return micFront_cmplx_input;
	}
	else if (name == BACK_CMPLX_INPUT){
		return micBack_cmplx_input;
	}
	else if (name == LEFT_OUTPUT){
		return micLeft_output;
	}
	else if (name == RIGHT_OUTPUT){
		return micRight_output;
	}
	else if (name == FRONT_OUTPUT){
		return micFront_output;
	}
	else if (name == BACK_OUTPUT){
		return micBack_output;
	}
	else{
		return NULL;
	}
}
