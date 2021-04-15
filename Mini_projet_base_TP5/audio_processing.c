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

#define FREQ_MAX_SPEED      	512  //8000 Hz
#define FREQ_SPEED_NUL_MIN  	272  //4250 - 4750 Hz --> On avance tout droit sur une range de fréquence
#define FREQ_SPEED_NUL_MAX  	304
#define FREQ_MIN_SPEED      	64   //1000 Hz
#define THRESHOLD           	30000 //seuil de détection du son
#define MAX_CORRECTION_SPEED	624 //step par seconde
#define COEF_CORRECTION			3

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

uint16_t rotation_speed_left = 0;   //coef entre 0 et 1 muiltiplié par 100 pour travailler avec des int
uint16_t rotation_speed_right = 0;  //appliqué a la vitesse du moteur pour la rotation de l'EPUCK

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
		must_send = 0;
		compute_motor_speed();
	} else {
		if (sem_ready) {
			must_send++;
		}
	}
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

//Détecte le pic en fréquence ansi que son intensité et associe une vitesse
//pour le moteur gauche et droite
void compute_motor_speed() {
	uint16_t pic_haut = 0;
	uint16_t pic_bas = 0;
	uint16_t pic_detect = 0;
	for(uint16_t  i = FREQ_MIN_SPEED; i < FREQ_MAX_SPEED; ++i) {
		if(micLeft_output[i] > THRESHOLD) {
			pic_haut = i;
		}
		if(micLeft_output[i] < THRESHOLD && pic_haut) {
			pic_bas = i;
			break;
		}
	}
	if (pic_haut && pic_bas) {
		pic_detect = (pic_haut + pic_bas)/2;
	} else {
		pic_detect = 0;
	}

	if (pic_detect > FREQ_SPEED_NUL_MIN && pic_detect < FREQ_SPEED_NUL_MAX) {
		rotation_speed_left_coef = 0;
		rotation_speed_right_coef = 0;
	} else if (pic_detect < FREQ_SPEED_NUL_MIN && pic_detect > FREQ_MIN_SPEED) {
		rotation_speed_left = -(FREQ_SPEED_NUL_MIN - pic_detect)*COEF_CORRECTION;
		rotation_speed_right = (FREQ_SPEED_NUL_MIN - pic_detect)*COEF_CORRECTION;
	} else if (pic_detect > FREQ_SPEED_NUL_MAX && pic_detect < FREQ_MAX_SPEED) {
		rotation_speed_left = (pic_detect - FREQ_SPEED_NUL_MAX)*COEF_CORRECTION;
		rotation_speed_right = -(pic_detect - FREQ_SPEED_NUL_MAX)*COEF_CORRECTION;
	} else {
		rotation_speed_left = 0;
		rotation_speed_right = 0;
	}
}
