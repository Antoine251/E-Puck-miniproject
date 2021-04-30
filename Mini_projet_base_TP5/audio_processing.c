#include "ch.h"
#include "hal.h"
#include <main.h>
#include <usbcfg.h>
#include <chprintf.h>
#include <mailboxe.h>

#include <motors.h>
#include <audio/microphone.h>
#include <audio_processing.h>
#include <communications.h>
#include <fft.h>
#include <arm_math.h>
#include <arm_const_structs.h>

#define FREQ_MAX_SPEED      	512  //8000 Hz
#define FREQ_SPEED_NUL_MIN  	382  //5979 - 6031 Hz --> On avance tout droit sur une range de fréquence
#define FREQ_SPEED_NUL_MAX  	386
#define FREQ_MIN_SPEED      	256    //4000 Hz
#define FREQ_MIN_DETECT			128    //2000 Hz (limite inférieur de l'application qui émet les fréquences
#define THRESHOLD           	10000  //seuil de détection du son
#define MAX_CORRECTION_SPEED	(FREQ_MAX_SPEED - FREQ_SPEED_NUL_MAX)*COEF_CORRECTION  //step par seconde
#define COEF_CORRECTION			1
#define NBR_VALEUR_MOYENNE		10

//liste des fréquences seuiles pour la détection de l'intensité
#define FREQ_4000				268
#define FREQ_4400				294
#define FREQ_5000				330
#define FREQ_5200				342
#define FREQ_5600				368
#define FREQ_6200				407
#define FREQ_6600				433
#define FREQ_7200				472
#define FREQ_7800				511

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

static int16_t mean_intensity = 0;

static int16_t rotation_speed_left = 0;
static int16_t rotation_speed_right = 0;
static int16_t speed_intensity = 0;

//proto************************
void do_band_filter(float* mic_complex_input, float32_t * magn_buffer);

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
	static int32_t somme_max_valu = 0;
	int16_t max_valu = 0;
	static uint16_t compteurbis = 0;

	for(uint16_t i = 0; i < num_samples; i += 4) {
		micRight_cmplx_input[compteur] = data[i];
		micRight_cmplx_input[compteur+1] = 0;
		micLeft_cmplx_input[compteur] = data[i+1];
		micLeft_cmplx_input[compteur+1] = 0;
		micBack_cmplx_input[compteur] = data[i+2];
		micBack_cmplx_input[compteur+1] = 0;
		micFront_cmplx_input[compteur] = data[i+3];
		micFront_cmplx_input[compteur+1] = 0;
		if (max_valu < data[i+1]) {
			max_valu = data[i+1];
		}

		if (compteur == 2 * FFT_SIZE) {
			doFFT_optimized(FFT_SIZE, micLeft_cmplx_input);
			for (uint16_t i = 0; i == 1024; i += 1) {
				chprintf((BaseSequentialStream *)&SDU1, "valeur en entrée : \n [");
				chprintf((BaseSequentialStream *)&SDU1, " %d,", micLeft_cmplx_input[i]);
				chprintf((BaseSequentialStream *)&SDU1, "] \n");
			}
			arm_cmplx_mag_f32(micLeft_cmplx_input, micLeft_output, FFT_SIZE);
			compteur = 0;
			sem_ready = 1;

			//ici
			do_band_filter(micLeft_cmplx_input, micLeft_output);

			//calculer la moyenne de la valeur max sur NBR_VALEUR_CYCLE cycles
			somme_max_valu += max_valu;
			//chprintf((BaseSequentialStream *)&SDU1, "max value = %d; somme = %d \n", max_valu, somme_max_valu);
			++compteurbis;
			if(compteurbis == NBR_VALEUR_MOYENNE) {
				mean_intensity = somme_max_valu/NBR_VALEUR_MOYENNE;
				compteurbis = 0;
				somme_max_valu = 0;
				//chprintf((BaseSequentialStream *)&SDU1, "  mean valu = %d \n", mean_intensity);
			}
		} else {
			compteur += 2;
		}
	}

	// activate the semaphore
	if (sem_ready) { 		//&& (must_send == 8)) {
		chBSemSignal(&sendToComputer_sem);
		//must_send = 0;
		compute_motor_speed();
	}
//	} else {
//		if (sem_ready) {
//			must_send++;
//		}
//	}
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

//Détecte le pic en fréquence et associe une vitessea additionner à la valeur déterminée
//par l'intensité du son pour le moteur gauche et droite
void compute_motor_speed() {
	//uint16_t pic_haut = 0;
	//uint16_t pic_bas = 0;
	uint16_t pic_detect = 0;
	uint16_t max_value = 0;
	for(uint16_t  i = FREQ_MIN_DETECT; i < FREQ_MAX_SPEED; ++i) {
		if(micLeft_output[i] > max_value && micLeft_output[i] > THRESHOLD) {
			max_value = micLeft_output[i];
			pic_detect = i;
		}
	}
//		if(micLeft_output[i] > THRESHOLD) {
//			pic_bas = i;
//		}
//		if(micLeft_output[i] < THRESHOLD && pic_bas) {
//			pic_haut = i;
//			break;
//		}
//	}
//	if (pic_haut && pic_bas) {
//		pic_detect = (pic_haut + pic_bas)/2;
//	} else {
//		pic_detect = 0;
//	}
	//chprintf((BaseSequentialStream *)&SDU1, "max value = %d \n", max_value);

	if (pic_detect > FREQ_SPEED_NUL_MIN && pic_detect < FREQ_SPEED_NUL_MAX) {
		rotation_speed_left = 0;
		rotation_speed_right = 0;
	} else if (pic_detect < FREQ_SPEED_NUL_MIN && pic_detect > FREQ_MIN_SPEED) {
		rotation_speed_left = -(FREQ_SPEED_NUL_MIN - pic_detect)*COEF_CORRECTION;
		rotation_speed_right = (FREQ_SPEED_NUL_MIN - pic_detect)*COEF_CORRECTION;
	} else if (pic_detect > FREQ_SPEED_NUL_MAX && pic_detect < FREQ_MAX_SPEED) {
		rotation_speed_left = (pic_detect - FREQ_SPEED_NUL_MAX)*COEF_CORRECTION;
		rotation_speed_right = -(pic_detect - FREQ_SPEED_NUL_MAX)*COEF_CORRECTION;
	} else if(pic_detect < FREQ_MIN_SPEED && pic_detect > FREQ_MIN_DETECT) {
		rotation_speed_left = -MAX_CORRECTION_SPEED;
		rotation_speed_right = MAX_CORRECTION_SPEED;
	} else {
		rotation_speed_left = 0;
		rotation_speed_right = 0;
	}

	//uint16_t pic_detect_ = pic_detect*15.625;
	//chprintf((BaseSequentialStream *)&SDU1, " pic detect = %d \n", pic_detect_);
	compute_speed_intensity(pic_detect);

	//chprintf((BaseSequentialStream *)&SDU1, " pic detect = %d;moteur gauche = %d; moteur droite = %d \n", pic_detect_, rotation_speed_left, rotation_speed_right);

	//Poste les vitesses calculées dans la mailboxe
	msg_t motor_speed_left_correction = speed_intensity + rotation_speed_left;
	msg_t motor_speed_right_correction = speed_intensity + rotation_speed_right;
	mailbox_t * mail_boxe_ptr = get_mailboxe_micro_adr();
	chSysLock();
	chMBPostI(mail_boxe_ptr, motor_speed_left_correction);
	chMBPostI(mail_boxe_ptr, motor_speed_right_correction);
	chSysUnlock();

//	chSysLock();
//	size_t mailboxe_size = chMBGetUsedCountI(get_mailboxe_adr());
//	chSysUnlock();
//	//chprintf((BaseSequentialStream *)&SDU1, " adresse = %d; taille mailboxe = %d \n", mail_boxe_ptr, mailboxe_size);

	//left_motor_set_speed(rotation_speed_left);
	//right_motor_set_speed(rotation_speed_right);
}

void compute_speed_intensity(uint16_t freq) {
	int16_t thres_34 = 0;
	int16_t thres_45 = 0;
	int16_t thres_56 = 0;
	int16_t thres_67 = 0;
	int16_t thres_78 = 0;

	if (freq > FREQ_4000 && freq < FREQ_4400) {
		thres_34 = THRES_34_4044(freq*15.625);
		thres_45 = THRES_45_4044(freq*15.625);
		thres_56 = THRES_56_4044(freq*15.625);
		thres_67 = THRES_67_4044(freq*15.625);
		thres_78 = THRES_78_4044(freq*15.625);
	} else if (freq > FREQ_4400 && freq < FREQ_5000) {
		thres_34 = THRES_34_4450(freq*15.625);
		thres_45 = THRES_45_4450(freq*15.625);
		thres_56 = THRES_56_4450(freq*15.625);
		thres_67 = THRES_67_4450(freq*15.625);
		thres_78 = THRES_78_4450(freq*15.625);
	} else if (freq > FREQ_5000 && freq < FREQ_5200) {
		thres_34 = THRES_34_5052(freq*15.625);
		thres_45 = THRES_45_5052(freq*15.625);
		thres_56 = THRES_56_5052(freq*15.625);
		thres_67 = THRES_67_5052(freq*15.625);
		thres_78 = THRES_78_5052(freq*15.625);
	} else if (freq > FREQ_5200 && freq < FREQ_5600) {
		thres_34 = THRES_34_5256(freq*15.625);
		thres_45 = THRES_45_5256(freq*15.625);
		thres_56 = THRES_56_5256(freq*15.625);
		thres_67 = THRES_67_5256(freq*15.625);
		thres_78 = THRES_78_5256(freq*15.625);
	} else if (freq > FREQ_5600 && freq < FREQ_6200) {
		thres_34 = THRES_34_5662(freq*15.625);
		thres_45 = THRES_45_5662(freq*15.625);
		thres_56 = THRES_56_5662(freq*15.625);
		thres_67 = THRES_67_5662(freq*15.625);
		thres_78 = THRES_78_5662(freq*15.625);
	} else if (freq > FREQ_6200 && freq < FREQ_6600) {
		thres_34 = THRES_34_6266(freq*15.625);
		thres_45 = THRES_45_6266(freq*15.625);
		thres_56 = THRES_56_6266(freq*15.625);
		thres_67 = THRES_67_6266(freq*15.625);
		thres_78 = THRES_78_6266(freq*15.625);
	} else if (freq > FREQ_6600 && freq < FREQ_7200) {
		thres_34 = THRES_34_6672(freq*15.625);
		thres_45 = THRES_45_6672(freq*15.625);
		thres_56 = THRES_56_6672(freq*15.625);
		thres_67 = THRES_67_6672(freq*15.625);
		thres_78 = THRES_78_6672(freq*15.625);
	} else if (freq > FREQ_7200 && freq < FREQ_7800) {
		thres_34 = THRES_34_7278(freq*15.625);
		thres_45 = THRES_45_7278(freq*15.625);
		thres_56 = THRES_56_7278(freq*15.625);
		thres_67 = THRES_67_7278(freq*15.625);
		thres_78 = THRES_78_7278(freq*15.625);
	}

	if (mean_intensity > INTENSITY_MIN && mean_intensity < thres_34) {
		speed_intensity = SPEED_MARCHE_ARRIERE;
	} else if (mean_intensity > thres_34 && mean_intensity < thres_45) {
		speed_intensity = VITESSE_NUL;
	} else if (mean_intensity > thres_45 && mean_intensity < thres_56) {
		speed_intensity = SPEED_1;
	} else if (mean_intensity > thres_56 && mean_intensity < thres_67) {
		speed_intensity = SPEED_2;
	} else if (mean_intensity > thres_67 && mean_intensity < thres_78) {
		speed_intensity = SPEED_3;
	} else if (mean_intensity > thres_78) {
		speed_intensity = SPEED_4;
	}
	if (thres_78 == 0) {
		speed_intensity = VITESSE_NUL;       //aucune fréquence n'est jouée, les threshold ne sont pas définis
	}

	//chprintf((BaseSequentialStream *)&SDU1, "speed_intensity = %d \n", speed_intensity);
}

void do_band_filter(float* mic_complex_input, float32_t * magn_buffer){
	uint16_t max_value = 0;
	uint16_t pic_detect = 0;
	for(uint16_t  i = FREQ_MIN_DETECT; i < FREQ_MAX_SPEED; ++i) {
		if(magn_buffer[i] > max_value && magn_buffer[i] > THRESHOLD) {
			max_value = magn_buffer[i];
			pic_detect = i;
		}
	}



	//enleve les vals pas autour du pic
	for(uint16_t  i = 0; i < FREQ_MAX_SPEED; i+=2) {
		if(i < (pic_detect*2 - 10) || i > (pic_detect*2 + 10)){
			mic_complex_input[i] = 0;
			mic_complex_input[FFT_SIZE - i] = 0;
		}
	}

	//FFT reverse
	arm_cfft_f32(&arm_cfft_sR_f32_len1024, mic_complex_input, 1, 1);


	//
}


