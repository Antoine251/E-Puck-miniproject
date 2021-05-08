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

//#include <arm_const_structs.h>

#define FREQ_MAX_SPEED      	448   //7000 Hz
#define FREQ_SPEED_NUL_MIN  	334   //5218 - 5281 Hz --> On avance tout droit sur une range de fréquence
#define FREQ_SPEED_NUL_MAX  	338
#define FREQ_MIN_SPEED      	224   //3500 Hz
#define FREQ_MIN_DETECT			64    //2000 Hz
#define FREQ_MAX_DETECT			512   //8000 Hz (max de détection de la fréquence)
#define THRESHOLD           	10000  //seuil de détection du son
#define MAX_CORRECTION_SPEED	(FREQ_MAX_SPEED - FREQ_SPEED_NUL_MAX)*COEF_CORRECTION  //step par seconde
#define COEF_CORRECTION			2
#define NBR_VALEUR_MOYENNE		1

//liste des fréquences seuiles pour la détection de l'intensité
#define FREQ_3578				229
#define FREQ_3765				241
#define FREQ_4500				288
#define FREQ_4812				308
#define FREQ_5421				347
#define FREQ_5984				383
#define FREQ_6296				403
#define FREQ_6718				430
#define FREQ_7031				450

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
static uint16_t pic_detect = 0;
static uint16_t pic_detect_last_value = 0;
static uint8_t change_freq = 0;                //=1 si la fréquence est en train d'être modifié



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
	int32_t max_valu = 0;
	static uint16_t compteurbis = 0;

	for(uint16_t i = 0; i < num_samples; i += 4) {
		micRight_cmplx_input[compteur] = data[i];
		micRight_cmplx_input[compteur+1] = 0;
		micLeft_cmplx_input[compteur] = data[i+2];    //i+1
		micLeft_cmplx_input[compteur+1] = 0;
		micBack_cmplx_input[compteur] = data[i+2];
		micBack_cmplx_input[compteur+1] = 0;
		micFront_cmplx_input[compteur] = data[i+3];
		micFront_cmplx_input[compteur+1] = 0;

		if (compteur == 2 * FFT_SIZE) {



			doFFT_optimized(FFT_SIZE, micLeft_cmplx_input);
			arm_cmplx_mag_f32(micLeft_cmplx_input, micLeft_output, FFT_SIZE);

//			uint32_t valeur_max2 = 0;
			//chprintf((BaseSequentialStream *)&SDU1, "valeur en entree : \n [");
//			for (uint16_t j = 0; j < FFT_SIZE; j += 1) {
//				uint32_t valeur = micLeft_output[j];
//				//chprintf((BaseSequentialStream *)&SDU1, " %d,", valeur);
//				if (valeur > valeur_max2) {
//					valeur_max2 = valeur;
//					pic_detect2 = j*15.625;
//				}
//			}
			//chprintf((BaseSequentialStream *)&SDU1, "] \n");
			//chprintf((BaseSequentialStream *)&SDU1, "               pic = %d \n", pic_detect2);

			do_band_filter(micLeft_cmplx_input, micLeft_output);

			arm_cmplx_mag_f32(micLeft_cmplx_input, micLeft_output, FFT_SIZE);
			//FFT reverse
			doFFT_inverse_optimized(FFT_SIZE, micLeft_cmplx_input);

//			chprintf((BaseSequentialStream *)&SDU1, "valeur en sortie : \n [");
//			for (uint16_t j = 0; j < 2*FFT_SIZE; j += 2) {
//				int32_t valeur = micLeft_cmplx_input[j];
//				chprintf((BaseSequentialStream *)&SDU1, " %d,", valeur);
//			}
//			chprintf((BaseSequentialStream *)&SDU1, "] \n");

			compteur = 0;
			sem_ready = 1;

//			chprintf((BaseSequentialStream *)&SDU1, "valeur en sortie : \n [");
//			for (uint16_t j = 0; j <= 1024; j += 1) {
//				int16_t valeur = micLeft_cmplx_input[j];								//A decale !
//				chprintf((BaseSequentialStream *)&SDU1, " %d,", valeur);
//			}
//			chprintf((BaseSequentialStream *)&SDU1, "] \n");

			//Vérifie si la fréquence est en train d'être modifié de façon abrupte
			uint16_t delta_freq = abs(pic_detect - pic_detect_last_value);
			if(delta_freq > 2 ) {
				change_freq = 1;
			} else {
				change_freq = 0;
			}


			//calculer la moyenne de la valeur max sur NBR_VALEUR_CYCLE cycles
			for(uint16_t n = 0; n < 2*FFT_SIZE; n+=2) {
				if (max_valu < micLeft_cmplx_input[n]) {
					max_valu = micLeft_cmplx_input[n];
				}
			}
			somme_max_valu += max_valu;
			//chprintf((BaseSequentialStream *)&SDU1, "max value = %d; somme = %d \n", max_valu, somme_max_valu);
			++compteurbis;
			if(compteurbis == NBR_VALEUR_MOYENNE) {
				mean_intensity = somme_max_valu/NBR_VALEUR_MOYENNE;
				compteurbis = 0;
				somme_max_valu = 0;
				chprintf((BaseSequentialStream *)&SDU1, "  					mean valu = %d \n", mean_intensity);
//				uint16_t pic_detect_ = pic_detect*15.625;
//				chprintf((BaseSequentialStream *)&SDU1, " pic detect = %d \n", pic_detect_);
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
//	uint16_t pic_detect = 0;
//	uint16_t max_value = 0;
//	for(uint16_t  i = FREQ_MIN_DETECT; i < FREQ_MAX_SPEED; ++i) {
//		if(micLeft_output[i] > max_value && micLeft_output[i] > THRESHOLD) {
//			max_value = micLeft_output[i];
//			pic_detect = i;
//		}
//	}
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
	} else if (pic_detect < FREQ_MIN_SPEED && pic_detect > FREQ_MIN_DETECT) {
		rotation_speed_left = -MAX_CORRECTION_SPEED;
		rotation_speed_right = MAX_CORRECTION_SPEED;
	} else if (pic_detect > FREQ_MAX_SPEED && pic_detect < FREQ_MAX_DETECT) {
		rotation_speed_left = MAX_CORRECTION_SPEED;
		rotation_speed_right = -MAX_CORRECTION_SPEED;
	} else {
		rotation_speed_left = 0;
		rotation_speed_right = 0;
	}

//	uint16_t pic_detect_ = pic_detect*15.625;
//	chprintf((BaseSequentialStream *)&SDU1, " pic detect = %d \n", pic_detect_);
	compute_speed_intensity(pic_detect);

	chprintf((BaseSequentialStream *)&SDU1, " pic detect = %d;moteur gauche = %d; moteur droite = %d \n", pic_detect, rotation_speed_left, rotation_speed_right);

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
	uint16_t thres_24 = 0;
	uint16_t thres_46 = 0;
	uint16_t thres_68 = 0;

	if (change_freq == 0) {
		if (freq >= FREQ_3578 && freq < FREQ_3765) {
			thres_24 = THRES_24_3537(freq*15.625);
			thres_46 = THRES_46_3537(freq*15.625);
			thres_68 = THRES_68_3537(freq*15.625);
		} else if (freq >= FREQ_3765 && freq < FREQ_4500) {
			thres_24 = THRES_24_3745(freq*15.625);
			thres_46 = THRES_46_3745(freq*15.625);
			thres_68 = THRES_68_3745(freq*15.625);
		} else if (freq >= FREQ_4500 && freq < FREQ_4812) {
			thres_24 = THRES_24_4548(freq*15.625);
			thres_46 = THRES_46_4548(freq*15.625);
			thres_68 = THRES_68_4548(freq*15.625);
		} else if (freq >= FREQ_4812 && freq < FREQ_5421) {
			thres_24 = THRES_24_4854(freq*15.625);
			thres_46 = THRES_46_4854(freq*15.625);
			thres_68 = THRES_68_4854(freq*15.625);
		} else if (freq >= FREQ_5421 && freq < FREQ_5984) {
			thres_24 = THRES_24_5459(freq*15.625);
			thres_46 = THRES_46_5459(freq*15.625);
			thres_68 = THRES_68_5459(freq*15.625);
		} else if (freq >= FREQ_5984 && freq < FREQ_6296) {
			thres_24 = THRES_24_5962(freq*15.625);
			thres_46 = THRES_46_5962(freq*15.625);
			thres_68 = THRES_68_5962(freq*15.625);
		} else if (freq >= FREQ_6296 && freq < FREQ_6718) {
			thres_24 = THRES_24_6267(freq*15.625);
			thres_46 = THRES_46_6267(freq*15.625);
			thres_68 = THRES_68_6267(freq*15.625);
		} else if (freq >= FREQ_6718 && freq <= FREQ_7031) {
			thres_24 = THRES_24_6770(freq*15.625);
			thres_46 = THRES_46_6770(freq*15.625);
			thres_68 = THRES_68_6770(freq*15.625);
		}

		if (mean_intensity > INTENSITY_MIN && mean_intensity < thres_24) {
			speed_intensity = SPEED_MARCHE_ARRIERE;
		} else if (mean_intensity > thres_24 && mean_intensity < thres_46) {
			speed_intensity = VITESSE_NUL;
		} else if (mean_intensity > thres_46 && mean_intensity < thres_68) {
			speed_intensity = SPEED_1;
		} else if (mean_intensity > thres_68) {
			speed_intensity = SPEED_2;
		}

		if (thres_24 == 0) {
			speed_intensity = VITESSE_NUL;       //aucune fréquence n'est jouée, les threshold ne sont pas définis
		}
	}

	chprintf((BaseSequentialStream *)&SDU1, "speed_intensity = %d \n", speed_intensity);
}

void do_band_filter(float * mic_complex_input, float32_t * magn_buffer){
	uint32_t max_value = 0;
	pic_detect_last_value = pic_detect;
	pic_detect = 0;
	for(uint16_t  i = FREQ_MIN_DETECT; i < FREQ_MAX_DETECT; ++i) {
		if (magn_buffer[i] > max_value && magn_buffer[i] > THRESHOLD) {
			max_value = magn_buffer[i];
			pic_detect = i;
		}
	}

	//enleve les vals pas autour du pic
	for(uint16_t  i = 0; i < 2*FFT_SIZE; i+=2) {
		//if(i < (pic_detect*2 - 1) || i > (pic_detect*2 + 1)){
		if(i != pic_detect*2) {
			mic_complex_input[i] = 0;
			mic_complex_input[i+1] = 0;
		}
	}
}


