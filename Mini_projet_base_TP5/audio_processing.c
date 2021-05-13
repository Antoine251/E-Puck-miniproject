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
#include <leds.h>

#define FREQ_MAX_SPEED      	448   		//7000 Hz
#define FREQ_SPEED_NUL_MIN  	334   		//5218 - 5281 Hz --> On avance tout droit sur une range de fréquence
#define FREQ_SPEED_NUL_MAX  	338
#define FREQ_MIN_SPEED      	224   		//3500 Hz
#define FREQ_MIN_DETECT			64    		//2000 Hz
#define FREQ_MAX_DETECT			512   		//8000 Hz (max de détection de la fréquence)
#define THRESHOLD           	10000  		//seuil de détection du son
#define MAX_CORRECTION_SPEED	(FREQ_MAX_SPEED - FREQ_SPEED_NUL_MAX)*COEF_CORRECTION  //step par seconde
#define COEF_CORRECTION			2			//coefficient multiplicateur pour la rotation du robot
#define BACK_MIC 				2

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

//2 times FFT_SIZE because these arrays contain complex numbers (real + imaginary)
static float mic_cmplx_input[2 * FFT_SIZE];

//Arrays containing the computed magnitude of the complex numbers
static float mic_output[FFT_SIZE];

static int16_t rotation_speed_left = 0;
static int16_t rotation_speed_right = 0;
static int16_t speed_intensity = 0;
static int16_t speed_intensity_last_value = 0;
static int16_t speed_intensity_2_last_value = 0;
static uint8_t change_freq = 0;                //=1 si la fréquence est en train d'être modifié


/***************************PROTOTYPE OF INTERNAL FUNCTIONS************************************/

void do_band_filter(float* mic_complex_input, uint16_t pic_detect);
uint8_t perturbation(void);
uint16_t compute_frequency(float32_t * mic_output);
void compute_rotation_speed(uint16_t pic_detect);
void compute_speed_intensity(uint16_t freq, int32_t mesured_intensity);
void compute_motor_speed(uint16_t pic_detect, int32_t mesured_intensity);

/***************************DECLARATION OF INTERNAL FUNCTIONS************************************/

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
	*	We get 160 samples per mic every 10ms
	*	So we fill the samples buffers to reach
	*	1024 samples, then we compute the FFTs.
	*/
	static uint16_t compteur = 0;
	int32_t max_intensity = 0;

	static uint16_t pic_detect = 0;
	static uint16_t pic_detect_last_value = 0;


	for(uint16_t i = 0; i < num_samples; i += 4) {
		mic_cmplx_input[compteur] = data[i+BACK_MIC];
		mic_cmplx_input[compteur+1] = 0;

		compteur += 2;

		if (compteur >= (2 * FFT_SIZE)) {

			doFFT_optimized(FFT_SIZE, mic_cmplx_input);
			arm_cmplx_mag_f32(mic_cmplx_input, mic_output, FFT_SIZE);

			pic_detect_last_value = pic_detect;    //Sauvegarde la valeur de la fréquence de l'entrée précédente
			pic_detect = compute_frequency(mic_output);

			do_band_filter(mic_cmplx_input, pic_detect);

			//FFT reverse
			doFFT_inverse_optimized(FFT_SIZE, mic_cmplx_input);

			//Vérifie si la fréquence est en train d'être modifié de façon abrupte
			uint16_t delta_freq = abs(pic_detect - pic_detect_last_value);
			if(delta_freq > 1 ) {
				change_freq = 1;
			} else {
				change_freq = 0;
			}

			if (pic_detect > FREQ_MIN_SPEED && pic_detect < FREQ_MAX_SPEED) {
				set_body_led(1);
			} else {
				set_body_led(0);
			}

			//calculer la moyenne de la valeur max sur NBR_VALEUR_CYCLE cycles
			for(uint16_t n = 0; n < 2*FFT_SIZE; n+=2) {
				if (max_intensity < mic_cmplx_input[n]) {
					max_intensity = mic_cmplx_input[n];
				}
			}

			uint16_t pic_detect_ = pic_detect*15.625;
			chprintf((BaseSequentialStream *)&SDU1, "  			                  		mean valu = %d         ; freq = %d \n", max_intensity, pic_detect_);

			compteur = 0;  //On rempli une nouvelle fois le buffer d'entré
			compute_motor_speed(pic_detect, max_intensity);
		}
	}
}

/*
*	Calcul la fréquence jouée par l'enceinte
*
*	params :
*	float32_t * mic_output			Norme de la FFT calculée !
*/
uint16_t compute_frequency(float32_t * mic_output) {
	uint32_t max_value = 0;
	uint16_t freq = 0;
	for(uint16_t  i = FREQ_MIN_DETECT; i < FREQ_MAX_DETECT; ++i) {
		if (mic_output[i] > max_value && mic_output[i] > THRESHOLD) {
			max_value = mic_output[i];
			freq = i;
		}
	}
	return freq;
}

/*
*	Associe à la fréquence calculée une vitesse de rotation ; calcul la vitesse du robot en fonction de l'intensité mesurée
*	Calcul la vitesse du robot en fonction de l'intensité mesurée
*	Additionne les deux termes de la vitesse et poste les valeurs dans la mailboxe
*
*	params :
*	uint16_t pic_detect			Fréquence mesurée
*	int32_t mesured_intensity	Intensité mesurée en entrée du micro après le filtrage
*/
void compute_motor_speed(uint16_t pic_detect, int32_t mesured_intensity) {

	compute_rotation_speed(pic_detect);

	compute_speed_intensity(pic_detect, mesured_intensity);

	//chprintf((BaseSequentialStream *)&SDU1, " pic detect = %d;moteur gauche = %d; moteur droite = %d \n", pic_detect, rotation_speed_left, rotation_speed_right);

	//Poste les vitesses calculées dans la mailboxe
	msg_t motor_speed_left_correction = speed_intensity + rotation_speed_left;
	msg_t motor_speed_right_correction = speed_intensity + rotation_speed_right;
	mailbox_t * mail_boxe_ptr = get_mailboxe_micro_adr();
	chSysLock();
	chMBPostI(mail_boxe_ptr, motor_speed_left_correction);
	chMBPostI(mail_boxe_ptr, motor_speed_right_correction);
	chSysUnlock();
}


/*
*	Calcul la vitesse de rotation qui correspond à la fréquence mesurée
*
*	params :
*	uint16_t pic_detect  		fréquence jouée
*/
void compute_rotation_speed(uint16_t pic_detect) {
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
}

/*
*	Calcul la vitesse en ligne droite associée à la fréquence et l'intensité mesurées
*
*	params :
*	uint16_t freq  					fréquence jouée
*	int32_t mesured_intensity		intensité mesurée
*/
void compute_speed_intensity(uint16_t freq, int32_t mesured_intensity) {
	int16_t old_intensity_v2 = 0;
	uint16_t thres_24 = 0;
	uint16_t thres_46 = 0;
	uint16_t thres_68 = 0;
	uint8_t fix_intensity = 0;	//variable booléenne pour savoir si la vitesse a été gelé (=1) ou non(=0)

	if (change_freq == 0) {										// La fréquence n'est pas en train d'être modifié,
		fix_intensity = 0;										// on mesure les thresholds en fonction de la fréqence
		if (freq >= FREQ_3578 && freq < FREQ_3765) {			// et on associe à notre intensité une vitesse
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

		if (mesured_intensity > INTENSITY_MIN && mesured_intensity < thres_24) {
			speed_intensity = SPEED_MARCHE_ARRIERE;
		} else if (mesured_intensity > thres_24 && mesured_intensity < thres_46) {
			speed_intensity = VITESSE_NUL;
		} else if (mesured_intensity > thres_46 && mesured_intensity < thres_68) {
			speed_intensity = SPEED_1;
		} else if (mesured_intensity > thres_68) {
			speed_intensity = SPEED_2;
		}

		if (thres_24 == 0) {
			speed_intensity = VITESSE_NUL;       //aucune fréquence n'est jouée, les threshold ne sont pas définis
		}

		old_intensity_v2 = speed_intensity_last_value;
		if (perturbation()) {
			speed_intensity_2_last_value = speed_intensity_last_value;
			speed_intensity_last_value = speed_intensity;
			speed_intensity = old_intensity_v2;
		} else {
			speed_intensity_2_last_value = speed_intensity_last_value;
			speed_intensity_last_value = speed_intensity;
		}

	} else {		//La fréquence est en train d'être modifié, on gèle la valeur de la vitesse à la vitesse d'il y a deux cycles
		if (fix_intensity == 0) {
			fix_intensity = 1;
			speed_intensity = speed_intensity_2_last_value;
			speed_intensity_last_value = speed_intensity_2_last_value;
		}
	}

	chprintf((BaseSequentialStream *)&SDU1, "speed_intensity = %d ;         %d   \n", speed_intensity, change_freq);
//	chprintf((BaseSequentialStream *)&SDU1, "speed_intensity = %d ;         %d   \n speed_intensity_1 = %d ; \n speed_intensity_2 = %d ; \n old_intensity = %d ; \n", speed_intensity, change_freq,speed_intensity_last_value,speed_intensity_2_last_value,old_intensity_v2);
}

void do_band_filter(float * mic_complex_input, uint16_t pic_detect) {
	//enleve les vals pas autour du pic
	for(uint16_t  i = 0; i < 2*FFT_SIZE; i+=2) {
		//if(i < (pic_detect*2 - 1) || i > (pic_detect*2 + 1)){
		if(i != pic_detect*2) {
			mic_complex_input[i] = 0;
			mic_complex_input[i+1] = 0;
		}
	}
}

//Finalement, on prend la valeur d'intensité qui est apparu le plus de fois dans les 3 dernieres mesures d'intensités
//--> Filtre les perturbations : des chutes d'intensités soudaines qui ne se répètent pas
uint8_t perturbation(void) {
	return (speed_intensity != speed_intensity_last_value && speed_intensity != speed_intensity_2_last_value
													      && speed_intensity_last_value == speed_intensity_2_last_value);
}


