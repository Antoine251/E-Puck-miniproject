#ifndef AUDIO_PROCESSING_H
#define AUDIO_PROCESSING_H


#define FFT_SIZE 	1024

typedef enum {
	//2 times FFT_SIZE because these arrays contain complex numbers (real + imaginary)
	LEFT_CMPLX_INPUT = 0,
	RIGHT_CMPLX_INPUT,
	FRONT_CMPLX_INPUT,
	BACK_CMPLX_INPUT,
	//Arrays containing the computed magnitude of the complex numbers
	LEFT_OUTPUT,
	RIGHT_OUTPUT,
	FRONT_OUTPUT,
	BACK_OUTPUT
} BUFFER_NAME_t;

void processAudioData(int16_t *data, uint16_t num_samples);

//Calcul la vitesse associée à l'intensité et à la fréquence du son
void compute_motor_speed(void);
void compute_speed_intensity(uint16_t freq);

/*
*	put the invoking thread into sleep until it can process the audio datas
*/
void wait_send_to_computer(void);

/*
*	Returns the pointer to the BUFFER_NAME_t buffer asked
*/
float* get_audio_buffer_ptr(BUFFER_NAME_t name);

//Ensemble des macros pour le calcul des thresholds en fonction de l'intensité
#define THRES_34_4044(f) 		-1.1884*f + 6418
#define THRES_34_4450(f) 		-0.6023*f + 3726
#define THRES_34_5052(f)		-0.8087*f + 4790
#define THRES_34_5256(f)		-0.4474*f + 2856
#define THRES_34_5662(f)		-0.1839*f + 1341
#define THRES_34_6266(f)		-0.1051*f + 840
#define THRES_34_6672(f)		-0.0498*f + 466
#define THRES_34_7278(f)		-0.0723*f + 631

#define THRES_45_4044(f) 		-1.5530*f + 8411
#define THRES_45_4450(f) 		-0.7972*f + 4940
#define THRES_45_5052(f)		-1.0638*f + 6314
#define THRES_45_5256(f)		-0.6253*f + 3968
#define THRES_45_5662(f)		-0.2356*f + 1727
#define THRES_45_6266(f)		-0.1498*f + 1181
#define THRES_45_6672(f)		-0.0590*f + 566
#define THRES_45_7278(f)		-0.1035*f + 894

#define THRES_56_4044(f) 		-2.0172*f + 10971
#define THRES_56_4450(f) 		-1.0596*f + 6572
#define THRES_56_5052(f)		-1.4056*f + 8356
#define THRES_56_5256(f)		-0.8471*f + 5367
#define THRES_56_5662(f)		-0.3169*f + 2319
#define THRES_56_6266(f)		-0.2029*f + 1594
#define THRES_56_6672(f)		-0.0723*f + 709
#define THRES_56_7278(f)		-0.1437*f + 1236

#define THRES_67_4044(f) 		-2.6330*f + 14362
#define THRES_67_4450(f) 		-1.3977*f + 8689
#define THRES_67_5052(f)		-1.8929*f + 11241
#define THRES_67_5256(f)		-0.1178*f + 7094
#define THRES_67_5662(f)		-0.4319*f + 3150
#define THRES_67_6266(f)		-0.2730*f + 2139
#define THRES_67_6672(f)		-0.0997*f + 966
#define THRES_67_7278(f)		-0.1880*f + 1617

#define THRES_78_4044(f) 		-3.4951*f + 19084
#define THRES_78_4450(f) 		-1.8790*f + 11661
#define THRES_78_5052(f)		-2.5000*f + 14863
#define THRES_78_5256(f)		-1.5050*f + 9538
#define THRES_78_5662(f)		-0.5788*f + 4213
#define THRES_78_6266(f)		-0.3502*f + 2759
#define THRES_78_6672(f)		-0.1387*f + 1326
#define THRES_78_7278(f)		-0.2471*f + 2126

#define INTENSITY_MIN 			30

// Vitesse du robot pour les différents niveaux d'intensité
#define SPEED_MARCHE_ARRIERE	-200
#define VITESSE_NUL				0
#define SPEED_1					200
#define SPEED_2					400
#define SPEED_3					600
#define SPEED_4					800

#endif /* AUDIO_PROCESSING_H */
