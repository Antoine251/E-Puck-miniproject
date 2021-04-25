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
#define THRES_34_4044(f) 		-1.2063*f + 6267
#define THRES_34_4450(f) 		-0.5642*f + 3442
#define THRES_34_5052(f)		-0.7925*f + 4584
#define THRES_34_5256(f)		-0.4463*f + 2783
#define THRES_34_5662(f)		-0.1867*f + 1329
#define THRES_34_6266(f)		-0.1088*f + 846
#define THRES_34_6672(f)		-0.0500*f + 459
#define THRES_34_7278(f)		-0.0733*f + 627
#define THRES_34_7880(f)		0.1250*f - 921

#define THRES_45_4044(f) 		-1.5762*f + 8214
#define THRES_45_4450(f) 		-0.7467*f + 4564
#define THRES_45_5052(f)		-1.0425*f + 6043
#define THRES_45_5256(f)		-0.6238*f + 3865
#define THRES_45_5662(f)		-0.2392*f + 1712
#define THRES_45_6266(f)		-0.1550*f + 1190
#define THRES_45_6672(f)		-0.0592*f + 557
#define THRES_45_7278(f)		-0.1050*f + 887
#define THRES_45_7880(f)		0.1800*f - 1335

#define THRES_56_4044(f) 		-2.0475*f + 10714
#define THRES_56_4450(f) 		-0.9925*f + 6072
#define THRES_56_5052(f)		-1.3775*f + 7997
#define THRES_56_5256(f)		-0.8450*f + 5228
#define THRES_56_5662(f)		-0.3217*f + 2298
#define THRES_56_6266(f)		-0.2100*f + 1605
#define THRES_56_6672(f)		-0.0725*f + 698
#define THRES_56_7278(f)		-0.1458*f + 1226
#define THRES_56_7880(f)		0.2525*f - 1881

#define THRES_67_4044(f) 		-2.6725*f + 14028
#define THRES_67_4450(f) 		-1.3092*f + 8029
#define THRES_67_5052(f)		-1.8550*f + 10758
#define THRES_67_5256(f)		-0.1150*f + 6910
#define THRES_67_5662(f)		-0.4383*f + 3121
#define THRES_67_6266(f)		-0.2825*f + 2155
#define THRES_67_6672(f)		-0.1000*f + 950
#define THRES_67_7278(f)		-0.1908*f + 1604
#define THRES_67_7880(f)		0.37*f - 2770

#define THRES_78_4044(f) 		-3.5475*f + 18640
#define THRES_78_4450(f) 		-1.7600*f + 10775
#define THRES_78_5052(f)		-2.4500*f + 14225
#define THRES_78_5256(f)		-1.5012*f + 9291
#define THRES_78_5662(f)		-0.5875*f + 4174
#define THRES_78_6266(f)		-0.3625*f + 2779
#define THRES_78_6672(f)		-0.1392*f + 1305
#define THRES_78_7278(f)		-0.2508*f + 2109
#define THRES_78_7880(f)		0.51*f - 3825

#define INTENSITY_MIN 			30

// Vitesse du robot pour les différents niveaux d'intensité
#define SPEED_MARCHE_ARRIERE	-200
#define VITESSE_NUL				0
#define SPEED_1					200
#define SPEED_2					400
#define SPEED_3					600
#define SPEED_4					800

#endif /* AUDIO_PROCESSING_H */
