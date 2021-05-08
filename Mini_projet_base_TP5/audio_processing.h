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
#define THRES_24_3537(f)     0.1631*f - 91
#define THRES_24_3745(f)     -0.4361*f + 2165
#define THRES_24_4548(f)     0.1458*f - 454
#define THRES_24_4854(f)     -0.2644*f + 1520
#define THRES_24_5459(f)     0.0853*f - 375
#define THRES_24_5962(f)     -0.0946*f + 701
#define THRES_24_6267(f)     0.1149*f - 618
#define THRES_24_6770(f)     -0.2204*f + 1635

#define THRES_46_3537(f)     0.3476*f - 366
#define THRES_46_3745(f)     -0.7884*f + 3911
#define THRES_46_4548(f)     0.2628*f - 820
#define THRES_46_4854(f)     -0.4713*f + 2713
#define THRES_46_5459(f)     0.1563*f - 689
#define THRES_46_5962(f)     -0.1843*f + 1349
#define THRES_46_6267(f)     0.2085*f - 1124
#define THRES_46_6770(f)     -0.3882*f + 2884

#define THRES_68_3537(f)     0.5348*f - 337
#define THRES_68_3745(f)     -1.3898*f + 6909
#define THRES_68_4548(f)     0.4728*f - 1472
#define THRES_68_4854(f)     -0.8563*f + 4923
#define THRES_68_5459(f)     0.2931*f - 1308
#define THRES_68_5962(f)     -0.3205*f + 2364
#define THRES_68_6267(f)     0.3329*f - 1750
#define THRES_68_6770(f)     -0.7173*f + 5305

#define INTENSITY_MIN 			50

// Vitesse du robot pour les différents niveaux d'intensité
#define SPEED_MARCHE_ARRIERE	-300
#define VITESSE_NUL				0
#define SPEED_1					300
#define SPEED_2					600

#endif /* AUDIO_PROCESSING_H */
