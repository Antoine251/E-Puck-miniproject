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

//Ensemble des macros pour le calcul des thresholds en fonction de l'intensité
#define THRES_24_3537(f)     -0.0922*f + 829
#define THRES_24_3745(f)     -0.351*f + 1804
#define THRES_24_4548(f)     -0.0697*f + 538
#define THRES_24_4854(f)     -0.1839*f + 1087
#define THRES_24_5459(f)     0.0617*f - 244
#define THRES_24_5962(f)     -0.0449*f + 394
#define THRES_24_6267(f)     0.077*f - 374
#define THRES_24_6770(f)     -0.1981*f + 1474
//
#define THRES_46_3537(f)     -0.127*f + 1346
#define THRES_46_3745(f)     -0.6276*f + 3230
#define THRES_46_4548(f)     -0.1466*f + 1066
#define THRES_46_4854(f)     -0.3255*f + 1927
#define THRES_46_5459(f)     0.1146*f - 459
#define THRES_46_5962(f)     -0.0913*f + 773
#define THRES_46_6267(f)     0.1422*f - 697
#define THRES_46_6770(f)     -0.3658*f + 2716
//
#define THRES_68_3537(f)     -0.2874*f + 2629
#define THRES_68_3745(f)     -1.101*f + 5692
#define THRES_68_4548(f)     -0.266*f + 1935
#define THRES_68_4854(f)     -0.5866*f + 3478
#define THRES_68_5459(f)     0.21*f - 841
#define THRES_68_5962(f)     -0.1611*f + 1380
#define THRES_68_6267(f)     0.2358*f - 1119
#define THRES_68_6770(f)     -0.635*f + 4731

#endif /* AUDIO_PROCESSING_H */
