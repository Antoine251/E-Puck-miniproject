#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include <main.h>
#include <chprintf.h>
#include <audio/microphone.h>
//#include <sensors/proximity.h>

#include <audio_processing.h>
#include <fft.h>
#include <communications.h>
#include <arm_math.h>
#include <motor.h>
#include <imu_obstacle.h>

#include <capteur_proxi.h> //decarer la thread dans le .h ??
//#include "proximity.h"

//uncomment to send the FFTs results from the real microphones
#define SEND_FROM_MIC

//uncomment to use double buffering to send the FFT to the computer
#define DOUBLE_BUFFERING

messagebus_t bus; 			//defini car utilisation de bus pour proximity.c //et imu ??
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);


static void serial_start(void)
{
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.
}

static void timer12_start(void){
    //General Purpose Timer configuration   
    //timer 12 is a 16 bit timer so we can measure time
    //to about 65ms with a 1Mhz counter
    static const GPTConfig gpt12cfg = {
        1000000,        /* 1MHz timer clock in order to measure uS.*/
        NULL,           /* Timer callback.*/
        0,
        0
    };

    gptStart(&GPTD12, &gpt12cfg);
    //let the timer count to max value
    gptStartContinuous(&GPTD12, 0xFFFF);
}

int main(void)
{

    halInit();
    chSysInit();
    mpu_init();

    messagebus_init(&bus, &bus_lock, &bus_condvar); //car bus utilis� dans proximity // et imu ??

    //starts the serial communication
    serial_start();
    //starts the USB communication
    usb_start();
    //starts timer 12
    timer12_start(); //necessaire ?
    //inits the motors
    motor_start();
    //inits the imu module
    imu_init();

    //inits and calibrates the thread to process sensors //ordre important ?
    capteur_proxi_start();

    //starts the microphones processing thread.
    //it calls the callback given in parameter when samples are ready
    mic_start(&processAudioData);

    /* Infinite loop. */
    while (1) {

    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
