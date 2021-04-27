#include "ch.h"
#include "hal.h"
#include <usbcfg.h>
#include <chprintf.h>
#include <sensors/imu.h>

#define Z_AXIS 2
#define NUMBER_SAMPLE_IMU 10

static THD_WORKING_AREA(obs_thd_wa, 256); //256 ou moins ?
static THD_FUNCTION(obs_thd, arg){

	chRegSetThreadName(__FUNCTION__);
	(void)arg;

	while(1){

		int16_t z_acc = 0;

		get_acc_filtered(Z_AXIS, NUMBER_SAMPLE_IMU);

		chprintf((BaseSequentialStream *)&SDU1, "imu values z_axis : %d \n", z_acc);

		chThdSleepMilliseconds(100); //10x par seconde
	}



}

void imu_init(void){ //ordre ?
	imu_start();
	chThdCreateStatic(obs_thd_wa, sizeof(obs_thd_wa), NORMALPRIO, obs_thd, NULL);
	calibrate_acc();
}

