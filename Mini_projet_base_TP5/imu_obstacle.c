#include "ch.h"
#include "hal.h"
#include <usbcfg.h>
#include <chprintf.h>
#include <sensors/imu.h>

#include <mailboxe.h>
#include <imu_obstacle.h>

#define Z_AXIS 							2
#define NUMBER_SAMPLE_IMU 				100
#define NUMBER_SAMPLE_IMU_CALIBRATION 	255
#define ACC_Z_TILT_THRESHOLD			30
#define MAX_TILT_COUNTER				3


static int16_t offset_acc_z = 0;

static THD_WORKING_AREA(obs_thd_wa, 256); //256 ou moins ?
static THD_FUNCTION(obs_thd, arg){

	chRegSetThreadName(__FUNCTION__);
	(void)arg;

	int16_t z_acc = 0;
	uint8_t tilt_counter = 0;
	uint8_t flat_counter = 0;


	uint8_t tilt_state = NO_BUMP_DETECTED;

	while(1){

		z_acc = get_acc_filtered(Z_AXIS, NUMBER_SAMPLE_IMU) - offset_acc_z;

		if(z_acc >= ACC_Z_TILT_THRESHOLD){ //une bosse est detectee
			tilt_counter++;
			flat_counter = 0;
			if(tilt_counter == MAX_TILT_COUNTER){
				tilt_counter = MAX_TILT_COUNTER - 1;
				tilt_state = BUMP_DETECTED;
			}
		}else{
			flat_counter++;
			tilt_counter = 0;
			if(flat_counter == MAX_TILT_COUNTER){
				tilt_counter = MAX_TILT_COUNTER - 1;
				tilt_state = NO_BUMP_DETECTED;
			}
		}

		//envoi d'information "bosse" via mailboxe
		msg_t etat_penche = tilt_state;
		chSysLock();
		chMBPostI(get_mailboxe_imu_adr(), etat_penche);
		chSysUnlock();

		//chprintf((BaseSequentialStream *)&SDU1, "imu values z_axis : %d \n", z_acc);

		chThdSleepMilliseconds(50); //20x par seconde
	}



}

void imu_init(void){ //ordre ?
	imu_start();
	chThdCreateStatic(obs_thd_wa, sizeof(obs_thd_wa), NORMALPRIO, obs_thd, NULL);
	calibrate_acc();
	offset_acc_z = get_acc_filtered(Z_AXIS, NUMBER_SAMPLE_IMU_CALIBRATION);
}

