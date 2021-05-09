#include "ch.h"
#include "hal.h"
#include <usbcfg.h>
#include <chprintf.h>
#include <sensors/imu.h>

#include <mailboxe.h>
#include <imu_obstacle.h>

#define Z_AXIS 							2
#define NUMBER_SAMPLE_IMU 				100
#define NUMBER_SAMPLE_IMU_CALIBRATION 	2000 //eleve pour avoir une bonne reference //probleme ici
#define ACC_Z_TILT_THRESHOLD			40
#define ACC_Z_FLAT_THRESHOLD			20
#define MAX_TILT_COUNTER				3


static int16_t offset_acc_z = 0;

/***************************INTERNAL FUNCTIONS************************************/

static THD_WORKING_AREA(obs_thd_wa, 256); //256 ou moins ?
static THD_FUNCTION(obs_thd, arg){

	chRegSetThreadName(__FUNCTION__);
	(void)arg;

	int16_t z_acc = 0;
	uint8_t tilt_counter = 0;
	uint8_t flat_counter = 0;

	uint8_t tilt_state = NO_BUMP_DETECTED;

	while(1){

		//recupere l'acceleration en z, vaut 0 si au plat
		z_acc = get_acc_filtered(Z_AXIS, NUMBER_SAMPLE_IMU) - offset_acc_z;

		//detection de bosse/plat (MAX_TILT_COUNTER valeurs consecutives necessaires)
		if (z_acc >= ACC_Z_TILT_THRESHOLD) {
			tilt_counter++;
			flat_counter = 0;
			if(tilt_counter == MAX_TILT_COUNTER){
				tilt_counter = MAX_TILT_COUNTER - 1;
				tilt_state = BUMP_DETECTED;
			}
		} else if (z_acc <= ACC_Z_FLAT_THRESHOLD) {
			flat_counter++;
			tilt_counter = 0;
			if(flat_counter == MAX_TILT_COUNTER){
				tilt_counter = MAX_TILT_COUNTER - 1;
				tilt_state = NO_BUMP_DETECTED;
			}
		}

		//envoi d'information "bosse" via mailboxe
		msg_t tilt_state_msg = tilt_state;
		chSysLock();
		chMBPostI(get_mailboxe_imu_adr(), tilt_state_msg);
		chSysUnlock();

		chThdSleepMilliseconds(50); //20x par seconde
	}

/*************************END INTERNAL FUNCTIONS**********************************/


/****************************PUBLIC FUNCTIONS*************************************/

}

void imu_init(void){
	imu_start();
	calibrate_acc();
	offset_acc_z = get_acc_filtered(Z_AXIS, NUMBER_SAMPLE_IMU_CALIBRATION);
	chThdCreateStatic(obs_thd_wa, sizeof(obs_thd_wa), NORMALPRIO, obs_thd, NULL);
}

/**************************END PUBLIC FUNCTIONS***********************************/

