#include "ch.h"
#include "hal.h"
#include <usbcfg.h>
#include <chprintf.h>
#include <motors.h>
#include <mailboxe.h>

static THD_WORKING_AREA(motor_thd_wa, 256);
static THD_FUNCTION(motor_thd, arg){

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    while(1) {
    	//reçoit les informations de audio processing
    	msg_t message_received_motor_left;
    	msg_t message_received_motor_right;
    	chSysLock();
    	chMBFetchI(get_mailboxe_micro_adr(), &message_received_motor_left);
    	chMBFetchI(get_mailboxe_micro_adr(), &message_received_motor_right);
    	chSysUnlock();
    	chprintf((BaseSequentialStream *)&SDU1, "moteur gauche = %d; moteur droite = %d \n", message_received1, message_received2);

    	//reçoit les informations des capteurs de proximité
    	msg_t message_received_proximity;
    	chSysLock();
    	chMBFetchI(get_mailboxe_micro_adr(), &message_received_proximity);
    	chSysUnlock();
    	chprintf((BaseSequentialStream *)&SDU1, "etat obstacle = %d \n", message_received_proximity);

    	chThdSleepMilliseconds(50);
    }
}

void motor_start(void){
	chThdCreateStatic(motor_thd_wa, sizeof(motor_thd_wa), NORMALPRIO, motor_thd, NULL);
	motors_init();
}
