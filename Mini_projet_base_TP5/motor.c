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

    	msg_t message_received1;
    	msg_t message_received2;
    	chSysLock();
    	chMBFetchI(get_mailboxe_adr(), &message_received1);
    	chMBFetchI(get_mailboxe_adr(), &message_received2);
    	chSysUnlock();

    	chprintf((BaseSequentialStream *)&SDU1, "moteur gauche = %d; moteur droite = %d \n", message_received1, message_received2);

    	chThdSleepMilliseconds(50);
    }
}

void motor_start(void){
	chThdCreateStatic(motor_thd_wa, sizeof(motor_thd_wa), NORMALPRIO, motor_thd, NULL);
	motors_init();
}
