#include "ch.h"
#include "hal.h"
#include <usbcfg.h>
#include <chprintf.h>
#include <motors.h>
#include <mailboxe.h>

#include <capteur_proxi.h> //include pour avoir les define des etat du proximity

//ajouter des com pour expliciter les actions faites

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
    	//chprintf((BaseSequentialStream *)&SDU1, "moteur gauche = %d; moteur droite = %d \n", message_received_motor_left, message_received_motor_right);

    	//reçoit les informations des capteurs de proximité
    	msg_t message_received_proximity;
    	chSysLock();
    	chMBFetchI(get_mailboxe_proximity_adr(), &message_received_proximity);
    	chSysUnlock();
    	//chprintf((BaseSequentialStream *)&SDU1, "etat obstacle = %d \n", message_received_proximity);

    	int16_t rotation_speed_right = message_received_motor_right;
    	int16_t rotation_speed_left = message_received_motor_left;
    	uint8_t etat_obstacle = message_received_proximity;

    	int16_t vit_moy = rotation_speed_left/2 + rotation_speed_right/2;

    	switch(etat_obstacle){
    	case PAS_OBSTACLE :
    		left_motor_set_speed(rotation_speed_left);
    		right_motor_set_speed(rotation_speed_right);
    		break;

    	case OBSTACLE_AVANT :
    		left_motor_set_speed(rotation_speed_left - vit_moy);
    		right_motor_set_speed(rotation_speed_right - vit_moy);
    		break;

    	case OBSTACLE_ARRIERE :
    		if(vit_moy < 0){
    			left_motor_set_speed(rotation_speed_left - vit_moy);
    			right_motor_set_speed(rotation_speed_right - vit_moy);
    		}else{
    			left_motor_set_speed(rotation_speed_left);
    			right_motor_set_speed(rotation_speed_right);
    		}
    		break;
    	}


    	chThdSleepMilliseconds(50); //20x par seconde
    }
}

void motor_start(void){
	chThdCreateStatic(motor_thd_wa, sizeof(motor_thd_wa), NORMALPRIO, motor_thd, NULL);
	motors_init();
}
