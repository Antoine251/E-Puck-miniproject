#include "ch.h"
#include "hal.h"
#include <usbcfg.h>
#include <chprintf.h>
#include <sensors/proximity.h>

#include <mailboxe.h>
#include <capteur_proxi.h>


#define PROXI_THRESHOLD 100 //distance à laquelle un obstacle est "proche"

#define PROXI_FFR 	 0 	//front front right  		proximity sensor (+18° relative to front)
#define PROXI_FRR 	 1 	//front right right 		proximity sensor (+49° relative to front)
#define PROXI_R 	 2	//right        				proximity sensor (+90° relative to front)
#define PROXI_BR 	 3 	//back right 				proximity sensor (+150° relative to front)
#define PROXI_BL 	 4 	//back left 				proximity sensor (-150° relative to front)
#define PROXI_L 	 5 	//left  					proximity sensor (-90° relative to front)
#define PROXI_FLL 	 6 	//front left left 			proximity sensor (-49° relative to front)
#define PROXI_FFL 	 7 	//front front left 			proximity sensor (-18° relative to front)


/***************************INTERNAL FUNCTIONS************************************/

static THD_WORKING_AREA(proxi_thd_wa, 256);
static THD_FUNCTION(proxi_thd, arg){

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    calibrate_ir();

	while(1){
		int proxi_value = 0;		//int car get_calibrated_prox(i) renvoie un int
		int max_proxi_value = 0;
		uint8_t max_proxi_chanel = 0;


		for(uint8_t i = 0; i < PROXIMITY_NB_CHANNELS; ++i){

			proxi_value = get_calibrated_prox(i);

			if(proxi_value > max_proxi_value){
				max_proxi_value = proxi_value;
				max_proxi_chanel = i;
			}
		}

		//detection de murs par les capteurs de proximite
		uint8_t obs_state = PAS_OBSTACLE;
		if(max_proxi_value > PROXI_THRESHOLD){
			//l'obstacle est devant (cotes inclus)
			if((max_proxi_chanel == PROXI_FFL) || (max_proxi_chanel == PROXI_FFR) ||
			   (max_proxi_chanel == PROXI_FLL) || (max_proxi_chanel == PROXI_FRR) ||
			   (max_proxi_chanel == PROXI_L)   || (max_proxi_chanel == PROXI_R)){
				obs_state = OBSTACLE_AVANT;
			}else{
				//l'obstacle est derriere
				obs_state = OBSTACLE_ARRIERE;
			}
		}else{
			//aucun obstacle est detecte
			obs_state = PAS_OBSTACLE;
		}

		//envoi d'information "obstacle" via mailboxe
		msg_t obs_state_msg_t = obs_state;
		chSysLock();
		chMBPostI(get_mailboxe_proximity_adr(), obs_state_msg_t);
		chSysUnlock();

		chThdSleepMilliseconds(50); //20x par seconde
	}
}

/*************************END INTERNAL FUNCTIONS**********************************/


/****************************PUBLIC FUNCTIONS*************************************/

void capteur_proxi_start(void){
	chThdCreateStatic(proxi_thd_wa, sizeof(proxi_thd_wa), NORMALPRIO, proxi_thd, NULL);
	proximity_start();

}

/**************************END PUBLIC FUNCTIONS***********************************/

