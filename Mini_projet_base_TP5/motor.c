#include "ch.h"
#include "hal.h"
#include <usbcfg.h>
#include <chprintf.h>
#include <motors.h>

#include <mailboxe.h>

#include <capteur_proxi.h> 	//include pour avoir les #define de etat_obstacle
#include <imu_obstacle.h>	//include pour avoir les #define de etat_penche

#define ON_A_BUMP			1
#define NOT_ON_A_BUMP		0
#define DO_A_TURN			1
#define DONT_DO_A_TURN		0
#define TURN_SPEED			500 //in step/sec
#define MAX_COUNTER_STEP 	52  // (perimetre_wheel/vitesse_tour)/(temps de la tread) = (16.8/(500/1000*13))/0.05



/*********************PROTOTYPES OF INTERNAL FUNCTIONS****************************/
void do_a_roll(void);
/*****************END OF PROTOTYPES OF INTERNAL FUNCTIONS*************************/

/***************************INTERNAL FUNCTIONS************************************/

static THD_WORKING_AREA(motor_thd_wa, 256); // /!\ longue tread, risque de depasser le stack attribue
static THD_FUNCTION(motor_thd, arg){

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    //mettre toute les decl en dehors du while ??

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

    	//recoit les informations de l'imu (axe z)
    	msg_t message_received_imu;
    	chSysLock();
    	chMBFetchI(get_mailboxe_imu_adr(), &message_received_imu);
    	chSysUnlock();
    	//chprintf((BaseSequentialStream *)&SDU1, "etat penche= %d \n", message_received_imu);



    	int16_t rotation_speed_right = message_received_motor_right;
    	int16_t rotation_speed_left = message_received_motor_left;
    	uint8_t etat_obstacle = message_received_proximity;
    	uint8_t etat_penche = message_received_imu;

    	int16_t vit_moy = rotation_speed_left/2 + rotation_speed_right/2;

    	//code pour detecter la fin d'une bosse
    	static uint8_t sur_une_bosse = NOT_ON_A_BUMP;
    	static uint8_t faire_un_tour = DONT_DO_A_TURN;
    	if(etat_penche == BUMP_DETECTED && sur_une_bosse == NOT_ON_A_BUMP){
    		sur_une_bosse = ON_A_BUMP;
    	}
    	if(etat_penche == NO_BUMP_DETECTED && sur_une_bosse == ON_A_BUMP){
    		faire_un_tour = DO_A_TURN;
    		sur_une_bosse = NOT_ON_A_BUMP;
    		//chprintf((BaseSequentialStream *)&SDU1, "test \n");
    	}

    	//obstacle a la prio sur bosse, donc tant qu'on a un obstacle, le robot ne fais pas de tour sur luimeme
    	//mais une fois le tour lanc�, il a la prio sur tout

    	static int turn_counter = 0;

    	if(faire_un_tour == DONT_DO_A_TURN){
			switch(etat_obstacle){
			case PAS_OBSTACLE : 	//avance suivant les instruction de l'utilisateur
					left_motor_set_speed(rotation_speed_left);
					right_motor_set_speed(rotation_speed_right);
				break;

			case OBSTACLE_AVANT : 	//autorise que la rotation sur place
				left_motor_set_speed(rotation_speed_left - vit_moy);
				right_motor_set_speed(rotation_speed_right - vit_moy);
				break;

			case OBSTACLE_ARRIERE : //bloque la direction en arri�re
				if(vit_moy < 0){
					left_motor_set_speed(rotation_speed_left - vit_moy);
					right_motor_set_speed(rotation_speed_right - vit_moy);
				}else{
					left_motor_set_speed(rotation_speed_left);
					right_motor_set_speed(rotation_speed_right);
				}
				break;
			}
    	}else{
    		//en train de faire le tour
    		left_motor_set_speed(TURN_SPEED);
    		right_motor_set_speed(TURN_SPEED);
    		turn_counter++;
    		if(turn_counter == MAX_COUNTER_STEP){
    			turn_counter = 0;
    			faire_un_tour = DONT_DO_A_TURN;

    		}

    	}


    	chThdSleepMilliseconds(50); //20x par seconde
    }
}

void do_a_roll(void){
	chprintf((BaseSequentialStream *)&SDU1, "je fais un tour \n");
	//chSysLock();
	//chSysUnlock();
}

/*************************END INTERNAL FUNCTIONS**********************************/


/****************************PUBLIC FUNCTIONS*************************************/

void motor_start(void){
	chThdCreateStatic(motor_thd_wa, sizeof(motor_thd_wa), NORMALPRIO, motor_thd, NULL);
	motors_init();
}

/**************************END PUBLIC FUNCTIONS***********************************/
