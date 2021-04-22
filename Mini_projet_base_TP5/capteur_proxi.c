#include "ch.h"
#include "hal.h"
#include <usbcfg.h>
#include <sensors/proximity.h>
#include <chprintf.h>





#define PROXI_THRESHOLD 0 // a definir -> trouver quelle est la valeur recu quand on touche dans le worst case, definir le seuil a partir de l�

//aucune id�e lequel est lequel (sensor = num�ro)
#define PROXI_FL 	 0 	//front left  		proximity sensor (-30� relative to front)
#define PROXI_FR 	 1 	//front right 		proximity sensor (30� relative to front)
#define PROXI_L 	 2	//left        		proximity sensor (-90� relative to front)
#define PROXI_R 	 3 	//right 			proximity sensor (90� relative to front)
#define PROXI_BLL 	 4 	//back left left 	proximity sensor (-131� relative to front)
#define PROXI_BRR 	 5 	//back right right  proximity sensor (131� relative to front)
#define PROXI_BBL 	 6 	//back back left 	proximity sensor (-162� relative to front)
#define PROXI_BBR 	 7 	//back back right 	proximity sensor (162� relative to front)


//detection avec les capteur IR
//threat interne a proximity.c
//peu de fonction externe
//droit de lib�rer le semaphore adc2_ready ? que utilis� dans adc_cb
//besoin d'une autre semaphore pour attendre que les donn�e soit publi�e, avant de les recup avec get_prox ?

static int proxi_values[PROXIMITY_NB_CHANNELS]; //besoin static ?


static THD_WORKING_AREA(proxi_thd_wa, 256); //256 ou moins ?
static THD_FUNCTION(proxi_thd, arg){

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    calibrate_ir(); // ou le mettre
	while(1){
		int max_proxi_value = 0;
		uint8_t max_proxi_chanel = 0;

		for(uint8_t i = 0; i < PROXIMITY_NB_CHANNELS; ++i){

			proxi_values[i] = get_calibrated_prox(i); //a quel capteur corespondent 0,1,2...7 ?

			if(proxi_values[i] > max_proxi_value){
				max_proxi_value = proxi_values[i];
				//chprintf((BaseSequentialStream *)&SDU1, "proxi values capteur %d : %d \n", i, proxi_values[i]);
				max_proxi_chanel = i;
			}
		}

		if(max_proxi_value > PROXI_THRESHOLD){ //an obstacle is seen
			if((max_proxi_chanel == PROXI_FL) || (max_proxi_chanel == PROXI_FR) ||
			   (max_proxi_chanel == PROXI_L) || (max_proxi_chanel == PROXI_R)){ //one of the 4 front sensor (including the side) sees an obstacle
				//avant -> mailbox autoriser que la rota
			}else{
				//arriere -> mailbox pas moy_roue < 0
			}
		}else{
			//mailbox no obstacle ??
		}

		chThdSleepMilliseconds(50); //20x par seconde
	}
}

void capteur_proxi_start(void){
	chThdCreateStatic(proxi_thd_wa, sizeof(proxi_thd_wa), NORMALPRIO, proxi_thd, NULL);
	proximity_start(); //probleme vient d'ici, a cause de messagebus ??

}

