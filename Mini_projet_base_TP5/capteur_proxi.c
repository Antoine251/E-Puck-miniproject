//#include "proximity.h"
//
//
//
//
//
//
//
//
////detection avec les capteur IR
////threat interne a proximity.c
////peu de fonction externe
////droit de lib�rer le semaphore adc2_ready ? que utilis� dans adc_cb
////besoin d'une autre semaphore pour attendre que les donn�e soit publi�e, avant de les recup avec get_prox ?
//
//static int proxi_values[PROXIMITY_NB_CHANNELS]; //besoin static ?
//
//
//static THD_WORKING_AREA(proxi_thd_wa, 256); //256 ou moins ?
//static THD_FUNCTION(proxi_thd, arg){
//
//	while(1){
//		for(uint8_t i = 0; i < PROXIMITY_NB_CHANNELS; ++i){
//			proxi_values = get_calibrated_prox(i);
//
//			chThdSleepMilliseconds(20);
//		}
//	}
//
//
//}
//
////a  mettre dans le main
