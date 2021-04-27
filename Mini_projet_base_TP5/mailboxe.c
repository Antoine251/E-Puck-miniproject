#include "ch.h"
#include "hal.h"

#define MAILBOXE_SIZE 	5

static msg_t buffer_mailboxe_micro[MAILBOXE_SIZE] = {0};
static msg_t buffer_mailboxe_proximity[MAILBOXE_SIZE] = {0};
static msg_t buffer_mailboxe_imu[MAILBOXE_SIZE] = {0};

static mailbox_t data_mailboxe_micro = _MAILBOX_DATA(data_mailboxe_micro, buffer_mailboxe_micro, MAILBOXE_SIZE);
static mailbox_t data_mailboxe_proximity = _MAILBOX_DATA(data_mailboxe_proximity, buffer_mailboxe_proximity, MAILBOXE_SIZE);
static mailbox_t data_mailboxe_imu = _MAILBOX_DATA(data_mailboxe_imu, buffer_mailboxe_imu, MAILBOXE_SIZE);


mailbox_t * get_mailboxe_micro_adr(void) {
	return &data_mailboxe_micro;
}

mailbox_t * get_mailboxe_proximity_adr(void) {
	return &data_mailboxe_proximity;
}

mailbox_t * get_mailboxe_imu_adr(void) {
	return &data_mailboxe_imu;
}
