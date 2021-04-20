#include "ch.h"
#include "hal.h"

#define MAILBOXE_SIZE 	20

static msg_t buffer_mailboxe[MAILBOXE_SIZE] = {0};
static mailbox_t data_mailboxe = _MAILBOX_DATA(data_mailboxe, buffer_mailboxe, MAILBOXE_SIZE);

mailbox_t * get_mailboxe_adr(void) {
	return &data_mailboxe;
}
