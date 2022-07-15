#ifndef SRC_HIL_H_
#define SRC_HIL_H_

#include "pixhawk.h"

#define HIL_SUCCESS 1
#define HIL_FAILURE 0

typedef struct HIL_data
{
	char init_char;

	int algorithm_trigger;
	int altitude_m;
	int altitude_cm;
	int circle_size;
	int square_size;

	u8 *uart_ptr;
	int uart_len;
} HIL_data;

int initHIL(HIL_data *data, u8 *buff, int buffLen);
int decodeReceivedHILdata(u8 *buff, HIL_data *data, int *parMask);
int decodeHILcommand(UART_buffsRx *buffs, int *parMask);

#endif /* SRC_HIL_H_ */
