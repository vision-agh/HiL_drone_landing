#ifndef SRC_PIXHAWK_H_
#define SRC_PIXHAWK_H_

#include <stdlib.h>
#include <math.h>
#include "xstatus.h"

#define DECODE_SUCCESS 0
#define DECODE_FAIL 1
#define DECODE_NOT_POSSIBLE -1

#define PIXHAWK_BODY_FRAME MAV_FRAME_LOCAL_OFFSET_NED

typedef unsigned char u8;

typedef struct UART_receive_buffers
{
	u8 **buffers;
	u8 *dataPtr;
	int bufferSize;

	int beginInd;
	int endInd;
	int pushInd;
	int popInd;

} UART_buffsRx;

int initUARTbuffsRx(UART_buffsRx *buffs, int nBuffs, int buffSize);
void resetUARTbuffRx(UART_buffsRx *buffs);
int nextUARTpushBuffer(UART_buffsRx *buffs);
int nextUARTpopBuffer(UART_buffsRx *buffs);
u8 currentUARTData(UART_buffsRx *buffs);
u8 moveUARTptr(UART_buffsRx *buffs);

#endif /* SRC_PIXHAWK_H_ */
