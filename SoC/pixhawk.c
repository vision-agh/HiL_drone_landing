#include "pixhawk.h"

int initUARTbuffsRx(UART_buffsRx *buffs, int nBuffs, int buffSize)
{
	buffs->buffers = NULL;

	// Allocate memory for pointers array
	buffs->buffers = malloc(nBuffs * sizeof(u8 *));
	if (buffs->buffers == NULL)
	{
		return XST_FAILURE;
	}

	// Allocate each pointer
	int fail = 0;
	int i = 0;
	for (i = 0; i < nBuffs; ++i)
	{
		(buffs->buffers)[i] = malloc(buffSize * sizeof(u8));
		if ((buffs->buffers)[i] == NULL)
		{
			fail = 1;
			break;
		}
	}

	// In case of allocation problem, free all previously allocated memory
	if (fail)
	{
		for (int j = i - 1; j >= 0; --j)
		{
			free((buffs->buffers)[i]);
		}

		free(buffs->buffers);

		return XST_FAILURE;
	}

	buffs->dataPtr = buffs->buffers[0];
	buffs->bufferSize = buffSize;
	buffs->beginInd = 0;
	buffs->endInd = nBuffs;
	buffs->popInd = 0;
	buffs->pushInd = 0;

	return XST_SUCCESS;
}

void resetUARTbuffRx(UART_buffsRx *buffs)
{
	buffs->popInd = 0;
	buffs->pushInd = 0;
}

int nextUARTpushBuffer(UART_buffsRx *buffs)
{
	buffs->pushInd = ++(buffs->pushInd) < buffs->endInd ? buffs->pushInd : buffs->beginInd;

	return buffs->pushInd;
}

int nextUARTpopBuffer(UART_buffsRx *buffs)
{
	buffs->popInd = ++(buffs->popInd) < buffs->endInd ? buffs->popInd : buffs->beginInd;

	return buffs->popInd;
}

u8 currentUARTData(UART_buffsRx *buffs)
{
	return *(buffs->dataPtr);
}

u8 moveUARTptr(UART_buffsRx *buffs)
{
	u8 retVal = *(buffs->dataPtr);

	if ((buffs->dataPtr - (buffs->buffers)[buffs->popInd]) < (buffs->bufferSize - 1))
	{
		buffs->dataPtr = buffs->dataPtr + 1;
	}
	else
	{
		if (buffs->popInd < buffs->endInd)
		{
			buffs->dataPtr = buffs->buffers[buffs->popInd + 1];
			buffs->popInd = buffs->popInd + 1;
		}
		else
		{
			buffs->dataPtr = buffs->buffers[0];
			buffs->popInd = 0;
		}
	}

	return retVal;
}