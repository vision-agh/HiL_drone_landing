#include "hil.h"

int initHIL(HIL_data *data, u8 *buff, int buffLen)
{
	data->init_char = 'Q';
	data->algorithm_trigger = 0;
	data->altitude_cm = 0;
	data->altitude_m = 0;
	data->circle_size = 0;
	data->square_size = 0;
	data->uart_ptr = buff;
	data->uart_len = buffLen;

	return HIL_SUCCESS;
}

int decodeReceivedHILdata(u8 *buff, HIL_data *data, int *parMask)
{
	u8 *iterPtr;
	int msgDecoded = 0;
	for (iterPtr = data->uart_ptr; (iterPtr != (data->uart_ptr - 1)) && (msgDecoded == 0); iterPtr = (iterPtr - buff) < (data->uart_len) - 1 ? iterPtr + 1 : buff)
	{
		switch (*iterPtr)
		{
		case 'A':
		{
			(*iterPtr) = '*';
			iterPtr++;
			if ((*iterPtr) == 'm')
			{
				iterPtr++;
				if ('0' <= (*iterPtr) && (*iterPtr) <= '9')
				{
					data->altitude_m = ((*iterPtr) - '0') * 10;
					iterPtr++;
					if ('0' <= (*iterPtr) && (*iterPtr) <= '9')
					{
						data->altitude_m += ((*iterPtr) - '0');
						msgDecoded = 1;
					}
				}
			}
			else if ((*iterPtr) == 'c')
			{
				iterPtr++;
				if ('0' <= (*iterPtr) && (*iterPtr) <= '9')
				{
					data->altitude_cm = ((*iterPtr) - '0') * 10;
					iterPtr++;
					if ('0' <= (*iterPtr) && (*iterPtr) <= '9')
					{
						data->altitude_cm += ((*iterPtr) - '0');
						msgDecoded = 1;
					}
				}
			}
			break;
		}

		case 'C':
		{
			(*iterPtr) = '*';
			iterPtr++;
			if ('0' <= (*iterPtr) && (*iterPtr) <= '9')
			{
				data->circle_size = ((*iterPtr) - '0') * 100;
				iterPtr++;
				if ('0' <= (*iterPtr) && (*iterPtr) <= '9')
				{
					data->circle_size += ((*iterPtr) - '0') * 10;
					iterPtr++;
					if ('0' <= (*iterPtr) && (*iterPtr) <= '9')
					{
						data->circle_size += ((*iterPtr) - '0');
						msgDecoded = 1;
					}
				}
			}
			break;
		}

		case 'S':
		{
			(*iterPtr) = '*';
			iterPtr++;
			if ('0' <= (*iterPtr) && (*iterPtr) <= '9')
			{
				data->square_size = ((*iterPtr) - '0') * 100;
				iterPtr++;
				if ('0' <= (*iterPtr) && (*iterPtr) <= '9')
				{
					data->square_size += ((*iterPtr) - '0') * 10;
					iterPtr++;
					if ('0' <= (*iterPtr) && (*iterPtr) <= '9')
					{
						data->square_size += ((*iterPtr) - '0');
						msgDecoded = 1;
					}
				}
			}
			break;
		}

		case 'T':
		{
			(*iterPtr) = '*';
			iterPtr++;
			if ((*iterPtr) == '1')
			{
				iterPtr++;
				if ((*iterPtr) == '1')
				{
					iterPtr++;
					if ((*iterPtr) == '1')
					{
						data->algorithm_trigger = 1;
						xil_printf("t:1\n");
						msgDecoded = 1;
					}
				}
			}
			else if ((*iterPtr) == '0')
			{
				iterPtr++;
				if ((*iterPtr) == '0')
				{
					iterPtr++;
					if ((*iterPtr) == '0')
					{
						data->algorithm_trigger = 0;
						xil_printf("t:0\n");
						msgDecoded = 1;
					}
				}
			}
			break;
		}
		}
	}

	data->uart_ptr = iterPtr;

	return HIL_SUCCESS;
}

int decodeHILcommand(UART_buffsRx *buffs, int *parMask)
{
	while (buffs->popInd != buffs->pushInd)
	{
		switch ((buffs->buffers)[buffs->popInd][0])
		{
		// Internal algorithm parametres request service
		case 'X':
		case 'x':
		{
			// Calculate param number
			int param_number = ((buffs->buffers)[buffs->popInd][1] - '0') * 10 + (buffs->buffers)[buffs->popInd][2] - '0';
			xil_printf("Parameter number = %d\r\n", param_number);

			if (param_number >= 0 && param_number < 32)
			{
				*parMask |= (1 << param_number);
			}

			break;
		}

		default:
		{
			xil_printf("Unrecognised command\r\n");
			resetUARTbuffRx(buffs);
			return HIL_FAILURE;
		}
		}

		nextUARTpopBuffer(buffs);
	}

	return HIL_SUCCESS;
}
