#include <inttypes.h>
#include "7seg.h"
#include "spi.h"


void enviaNumero(uint8_t n,uint8_t c)
{
	c = 1<<(c+4);
	switch(n)
	{
		case 0:
			SPI(9|c); SPI(8|c); SPI(7|c); SPI(5|c); SPI(4|c); SPI(3|c);
			break;
		case 1:
			SPI(7|c); SPI(5|c);
			break;
		case 2:
			SPI(8|c); SPI(7|c); SPI(6|c); SPI(3|c); SPI(4|c);
			break;
		case 3:
			SPI(8|c); SPI(7|c); SPI(6|c); SPI(5|c); SPI(4|c); SPI(6|c);
			break;
		case 4:
			SPI(9|c); SPI(7|c); SPI(6|c); SPI(5|c);
			break;
		case 5:
			SPI(9|c); SPI(8|c); SPI(6|c); SPI(5|c); SPI(4|c);
			break;
		case 6:
			SPI(9|c); SPI(6|c); SPI(5|c); SPI(4|c); SPI(3|c);
			break;
		case 7:
			SPI(8|c); SPI(7|c); SPI(5|c);
			break;
		case 8:
			SPI(9|c); SPI(8|c); SPI(7|c); SPI(6|c); SPI(5|c); SPI(4|c); SPI(3|c);
			break;
		case 9:
			SPI(9|c); SPI(8|c); SPI(7|c); SPI(6|c); SPI(5|c);
			break;
		default:
			break;
	}
}

void display72(uint8_t x)
{
	enviaNumero(x%10,0);
	enviaNumero(x/10,1);
}
