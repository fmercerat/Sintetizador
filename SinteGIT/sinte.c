#include <avr/pgmspace.h>
#include <inttypes.h>

#include "sinte.h"


uint16_t g_rand = 0xACE1u;

uint8_t Sierra(uint8_t x)
{
	return x;
}

uint8_t Triang(uint8_t x)
{
	if(x < 128)
		return (x << 1);
	else
		return (256 - ( (x-127) << 1 ) );
}

uint8_t Cuadrada(uint8_t x)
{
	if(x < 128)
		return 255;
	else
		return 0;
}

uint8_t Pulso(uint8_t x, uint8_t y)
{
	if(x < y)
			return 255;
		else
			return 0;
}

void nextRand(void)
{
  g_rand = (g_rand >> 1) ^ (-(g_rand & 1u) & 0xB400u);
}

uint8_t Ruido(void)
{
	nextRand();
	return (uint8_t)g_rand;
}


