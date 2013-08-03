#include <avr/io.h>
#include "spi.h"

void toggleLatch()
{
	SHIFT_PORT |= LATCH;
	SHIFT_PORT &= ~LATCH;
}

void setupIOSPI()
{
	SHIFT_REGISTER |= (DATA | LATCH | CLOCK); //Set control pins as outputs
	SHIFT_PORT &= ~(DATA | LATCH | CLOCK);        //Set control pins low
	SHIFT_PORT &= ~LATCH;	//Pull LATCH low (Important: this is necessary to start the SPI transfer!)
}

void setupSPI()
{
	SPCR = (1<<SPE) | (1<<MSTR);  //Start SPI as Master
}

void SPI(unsigned char data)
{
	//Shift in some data
	SPDR = data;
	while(!(SPSR & (1<<SPIF)));
	toggleLatch();
}

void iniciarSPI()
{
	setupIOSPI();
	setupSPI();
}
