#ifndef _spi_h_
#define _spi_h_

#define SHIFT_REGISTER DDRB
#define SHIFT_PORT PORTB
#define DATA (1<<PB3)           //MOSI (SI)
#define LATCH (1<<PB2)          //SS   (RCK)
#define CLOCK (1<<PB5)          //SCK  (SCK)

void toggleLatch();
void setupIOSPI();
void setupSPI();
void SPI(unsigned char data);
void iniciarSPI();

#endif
