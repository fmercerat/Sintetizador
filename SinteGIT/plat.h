#ifndef _plat_h_
#define _plat_h_

#include "pila.h"

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#define CLK_S 16000000

#define PUERTO_LEDR PORTB
#define PUERTO_LEDA PORTD

#define REGISTRO_LEDR DDRB
#define REGISTRO_LEDA DDRD

#define LEDR PB0
#define LEDA PD7

#define PUERTO_SW1 PORTD
#define PUERTO_SW2 PORTD

#define REGISTRO_SW1 DDRD
#define REGISTRO_SW2 DDRD

#define PIN_SW1 PIND
#define PIN_SW2 PIND

#define SW1 PD5
#define SW2 PD6

#define UART_MIDI 31

#define UART_RXTX 0
#define UART_RX 1
#define UART_TX 2
#define UART_CON_INTERRUPCIONES 0
#define UART_SIN_INTERRUPCIONES 1
#define UART_ASINC 0
#define UART_SINC 1


void iniciaLedR(void);
void iniciaLedA(void);
void iniciaSW1(void);
void iniciaSW2(void);

void prendeLedR(void);
void prendeLedA(void);
void apagaLedR(void);
void apagaLedA(void);
void permutaLedR(void);
void permutaLedA(void);

uint8_t estadoSW1(void);
uint8_t estadoSW2(void);

void esperarPresionSW1(void);
void esperarPresionSW2(void);

void iniciaPlataforma(void);

void iniciaTimer1s(void);
void iniciaTimer(uint16_t);

void iniciarUART(uint8_t prescaler, uint8_t modo, uint8_t interrupciones, uint8_t sinc);
void iniciarUARTMIDI();

void iniciaLCD(void);
void escribeLCD(const char *s);

void iniciarPWM(void);
void iniciarPWMICR(void);

void iniciaTimer01ms(void);
uint8_t comparaTimer0(uint16_t);

#define MAX_LOOKUP 0x7FFF

#define INCREMENT_NOTE(note, i)                 \
  if (note)                                     \
    {                                           \
      if (i + note < MAX_LOOKUP)                \
        {                                       \
          i += note;                            \
        }                                       \
      else                                      \
        {                                       \
          i = note - (MAX_LOOKUP - i);          \
        }                                       \
    }                                           \
  else                                          \
    {                                           \
      i = 0;                                    \
    }


void inicia20KHzT2(void);
void iniciaPWMOCR1A(void);
void inicia20KHzT0(void);

void iniciarPila(pila *p);

typedef int16_t fixed;

#define FIXED_SHIFT		7
#define FIXED_SCALE		(1 << FIXED_SHIFT)

#define i2fp(a)			((a) << FIXED_SHIFT)
#define fp2i(a)			((a) >> FIXED_SHIFT)

#define fp_add(a, b)	((a) + (b))
#define fp_sub(a, b)	((a) - (b))

#define FP_FRACMASK FIXED_SCALE-1 ;

#define fp_mul(x,y)		((fixed)(((int32_t)(x) * (int32_t)(y)) >> FIXED_SHIFT))


#define FP_ONE		(1 << FIXED_SHIFT)

#endif
