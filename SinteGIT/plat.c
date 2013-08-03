#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>

#include "pila.h"
#include "plat.h"
//#include "lcd.h"

//static uint16_t over;
/*
void inicia20KHzT2(void)
{
	OCR2 = 99; // 20KHz
	TCCR2 |= (1 << WGM21);		// CTC
	TIMSK |= (1 << OCIE2);		// Inte3rrupt
	TCCR2 |= (1 << CS21);		// 8 Prescaler
}
*/
void iniciaPWMOCR1A(void)
{
	DDRB |= (1 << DDB1);		// OCR1A es salida
	TCCR1A |= (1 << COM1A1);	// Non inverting mode
	TCCR1A |=  (1 << WGM10);	// 8bit
	TCCR1B |=  (1 << WGM12) | (1 << CS10);		// Prescaler 8
//	ICR1 = 0xFFFF;
	OCR1A = 0;
}

void inicia20KHzT0(void)
{
	TIMSK |= (1 << TOIE0);
	TCCR0 |= (1 << CS01);

	// Es necesario incluir en el ISR
	// TCNT0 += 156;
}

void iniciaLedR(void)
{
	REGISTRO_LEDR |= (1 << LEDR);			// Pone en modo salida ese bit
}

void iniciaLedA(void)
{
	REGISTRO_LEDA |= (1 << LEDA);			// Pone en modo salida ese bit
}

void iniciaSW1(void)
{
	REGISTRO_SW1 &= ~(1 << SW1);			// Pone en modo entrada ese bit
	PUERTO_SW1 |= (1 << SW1);				// Activa Pull-UP
}

void iniciaSW2(void)
{
	REGISTRO_SW2 &= ~(1 << SW2);			// Pone en modo entrada ese bit
	PUERTO_SW2 |= (1 << SW2);				// Activa Pull-UP
}

void prendeLedR(void)
{
	PUERTO_LEDR |= (1 << LEDR);				// Pone en 1 el valor del bit para prender
}

void prendeLedA(void)
{
	PUERTO_LEDA |= (1 << LEDA);				// Pone en 1 el valor del bit para prender
}

void apagaLedR(void)
{
	PUERTO_LEDR &= ~(1 << LEDR);			// Pone en 0 el valor del bit para prender
}

void apagaLedA(void)
{
	PUERTO_LEDA &= ~(1 << LEDA);			// Pone en 0 el valor del bit para prender
}

void permutaLedR(void)
{
	PUERTO_LEDR ^= (1 << LEDR);				// Invierte el valor del bit
}

void permutaLedA(void)
{
	PUERTO_LEDA ^= (1 << LEDA);				// Invierte el valor del bit
}
uint8_t estadoSW1()							// 1 activado, 0 desactivado
{
	if(bit_is_clear(PIN_SW1,SW1))		// Si hay un 0 es que esta a tierra el pin
		return 1;

	return 0;
}
uint8_t estadoSW2()							// 1 activado, 0 desactivado
{
	if(bit_is_clear(PIN_SW2,SW2))		// Si hay un 0 es que esta a tierra el pin
		return 1;

	return 0;
}
/*
uint8_t estadoSW1()							// 1 activado, 0 desactivado
{
	if(bit_is_clear(PIN_SW1,SW1))		// Si hay un 0 es que esta a tierra el pin
	{										// por lo que se esta presionando el boton
//		_delay_ms(25);
		if(bit_is_clear(PIN_SW1,SW1))
			return 1;
	}

	return 0;
}

uint8_t estadoSW2()							// 1 activado, 0 desactivado
{
	if(bit_is_clear(PIN_SW2,SW2))		// Si hay un 0 es que esta a tierra el pin
	{										// por lo que se esta presionando el boton
//		_delay_ms(25);
		if(bit_is_clear(PIN_SW2,SW2))
			return 1;
	}

	return 0;
}*/

void esperarPresionSW1(void)
{
	loop_until_bit_is_clear(PIN_SW1, SW1);
}

void esperarPresionSW2(void)
{
	loop_until_bit_is_clear(PIN_SW2, SW2);
}

void iniciaPlataforma(void)
{
	iniciaLedA();
	iniciaLedR();
	iniciaSW1();
	iniciaSW2();
}
/*
void iniciaTimer1s(void)
{
	OCR1A = 15624;		//  [ (clock_speed / Prescaler) * tiempo_deseado ] - 1
	TCCR1B |= (1 << WGM12);					// Define en CTC (clear timer on compare)
	TIMSK |= (1 << OCIE1A);					// Habilita interruciones en comparacion
	TCCR1B |= (1 << CS12) | (1 << CS10);  	// Pone el prescaler en 1024
}

void iniciaTimer(uint16_t segs)
{
	uint16_t comp = (uint16_t) ((CLK_S / 1024) * segs) - 1;
	OCR1A = comp;		//  [ (clock_speed / Prescaler) * tiempo_deseado ] - 1
	TCCR1B |= (1 << WGM12);					// Define en CTC (clear timer on compare)
	TIMSK |= (1 << OCIE1A);					// Habilita interruciones en comparacion
	TCCR1B |= (1 << CS12) | (1 << CS10);  	// Pone el prescaler en 1024
}
*/
void iniciarUART(uint8_t prescaler, uint8_t modo, uint8_t interrupciones, uint8_t sinc)
{
	UBRRH = (prescaler >> 8);		// Ajustes de prescaler
	UBRRL = prescaler;

	switch(modo){
		case UART_TX:
			UCSRB |= (1 << TXEN);
			break;

		case UART_RX:
			UCSRB |= (1 << RXEN);
			break;

		case UART_RXTX:
			UCSRB |= (1 << RXEN) | (1 << TXEN);
			break;

		default:
			UCSRB |= (1 << RXEN) | (1 << TXEN);
			break;
	}

	switch(interrupciones){
		case UART_CON_INTERRUPCIONES:
			UCSRB |= (1 << RXCIE);
			break;

		case UART_SIN_INTERRUPCIONES:
			UCSRB &= ~(1 << RXCIE);
			break;

		default:
			break;
	}

	switch(sinc){
		case UART_SINC:
			UCSRC |= (1 << URSEL) |(1 << UCSZ1) | (1 << UCSZ0);			// 8bits sincronicos
			break;

		case UART_ASINC:
			UCSRC |= (1 << UCSZ1) | (1 << UCSZ0);						// 8bits asincronicos
			break;

		default:
			break;
	}
}

void iniciarUARTMIDI()
{
	iniciarUART(31, UART_RXTX, UART_CON_INTERRUPCIONES, UART_SINC);
}
/*
void iniciaLCD()
{
//	lcd_init(LCD_DISP_ON);
//	lcd_clrscr();
}

void escribeLCD(const char *s)
{
//	lcd_clrscr();
//	lcd_home();
//	lcd_puts(s);
}
*/
/*
void iniciarPWM(void)
{
	DDRB |= (1 << PB1);						// PB1-OCR1A  Es salida

	OCR1A = 0;								// Pone en 0 el comparador

	TCCR1A |= (1 << COM1A1);				// Non inverting mode
	TCCR1A |= (1 << WGM10);					// Fast PWM, 8bit
	TCCR1B |= (1 << WGM12) | (1 << CS11);	// Fast PWM, 8bit,  No prescaler
}

void iniciarPWMICR(void)
{
	DDRB |= (1 << PB1);						// PB1-OCR1A  Es salida

	OCR1A = 0;								// Pone en 0 el comparador

	TCCR1A |= (1 << COM1A1) | (1 << WGM11);	//  Non inverting mode Phase corrected PWM
	TCCR1B |= (1 << WGM13) | (1 << CS11) | (1 << CS10);	// Fast PWM, 8bit,  64 prescaler
}
*/
/*
void iniciaTimer01ms(void)
{
	TIMSK |= (1 << TOIE0);					// Habilita interrupciones en overflow timer 0
	TCCR0 |= (1 << CS01) | (1 << CS00);		// Prescaler 64


 *  Hay que agregar
 *
 * 	ISR (TIMER0_OVF_vect)  // timer0 overflow interrupt
 *	{
 *		TCNT0 += 6;
 *   	over++;
 *	}
 *
 *	static uint16_t over;
 */
//}
/*
uint8_t comparaTimer0(uint16_t dato)
{
	if(over >= dato)
	{
		over = 0;
		return 1;
	}
	else
		return 0;
}
*/
void iniciarPila(pila *p)
{
	uint8_t i;

	crear_pila(p);
	for(i=0;i<TAMANO_MAXIMO_PILA;i++)
		apilar(0,p);
	while(desMidi(p));
}


