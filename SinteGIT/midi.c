#include <avr/io.h>
#include <stdint.h>

#include "plat.h"
#include "midi.h"
#include "colas.h"
//#include "uart.h"

//extern volatile uint8_t indiceParsing = 0;
static MIDI m;
//static uint8_t indiceParsing = 0;
static cola col, envio;
static uint8_t tamanoEsperado;

/*
 * Agrega un dato a la cola de datos del USART
 */
void agregarBuffer(uint8_t data)
{
	insertar(data, &col);
}

/*
 * Devuelve separados los datos del primer byte, en canal y tipo de comando
 */
COMANDO separaCMD(uint8_t data)
{
	COMANDO aux;
	aux.cmd = (data >> 4);
	aux.canal = (data & 0x0F);

	return aux;
}

// iniciar cola, necesario
void iniciarCola(void)
{
	crear_cola(&col);
	crear_cola(&envio);
	tamanoEsperado = 0;
	resetM();
}

void resetM()
{
	m.cmd.canal = 0;
	m.cmd.cmd = 0;
	m.data1 = 0;
	m.data2 = 0;
	m.listo = 0;
	m.par = 0;
}

void ejecutaMidi(void)
{

}

uint8_t juntaCMD(COMANDO cm)
{
	return (cm.canal + (cm.cmd << 4));
}

/*void enviarMidi(MIDI mi)
{
	uint8_t aux;
	aux = juntaCMD(mi.cmd);
	uart_putc(aux);
	uart_putc(mi.data1);
	uart_putc(mi.data2);

}
*/
MIDI parsea2()
{
	return m;
}

uint8_t distintoMidi(MIDI a, MIDI b)
{
	return ((juntaCMD(a.cmd) != juntaCMD(b.cmd)) || (a.data1 != b.data1) || (a.data2 != b.data2));
}

uint8_t listoEnvio(void)
{
	return m.listo;
}

uint8_t parsea(void)
{
	if(cola_vacia(col))			// Si no hay datos, no puedo empezar
		return 0;

	if(primero(col) > 127)
	{
		switch(separaCMD(primero(col)).cmd)
		{
			case SYSEX:

				if(quitar_primero(&col) == 0xF8)
				{
					m.cmd.cmd = CLOCK;
					m.listo = 1;
					return 1;
				}
				m.listo = 0;
				return 0;
				break;

			case PROG_CH:
			case CH_PRES:
				tamanoEsperado = 2;
				if(tamanoCola(&col) < tamanoEsperado)	// Faltan datos
					return 0;
				break;

			case NOTE_OFF:
			case NOTE_ON:
			case CC:
			case PITCH_W:
			case POLY_KEY:
				tamanoEsperado = 3;
				if(tamanoCola(&col) < tamanoEsperado)
					return 0;
				break;

			default:
				return -1;
				break;
		}
	}
	else
	{
		quitar_primero(&col);						// quito el primero asi veo que viene
		return 0;									// porque no es cabecera
	}

	if(tamanoEsperado == 0)							// no tendria que entrar
		return -2;
	// Si estoy aca, es que hay por lo menos la misma cantidad de mensajes que los necesarios
	// para completar una estructura MIDI

	m.cmd = separaCMD(quitar_primero(&col));
	m.data1 = quitar_primero(&col);

	if(tamanoEsperado == 2)
	{
		m.listo = 1;
		m.par = 1;
		return 1;
	}
	if(tamanoEsperado == 3)
	{
		m.data2 = quitar_primero(&col);
		m.listo = 0;
		m.par = 2;
		return 1;
	}

	return -1;		// no tendria que pasar
}

void resetMIDI(MIDI *mmm)
{
	mmm->cmd.cmd = 0;
	mmm->cmd.canal = 0;
	mmm->data1 = 0;
	mmm->data2 = 0;
	mmm->listo = 0;
	mmm->par = 0;
}
