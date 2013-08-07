#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
//#include <util/delay.h>
#include <inttypes.h>
#include <stdio.h>

#include "notas.h"
#include "tablas.h"
#include "plat.h"
#include "sinte.h"
#include "midi.h"
#include "pila.h"
//#include "7seg.h"
//#include "spi.h"
//#include "lcd.h"


uint16_t Cont[4];		// Contador para el oscilador
uint8_t Nota[4];		// Nota que va a incrementar el oscilador
uint8_t Data;			// Informacion UART
uint8_t LFO;			// Resultado del oscilador de LFO - valores entre 0x00 y 0xFF, cambia velocidad con
uint8_t velLFO;			// Velocidad vibrato
uint16_t contLFO;		// Contador para funcionamiento de LFO
uint8_t profVibrato;	// Prof vibrato
uint8_t profFiltroLFO;	// Profundidad del LFO en el filtro
uint8_t profVCALFO;		// Profundidad del LFO en el VCA
uint16_t mod;			// Variable para modular el OSC1
uint8_t salida;			// Resultado de los osc
uint8_t adsrCont;		// Contador para el oscilador del ADSR
uint8_t adsrVel[2];		// 0 -> attack   1 -> Release
uint8_t adsrIndex;		// Indice del ADSR, es la salida
uint8_t adVel[2];		// 0 -> attack   1 -> Release
uint8_t adIndex;		// Indice attack decay
uint8_t adInicio;		// Valor desde el que comienza el ataque del filtro
uint8_t adCont;			// Contador para el osc del AD
uint8_t adSust;			// Valor hasta donde cae el decay
uint8_t adTope;			// Saber cuando ya se llego al tope
uint8_t gate;			// Variable para saber cuando hay pulsada una tecla
uint8_t act;			// Variable para saber cuando actualizar PWM asi no se hacen calculos innecesariamente
uint16_t pitchw;		// Variable para controlar cambios realizados por pitch wheel en el oscilador
uint8_t freqFiltro;		// Frecuencia del filtro 0-127
uint8_t freqRes;		// Frecuencia de resonancia
uint8_t continua;		// Valor en el que oscila el VCA
uint8_t fOndaOsc[4];	// Formas de onda del oscilador -> 0 y 1 Nota1 --- 2 y 3 Nota2
uint16_t bpmCont;		// Contador para bpm
uint16_t bpmTop;		// Valor de tope para un bpm determinado
uint8_t volOsc[4];		// Volumenes Osc (0-255)
int8_t oscShift[2];	// Trasposicion de la nota
uint8_t egReset[2];		// 0 AR(VCA) --- 1 AD(VCF)
uint8_t volRuido;
uint8_t sampleHold;		// Activado Sample&Hold
//uint8_t contSH;			// por ahora funciona con el LFO para ahorrar un control
//uint8_t velSH;			//
int8_t arp[8];			//
uint8_t arpIni;			// valor inicial de la nota
uint16_t contArp;
int8_t arpOut;
uint8_t arpeg;			// Seleccion arpegiador
uint8_t arpSH;			// Seleccion ARP y SH 	0 00 - 1 01 - 2 10 - 3 11    // 0 - off | 1 - on

uint8_t dist;			// nivel de distorsion
uint8_t contSPI;		// actualizar display
uint8_t programa;		// numero de programa
/*
 * Reverb
 */
uint8_t buffer[512];
uint16_t bufferIndex;
int iw, iw1;
uint16_t delayTime;
uint8_t profReverb;
uint8_t noTail;

uint8_t prueba;


fixed lpIn,hpIn,difr,fSpeed,fHeight,fDelay,oscFMix;		// Variables para el filtro
uint8_t fCut,fRes;										// Variables para el filtro

void actualizafRes(fixed ff);
void actualizafCut(fixed ff);
void iniciaFiltro(void);
uint8_t correFiltro(fixed entrada);
uint8_t Seno(uint8_t xxx);
uint8_t Super(uint8_t xxx);
uint8_t Super2(uint8_t xxx);

void ejecutaADSR(void);
void ejecutaLFO(void);
void ejecutaAD(void);

uint8_t reverb(uint8_t entrada, uint8_t profundidad, uint16_t delay);
uint8_t onda(uint8_t lugar, uint8_t forma, uint8_t parametro2, uint8_t div);
void grabador(void);
void lector(void);
int8_t shiftMIDI(uint8_t valorMIDI);

int main()
{
	MIDI mm,mp;
	uint8_t aux,i;
	pila p;
//	uint8_t auxFiltro = 0;
//	uint16_t j=0;
//	char Cad[36];
	uint16_t adsrAux;
	uint8_t auxVCA=0;
	uint16_t pw = 0x2000;
	uint16_t pwp = 0x2000;
	uint8_t estSW1 = 0;
	uint8_t estSW2 = 0;

	crear_pila(&p);
	for(i=0;i<TAMANO_MAXIMO_PILA;i++)
		apilar(0,&p);
	while(desMidi(&p));

	iniciaFiltro();

	iniciaPlataforma();
	inicia20KHzT0();
	iniciaPWMOCR1A();
	iniciarCola();
	iniciarUARTMIDI();

//	iniciaLCD();

	resetMIDI(&mp);
	resetMIDI(&mm);

	Cont[0] = 0;
	Cont[1] = 0;
	Cont[2] = 0;
	Cont[3] = 0;
	Nota[0] = 0;
	Nota[1] = 0;
	Nota[2] = 0;
	Nota[3] = 0;
	volOsc[0] = 255;
	volOsc[1] = 0;
	volOsc[2] = 255;
	volOsc[3] = 0;
	egReset[0] = 0;
	egReset[1] = 0;

	profVibrato = 0;
	LFO = 0;
	velLFO = 60;
	act = 0;
	profFiltroLFO = 127;
	freqFiltro = FP_ONE;
	profVCALFO = 0;
	continua = 0;

	gate = 0;
	adsrCont = 0;
	adsrIndex = 0;
	adsrVel[0] = 0;
	adsrVel[1] = 0;
	adVel[0] = 0;
	adVel[1] = 1;
	adInicio = 0;

	fOndaOsc[0] = CUADRADA;
	fOndaOsc[1] = TRIANGULO;
	fOndaOsc[2] = TRIANGULO;
	fOndaOsc[3] = TRIANGULO;

	oscShift[0] = 7;
	oscShift[1] = 7;

	sampleHold = 0;

	arp[0] = 0;
	arp[1] = 2;
	arp[2] = 0;
	arp[3] = 2;
	arp[4] = 3;
	arp[5] = 2;
	arp[6] = 0;
	arp[7] = 7;

	arpIni = 0;
	arpeg = 0;		//	Apagado inicialmente
	dist = 0;

	profReverb = 0;
	delayTime = 60;
	noTail = 1;

	pitchw = 0;

	LFO = 0;
	contLFO = 0;
	prendeLedR();
	DDRB |= (1 << PB0);
	PORTB |= (1 << PB0);

	iniciaFiltro();

	for(bufferIndex = 0; bufferIndex < 512; bufferIndex++)
	{
		buffer[bufferIndex] = 0;
	}

	bufferIndex = 0;

	sei();

	actualizafCut(freqFiltro);
	while(1)
	{
		aux = parsea();
		if(aux == 1)
			mm = parsea2();
		resetM();

		if(distintoMidi(mm,mp))
		{
			switch(mm.cmd.cmd)
			{
				case NOTE_ON:
				{
					Nota[0] = mm.data1;
	//				Nota[2] = Nota[0] - 12;
					apilar(Nota[0],&p);
	//				ultimaNON = Nota[0];
					if(p.cima>0)
						Nota[2] = p.vector[p.cima-1];
					else
						Nota[2] = 0;
					prendeLedA();
					if(gate)
					{
						if(egReset[0])
						{
							adsrCont = 0;
							adsrIndex = 0;
						}
						if(egReset[1])
						{
							adCont = 0;
							adIndex = adInicio;
							adTope = 0;
						}
					}
					else
					{
						adsrCont = 0;
						adsrIndex = 0;
						adCont = 0;
						adIndex = adInicio;
						adTope = 0;
					}
					gate = 1;
					break;
				}

				case NOTE_OFF:
				{
					if(mm.data1 == cima(p))
					{
						if(!pila_vacia(p))
						{
							p.vector[p.cima] = 0;
							desapilar(&p);
							if(!pila_vacia(p))
							{
								Nota[0] = p.vector[p.cima];
								if(p.cima-1>0)
									Nota[2] = p.vector[p.cima-1];
								else
									Nota[2] = 0;
							}
							else
							{
			//					Nota[0] = 0;
								apagaLedA();
								gate = 0;
							}
						}
					}
					else
					{
						eliminar2(&p,mm.data1);
						if(mm.data1 == Nota[2])
						{	if(p.cima>0)
								Nota[2] = p.vector[p.cima-1];
							else
								Nota[2] = 0;
						}
					}

					break;
				}

				case PITCH_W:
				{
					pw = mm.data1 + (mm.data2 << 7);

					if(pw >= 0x2000)
					{
	//					if((pw-pwp) >= 0)
						if(pw >= pwp)
							pitchw += (pw-pwp) >> 7;
						else
							pitchw -= (pwp-pw) >> 7;
					}
					else
					{
	//					if((pw-pwp) < 0)
						if(pw < pwp)
							pitchw -= (pwp-pw) >> 7;
						else
							pitchw += (pw-pwp) >> 7;
					}

					if(pw == 0x2000)
						pitchw = 0;

					pwp = pw;

					break;
				}
				case PROG_CH:
				{
					programa = mm.data1;
					break;
				}

				case CC:
				{
					switch(mm.data1)
					{
						case 74:
							actualizafCut(mm.data2);
							freqFiltro = mm.data2;
							break;

						case 71:
							actualizafRes(mm.data2);
							freqRes = mm.data2;
							break;

						case 27:
							velLFO = 64 - (mm.data2 >> 1);
							break;

						case 76:
							profVibrato =mm.data2;
							break;

						case 73:
							adsrVel[0] = mm.data2;
							break;

						case 72:
							adsrVel[1] = mm.data2;
							break;

						case 70:
							continua = mm.data2 << 1;
							break;

						case 80:	// profundidad filtro
							profFiltroLFO = 130 - mm.data2;
							if(profFiltroLFO >= 126)
								actualizafCut(freqFiltro);
							break;

						case 81:
							profVCALFO = mm.data2;
							break;

						case 1:
					//		freqFiltro = mm.data2;
					//		actualizafCut(freqFiltro);
					//		profVibrato = mm.data2;
							dist = mm.data2;
							break;

						case 77:		//	Forma OSC1
							fOndaOsc[0] = mm.data2 >> 4;
							break;

						case 79:
							fOndaOsc[1] = mm.data2 >> 4;
							break;

						case 78:		//	Forma OSC2
							fOndaOsc[2] = mm.data2 >> 4;
							break;

						case 20:
							volOsc[0] = mm.data2 << 1;
							break;

						case 21:
							volOsc[1] = mm.data2 << 1;
							break;

						case 23:
							adVel[0] = mm.data2;
							break;

						case 24:
							adVel[1] = mm.data2;
							break;

						case 18:
							oscShift[0] = shiftMIDI(mm.data2);
							break;

						case 94:
							profReverb = mm.data2 << 1;
							break;

						case 13:
							delayTime = mm.data2 << 2;
							break;

						case 28:
							noTail = mm.data2 >> 6;
							break;

						case 83:
							arpSH = mm.data2 >> 5;
							sampleHold = arpSH >> 1;
							arpeg = arpSH & 1;
							actualizafCut(freqFiltro);
							break;

						case 22:
							volRuido = mm.data2;
							break;

						default:
							break;
					}

					break;
				}
			}
		}

		mp=mm;
/*		if(gate)
		{
			salida = ((Cuadrada(Cont[0]>>7) + Triang(Cont[2]>>7))>>1);
			OCR1A = salida;
		}
		else
			OCR1A = 0;
*/
		if(act)
		{
//			display72(programa);
//			salida = ((Cuadrada(Cont[0]>>7) + Triang(Cont[2]>>7))>>1); //+ (Seno(LFO)>>4);
			if(fOndaOsc[2] == NADA)
				salida = (onda(Cont[0]>>7, fOndaOsc[0], 25, volOsc[0]) +
						  ((Ruido()*volRuido)>>7) +
						  onda(Cont[1]>>7, fOndaOsc[1], 25, volOsc[1]) +
						  onda(Cont[3]>>7, fOndaOsc[3], 25, volOsc[3]));// >> DIV2;
			else
				salida = (onda(Cont[0]>>7, fOndaOsc[0], 25, volOsc[0]) +
						  onda(Cont[1]>>7, fOndaOsc[1], 25, volOsc[1]) +
						  onda(Cont[2]>>7, fOndaOsc[2], 25, volOsc[2]) +
						  onda(Cont[3]>>7, fOndaOsc[3], 15, volOsc[3])) >> DIV4;
			adsrAux = salida;

			if(adVel[0] > 1)
				actualizafCut(adIndex>>1);

			if(profFiltroLFO < 126)
			{
				auxVCA = fCut + onda(LFO,SENO,0,127-profFiltroLFO+continua);//freqFiltro + onda(LFO,TRIANGULO,0,127-profFiltroLFO);
//				auxVCA = freqFiltro + Triang(LFO)/profFiltroLFO;
				if(auxVCA > 127)
					auxVCA = 127;
				actualizafCut(auxVCA);
			}

			if(sampleHold)
			{
				if(LFO == 0)
				{
					auxVCA = Ruido() >> 1;
					auxVCA += 15;
					if(auxVCA > 127)
						auxVCA = 127;
					actualizafCut(auxVCA);
				}
			}

//			if(fCut < 125)
				adsrAux = correFiltro(salida);

			adsrAux = (adsrAux * adsrIndex) >> 8;



			if(profVCALFO)		// 31<<2 = 124
			{
				auxVCA = Triang(LFO)/(33-(profVCALFO>>2));

				if(auxVCA + continua > 255)		//	64 Nivel de continua
					auxVCA = 255;
				else
					auxVCA = auxVCA + continua;

				adsrAux = (adsrAux * auxVCA) >> 7;
			}

			if(profReverb)
			{
				if(noTail)
				{
					if(gate)
					{
						adsrAux = reverb(adsrAux, profReverb, delayTime);
					}
					else
					{
						reverb(0, profReverb, delayTime);
					}
				}
				else
				{
					adsrAux = reverb(adsrAux, profReverb, delayTime);
				}
			}

			if(dist)
			{
				adsrAux = (adsrAux * dist) >> 5;
				if(adsrAux > 255)
					adsrAux = 255;
				adsrAux = ((adsrAux * adsrAux) >> 10) + ((((adsrAux * adsrAux)>>8)*adsrAux)>>7);

				if(adsrAux > 255)
					adsrAux = 255;
			}

			OCR1A = adsrAux;
			act = 0;
		}

		if(estadoSW1() && !estSW1)
		{
			estSW1 = 1;
			grabador();
		}
		if(!estadoSW1() && estSW1)
			estSW1 = 0;

		if(estadoSW2() && !estSW2)
		{
			estSW2 = 1;
			lector();
		}
		if(!estadoSW2() && estSW2)
			estSW2 = 0;

	}
}

ISR(TIMER0_OVF_vect)
{
//	TCNT0 += 192;
	TCNT0 += 156;
	if(Nota[0])
	{
//		mod = notas[Nota[0]] + pitchw + (Seno(LFO) / (profVibrato+1)) ;//(Seno(LFO) >>  (profVibrato >> 4));	// Control profundidad vibrato 0~64
		mod = notas[Nota[0] + arp[arpIni]] + pitchw;
		if(profVibrato)
		{
			mod += (Seno(LFO) * profVibrato) >> 9;
		}
		INCREMENT_NOTE(mod,Cont[0]);
		INCREMENT_NOTE(notas[Nota[0] + oscShift[0]] + ((Seno(LFO) * profVibrato) >> 9) + pitchw,Cont[1]);

		if(fOndaOsc[2] == NADA)
		{
			INCREMENT_NOTE(0,Cont[2]);
			INCREMENT_NOTE(notas[Nota[0] + oscShift[1]] + ((Seno(LFO) * profVibrato) >> 9) + pitchw,Cont[3]);
		}
		else
		{
			INCREMENT_NOTE(notas[Nota[2]],Cont[2]);
		}

	}

	act = 1;

	ejecutaAD();
	ejecutaADSR();
	ejecutaLFO();


	if(arpeg)
	{
//		contArp++;
//		if(contArp > 5000)
//			contArp = 0;
//		if(!contArp)
		if(!LFO)
		{
			arpIni++;
			if(arpIni > 7)
				arpIni = 0;
		}
		if(!gate)
		{
			arpIni = 0;
//			contArp = 0;
		}
	}
}

ISR (USART_RXC_vect)
{
	Data = UDR;                     // Read data from the RX buffer
	agregarBuffer(Data);
}

void ejecutaLFO(void)
{
	contLFO++;
	if(contLFO >= velLFO)	// Control velocidad vibrato 0~64
	{
		contLFO = 0;
		LFO++;
		if(LFO > 255)
			LFO = 0;
	}
}

void ejecutaADSR(void)
{
	if(gate)
	{
		if(adsrCont == adsrVel[0])
		{
			adsrCont = 0;
			if(adsrIndex < 255)
				adsrIndex++;
			if(adsrVel[0]==0)
				adsrIndex = 255;
		}

	}
	else
	{
		if(adsrCont == adsrVel[1])
		{
			adsrCont = 0;
			if(adsrIndex > 1)
			{
				adsrIndex--;
			}
			if(adsrVel[1]==0)
				adsrIndex = 0;
		}
	}
	if(!egReset[1] && adsrIndex == 0)
		adIndex = 0;

	adsrCont++;

	if(adsrCont > 255)
			adsrCont = 0;
}

/*		j++;
		if(j==5000)
		{
			sprintf(Cad,"%d %d %d %d %d\n%d   %d %d",p.vector[0],p.vector[1],p.vector[2],p.vector[3],p.vector[4],p.cima+1,Nota[0],Nota[2]);
//			sprintf(Cad,"%d  %d",p.cima+1,p.vector[p.cima]);
			escribeLCD(Cad);
			j=0;
		}
*/

uint8_t Seno(uint8_t xxx)
{
	return pgm_read_byte(&seno[xxx]);
}

uint8_t Super(uint8_t xxx)
{
	return pgm_read_byte(&super[xxx]);

}

uint8_t Super2(uint8_t xxx)
{
	return pgm_read_byte(&super2[xxx]);
}

void actualizafRes(fixed ff)
{
	fixed aux;
	aux = FP_ONE - ff;
	fRes = FP_ONE - (fp_mul(fp_mul(aux,aux),aux));
}

void actualizafCut(fixed ff)
{
	fCut = fp_mul(ff, ff);
}

void iniciaFiltro(void)
{
	freqFiltro = FP_ONE;
	actualizafCut(freqFiltro);
	actualizafRes(0);
	lpIn=hpIn=difr=fSpeed=fHeight=fDelay=0;
	oscFMix = 0;
}

uint8_t correFiltro(fixed entrada)
{
	lpIn = fp_mul(entrada, FP_ONE - oscFMix);
	hpIn = - fp_mul(entrada,oscFMix);

	difr = lpIn - fHeight;

	fSpeed = fp_mul(fSpeed, fRes);
	fSpeed = fSpeed + fp_mul(difr, fCut);

	fHeight += fSpeed;
	fHeight += fDelay - hpIn;

	fDelay = hpIn;

	if(fHeight < 0)
		fHeight = 0;
	if(fHeight > 255)
		fHeight = 255;

	return fHeight;
}

void ejecutaAD(void)
{
	if(gate)
	{
		if(adCont == adVel[0] && !adTope)
		{
			adCont = 0;
			if(adIndex < 255)
				adIndex++;
			else
				adTope = 1;
		}

		if(adTope && adVel[1]==adCont)
		{
			adCont = 0;
			if(adIndex > (freqFiltro<<1))//adSust)
				adIndex--;
		}
	}

	adCont++;

	if(adCont > 255)
			adCont = 0;
}

uint8_t onda(uint8_t lugar, uint8_t forma, uint8_t parametro2, uint8_t div)
{
	switch(forma)
	{
		case CUADRADA:
			return (Cuadrada(lugar) * div) >> 8;
			break;

		case PULSO:
			return (Pulso(lugar, parametro2) * div) >> 8;
			break;

		case SENO:
			return (Seno(lugar) * div) >> 8;
			break;

		case TRIANGULO:
			if(parametro2 < 64)
			{
				return (Triang(lugar) * div) >> 8;
			}
			else
			{
				return ((255 - Triang(lugar)) * div) >> 8;
			}
			break;

		case SUPER:
			return (Super(lugar) * div) >> 8;
			break;

		case SUPER2:
			return (Super2(lugar) * div) >> 8;
			break;

		case SIERRA:
			if(parametro2 < 64)
			{
				return (Sierra(lugar) * div) >> 8;
			}
			else
			{
				return ((255 - Sierra(lugar)) * div) >> 8;
			}

			break;

		case NADA:
			return 0;
			break;

		case RUIDO:
			return (Ruido() * div) >> 8;
			break;

		default:
			return (Sierra(lugar) * div) >> 8;
			break;
	}

	return (Sierra(lugar) * div) >> 8;
}

uint8_t reverb(uint8_t x, uint8_t prof, uint16_t delay)
{
	iw = 127 - buffer[bufferIndex];
	iw = iw * prof/255;
	iw1 = 127 - x;
	iw1 = iw1 + iw;

	if (iw1 < -127) iw1=-127;		// Limites
	if (iw1 > 127) iw1=127;

	buffer[bufferIndex] = 127 + iw1;

	bufferIndex++;
//	bufferIndex = bufferIndex & 511;

	if(bufferIndex == delay) bufferIndex = 0;

	return iw1+127;
}

void grabador(void)
{
	uint8_t aux;
	aux= programa*31;

	eeprom_update_byte((uint8_t *)0 + aux, velLFO);
	eeprom_update_byte((uint8_t *)1 + aux, profVibrato );
	eeprom_update_byte((uint8_t *)2 + aux, profFiltroLFO );
	eeprom_update_byte((uint8_t *)3 + aux, profVCALFO );
	eeprom_update_byte((uint8_t *)4 + aux, adsrVel[0] );
	eeprom_update_byte((uint8_t *)5 + aux, adsrVel[1] );
	eeprom_update_byte((uint8_t *)6 + aux, adVel[0] );
	eeprom_update_byte((uint8_t *)7 + aux, adVel[1] );
	eeprom_update_byte((uint8_t *)8 + aux, adSust );
	eeprom_update_byte((uint8_t *)9 + aux, freqFiltro );
	eeprom_update_byte((uint8_t *)10 + aux, freqRes );
	eeprom_update_byte((uint8_t *)11 + aux, continua );
	eeprom_update_byte((uint8_t *)12 + aux, fOndaOsc[0] );
	eeprom_update_byte((uint8_t *)13 + aux, fOndaOsc[1] );
	eeprom_update_byte((uint8_t *)14 + aux, fOndaOsc[2] );
	eeprom_update_byte((uint8_t *)15 + aux, fOndaOsc[3] );
	eeprom_update_byte((uint8_t *)16 + aux, volOsc[0] );
	eeprom_update_byte((uint8_t *)17 + aux, volOsc[1] );
	eeprom_update_byte((uint8_t *)18 + aux, volOsc[2] );
	eeprom_update_byte((uint8_t *)19 + aux, volOsc[3] );
	eeprom_update_byte((uint8_t *)20 + aux, oscShift[0] );
	eeprom_update_byte((uint8_t *)21 + aux, oscShift[1] );
	eeprom_update_byte((uint8_t *)22 + aux, egReset[0] );
	eeprom_update_byte((uint8_t *)23 + aux, egReset[1] );
	eeprom_update_byte((uint8_t *)24 + aux, volRuido );
	eeprom_update_byte((uint8_t *)25 + aux, delayTime );
	eeprom_update_byte((uint8_t *)26 + aux, profReverb );
	eeprom_update_byte((uint8_t *)27 + aux, noTail );
	eeprom_update_byte((uint8_t *)28 + aux, sampleHold );
	eeprom_update_byte((uint8_t *)29 + aux, arpSH );
	eeprom_update_byte((uint8_t *)30 + aux, dist );
}

void lector(void)
{
	uint8_t aux;
	aux = programa * 31;

	velLFO = eeprom_read_byte((uint8_t *)0 + aux);
	profVibrato = eeprom_read_byte((uint8_t *)1 + aux);
	profFiltroLFO = eeprom_read_byte((uint8_t *)2 + aux);
	profVCALFO = eeprom_read_byte((uint8_t *)3 + aux);
	adsrVel[0] = eeprom_read_byte((uint8_t *)4 + aux);
	adsrVel[1] = eeprom_read_byte((uint8_t *)5 + aux);
	adVel[0] = eeprom_read_byte((uint8_t *)6 + aux);
	adVel[1] = eeprom_read_byte((uint8_t *)7 + aux);
	adSust = eeprom_read_byte((uint8_t *)8 + aux);
	freqFiltro = eeprom_read_byte((uint8_t *)9 + aux);
	freqRes = eeprom_read_byte((uint8_t *)10 + aux);
	continua = eeprom_read_byte((uint8_t *)11 + aux);
	fOndaOsc[0] = eeprom_read_byte((uint8_t *)12 + aux);
	fOndaOsc[1] = eeprom_read_byte((uint8_t *)13 + aux);
	fOndaOsc[2] = eeprom_read_byte((uint8_t *)14 + aux);
	fOndaOsc[3] = eeprom_read_byte((uint8_t *)15 + aux);
	volOsc[0] = eeprom_read_byte((uint8_t *)16 + aux);
	volOsc[1] = eeprom_read_byte((uint8_t *)17 + aux);
	volOsc[2] = eeprom_read_byte((uint8_t *)18 + aux);
	volOsc[3] = eeprom_read_byte((uint8_t *)19 + aux);
	oscShift[0] = eeprom_read_byte((uint8_t *)20 + aux);
	oscShift[1] = eeprom_read_byte((uint8_t *)21 + aux);
	egReset[0] = eeprom_read_byte((uint8_t *)22 + aux);
	egReset[1] = eeprom_read_byte((uint8_t *)23 + aux);
	volRuido = eeprom_read_byte((uint8_t *)24 + aux);
	delayTime = eeprom_read_byte((uint8_t *)25 + aux);
	profReverb = eeprom_read_byte((uint8_t *)26 + aux);
	noTail = eeprom_read_byte((uint8_t *)27 + aux);
	sampleHold = eeprom_read_byte((uint8_t *)28 + aux);
	arpSH = eeprom_read_byte((uint8_t *)29 + aux);
	dist = eeprom_read_byte((uint8_t *)30 + aux);

	sampleHold = arpSH >> 1;
	arpeg = arpSH & 1;
	actualizafCut(freqFiltro);
	actualizafRes(freqRes);
	LFO = 0;
	contLFO = 0;
}

int8_t shiftMIDI(uint8_t valorMIDI)
{
	return (valorMIDI * 49 / 128) - 24;
}
