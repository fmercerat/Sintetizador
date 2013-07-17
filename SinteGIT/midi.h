#ifndef _midi_h_
#define _midi_h_

#include <stdint.h>
#include "colas.h"

#define NOTE_OFF	0b1000
#define NOTE_ON		0b1001
#define POLY_KEY	0b1010
#define CC			0b1011
#define PROG_CH		0b1100
#define CH_PRES		0b1101
#define PITCH_W		0b1110
#define SYSEX		0b1111
#define CLOCK       0b11111000

/*
	NoteOff	              = 0x80,	///< Note Off
	NoteOn                = 0x90,	///< Note On
	AfterTouchPoly        = 0xA0,	///< Polyphonic AfterTouch
	ControlChange         = 0xB0,	///< Control Change / Channel Mode
	ProgramChange         = 0xC0,	///< Program Change
	AfterTouchChannel     = 0xD0,	///< Channel (monophonic) AfterTouch
	PitchBend             = 0xE0,	///< Pitch Bend
	SystemExclusive       = 0xF0,	///< System Exclusive
	TimeCodeQuarterFrame  = 0xF1,	///< System Common - MIDI Time Code Quarter Frame
	SongPosition          = 0xF2,	///< System Common - Song Position Pointer
	SongSelect            = 0xF3,	///< System Common - Song Select
	TuneRequest           = 0xF6,	///< System Common - Tune Request
	Clock                 = 0xF8,	///< System Real Time - Timing Clock
	Start                 = 0xFA,	///< System Real Time - Start
	Continue              = 0xFB,	///< System Real Time - Continue
	Stop                  = 0xFC,	///< System Real Time - Stop
	ActiveSensing         = 0xFE,	///< System Real Time - Active Sensing
	SystemReset           = 0xFF,	///< System Real Time - System Reset
	InvalidType           = 0x00    ///< For notifying errors
*/

typedef struct comando {
	uint8_t cmd;
	uint8_t canal;
}COMANDO;

typedef struct midi{
	COMANDO cmd;
	uint8_t data1;
	uint8_t data2;
	uint8_t par;
	uint8_t listo;
}MIDI;

#define ultimoMidi bufferMIDI[indiceBufferMIDIUL]

void agregarBuffer(uint8_t);
void parsea3(void);
COMANDO separaCMD(uint8_t);
uint8_t juntaCMD(COMANDO);
void ejecutaMidi(void);
void iniciarCola(void);
void enviarMidi(MIDI);
uint8_t transmitirMIDI(void);
MIDI parsea2(void);
uint8_t distintoMidi(MIDI a, MIDI b);
uint8_t listoEnvio(void);
uint8_t parsea(void);
void resetM(void);
void resetMIDI(MIDI *);


#endif
