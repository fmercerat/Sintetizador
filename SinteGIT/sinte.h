#ifndef _sinte_h_
#define _sinte_h_

#include <inttypes.h>

#define	CUADRADA	0
#define PULSO		1
#define SENO		2
#define	TRIANGULO	3
#define SUPER		4
#define SUPER2		5
#define	SIERRA		6
#define NADA		7
#define RUIDO		8

#define DIV2		1
#define DIV4		2
#define DIV8		3
#define DIV16		4
#define DIV32		5
#define DIV64		6

#define NORMAL		0
#define INVERTIDA	127

#define MONO		0
#define POLY		1
#define SPLIT		2

uint8_t Sierra(uint8_t);
uint8_t Triang(uint8_t);
uint8_t Cuadrada(uint8_t);
uint8_t Pulso(uint8_t, uint8_t);
void nextRand(void);
uint8_t Ruido(void);

#define MAX_LOOKUP 0x7FFF

#define INCREMENTAR_NOTA(note, i)               \
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


#endif
