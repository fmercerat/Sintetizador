#ifndef _pila_h_
#define _pila_h_

#include <inttypes.h>

#ifndef TAMANO_MAXIMO_PILA
#define TAMANO_MAXIMO_PILA 16
#endif

typedef uint8_t tipo_elemento;

typedef struct {
	int cima;
	tipo_elemento vector[TAMANO_MAXIMO_PILA];
} pila;

void crear_pila(pila *);
uint8_t pila_vacia(pila);
uint8_t apilar(tipo_elemento, pila *);
tipo_elemento cima(pila);
uint8_t desapilar(pila *);
uint8_t eliminar(pila *, tipo_elemento);
uint8_t eliminar2(pila *, tipo_elemento);
uint8_t desMidi(pila *);

#endif
