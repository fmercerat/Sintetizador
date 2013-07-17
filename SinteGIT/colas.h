#ifndef _colas_h_
#define _colas_h_

#include <stdint.h>

#ifndef TAMANO_MAXIMO_COLA
#define TAMANO_MAXIMO_COLA 32
#endif

typedef uint8_t tipo_elemento;

typedef struct {
	uint8_t cabeza, final, tamano;
	tipo_elemento vector[TAMANO_MAXIMO_COLA];
} cola;

void crear_cola(cola *);
uint8_t cola_vacia(cola);
void insertar(tipo_elemento, cola *);
tipo_elemento quitar_primero(cola *);
tipo_elemento primero(cola);
uint8_t tamanoCola(cola *);


#endif
