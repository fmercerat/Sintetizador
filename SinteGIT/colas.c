#include <stdlib.h>
#include <stdio.h>
#include "colas.h"

void crear_cola(cola *c)
{
	c->tamano = 0;
	c->cabeza = 0;
	c->final = -1;
}

uint8_t cola_vacia(cola c)
{
	return (c.tamano == 0);
}

void incrementar(uint8_t *x)
{ /* privado */
	if (++(*x) == TAMANO_MAXIMO_COLA)
		*x = 0;
}

void insertar(tipo_elemento x, cola *c)
{
	if (c->tamano == TAMANO_MAXIMO_COLA)
	{
		return;
	}

	c->tamano++;
	incrementar(&(c->final));
	c->vector[c->final] = x;
}

tipo_elemento primero(cola c)
{
	return(c.vector[c.cabeza]);
}

tipo_elemento quitar_primero(cola *c)
{
	tipo_elemento x;

	c->tamano--;
	x = c->vector[c->cabeza];
	incrementar(&(c->cabeza));

	return x;
}

uint8_t tamanoCola(cola *c)
{
	return c->tamano;
}

