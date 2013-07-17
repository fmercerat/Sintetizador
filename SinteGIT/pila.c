#include "pila.h"

void crear_pila(pila *p)
{
	p->cima = -1;
}

uint8_t pila_vacia(pila p)
{
	return (p.cima == -1);
}

uint8_t apilar(tipo_elemento x, pila *p)
{
	if (++p->cima == TAMANO_MAXIMO_PILA)
	{
		return -1;
	}

	p->vector[p->cima] = x;
	return 0;
}

tipo_elemento cima(pila p)
{
	if (pila_vacia(p))
	{
		return -1;
	}
	return p.vector[p.cima];
}

uint8_t desapilar(pila *p)
{
	if (pila_vacia(*p))
	{
		return -1;
	}

	p->cima--;

	return 0;
}

uint8_t eliminar(pila *p, tipo_elemento xxx)
{
	uint8_t i;

	if(p->cima == -1)
		return -1;

	for(i=0; i < p->cima; i++)
	{
		if(p->vector[i] == xxx)
		{
			p->vector[i] = 0;
			return 1;
		}
	}

	return 0;
}

uint8_t eliminar2(pila *p, tipo_elemento xxx)
{
	uint8_t i,j=0;

	if(p->cima == -1)
		return -1;

	for(i=0; i < p->cima; i++)
	{
		if(p->vector[i] == xxx)
		{
			p->vector[i] = 0;
			j=i;
		}
	}

	for(i=j;i<=p->cima;i++)
	{
		p->vector[i] = p->vector[i+1];
	}
	p->vector[p->cima] = 0;
	p->cima--;

	return 0;
}

uint8_t desMidi(pila *p)
{
	if (pila_vacia(*p))
	{
		return 0;
	}

	p->vector[p->cima] = 0;

	while((p->vector[p->cima] == 0))
	{
		if(p->cima==0)
		{
			p->cima--;
			return 0;
		}
		p->cima--;
	}

	return 1;


}
