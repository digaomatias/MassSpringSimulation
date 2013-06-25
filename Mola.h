#ifndef MOLA_H
#define MOLA_H

#include "Vertice.h"

class Mola
{
private:
	float distancia_em_repouso; // A largura entre partícula p1 e p2 em posição de repouso
	int mola_k;

public:
	Vertice *p1, *p2; // As duas partículas ligadas por essa aresta

	Mola(Vertice *p1, Vertice *p2) :  p1(p1),p2(p2), mola_k(100)
	{
		Vector3 vec = p1->getPosicao()-p2->getPosicao();
		distancia_em_repouso = vec.length();
	}

	void processaMola();
};

#endif