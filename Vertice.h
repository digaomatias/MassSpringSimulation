#ifndef VERTICE_H
#define VERTICE_H

#include "Vector3.h"

#define AMORTECIMENTO 0.01 // for�a oposta, pra n�o ficar chacoalhando eternamente
#define DT 0.5*0.5 // varia��o de tempo (delta t) a cada passo de execu��o

class Vertice
{
private:
	bool movimentavel;// usado pra marcar uma particula que pode ser movimentada (� afetada por for�as)
	bool selecionada; // pra identificar part�culas que estao selecionadas

	float massa; // a massa da particula. Usamos 1 por default
	Vector3 posicao; // posi��o da particula no espa�o 3D
	Vector3 old_posicao; // Posi��o anterior da part�cula, usado para calcular a integra��o de verlet.
	Vector3 aceleracao; // Vetor de acelera��o da part�cula
	Vector3 normal_acumulada; // Usado pra OpenGL soft shading


public:
	Vertice(Vector3 posicao) : posicao(posicao), old_posicao(posicao),aceleracao(Vector3(0,0,0)), massa(1), movimentavel(true), normal_acumulada(Vector3(0,0,0)), selecionada(false){}
	Vertice() {}

	void addForca(Vector3 f);

	/* Aqui ocorre um passo de tempo (c�lculo das posi��es baseado no DT)	   
	   De acordo com a equal��o "for�a = massa * acelera��o" a proxima posi��o � encontrada atrav�s de 
	   do m�todo de integra��o verlet
	   
	   Integra��o verlet pode ser encontrado aqui http://www.cems.uvm.edu/~tlakoba/math337/verlet_on_gamedevnet_1.htm e 
	   aqui http://en.wikipedia.org/wiki/Verlet_integration
	   */
	void passoDeTempo();

	Vector3& getPosicao();

	void resetAceleracao();

	void offsetPosicao(const Vector3 v);

	void setNaoMovimentavel();

	void toggleSelecionado();

	bool isSelecionado();

	void adicionaNormal(Vector3 normal);

	Vector3& getNormal(); // notice, the normal is not unit length

	void resetNormal();

};

#endif