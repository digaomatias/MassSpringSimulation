#ifndef VERTICE_H
#define VERTICE_H

#include "Vector3.h"

#define AMORTECIMENTO 0.01 // força oposta, pra não ficar chacoalhando eternamente
#define DT 0.5*0.5 // variação de tempo (delta t) a cada passo de execução

class Vertice
{
private:
	bool movimentavel;// usado pra marcar uma particula que pode ser movimentada (é afetada por forças)
	bool selecionada; // pra identificar partículas que estao selecionadas

	float massa; // a massa da particula. Usamos 1 por default
	Vector3 posicao; // posição da particula no espaço 3D
	Vector3 old_posicao; // Posição anterior da partícula, usado para calcular a integração de verlet.
	Vector3 aceleracao; // Vetor de aceleração da partícula
	Vector3 normal_acumulada; // Usado pra OpenGL soft shading


public:
	Vertice(Vector3 posicao) : posicao(posicao), old_posicao(posicao),aceleracao(Vector3(0,0,0)), massa(1), movimentavel(true), normal_acumulada(Vector3(0,0,0)), selecionada(false){}
	Vertice() {}

	void addForca(Vector3 f);

	/* Aqui ocorre um passo de tempo (cálculo das posições baseado no DT)	   
	   De acordo com a equalção "força = massa * aceleração" a proxima posição é encontrada através de 
	   do método de integração verlet
	   
	   Integração verlet pode ser encontrado aqui http://www.cems.uvm.edu/~tlakoba/math337/verlet_on_gamedevnet_1.htm e 
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