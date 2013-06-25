#include "Vertice.h"

void Vertice::addForca(Vector3 f)
{
	aceleracao += f/massa;
}

void Vertice::passoDeTempo()
{
	if(movimentavel)
	{
		Vector3 temp = posicao;
		posicao = posicao + (posicao-old_posicao)*(1.0-AMORTECIMENTO) + aceleracao*DT;
		old_posicao = temp;
		aceleracao = Vector3(0,0,0); // Reset a aceleracao, já que a mesma já foi processada (posicao do vertice no espaço atualizado)	
	}
}

Vector3& Vertice::getPosicao() {return posicao;}

void Vertice::resetAceleracao() {aceleracao = Vector3(0,0,0);}

void Vertice::offsetPosicao(const Vector3 v) { if(movimentavel) posicao += v;}

void Vertice::setNaoMovimentavel() {movimentavel = false;}

void Vertice::toggleSelecionado() { selecionada = !selecionada; }

bool Vertice::isSelecionado() { return selecionada; }

void Vertice::adicionaNormal(Vector3 normal)
{
	normal_acumulada += normal.normalized();
}

Vector3& Vertice::getNormal() { return normal_acumulada;}

void Vertice::resetNormal() 
{
	normal_acumulada = Vector3(0,0,0);
}