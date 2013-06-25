#include "Mola.h"

void Mola::processaMola()
{
	Vector3 p1_a_p2 = p2->getPosicao()-p1->getPosicao(); // vetor distancia de p1 e p2
	float distancia_atual = p1_a_p2.length(); // escalar da distancia
	Vector3 vetor_de_correcao = p1_a_p2*(1 - distancia_em_repouso/distancia_atual); // O vetor de offset que pode mover p1 at� a distancia de repouso de t2 (processo de relaxamento da mola)
	Vector3 meio_vetor_de_correcao = vetor_de_correcao*0.5; // S� divide por 2 pra poder mover tanto p1 quanto p2 pra posi��o de repouso.
	p1->offsetPosicao(meio_vetor_de_correcao); // Move p1 na dire��o de p2
	p2->offsetPosicao(-meio_vetor_de_correcao); // Move p2 na dire��o oposta (em dire��o a p1)
}