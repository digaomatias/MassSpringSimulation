#ifndef MASSAMOLA_H
#define MASSAMOLA_H

#ifdef _WIN32
#include <windows.h> 
#endif
#include <GL/gl.h>
#include <GL/glut.h> 
#include <math.h>
#include <vector>
#include <iostream>
#include <omp.h>

#include "Mola.h"

/* Constantes de f�sica */
#define ITERACOES_MOLA 5 // Essa � a nossa vers�o de K. Vai determinar quantas vezes o processamento da mola � rodado por passo de tempo.
#define SELECTION_UP 1
#define SELECTION_DOWN 2
#define SELECTION_LEFT 3
#define SELECTION_RIGHT 4

class MassaMola
{
private:

	int num_vertices_width; // number of Vertices in "width" direction
	int num_vertices_height; // number of Vertices in "height" direction
	// total number of Vertices is num_vertices_width*num_vertices_height

	//vari�veis pra controlar a part�cula selecionada
	int selected_x;
	int selected_y;

	std::vector<Vertice> Vertices; // all Vertices that are part of this cloth
	std::vector<Mola> constraints; // all constraints between Vertices as part of this cloth

	bool spherized_Vertice;

	Vertice* getVertice(int x, int y);
	void criaMola(Vertice *p1, Vertice *p2);


	/* M�todo usado por drawShaded() e addVentoTriangulo() pra calcular o
	vetor normal do tri�ngulo definido pela posi��o das part�culas p1, p2, and p3.
	A magnitude do vetor normal � igual a area do paralelograma definido por p1, p2 e p3
	*/
	Vector3 calcTrianguloNormal(Vertice *p1,Vertice *p2,Vertice *p3);

	/* M�todo usado pelo aplicaVento() para calcular a for�a do vendo em um �nico tri�ngulo, definido por 
	p1,p2,p3*/
	void addVentoTriangulo(Vertice *p1,Vertice *p2,Vertice *p3, const Vector3 direction);

	/* M�todo usado pelo drawShaded pra pintar um tri�ngulo com uma cor*/
	void desenhaTriangulo(Vertice *p1, Vertice *p2, Vertice *p3, const Vector3 color);

	void esferizaVertice(Vertice Vertice, float sphereRadius); //Mostra uma esfera na particula para identific�-la

public:
		
	MassaMola(float width, float height, int num_vertices_width, int num_vertices_height, float massaTotal) : num_vertices_width(num_vertices_width), num_vertices_height(num_vertices_height)
	{
		inicializa(width, height, massaTotal);
	}

	void inicializa(int width, int height, float massaTotal);
	
	void alternaVerticeEsferizado();

	/* desenha o tecido como uma malha smooth shaded (e colorido de acordo com a cor)  triangular OPEN GL
	Chamado pelo m�todo display()
		*/
	void drawShaded();

	/* aqui � onde ocorre um passo de tempo para todas as part�culas.
	*/
	void passoDeTempo();

	/* usado pra aplicar gravidade, ou qualquer outro vetor, a todas as part�culas*/
	void addForca(const Vector3 direction);

	/* for�a aplicada apenas � particula selecionada */
	void addForcaToSelectedVertice(const Vector3 direction);

	/* Usado pra adicionar for�a do vento a todas part�culas, � adicionado pra cada triangulo j� que a for�a final � proporcional a �rea do tri�ngulo de acordo com a dire��o do vento*/
	void aplicaVento(const Vector3 direction);
	
	void seleciona(int direction);
};

#endif