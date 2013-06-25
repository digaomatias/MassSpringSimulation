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

/* Constantes de física */
#define ITERACOES_MOLA 5 // Essa é a nossa versão de K. Vai determinar quantas vezes o processamento da mola é rodado por passo de tempo.
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

	//variáveis pra controlar a partícula selecionada
	int selected_x;
	int selected_y;

	std::vector<Vertice> Vertices; // all Vertices that are part of this cloth
	std::vector<Mola> constraints; // all constraints between Vertices as part of this cloth

	bool spherized_Vertice;

	Vertice* getVertice(int x, int y);
	void criaMola(Vertice *p1, Vertice *p2);


	/* Método usado por drawShaded() e addVentoTriangulo() pra calcular o
	vetor normal do triângulo definido pela posição das partículas p1, p2, and p3.
	A magnitude do vetor normal é igual a area do paralelograma definido por p1, p2 e p3
	*/
	Vector3 calcTrianguloNormal(Vertice *p1,Vertice *p2,Vertice *p3);

	/* Método usado pelo aplicaVento() para calcular a força do vendo em um único triângulo, definido por 
	p1,p2,p3*/
	void addVentoTriangulo(Vertice *p1,Vertice *p2,Vertice *p3, const Vector3 direction);

	/* Método usado pelo drawShaded pra pintar um triângulo com uma cor*/
	void desenhaTriangulo(Vertice *p1, Vertice *p2, Vertice *p3, const Vector3 color);

	void esferizaVertice(Vertice Vertice, float sphereRadius); //Mostra uma esfera na particula para identificá-la

public:
		
	MassaMola(float width, float height, int num_vertices_width, int num_vertices_height, float massaTotal) : num_vertices_width(num_vertices_width), num_vertices_height(num_vertices_height)
	{
		inicializa(width, height, massaTotal);
	}

	void inicializa(int width, int height, float massaTotal);
	
	void alternaVerticeEsferizado();

	/* desenha o tecido como uma malha smooth shaded (e colorido de acordo com a cor)  triangular OPEN GL
	Chamado pelo método display()
		*/
	void drawShaded();

	/* aqui é onde ocorre um passo de tempo para todas as partículas.
	*/
	void passoDeTempo();

	/* usado pra aplicar gravidade, ou qualquer outro vetor, a todas as partículas*/
	void addForca(const Vector3 direction);

	/* força aplicada apenas à particula selecionada */
	void addForcaToSelectedVertice(const Vector3 direction);

	/* Usado pra adicionar força do vento a todas partículas, é adicionado pra cada triangulo já que a força final é proporcional a área do triângulo de acordo com a direção do vento*/
	void aplicaVento(const Vector3 direction);
	
	void seleciona(int direction);
};

#endif