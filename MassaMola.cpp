#include "MassaMola.h"

	void MassaMola::inicializa(int width, int height)
	{
		spherized_Vertice = false;
		selected_x = 0;
		selected_y = 0;
		Vertices.resize(num_vertices_width*num_vertices_height); //Array contendo todas as partículas

		// criando partículas em um grid de particulas (0,0,0) até (width,-height,0)
		for(int x=0; x<num_vertices_width; x++)
		{
			for(int y=0; y<num_vertices_height; y++)
			{
				Vector3 pos = Vector3(width * (x/(float)num_vertices_width),
								-height * (y/(float)num_vertices_height),
								0);
				Vertices[y*num_vertices_width+x]= Vertice(pos); // insere partículas na coluna x e linha y
				//a primeira particula tem que ser selecionada
				if(y == 0 && x == 0)
					Vertices[y*num_vertices_width+x].toggleSelecionado();
			}
		}

		// Conectando arestas de partículas vizinhas imediatas (distância 1 e sqrt(2) no grid)
		for(int x=0; x<num_vertices_width; x++)
		{
			for(int y=0; y<num_vertices_height; y++)
			{
				if (x<num_vertices_width-1) criaMola(getVertice(x,y),getVertice(x+1,y));
				if (y<num_vertices_height-1) criaMola(getVertice(x,y),getVertice(x,y+1));
				if (x<num_vertices_width-1 && y<num_vertices_height-1) criaMola(getVertice(x,y),getVertice(x+1,y+1));
				if (x<num_vertices_width-1 && y<num_vertices_height-1) criaMola(getVertice(x+1,y),getVertice(x,y+1));
			}
		}


		// Conectando arestas de partículas vizinhas secundários (distância 2 e sqrt(4) no grid)
		for(int x=0; x<num_vertices_width; x++)
		{
			for(int y=0; y<num_vertices_height; y++)
			{
				if (x<num_vertices_width-2) criaMola(getVertice(x,y),getVertice(x+2,y));
				if (y<num_vertices_height-2) criaMola(getVertice(x,y),getVertice(x,y+2));
				if (x<num_vertices_width-2 && y<num_vertices_height-2) criaMola(getVertice(x,y),getVertice(x+2,y+2));
				if (x<num_vertices_width-2 && y<num_vertices_height-2) criaMola(getVertice(x+2,y),getVertice(x,y+2));			}
		}


		// Setando as 3 partículas mais da esquerda superior e direita superior como não movíveis
		for(int i=0;i<3; i++)
		{
			getVertice(0+i ,0)->offsetPosicao(Vector3(0.5,0.0,0.0)); //move a particula um pouco mais pro centro pra parecer mais natural
			getVertice(0+i ,0)->setNaoMovimentavel(); 

			getVertice(0+i ,0)->offsetPosicao(Vector3(-0.5,0.0,0.0)); //move a particula um pouco mais pro centro pra parecer mais natural
			getVertice(num_vertices_width-1-i ,0)->setNaoMovimentavel();
		}
	}

	Vertice* MassaMola::getVertice(int x, int y) {return &Vertices[y*num_vertices_width + x];}
	void MassaMola::criaMola(Vertice *p1, Vertice *p2) {constraints.push_back(Mola(p1,p2));}


	/* Método usado por drawShaded() e addWindForcesForTriangle() pra calcular o
	vetor normal do triângulo definido pela posição das partículas p1, p2, and p3.
	A magnitude do vetor normal é igual a area do paralelograma definido por p1, p2 e p3
	*/
	Vector3 MassaMola::calcTriangleNormal(Vertice *p1,Vertice *p2,Vertice *p3)
	{
		Vector3 pos1 = p1->getPosicao();
		Vector3 pos2 = p2->getPosicao();
		Vector3 pos3 = p3->getPosicao();

		Vector3 v1 = pos2-pos1;
		Vector3 v2 = pos3-pos1;

		return v1.cross(v2);
	}

	/* Método usado pelo aplicaVento() para calcular a força do vendo em um único triângulo, definido por 
	p1,p2,p3*/
	void MassaMola::addWindForcesForTriangle(Vertice *p1,Vertice *p2,Vertice *p3, const Vector3 direction)
	{
		Vector3 normal = calcTriangleNormal(p1,p2,p3);
		Vector3 d = normal.normalized();
		Vector3 force = normal*(d.dot(direction));
		p1->addForca(force);
		p2->addForca(force);
		p3->addForca(force);
	}

	/* Método usado pelo drawShaded pra pintar um triângulo com uma cor*/
	void MassaMola::drawTriangle(Vertice *p1, Vertice *p2, Vertice *p3, const Vector3 color)
	{
		glColor3fv( (GLfloat*) &color );

		glNormal3fv((GLfloat *) &(p1->getNormal().normalized() ));
		glVertex3fv((GLfloat *) &(p1->getPosicao() ));

		glNormal3fv((GLfloat *) &(p2->getNormal().normalized() ));
		glVertex3fv((GLfloat *) &(p2->getPosicao() ));

		glNormal3fv((GLfloat *) &(p3->getNormal().normalized() ));
		glVertex3fv((GLfloat *) &(p3->getPosicao() ));
	}

	void MassaMola::spherizeVertice(Vertice Vertice, float sphereRadius) //Mostra uma esfera na particula para identificá-la
	{
		glPushMatrix();
		glTranslatef(Vertice.getPosicao().f[0], Vertice.getPosicao().f[1], Vertice.getPosicao().f[2]); //Posiciona a esfera glut na posição da particula
		if(Vertice.isSelecionado())
			glColor3f(0.2f,1.0f,0.5f);
		else
			glColor3f(1.0f,0.2f,0.5f);
		glutSolidSphere(sphereRadius,5,5);
		glPopMatrix();
	}
		
	void MassaMola::toggleSpherizedVertice()
	{
		spherized_Vertice = !spherized_Vertice;
	}

	/* desenha o tecido como uma malha smooth shaded (e colorido de acordo com a cor)  triangular OPEN GL
	Chamado pelo método display()
		*/
	void MassaMola::drawShaded()
	{
		// reseta os normais que foram escritos no frame anterior
		std::vector<Vertice>::iterator Vertice;
		for(Vertice = Vertices.begin(); Vertice != Vertices.end(); Vertice++)
		{
			(*Vertice).resetNormal();
		}

		//cria os normais por partícula através da adição de todos os triangulos normais (hard)  os quais cada partícula faz parte
		//create smooth per Vertice normals by adding up all the (hard) triangle normals that each Vertice is part of
		for(int x = 0; x<num_vertices_width-1; x++)
		{
			for(int y=0; y<num_vertices_height-1; y++)
			{
				Vector3 normal = calcTriangleNormal(getVertice(x+1,y),getVertice(x,y),getVertice(x,y+1));
				getVertice(x+1,y)->adicionaNormal(normal);
				getVertice(x,y)->adicionaNormal(normal);
				getVertice(x,y+1)->adicionaNormal(normal);

				normal = calcTriangleNormal(getVertice(x+1,y+1),getVertice(x+1,y),getVertice(x,y+1));
				getVertice(x+1,y+1)->adicionaNormal(normal);
				getVertice(x+1,y)->adicionaNormal(normal);
				getVertice(x,y+1)->adicionaNormal(normal);
			}
		}

		if(spherized_Vertice)
		{
			for(int x = 0; x<num_vertices_width; x++)
			{
				for(int y=0; y<num_vertices_height; y++)
				{
					//Cria as bolinhas nas partículas caso a opção esteja ligada
					spherizeVertice(*getVertice(x, y), 0.07);
				}
			}
		}

		glBegin(GL_TRIANGLES);
		for(int x = 0; x<num_vertices_width-1; x++)
		{
			for(int y=0; y<num_vertices_height-1; y++)
			{
				Vector3 color(0,0,0);
				if (x%2) // red and white color is interleaved according to which column number
					color = Vector3(0.3f,0.5f,0.2f);
				else
					color = Vector3(1.0f,1.0f,1.0f);

				drawTriangle(getVertice(x+1,y),getVertice(x,y),getVertice(x,y+1),color);
				drawTriangle(getVertice(x+1,y+1),getVertice(x+1,y),getVertice(x,y+1),color);
			}
		}
		glEnd();
	}

	/* aqui é onde ocorre um passo de tempo para todas as partículas.
	*/
	void MassaMola::passoDeTempo()
	{
		for(int i=0; i<ITERACOES_MOLA; i++)
		{
			omp_set_num_threads(4);			
			#pragma omp parallel for
			for(int j = 0; j < constraints.size(); j++ )
			{
				constraints[j].processaMola();
			}
		}

		std::vector<Vertice>::iterator Vertice;
		for(Vertice = Vertices.begin(); Vertice != Vertices.end(); Vertice++)
		{
			(*Vertice).passoDeTempo(); // calcula a posição de cada partícula no passo seguinte.
		}
	}

	/* usado pra aplicar gravidade, ou qualquer outro vetor, a todas as partículas*/
	void MassaMola::addForca(const Vector3 direction)
	{
		std::vector<Vertice>::iterator Vertice;
		for(Vertice = Vertices.begin(); Vertice != Vertices.end(); Vertice++)
		{
			(*Vertice).addForca(direction); // adiciona o vetor de força a cada partícula
		}

	}

	/* força aplicada apenas à particula selecionada */
	void MassaMola::addForcaToSelectedVertice(const Vector3 direction)
	{
		Vertice* p = getVertice(selected_x, selected_y);		
		Vector3 normal = p->getNormal();
		Vector3 d = normal.normalized();
		Vector3 force = normal*(d.dot(direction));
		p->addForca(force);
	}

	/* Usado pra adicionar força do vento a todas partículas, é adicionado pra cada triangulo já que a força final é proporcional a área do triângulo de acordo com a direção do vento*/
	void MassaMola::aplicaVento(const Vector3 direction)
	{
		for(int x = 0; x<num_vertices_width-1; x++)
		{
			for(int y=0; y<num_vertices_height-1; y++)
			{
				addWindForcesForTriangle(getVertice(x+1,y),getVertice(x,y),getVertice(x,y+1),direction);
				addWindForcesForTriangle(getVertice(x+1,y+1),getVertice(x+1,y),getVertice(x,y+1),direction);
			}
		}
	}
	
	void MassaMola::seleciona(int direction)
	{
		Vertice* p = getVertice(selected_x, selected_y);
		p->toggleSelecionado();

		switch(direction)
		{
			case SELECTION_UP:
				selected_y--;
				if(selected_y < 0)
					selected_y = num_vertices_height-1;
			break;
			case SELECTION_DOWN:
				selected_y++;
				if(selected_y == num_vertices_height)
					selected_y = 0;				
			break;
			case SELECTION_LEFT:
				selected_x--;
				if(selected_x < 0)
					selected_x = num_vertices_width-1;				
			break;
			case SELECTION_RIGHT:
				selected_x++;
				if(selected_x == num_vertices_width)
					selected_x = 0;
			break;
		}

		p = getVertice(selected_x, selected_y);
		p->toggleSelecionado();
	}

