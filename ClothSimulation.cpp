/*
conteúdo do arquivo;
* includes
* constantes de física

* class Vec3
* class Particle (com posição Vec3)
* class Constraint (ligando duas partículas)
* class MassSpring (tecido com partículas e arestas)

*/


#ifdef _WIN32
#include <windows.h> 
#endif
#include <GL/gl.h>
#include <GL/glut.h> 
#include <math.h>
#include <vector>
#include <iostream>
#include <omp.h>


/* Constantes de física */
#define DAMPING 0.01 // força oposta, pra não ficar chacoalhando eternamente
#define TIME_STEPSIZE2 0.5*0.5 // how large time step each particle takes each frame
#define CONSTRAINT_ITERATIONS 4 // how many iterations of constraint satisfaction each frame (more is rigid, less is soft)
#define SELECTION_UP 1
#define SELECTION_DOWN 2
#define SELECTION_LEFT 3
#define SELECTION_RIGHT 4


class Vec3 // simplesmente um vetor com 3 floats x, y e z.
{	
public:
	float f[3];

	Vec3(float x, float y, float z)
	{
		f[0] =x;
		f[1] =y;
		f[2] =z;
	}

	Vec3() {}

	float length()
	{
		return sqrt(f[0]*f[0]+f[1]*f[1]+f[2]*f[2]);
	}

	Vec3 normalized()
	{
		float l = length();
		return Vec3(f[0]/l,f[1]/l,f[2]/l);
	}

	void operator+= (const Vec3 &v)
	{
		f[0]+=v.f[0];
		f[1]+=v.f[1];
		f[2]+=v.f[2];
	}

	Vec3 operator/ (const float &a)
	{
		return Vec3(f[0]/a,f[1]/a,f[2]/a);
	}

	Vec3 operator- (const Vec3 &v)
	{
		return Vec3(f[0]-v.f[0],f[1]-v.f[1],f[2]-v.f[2]);
	}

	Vec3 operator+ (const Vec3 &v)
	{
		return Vec3(f[0]+v.f[0],f[1]+v.f[1],f[2]+v.f[2]);
	}

	Vec3 operator* (const float &a)
	{
		return Vec3(f[0]*a,f[1]*a,f[2]*a);
	}

	Vec3 operator-()
	{
		return Vec3(-f[0],-f[1],-f[2]);
	}

	Vec3 cross(const Vec3 &v)
	{
		return Vec3(f[1]*v.f[2] - f[2]*v.f[1], f[2]*v.f[0] - f[0]*v.f[2], f[0]*v.f[1] - f[1]*v.f[0]);
	}

	float dot(const Vec3 &v)
	{
		return f[0]*v.f[0] + f[1]*v.f[1] + f[2]*v.f[2];
	}
};

class Particle
{
private:
	bool movable; // usado pra marcar uma particula que pode ser movimentada (é afetada por forças)
	bool selected; // pra identificar partículas que estao selecionadas

	float mass; // a massa da particula. Usamos 1 por default
	Vec3 pos; // posição da particula no espaço 3D
	Vec3 old_pos; // Posição anterior da partícula, usado para calcular a integração de verlet.
	Vec3 acceleration; // Vetor de aceleração da partícula
	Vec3 accumulated_normal; // Usado pra OpenGL soft shading


public:
	Particle(Vec3 pos) : pos(pos), old_pos(pos),acceleration(Vec3(0,0,0)), mass(1), movable(true), accumulated_normal(Vec3(0,0,0)), selected(false){}
	Particle() {}

	void addForce(Vec3 f)
	{
		acceleration += f/mass;
	}

	/* This is one of the important methods, where the time is progressed a single step size (TIME_STEPSIZE)
	   The method is called by MassSpring.time_step()
	   Given the equation "force = mass * acceleration" the next position is found through verlet integration*/
	void timeStep()
	{
		if(movable)
		{
			Vec3 temp = pos;
			pos = pos + (pos-old_pos)*(1.0-DAMPING) + acceleration*TIME_STEPSIZE2;
			old_pos = temp;
			acceleration = Vec3(0,0,0); // acceleration is reset since it HAS been translated into a change in position (and implicitely into velocity)	
		}
	}

	Vec3& getPos() {return pos;}

	void resetAcceleration() {acceleration = Vec3(0,0,0);}

	void offsetPos(const Vec3 v) { if(movable) pos += v;}

	void makeUnmovable() {movable = false;}

	void toggleSelected() { selected = !selected; }

	bool isSelected() { return selected; }

	void addToNormal(Vec3 normal)
	{
		accumulated_normal += normal.normalized();
	}

	Vec3& getNormal() { return accumulated_normal;} // notice, the normal is not unit length

	void resetNormal() {accumulated_normal = Vec3(0,0,0);}

};

class Constraint
{
private:
	float rest_distance; // A largura entre partícula p1 e p2 em posição de repouso

public:
	Particle *p1, *p2; // As duas partículas ligadas por essa aresta

	Constraint(Particle *p1, Particle *p2) :  p1(p1),p2(p2)
	{
		Vec3 vec = p1->getPos()-p2->getPos();
		rest_distance = vec.length();
	}

	/* This is one of the important methods, where a single constraint between two particles p1 and p2 is solved
	the method is called by MassSpring.time_step() many times per frame*/
	void satisfyConstraint()
	{
		Vec3 p1_to_p2 = p2->getPos()-p1->getPos(); // vector from p1 to p2
		float current_distance = p1_to_p2.length(); // current distance between p1 and p2
		Vec3 correctionVector = p1_to_p2*(1 - rest_distance/current_distance); // The offset vector that could moves p1 into a distance of rest_distance to p2
		Vec3 correctionVectorHalf = correctionVector*0.5; // Lets make it half that length, so that we can move BOTH p1 and p2.
		p1->offsetPos(correctionVectorHalf); // correctionVectorHalf is pointing from p1 to p2, so the length should move p1 half the length needed to satisfy the constraint.
		p2->offsetPos(-correctionVectorHalf); // we must move p2 the negative direction of correctionVectorHalf since it points from p2 to p1, and not p1 to p2.	
	}

};

class MassSpring
{
private:

	int num_particles_width; // number of particles in "width" direction
	int num_particles_height; // number of particles in "height" direction
	// total number of particles is num_particles_width*num_particles_height

	//variáveis pra controlar a partícula selecionada
	int selected_x;
	int selected_y;

	std::vector<Particle> particles; // all particles that are part of this cloth
	std::vector<Constraint> constraints; // all constraints between particles as part of this cloth

	bool spherized_particle;

	Particle* getParticle(int x, int y) {return &particles[y*num_particles_width + x];}
	void makeConstraint(Particle *p1, Particle *p2) {constraints.push_back(Constraint(p1,p2));}


	/* Método usado por drawShaded() e addWindForcesForTriangle() pra calcular o
	vetor normal do triângulo definido pela posição das partículas p1, p2, and p3.
	A magnitude do vetor normal é igual a area do paralelograma definido por p1, p2 e p3
	*/
	Vec3 calcTriangleNormal(Particle *p1,Particle *p2,Particle *p3)
	{
		Vec3 pos1 = p1->getPos();
		Vec3 pos2 = p2->getPos();
		Vec3 pos3 = p3->getPos();

		Vec3 v1 = pos2-pos1;
		Vec3 v2 = pos3-pos1;

		return v1.cross(v2);
	}

	/* Método usado pelo windForce() para calcular a força do vendo em um único triângulo, definido por 
	p1,p2,p3*/
	void addWindForcesForTriangle(Particle *p1,Particle *p2,Particle *p3, const Vec3 direction)
	{
		Vec3 normal = calcTriangleNormal(p1,p2,p3);
		Vec3 d = normal.normalized();
		Vec3 force = normal*(d.dot(direction));
		p1->addForce(force);
		p2->addForce(force);
		p3->addForce(force);
	}

	/* Método usado pelo drawShaded pra pintar um triângulo com uma cor*/
	void drawTriangle(Particle *p1, Particle *p2, Particle *p3, const Vec3 color)
	{
		glColor3fv( (GLfloat*) &color );

		glNormal3fv((GLfloat *) &(p1->getNormal().normalized() ));
		glVertex3fv((GLfloat *) &(p1->getPos() ));

		glNormal3fv((GLfloat *) &(p2->getNormal().normalized() ));
		glVertex3fv((GLfloat *) &(p2->getPos() ));

		glNormal3fv((GLfloat *) &(p3->getNormal().normalized() ));
		glVertex3fv((GLfloat *) &(p3->getPos() ));
	}

	void spherizeParticle(Particle particle, float sphereRadius) //Mostra uma esfera na particula para identificá-la
	{
		glPushMatrix();
		glTranslatef(particle.getPos().f[0], particle.getPos().f[1], particle.getPos().f[2]); //Posiciona a esfera glut na posição da particula
		if(particle.isSelected())
			glColor3f(0.2f,1.0f,0.5f);
		else
			glColor3f(1.0f,0.2f,0.5f);
		glutSolidSphere(sphereRadius,5,5);
		glPopMatrix();
	}

public:
		
	MassSpring(float width, float height, int num_particles_width, int num_particles_height) : num_particles_width(num_particles_width), num_particles_height(num_particles_height)
	{
		spherized_particle = false;
		selected_x = 0;
		selected_y = 0;
		particles.resize(num_particles_width*num_particles_height); //Array contendo todas as partículas

		// criando partículas em um grid de particulas (0,0,0) até (width,-height,0)
		for(int x=0; x<num_particles_width; x++)
		{
			for(int y=0; y<num_particles_height; y++)
			{
				Vec3 pos = Vec3(width * (x/(float)num_particles_width),
								-height * (y/(float)num_particles_height),
								0);
				particles[y*num_particles_width+x]= Particle(pos); // insere partículas na coluna x e linha y
				//a primeira particula tem que ser selecionada
				if(y == 0 && x == 0)
					particles[y*num_particles_width+x].toggleSelected();
			}
		}

		// Conectando arestas de partículas vizinhas imediatas (distância 1 e sqrt(2) no grid)
		for(int x=0; x<num_particles_width; x++)
		{
			for(int y=0; y<num_particles_height; y++)
			{
				if (x<num_particles_width-1) makeConstraint(getParticle(x,y),getParticle(x+1,y));
				if (y<num_particles_height-1) makeConstraint(getParticle(x,y),getParticle(x,y+1));
				if (x<num_particles_width-1 && y<num_particles_height-1) makeConstraint(getParticle(x,y),getParticle(x+1,y+1));
				if (x<num_particles_width-1 && y<num_particles_height-1) makeConstraint(getParticle(x+1,y),getParticle(x,y+1));
			}
		}


		// Conectando arestas de partículas vizinhas secundários (distância 2 e sqrt(4) no grid)
		for(int x=0; x<num_particles_width; x++)
		{
			for(int y=0; y<num_particles_height; y++)
			{
				if (x<num_particles_width-2) makeConstraint(getParticle(x,y),getParticle(x+2,y));
				if (y<num_particles_height-2) makeConstraint(getParticle(x,y),getParticle(x,y+2));
				if (x<num_particles_width-2 && y<num_particles_height-2) makeConstraint(getParticle(x,y),getParticle(x+2,y+2));
				if (x<num_particles_width-2 && y<num_particles_height-2) makeConstraint(getParticle(x+2,y),getParticle(x,y+2));			}
		}


		// Setando as 3 partículas mais da esquerda superior e direita superior como não movíveis
		for(int i=0;i<3; i++)
		{
			getParticle(0+i ,0)->offsetPos(Vec3(0.5,0.0,0.0)); //move a particula um pouco mais pro centro pra parecer mais natural
			getParticle(0+i ,0)->makeUnmovable(); 

			getParticle(0+i ,0)->offsetPos(Vec3(-0.5,0.0,0.0)); //move a particula um pouco mais pro centro pra parecer mais natural
			getParticle(num_particles_width-1-i ,0)->makeUnmovable();
		}
	}

	
	void toggleSpherizedParticle()
	{
		spherized_particle = !spherized_particle;
	}

	/* desenha o tecido como uma malha smooth shaded (e colorido de acordo com a cor)  triangular OPEN GL
	Chamado pelo método display()
	O tecido consiste de triangulos para quatro partículas, conforme vistas no grid abaixo

	(x,y)   *--* (x+1,y)
	        | /|
	        |/ |
	(x,y+1) *--* (x+1,y+1)

	*/
	void drawShaded()
	{
		// reseta os normais que foram escritos no frame anterior
		std::vector<Particle>::iterator particle;
		for(particle = particles.begin(); particle != particles.end(); particle++)
		{
			(*particle).resetNormal();
		}

		//cria os normais por partícula através da adição de todos os triangulos normais (hard)  os quais cada partícula faz parte
		//create smooth per particle normals by adding up all the (hard) triangle normals that each particle is part of
		for(int x = 0; x<num_particles_width-1; x++)
		{
			for(int y=0; y<num_particles_height-1; y++)
			{
				Vec3 normal = calcTriangleNormal(getParticle(x+1,y),getParticle(x,y),getParticle(x,y+1));
				getParticle(x+1,y)->addToNormal(normal);
				getParticle(x,y)->addToNormal(normal);
				getParticle(x,y+1)->addToNormal(normal);

				normal = calcTriangleNormal(getParticle(x+1,y+1),getParticle(x+1,y),getParticle(x,y+1));
				getParticle(x+1,y+1)->addToNormal(normal);
				getParticle(x+1,y)->addToNormal(normal);
				getParticle(x,y+1)->addToNormal(normal);
			}
		}

		if(spherized_particle)
		{
			for(int x = 0; x<num_particles_width; x++)
			{
				for(int y=0; y<num_particles_height; y++)
				{
					//Cria as bolinhas nas partículas caso a opção esteja ligada
					spherizeParticle(*getParticle(x, y), 0.07);
				}
			}
		}

		glBegin(GL_TRIANGLES);
		for(int x = 0; x<num_particles_width-1; x++)
		{
			for(int y=0; y<num_particles_height-1; y++)
			{
				Vec3 color(0,0,0);
				if (x%2) // red and white color is interleaved according to which column number
					color = Vec3(0.3f,0.5f,0.2f);
				else
					color = Vec3(1.0f,1.0f,1.0f);

				drawTriangle(getParticle(x+1,y),getParticle(x,y),getParticle(x,y+1),color);
				drawTriangle(getParticle(x+1,y+1),getParticle(x+1,y),getParticle(x,y+1),color);
			}
		}
		glEnd();
	}

	/* aqui é onde ocorre um passo de tempo para todas as partículas.
	*/
	void timeStep()
	{
		for(int i=0; i<CONSTRAINT_ITERATIONS; i++) // iterate over all constraints several times
		{
			omp_set_num_threads(4);			
#pragma omp parallel for
			for(int j = 0; j < constraints.size(); j++ )
			{
				constraints[j].satisfyConstraint();
			}
		}

		std::vector<Particle>::iterator particle;
		for(particle = particles.begin(); particle != particles.end(); particle++)
		{
			(*particle).timeStep(); // calcula a posição de cada partícula no passo seguinte.
		}
	}

	/* usado pra aplicar gravidade, ou qualquer outro vetor, a todas as partículas*/
	void addForce(const Vec3 direction)
	{
		std::vector<Particle>::iterator particle;
		for(particle = particles.begin(); particle != particles.end(); particle++)
		{
			(*particle).addForce(direction); // adiciona o vetor de força a cada partícula
		}

	}

	/* força aplicada apenas à particula selecionada */
	void addForceToSelectedParticle(const Vec3 direction)
	{
		Particle* p = getParticle(selected_x, selected_y);		
		Vec3 normal = p->getNormal();
		Vec3 d = normal.normalized();
		Vec3 force = normal*(d.dot(direction));
		p->addForce(force);
	}

	/* Usado pra adicionar força do vento a todas partículas, é adicionado pra cada triangulo já que a força final é proporcional a área do triângulo de acordo com a direção do vento*/
	void windForce(const Vec3 direction)
	{
		for(int x = 0; x<num_particles_width-1; x++)
		{
			for(int y=0; y<num_particles_height-1; y++)
			{
				addWindForcesForTriangle(getParticle(x+1,y),getParticle(x,y),getParticle(x,y+1),direction);
				addWindForcesForTriangle(getParticle(x+1,y+1),getParticle(x+1,y),getParticle(x,y+1),direction);
			}
		}
	}

	/* used to detect and resolve the collision of the cloth with the ball.
	This is based on a very simples scheme where the position of each particle is simply compared to the sphere and corrected.
	This also means that the sphere can "slip through" if the ball is small enough compared to the distance in the grid bewteen particles
	*/
	void ballCollision(const Vec3 center,const float radius )
	{
		std::vector<Particle>::iterator particle;
		for(particle = particles.begin(); particle != particles.end(); particle++)
		{
			Vec3 v = (*particle).getPos()-center;
			float l = v.length();
			if ( v.length() < radius) // if the particle is inside the ball
			{
				(*particle).offsetPos(v.normalized()*(radius-l)); // project the particle to the surface of the ball
			}
		}
	}

	void select(int direction)
	{
		Particle* p = getParticle(selected_x, selected_y);
		p->toggleSelected();

		switch(direction)
		{
			case SELECTION_UP:
				selected_y--;
				if(selected_y < 0)
					selected_y = num_particles_height-1;
			break;
			case SELECTION_DOWN:
				selected_y++;
				if(selected_y == num_particles_height)
					selected_y = 0;				
			break;
			case SELECTION_LEFT:
				selected_x--;
				if(selected_x < 0)
					selected_x = num_particles_width-1;				
			break;
			case SELECTION_RIGHT:
				selected_x++;
				if(selected_x == num_particles_width)
					selected_x = 0;
			break;
		}

		p = getParticle(selected_x, selected_y);
		p->toggleSelected();
	}

	void doFrame()
	{

	}
};

/***** Acima estão as definições das classes; Vec3, Particle, Constraint, and MassSpring *****/

 
MassSpring massSpring1(14,10,30,20); // Com esses valores ficou extremamente lento: 14, 10, 55, 45
Vec3 ball_pos(7,-5,0); // O centro da bola
float ball_radius = 1.5; // O raio da bola



/***** Toda a parte de controle opengl é feita abaixo *****/

float ball_time = 0; // contador usado pra calcular a posição z da bola
float view_distance = 0;
static BOOL g_bButton1Down = FALSE;
static int g_yClick = 0;
#define VIEWING_DISTANCE_MIN  1.0

void init(GLvoid)
{
	glShadeModel(GL_SMOOTH);
	glClearColor(0.2f, 0.2f, 0.4f, 0.5f);				
	glClearDepth(1.0f);
	glEnable(GL_DEPTH_BITS);
	glDepthFunc(GL_LEQUAL);
	glEnable(GL_COLOR_MATERIAL);
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_FASTEST);
	
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	GLfloat lightPos[4] = {-1.0,1.0,0.5,0.0};
	glLightfv(GL_LIGHT0,GL_POSITION,(GLfloat *) &lightPos);

	glEnable(GL_LIGHT1);

	GLfloat lightAmbient1[4] = {0.0,0.0,0.0,0.0};
	GLfloat lightPos1[4] = {1.0,0.0,-0.2,0.0};
	GLfloat lightDiffuse1[4] = {0.5,0.5,0.3,0.0};

	glLightfv(GL_LIGHT1,GL_POSITION,(GLfloat *) &lightPos1);
	glLightfv(GL_LIGHT1,GL_AMBIENT,(GLfloat *) &lightAmbient1);
	glLightfv(GL_LIGHT1,GL_DIFFUSE,(GLfloat *) &lightDiffuse1);

	glLightModeli(GL_LIGHT_MODEL_TWO_SIDE,GL_TRUE);
}

void drawBall()
{
	glPushMatrix();
	glTranslatef(ball_pos.f[0],ball_pos.f[1],ball_pos.f[2]); //Posiciona a esfera glut na posição da bola
	glColor3f(0.4f,0.8f,0.5f);
	glutSolidSphere(ball_radius-0.1,50,50); // desenha a bola mas com um raio levemente menor, senão pode haver artefatos dos pixels da bola e da roupa se penetrando
	glPopMatrix();
}

bool windEnabled;
void toggleWindEnabled()
{
	windEnabled = !windEnabled;
}

bool gravityEnabled;
void toggleGravityEnabled()
{
	gravityEnabled = !gravityEnabled;
}

bool ballEnabled;
void toggleBallEnabled()
{
	ballEnabled = !ballEnabled;
}

void escreve(int x, int y, float r, float g, float b, void* font, char *string)
{
  glColor3f( r, g, b );
  glRasterPos2f(x, y);
  int len, i;
  len = (int)strlen(string);
  for (i = 0; i < len; i++) {
	  glutBitmapCharacter(font, string[i]);
  }
}

void writeMenu()
{
	escreve(-3, 0, 1.0f, 0.0f, 0.0f, GLUT_BITMAP_HELVETICA_12, "F1: Esfera");
	escreve(-3, -1, 1.0f, 0.0f, 0.0f, GLUT_BITMAP_HELVETICA_12, "F2: Vento");
	escreve(-3, -2, 1.0f, 0.0f, 0.0f, GLUT_BITMAP_HELVETICA_12, "F3: Gravidade");
	escreve(-3, -3, 1.0f, 0.0f, 0.0f, GLUT_BITMAP_HELVETICA_12, "b: Bola Particula");
	escreve(-3, -4, 1.0f, 0.0f, 0.0f, GLUT_BITMAP_HELVETICA_12, "w: Wireframe");
}

/* Desenha*/
void Display(void)
{
	// Calculando posições
	if(gravityEnabled)
		massSpring1.addForce(Vec3(0,-0.2,0)*TIME_STEPSIZE2); // Adiciona gravidade a cade frame, na direção pra baixo
	
	if(windEnabled)
		massSpring1.windForce(Vec3(0.5,0,0.2)*TIME_STEPSIZE2); // gera vento a cada frame
	massSpring1.timeStep(); // calcula a posição da particula no frame seguinte

	// Desenhando
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glLoadIdentity();

	gluLookAt(0, 0, view_distance, 0, 0, -1, 0, 1, 0);

	glDisable(GL_LIGHTING); // Pinta o fundo bonito
	glBegin(GL_POLYGON);
	glColor3f(0.7f,0.7f,1.0f);
	glVertex3f(-250.0f,-120.0f,-130.0f);
	glVertex3f(250.0f,-120.0f,-130.0f);
	glColor3f(0.3f,0.8f,0.3f);	
	glVertex3f(200.0f,100.0f,-100.0f);
	glVertex3f(-200.0f,100.0f,-100.0f);
	glEnd();
	glEnable(GL_LIGHTING);

	glTranslatef(-6.5,6,-9.0f); // Posiciona a câmera centralizada no tecido
	glRotatef(25,0,1,0); // Rotaciona pra ver o tecido pelo lado
	massSpring1.drawShaded(); // Desenha o tecido shaded.
	
	//Desenha uma bola de raio ball_radius
	if(ballEnabled)
	{
		massSpring1.ballCollision(ball_pos,ball_radius); // trata a colisão com a bola
		ball_time++;
		ball_pos.f[2] = cos(ball_time/50.0)*7;
		drawBall();
	}

	writeMenu();
	
	glutSwapBuffers();
	glutPostRedisplay();
}

void reshape(int w, int h)  
{
	glViewport(0, 0, w, h);
	glMatrixMode(GL_PROJECTION); 
	glLoadIdentity();  
	if (h==0)  
		gluPerspective(80,(float)w,1.0,5000.0);
	else
		gluPerspective (80,( float )w /( float )h,1.0,5000.0 );
	glMatrixMode(GL_MODELVIEW);  
	glLoadIdentity(); 
}

bool wireframe = false;
void toggleWireframeMode()
{
	if(wireframe)
		{
			glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );
			wireframe = false;
		}
		else
		{
			glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );
			wireframe = true;
		}
}

void keyboard( unsigned char key, int x, int y ) 
{
	switch ( key ) {
	case 27:    
		exit ( 0 );
	case 32:
		//Aplica uma força na partícula selecionada.
		massSpring1.addForceToSelectedParticle(Vec3(2,0,0.8)*TIME_STEPSIZE2);
		break;
	case 119:
		toggleWireframeMode();		
		break;
	case 'b':
		massSpring1.toggleSpherizedParticle();
		break;
	default: 
		break;
	}
}


void Special( int a_keys, int x, int y ) 
{
	switch(a_keys) {
	case GLUT_KEY_UP:
		//Move a seleção pra cima
		massSpring1.select(SELECTION_UP);
		break;
	case GLUT_KEY_DOWN: 
		//Muda a seleção pra baixo
		massSpring1.select(SELECTION_DOWN);
		break;
	case GLUT_KEY_LEFT:
		//Muda a seleção pra esquerda
		massSpring1.select(SELECTION_LEFT);
		break;
	case GLUT_KEY_RIGHT: 
		//Muda a seleção pra direita
		massSpring1.select(SELECTION_RIGHT);
		break;
	case GLUT_KEY_F1:
		toggleBallEnabled();
		break;
	case GLUT_KEY_F2:
		toggleWindEnabled();
		break;
	case GLUT_KEY_F3:
		toggleGravityEnabled();
		break;
	default:
		break;
	}
}

void MouseButton(int button, int state, int x, int y)
{
  // Respond to mouse button presses.
  // If button1 pressed, mark this state so we know in motion function.
  if (button == GLUT_LEFT_BUTTON)
    {
      g_bButton1Down = (state == GLUT_DOWN) ? TRUE : FALSE;
	  g_yClick = y - 3 * view_distance;
    }
}
void MouseMotion(int x, int y)
{
  // If button1 pressed, zoom in/out if mouse is moved up/down.
  if (g_bButton1Down)
    {
      view_distance = (y - g_yClick) / 3.0;
      if (view_distance < VIEWING_DISTANCE_MIN)
         view_distance = VIEWING_DISTANCE_MIN;
      glutPostRedisplay();
    }
}

int main ( int argc, char** argv ) 
{
	ballEnabled = gravityEnabled = windEnabled = false;
	glutInit( &argc, argv );

	
	glutInitDisplayMode( GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH ); 
	glutInitWindowSize(1280, 720 ); 

	glutCreateWindow( "Trabalho de CGII - Rodrigo e Leonardo Panatta." );
	init();
	glutDisplayFunc(Display);  
	glutReshapeFunc(reshape);

	glutKeyboardFunc(keyboard);
	glutSpecialFunc(Special);
	glutMouseFunc (MouseButton);
    glutMotionFunc (MouseMotion);

	glutMainLoop();
}