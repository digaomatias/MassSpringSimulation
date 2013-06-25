
#include "MassaMola.h"

/***** Acima estão as definições das classes; Vector3, Vertice, Mola, and MassaMola *****/

 
MassaMola massaMola(14,10,30,20, 60*20); // Com esses valores ficou extremamente lento: 14, 10, 55, 45

/***** Toda a parte de controle opengl é feita abaixo *****/
float view_distance = 2;
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
		massaMola.addForca(Vector3(0,-0.2,0)*DT); // Adiciona gravidade a cade frame, na direção pra baixo
	
	if(windEnabled)
		massaMola.aplicaVento(Vector3(0.5,0,0.2)*DT); // gera vento a cada frame
	massaMola.passoDeTempo(); // calcula a posição da particula no frame seguinte

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
	massaMola.drawShaded(); // Desenha o tecido shaded.
	
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
		massaMola.addForcaToSelectedVertice(Vector3(2,0,0.8)*DT);
		break;
	case 119:
		toggleWireframeMode();		
		break;
	case 'b':
		massaMola.alternaVerticeEsferizado();
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
		massaMola.seleciona(SELECTION_UP);
		break;
	case GLUT_KEY_DOWN: 
		//Muda a seleção pra baixo
		massaMola.seleciona(SELECTION_DOWN);
		break;
	case GLUT_KEY_LEFT:
		//Muda a seleção pra esquerda
		massaMola.seleciona(SELECTION_LEFT);
		break;
	case GLUT_KEY_RIGHT: 
		//Muda a seleção pra direita
		massaMola.seleciona(SELECTION_RIGHT);
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
	gravityEnabled = windEnabled = false;
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