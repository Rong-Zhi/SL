/*======================================================================
========================================================================
                      
                              initUserGraphics.c
 
========================================================================
Remarks:

         Functions needed for user graphics
         simulation

======================================================================*/

#include "SL.h"
#include "SL_user.h"
#include "SL_man.h"

// openGL includes
#ifdef powerpc
#include <GLUT/glut.h>
#else
#include "GL/glut.h"
#endif

#include "SL_openGL.h"
#include "SL_userGraphics.h"

#include "math.h"


void draw_disk_no_push(float radius, float height, GLfloat *col) {
	GLUquadric* nQ;
	
	nQ = gluNewQuadric();
	gluQuadricDrawStyle(nQ, GLU_FILL); 
	glColor3f(col[1], col[2], col[3]);
	gluCylinder(nQ, radius, radius, height, 20, 20);
	gluDisk(nQ, 0.0, radius, 20, 20);
	glTranslatef(0, 0, +height/10);
	gluDisk(nQ, 0.0, radius, 20, 20);
	gluDeleteQuadric(nQ); 
}

void draw_disk(float x, float y, float z, float radius, float height, GLfloat *col) {
	GLUquadric* nQ;

	glPushMatrix();
	glTranslated((GLdouble)x,(GLdouble)y, (GLdouble)z);

	draw_disk_no_push(radius,height,col);
	glPopMatrix();
}

void draw_cube(float x, float y, float z, float l, float w, float h, GLfloat *col) {
	glPushMatrix();
	glColor3f(col[1], col[2], col[3]);
	glTranslatef(x, y, z);
	glScalef(l, w, h);
	glutSolidCube(1); 
	glPopMatrix();
}

void draw_rotated_beam(double x, double y, double z, double l, double w, double h, GLfloat *col, SL_quat q) {
	double n = sqrt(1-sqr(q.q[2])+sqr(q.q[3])+sqr(q.q[4]));
	glPushMatrix();
	glColor3f(col[1], col[2], col[3]);
	glTranslatef(x, y , z);
	glRotated((2*180./PI)*acos(q.q[1]), q.q[2]/n, q.q[3]/n, q.q[4]/n); 
	glScalef(l, w, h);		
	glutSolidCube(1); 
	glPopMatrix();
}

void draw_rotated_disk(float x, float y, float z, float radius, float height, GLfloat *col, SL_quat q) {
	double n; 

	glPushMatrix();
	glTranslated((GLdouble)x,(GLdouble)y, (GLdouble)z);

	n = sqrt(1-sqr(q.q[2])+sqr(q.q[3])+sqr(q.q[4]));
	glRotated((2*180./PI)*acos(q.q[1]), q.q[2]/n, q.q[3]/n, q.q[4]/n); 

	draw_disk_no_push(radius,height,col);
	glPopMatrix();
}

void draw_ball (double x, double y, double z, double radius, GLfloat * col)
{
	glPushMatrix ();
	glTranslated ((GLdouble) x, (GLdouble) y, (GLdouble) z);
	glColor3f (col[1], col[2], col[3]);
	glutSolidSphere (radius, 8, 8);
	glPopMatrix ();
}

void draw_string (double xc, double yc, double zc,
			 double xa, double ya, double za,
			 double xb, double yb, double zb, GLfloat * col)
{
	glDisable(GL_LIGHTING);		// no orientation -> else rendered incorrectly
	glLineWidth(2.);
	glBegin (GL_LINE_STRIP);
	glColor3f (col[1], col[2], col[3]);
	glVertex3d (xc, yc, zc);	// origin of the line
	glVertex3d (xa, ya, za);	// via point of the line
	glVertex3d (xb, yb, zb);	// ending point of the line
	glEnd ();
	glLineWidth(1.);
	glEnable(GL_LIGHTING);
}

void draw_rotated_cup (float x, float y, float z, float radius, float height, float wall, float rim, GLfloat * col, SL_quat q)
{
	glPushMatrix ();
	glTranslated ((GLdouble) x, (GLdouble) y, (GLdouble) z);
	
	glRotated ((180. / PI) * 2 * acos (q.q[1]), q.q[2], q.q[3], q.q[4]);
		
	GLUquadric *nQ;
	nQ = gluNewQuadric ();
	gluQuadricDrawStyle (nQ, GLU_FILL);
	glColor3f (col[1], col[2], col[3]);
	//bottom
	gluDisk (nQ, 0.0, radius-rim, 20, 20);
	//bottom rim
	gluCylinder (nQ, radius-rim, radius, rim, 20, 20);
	//outside
	glTranslated (0., 0., rim);
	gluCylinder (nQ, radius, radius, height-2*rim, 20, 20);
	//inside
	glTranslated (0., 0., wall-rim);
	gluCylinder (nQ, radius-wall, radius-wall, height-wall, 20, 20);
	//inside bottom
	gluDisk (nQ, 0.0, radius-wall, 20, 20);
	//top rim
	glTranslated (0., 0., height-wall-rim);
	gluCylinder (nQ, radius, radius-rim, rim, 20, 20);
	//top
	glTranslated (0., 0., rim);
	gluDisk (nQ, radius-wall, radius-rim, 20, 20);
	gluDeleteQuadric (nQ);
	
	glPopMatrix ();
}


void clearUserGraphicsWrapper ( void *b ) {
	(void)b;
	clearUserGraphics();
}


/*****************************************************************************
******************************************************************************
Function Name	: initUserGraphics
Date		: June 1999
	 
Remarks:

			allows adding new graphics functions to openGL interface

******************************************************************************
Paramters:	(i/o = input/output)

	none	 

*****************************************************************************/
int initUserGraphics(void)
{
	
	addToUserGraphics("clearUserGraphics","Clear User Graphics",&(clearUserGraphicsWrapper),0);

#ifdef _CMAKE_SL_TASK_AUTO_ADD_
#include <user_graphics_autodetect.h>
#endif // _CMAKE_SL_TASK_AUTO_ADD_

	switchCometDisplay(TRUE,500);

	return TRUE;

}
