/***********************************************************************
 * This file contains the functions for the graphic simulation of the
 * task.
 **********************************************************************/
#include "SL_system_headers.h"
#include "SL.h"
#include "GL/glut.h"

#include "ballonbeam.h"
#include "ballonbeam_env.h"
#include "initUserGraphics.h"
#include "SL_userGraphics.h"

static void display_ballonbeam(void *envP)
{
    ballonbeam_environmentVar *env = envP;

	GLfloat col[4]			= {(float)1.0,(float)1.0,(float)0.0,(float)1.0};
	GLfloat black[4]		= {(float)1., (float)0., (float)0., (float)0.};
	GLfloat wood[4]			= {(float)1., (float)0.8,(float)0.6,(float)0.4};
	GLfloat string_color[4] = {(float)1., (float)1., (float)1., (float)1.};
	GLfloat colTarget[4]	= {(float)0.0,(float)1.0,(float)0.0,(float)1.0};
		
	// draw the ball
	glPushMatrix();
	glTranslated((GLdouble)env->ballX, (GLdouble)env->ballY, (GLdouble)env->ballZ);
	glColor4fv(col);
	glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, col);
	glutSolidSphere(ballRadius,8,8);
	glPopMatrix();
	
	// draw the beam
	draw_rotated_beam(env->beamX, env->beamY, env->beamZ, beamLength, beamWidth, beamHeight, wood, env->varOrient);
}

void add_ballonbeam_graphics(void) {
	addToUserGraphics("ballOnBeam","Display Ball On A Beam",&(display_ballonbeam),sizeof(ballonbeam_environmentVar));
}

