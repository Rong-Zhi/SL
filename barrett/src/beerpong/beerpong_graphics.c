/***********************************************************************
 * This file contains the functions for the graphic simulation of the
 * task.
 **********************************************************************/
#include "SL_system_headers.h"
#include "SL.h"
#include "GL/glut.h"

#include "beerpong.h"
#include "beerpong_env.h"
#include "initUserGraphics.h"
#include "SL_userGraphics.h"


void display_beerpong (void *envP)
{
    beerpong_environmentVar *env = envP;

	GLfloat cup_color[4]  =	{ (float)1., (float)1., (float)1.,   (float)1.  };
	GLfloat ball_color[4] = { (float)0., (float)1., (float)1.,   (float)0.  };
	GLfloat wood[4]       = { (float)1., (float)0.8, (float)0.6, (float)0.4 };

	// draw the cup
	draw_rotated_cup (env->cupX, env->cupY, env->cupZ, cup_radius,
						cup_height, cup_wall, cup_rim, cup_color, env->varOrient);
						
	// draw the ball
	draw_ball (env->ballX, env->ballY, env->ballZ, ball_radius, ball_color);	
	
	// draw the top of the table
	draw_cube(table_center, dist_to_table-.5*table_length, floor_level+table_height+.5*table_thickness, 
					table_width, table_length, table_thickness, wood);

	// draw the legs of the table
	draw_cube(table_center+.5*table_width-table_width/6., dist_to_table-table_width/6., floor_level+.5*table_height, 
				table_thickness, table_thickness, table_height, wood);
	draw_cube(table_center-.5*table_width+table_width/6., dist_to_table-table_width/6., floor_level+.5*table_height, 
				table_thickness, table_thickness, table_height, wood);
	draw_cube(table_center+.5*table_width-table_width/6., dist_to_table-table_length+table_width/6., floor_level+.5*table_height, 
				table_thickness, table_thickness, table_height, wood);
	draw_cube(table_center-.5*table_width+table_width/6., dist_to_table-table_length+table_width/6., floor_level+.5*table_height, 
				table_thickness, table_thickness, table_height, wood);
}

void add_beerpong_graphics(void) {
	addToUserGraphics("beerpong","Display Beerpong",&(display_beerpong),sizeof(beerpong_environmentVar));
}

