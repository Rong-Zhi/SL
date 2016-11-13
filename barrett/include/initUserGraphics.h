// openGL includes
#ifdef powerpc
#include <GLUT/glut.h>
#else
#include "GL/glut.h"
#endif


void draw_disk_no_push(float radius, float height, GLfloat *col);

void draw_disk(float x, float y, float z, float radius, float height, GLfloat *col);

void draw_cube(float x, float y, float z, float l, float w, float h, GLfloat *col);

void draw_rotated_beam(double x, double y, double z, double l, double w, double h, GLfloat *col, SL_quat q);

void draw_rotated_disk(float x, float y, float z, float radius, float height, GLfloat *col, SL_quat q);

void draw_ball (double x, double y, double z, double radius, GLfloat * col);

void draw_string (double xc, double yc, double zc, double xa, double ya, double za, double xb, double yb, double zb, GLfloat * col);

void draw_rotated_cup (float x, float y, float z, float radius, float height, float wall, float rim, GLfloat * col, SL_quat q);
