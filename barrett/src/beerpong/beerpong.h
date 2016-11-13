#ifndef BEERPONG_H
#define BEERPONG_H

static const double cup_radius      = 0.04;
static const double cup_height      = 0.1;
static const double cup_wall        = 0.008;
static const double cup_rim         = 0.0030;
static const double cup_z           = 0;
static const double ball_radius     = 0.02;
static const double table_height    = 0.62;
static const double table_length    = 1.2;
static const double table_width     = 0.7;
static const double table_thickness = 0.05;
static const double table_center    = 0.0;
static const double dist_to_table   = -1.5;
static const double floor_level     = -1.45;
static const double restitution     = 0.5;
static const double launcher_l      = 0.16;
static const double launcher_w      = 0.04;
static const double launcher_h      = 0.02;

extern SL_Cstate cup_state;
extern SL_Cstate ball_state;

int sim_beerpong_state();
void send_beerpong_graphics();
void init_beerpong_state();
int add_beerpong_vars();

#endif
