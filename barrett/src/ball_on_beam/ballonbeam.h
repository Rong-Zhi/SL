// Header guard
#ifndef BALLONBEAM_H
#define BALLONBEAM_H

// Constant parameters
static const double beamWidth   = 0.2;
static const double beamLength  = 1.f;
static const double beamHeight  = 0.01;
static const double ballRadius  = 0.04;
static const double restitution = 0.75;

// These variables are used by 'ballonbeam_sim.c' and 'ballonbeam_graphics.c'.
// The first file redeclare and assign them a value, while the second one only uses them.
extern SL_Cstate beamState;
extern SL_Cstate ballState;

// Functions implemented by 'ballonbeam_sim.c' and used also by 'ballonbeam_task.c'.
int addBallOnBeamVars ();
void resetBallOnBeamSimulation();
int simBallOnBeamTask();
void sendBallOnBeamGraphics();

#endif
