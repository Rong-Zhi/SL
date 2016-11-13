// Header guard
#ifndef BALLONBEAM_ENV_H
#define BALLONBEAM_ENV_H

// This struct contains the variables of the environment. In the Ball on
// Beam task they are the position of the ball, of the beam and the
// orientation of the beam.
typedef struct ballonbeam_environmentVar {
    double beamX;
    double beamY;
    double beamZ;
    double ballX;
    double ballY;
    double ballZ;
	SL_quat varOrient;
} ballonbeam_environmentVar;

#endif
