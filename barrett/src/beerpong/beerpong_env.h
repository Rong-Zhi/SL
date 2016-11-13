#ifndef BEERPONG_ENV_H
#define BEERPONG_ENV_H

typedef struct beerpong_environmentVar {
	double ballX;
	double ballY;
	double ballZ;
	double cupX;
	double cupY;
	double cupZ;
	SL_quat varOrient;
} beerpong_environmentVar;

#endif
