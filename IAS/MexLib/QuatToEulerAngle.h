
/*
 * QuatToEulerAngles.h
 *
 *  Created on: Mar 29, 2014
 *      Author: rueckert
 */

#ifndef QUATTOEULERANGLE_H_
#define QUATTOEULERANGLE_H_

void quatToEulerAngles(SL_quat *quat, double *eulerAngles) {

	double threshold = 0.001;
	double qx, qy, qz, qw, heading, attitude, bank;
	qw = quat->q[_Q0_];
	qx = quat->q[_Q1_];
	qy = quat->q[_Q2_];
	qz = quat->q[_Q3_];

	heading = atan2(2.0*qy*qw-2.0*qx*qz , 1.0 - 2.0*qy*qy - 2.0*qz*qz);
	attitude = asin(2.0*qx*qy + 2.0*qz*qw);
	bank = atan2(2.0*qx*qw-2.0*qy*qz , 1.0 - 2.0*qx*qx - 2.0*qz*qz);

	if((qx*qy + qz*qw - 0.5)*(qx*qy + qz*qw - 0.5) < threshold) { //(north pole)
		   heading = 2.0 * atan2(qx,qw);
		   bank = 0;
	}
    if((qx*qy + qz*qw + 0.5)*(qx*qy + qz*qw + 0.5) < threshold) { //(south pole)
		   heading = -2.0 * atan2(qx,qw);
		   bank = 0;
    }
    eulerAngles[_A_] = attitude;
    eulerAngles[_B_] = heading;
    eulerAngles[_G_] = bank;
}


#endif /* QUATTOEULERANGLE_H_ */
