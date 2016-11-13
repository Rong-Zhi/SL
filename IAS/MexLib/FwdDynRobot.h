/*
 * FwdDynRobot.h
 *
 *  Created on: Mar 29, 2014
 *      Author: rueckert
 */

#ifndef FWDDYNROBOT_H_
#define FWDDYNROBOT_H_

#include "NumericDerivation.h"
#include "NDObjectRobot.h"

// SL general includes of system headers
#include "SL_system_headers.h"

/* private includes */
#include "SL.h"
#include "SL_common.h"
#include "SL_dynamics.h"
#include "utility.h"
#include "utility_macros.h"

#define MSG_COULD_NOT_INIT_DYNAMICS "MexLib: could not initialize the dynamics via init_dynamics()";

template <class T>
class FwdDynRobot : public NumericDerivation<T> {
protected:
	double* x_;
	double* u_;
	int szX_;
	int szU_;
	double* outArrayPtrXdot_;
	double* outArrayPtrXdotX_;
	double* outArrayPtrXdotU_;

public:
	FwdDynRobot(double *x, double *u, int szX, int szU,
			double* outArrayPtrXdot, double* outArrayPtrXdotX, double* outArrayPtrXdotU)
	: NumericDerivation<T>(), x_(x), u_(u), szX_(szX), szU_(szU),
			  outArrayPtrXdot_(outArrayPtrXdot), outArrayPtrXdotX_(outArrayPtrXdotX),
			  outArrayPtrXdotU_(outArrayPtrXdotU) { }
	~FwdDynRobot() {}

	int initiateComputation() {

//		setDefaultEndeffector();
		if (!init_dynamics())
			throw MSG_COULD_NOT_INIT_DYNAMICS; //return NULL;

		double xAndU[szX_ + szU_];
		int i, retVal;
		for(i=0; i<szX_; i++)
			xAndU[i] = x_[i];
		for(i=0; i<szU_; i++)
			xAndU[szX_+i] = u_[i];

		NDObjectRobot single(xAndU, outArrayPtrXdot_, szX_, szU_);
		if(outArrayPtrXdotX_ == 0) {
			return NumericDerivation<T>::initiateFunctionDerivative(&single);
		}
		else {
			double resultsYdotX[szX_*(szX_+szU_)];
			retVal = NumericDerivation<T>::initiatePartitialDerivative(&single, resultsYdotX);
			for(i=0; i<szX_*szX_; i++)
				outArrayPtrXdotX_[i] = resultsYdotX[i];
			for(i=0; i<szX_*szU_; i++)
				outArrayPtrXdotU_[i] = resultsYdotX[szX_*szX_ + i];
			return retVal;
		}
	}

	int getSzU() const {
		return szU_;
	}

	int getSzX() const {
		return szX_;
	}

	double* getU() const {
		return u_;
	}

	double* getX() const {
		return x_;
	}

	double* getOutArrayPtrXdot() const {
		return outArrayPtrXdot_;
	}

	double* getOutArrayPtrXdotU() const {
		return outArrayPtrXdotU_;
	}

	double* getOutArrayPtrXdotX() const {
		return outArrayPtrXdotX_;
	}
};

#endif /* FWDDYNROBOT_H_ */
