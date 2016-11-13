/*
 * NumericDerivationObject.h
 *
 *  Created on: Mar 29, 2014
 *      Author: rueckert
 */

#ifndef NUMERICDERIVATIONOBJECT_H_
#define NUMERICDERIVATIONOBJECT_H_

#include "stddef.h"

class NumericDerivationObject {
protected:
	double* x_;
	double* y_;
	int numInputs_;
	int numOutputs_;

public:

	NumericDerivationObject(double *x, double *y, int numInputs, int numOutputs)
	: x_(x), y_(y), numInputs_(numInputs), numOutputs_(numOutputs) {}

	NumericDerivationObject(const NumericDerivationObject& cpyObj)
	: x_(NULL), y_(NULL), numInputs_(cpyObj.numInputs_), numOutputs_(cpyObj.numOutputs_) { }

	virtual ~NumericDerivationObject() {}

	NumericDerivationObject & operator=(const NumericDerivationObject &cpyObj) {
		x_ = NULL;
		y_ = NULL;
		numInputs_ = cpyObj.numInputs_;
		numOutputs_ = cpyObj.numOutputs_;
        return *this;
	}

	virtual void * RunThread() = 0;

	double* getX() const {
		return x_;
	}

	double getX(int index) const {
		return x_[index];
	}

	void setX(double* x) {
		x_ = x;
	}

	double* getY() const {
		return y_;
	}

	double getY(int index) const {
		return y_[index];
	}

	void setY(double* y) {
		y_ = y;
	}

	int getNumInputs() const {
		return numInputs_;
	}

	int getNumOutputs() const {
		return numOutputs_;
	}
};

#endif /* NUMERICDERIVATIONOBJECT_H_ */
