
/*
 * NumericDerivation.h
 *
 *  Created on: Mar 29, 2014
 *      Author: rueckert
 */

#ifndef NUMERICDERIVATION_H_
#define NUMERICDERIVATION_H_

#include "NumericDerivationObject.h"
#include <vector>

#include "pthread.h" /* for threading */
#include "mex.h"
#include "SL_user.h"

#define MSG_COULD_NOT_CREATE_THREAD "MexLib: Unable to create a pthread";

template <class T>
class NumericDerivation {

private:
	bool flgUseFwdDerivatives_;
	bool functionDerivativeWasCalled_;

public:
	NumericDerivation() { functionDerivativeWasCalled_ = false; flgUseFwdDerivatives_ = true; }
	virtual ~NumericDerivation() {}
	void SetModeFwdDerivative(){ flgUseFwdDerivatives_ = true; }
	void SetModeCentralDerivative() { flgUseFwdDerivatives_ = false; }

protected:
	int initiateFunctionDerivative(T *args) {
		functionDerivativeWasCalled_ = true;
		pthread_t single_thread;

		int rc = pthread_create(&single_thread, NULL, &NumericDerivation::thread_entry, (void*)args);
		if(rc != 0)
			throw MSG_COULD_NOT_CREATE_THREAD;
		pthread_join(single_thread, NULL); /* Wait until thread is finished */
		return rc;
	}

	int initiatePartitialDerivative(T *args, double* resultsYdotX){

		if(!functionDerivativeWasCalled_) {
			initiateFunctionDerivative(args);
		}

		if(flgUseFwdDerivatives_) {
			computeFwdDerivatives(args, resultsYdotX);
		} else {
			computeCentralDerivatives(args, resultsYdotX);
		}
		return 0;
	}

	int initiatePartitialDerivative(T *args, double* resultsYdotX,
			double* resultsEFdotX){

		if(!functionDerivativeWasCalled_) {
			initiateFunctionDerivative(args);
		}

		if(flgUseFwdDerivatives_) {
			computeFwdDerivatives(args, resultsYdotX, resultsEFdotX);
		} else {
			computeCentralDerivatives(args, resultsYdotX, resultsEFdotX);
		}
		return 0;
	}

private:

	void computeFwdDerivatives(T *args, double* resultsYdotX) {
		int numThreads = args->getNumInputs();
		int threadId;

		std::vector<T*> vecParamObjectsPointers;
		std::vector<double*> vecInputs;
		std::vector<double*> vecOutputs;
		generateThreadData(args, getEpsilon(), vecParamObjectsPointers,
				vecInputs, vecOutputs);

		runThreads(vecParamObjectsPointers);
		combineResultsFwdDerivatives(args, vecOutputs, resultsYdotX);

		for(threadId = 0; threadId < numThreads; threadId++) {
			delete vecInputs[threadId];
			delete vecOutputs[threadId];
			delete vecParamObjectsPointers[threadId];
		}
	}

	void computeFwdDerivatives(T *args, double* resultsYdotX,
			double *resultsEFdotX) {

		int numThreads = args->getNumInputs();
		int threadId;

		std::vector<T*> vecParamObjectsPointers;
		std::vector<double*> vecInputs;
		std::vector<double*> vecOutputs;
		std::vector<double*> vecOutputsEF;
		generateThreadData(args, getEpsilon(), vecParamObjectsPointers,
				vecInputs, vecOutputs, vecOutputsEF);

		runThreads(vecParamObjectsPointers);
		combineResultsFwdDerivatives(args, vecOutputs, resultsYdotX, vecOutputsEF, resultsEFdotX);

		for(threadId = 0; threadId < numThreads; threadId++) {
			delete vecInputs[threadId];
			delete vecOutputs[threadId];
			delete vecOutputsEF[threadId];
			delete vecParamObjectsPointers[threadId];
		}
	}

	void computeCentralDerivatives(T *args, double* resultsYdotX) {
		int numThreadsFwd = args->getNumInputs();
		int threadId;

		std::vector<T*> vecParamObjectsPointersFWD;
		std::vector<double*> vecInputsFWD;
		std::vector<double*> vecOutputsFWD;
		generateThreadData(args, getEpsilon(), vecParamObjectsPointersFWD,
				vecInputsFWD, vecOutputsFWD);
		std::vector<T*> unionParamObjectsBWD;
		std::vector<double*> vecInputsBWD;
		std::vector<double*> vecOutputsBWD;
		generateThreadData(args, (-1.0)*getEpsilon(), unionParamObjectsBWD,
				vecInputsBWD, vecOutputsBWD);

		std::vector<T*> unionParamObjectsFWDBWD;
		for(threadId=0; threadId < numThreadsFwd; threadId++){
			unionParamObjectsFWDBWD.push_back( vecParamObjectsPointersFWD[threadId] );
			unionParamObjectsFWDBWD.push_back( unionParamObjectsBWD[threadId] );
		}

		runThreads(unionParamObjectsFWDBWD);
		combineResultsCentralDerivatives(args, vecOutputsFWD, vecOutputsBWD, resultsYdotX);

		for(threadId = 0; threadId < numThreadsFwd; threadId++) {
			delete vecInputsFWD[threadId];
			delete vecOutputsFWD[threadId];
			delete vecInputsBWD[threadId];
			delete vecOutputsBWD[threadId];
		}
		for(threadId = 0; threadId < unionParamObjectsFWDBWD.size(); threadId++) {
			delete unionParamObjectsFWDBWD[threadId];
		}
	}

	void computeCentralDerivatives(T *args, double* resultsYdotX, double *resultsEFdotX) {
			int numThreadsFwd = args->getNumInputs();
			int threadId;

			std::vector<T*> vecParamObjectsPointersFWD;
			std::vector<double*> vecInputsFWD;
			std::vector<double*> vecOutputsFWD;
			std::vector<double*> vecOutputsFWDEF;
			generateThreadData(args, getEpsilon(), vecParamObjectsPointersFWD,
					vecInputsFWD, vecOutputsFWD, vecOutputsFWDEF);
			std::vector<T*> unionParamObjectsBWD;
			std::vector<double*> vecInputsBWD;
			std::vector<double*> vecOutputsBWD;
			std::vector<double*> vecOutputsBWDEF;
			generateThreadData(args, (-1.0)*getEpsilon(), unionParamObjectsBWD,
					vecInputsBWD, vecOutputsBWD, vecOutputsBWDEF);

			std::vector<T*> unionParamObjectsFWDBWD;
			for(threadId=0; threadId < numThreadsFwd; threadId++){
				unionParamObjectsFWDBWD.push_back( vecParamObjectsPointersFWD[threadId] );
				unionParamObjectsFWDBWD.push_back( unionParamObjectsBWD[threadId] );
			}

			runThreads(unionParamObjectsFWDBWD);
			combineResultsCentralDerivatives(args, vecOutputsFWD, vecOutputsBWD, resultsYdotX,
					vecOutputsFWDEF, vecOutputsBWDEF, resultsEFdotX);

			for(threadId = 0; threadId < numThreadsFwd; threadId++) {
				delete vecInputsFWD[threadId];
				delete vecOutputsFWD[threadId];
				delete vecOutputsFWDEF[threadId];
				delete vecInputsBWD[threadId];
				delete vecOutputsBWD[threadId];
				delete vecOutputsBWDEF[threadId];
			}
			for(threadId = 0; threadId < unionParamObjectsFWDBWD.size(); threadId++) {
				delete unionParamObjectsFWDBWD[threadId];
			}
		}

	void generateThreadData(T *args, double epsilon, std::vector<T*> &vecParamObjectsPointers,
			std::vector<double*> &vecInputs, std::vector<double*> &vecOutputs) {

		int numThreads = args->getNumInputs();
		T *tmpArgs = NULL;
		double *tmpInput;
		double *tmpOutput;
		int threadId, inputId;

		for(threadId = 0; threadId < numThreads; threadId++) {

			tmpArgs = new T(*args);
			tmpInput = new double[ args->getNumInputs() ];
			tmpOutput = new double[ args->getNumOutputs() ];

			vecParamObjectsPointers.push_back( tmpArgs );
			vecInputs.push_back( tmpInput );
			vecOutputs.push_back( tmpOutput );

			for(inputId=0; inputId < args->getNumInputs(); inputId++) {
				tmpInput[inputId] = args->getX(inputId);
				if(threadId == inputId)
					tmpInput[inputId] += epsilon;
			}
			tmpArgs->setX( tmpInput );
			tmpArgs->setY( tmpOutput );
		}
	}

	void generateThreadData(T *args, double epsilon, std::vector<T*> &vecParamObjectsPointers,
			std::vector<double*> &vecInputs, std::vector<double*> &vecOutputs,
			std::vector<double*> &vecOutputsEF) {

		int numThreads = args->getNumInputs();
		T *tmpArgs = NULL;
		double *tmpInput;
		double *tmpOutput;
		double *tmpOutputEF;
		int threadId, inputId;

		for(threadId = 0; threadId < numThreads; threadId++) {

			tmpArgs = new T(*args);
			tmpInput = new double[ args->getNumInputs() ];
			tmpOutput = new double[ args->getNumOutputs() ];
			tmpOutputEF = new double[ 6 * N_ENDEFFS ];

			vecParamObjectsPointers.push_back( tmpArgs );
			vecInputs.push_back( tmpInput );
			vecOutputs.push_back( tmpOutput );
			vecOutputsEF.push_back( tmpOutputEF );

			for(inputId=0; inputId < args->getNumInputs(); inputId++) {
				tmpInput[inputId] = args->getX(inputId);
				if(threadId == inputId)
					tmpInput[inputId] += epsilon;
			}
			tmpArgs->setX( tmpInput );
			tmpArgs->setY( tmpOutput );
			tmpArgs->setEndeffectorPosAndOrients( tmpOutputEF );
		}
	}

	void combineResultsFwdDerivatives(T *args, std::vector<double*> &vecOutputs, double* resultsYdotX) {

		int numThreads, threadId, outputId, yIdPerThread;
		numThreads = vecOutputs.size();
		double *tmpDotY =  args->getY();
		double *tmpOutput;
		double diff = 0.0;
		double epsilonTmp = 1.0/getEpsilon();

		for(threadId = 0; threadId < numThreads; threadId++) {
			tmpOutput = vecOutputs[threadId];
			for(outputId=0; outputId < args->getNumOutputs(); outputId++) {
				yIdPerThread = threadId*(args->getNumOutputs()) + outputId;
				diff = tmpOutput[outputId] - tmpDotY[outputId];
				resultsYdotX[yIdPerThread] = diff * epsilonTmp;
			}
		}
	}

	void combineResultsFwdDerivatives(T *args, std::vector<double*> &vecOutputs, double* resultsYdotX,
			std::vector<double*> &vecOutputsEF, double* resultsEFdotX) {

		int numThreads, threadId, outputId, efIdPerThread;
		numThreads = vecOutputs.size();
		double *tmpDotEF =  args->getEndeffectorPosAndOrients();
		double *tmpOutputEF;
		double diff = 0.0;
		double epsilonTmp = 1.0/getEpsilon();

		combineResultsFwdDerivatives(args, vecOutputs, resultsYdotX);

		for(threadId = 0; threadId < numThreads; threadId++) {
			tmpOutputEF = vecOutputsEF[threadId];
			for(outputId=0; outputId < 6; outputId++) {
				efIdPerThread = threadId*(6) + outputId;
				diff = tmpOutputEF[outputId] - tmpDotEF[outputId];
				resultsEFdotX[efIdPerThread] = diff * epsilonTmp;
			}
		}
	}

	void combineResultsCentralDerivatives(T *args, std::vector<double*> &vecOutputsFwd, std::vector<double*> &vecOutputsBwd, double* resultsYdotX) {

		int numThreads, threadId, outputId, yIdPerThread;
		numThreads = vecOutputsFwd.size();
		double *tmpOutputFwd;
		double *tmpOutputBwd;
		double diff = 0.0;
		double epsilonTmp = 1.0/(2.0*getEpsilon());

		for(threadId = 0; threadId < numThreads; threadId++) {
			tmpOutputFwd = vecOutputsFwd[threadId];
			tmpOutputBwd = vecOutputsBwd[threadId];
			for(outputId=0; outputId < args->getNumOutputs(); outputId++) {
				yIdPerThread = threadId*(args->getNumOutputs()) + outputId;
				diff = tmpOutputFwd[outputId] - tmpOutputBwd[outputId];
				resultsYdotX[yIdPerThread] = diff * epsilonTmp;
			}
		}
	}

	void combineResultsCentralDerivatives(T *args, std::vector<double*> &vecOutputsFwd,
			std::vector<double*> &vecOutputsBwd, double* resultsYdotX,
			std::vector<double*> &vecOutputsFwdEF, std::vector<double*> &vecOutputsBwdEF, double* resultsEFdotX) {

			int numThreads, threadId, outputId, efIdPerThread;
			numThreads = vecOutputsFwd.size();
			double *tmpOutputFwdEF;
			double *tmpOutputBwdEF;
			double diff = 0.0;
			double epsilonTmp = 1.0/(2.0*getEpsilon());

			combineResultsCentralDerivatives(args, vecOutputsFwd, vecOutputsBwd, resultsYdotX);

			for(threadId = 0; threadId < numThreads; threadId++) {
				tmpOutputFwdEF = vecOutputsFwdEF[threadId];
				tmpOutputBwdEF = vecOutputsBwdEF[threadId];
				for(outputId=0; outputId < 6; outputId++) {
					efIdPerThread = threadId*(6) + outputId;
					diff = tmpOutputFwdEF[outputId] - tmpOutputBwdEF[outputId];
					resultsEFdotX[efIdPerThread] = diff * epsilonTmp;
				}
			}
		}

	double getEpsilon() { return 0.00001; } //0.00001;

	static void * thread_entry(void * functionObject) {
		void *retVal = static_cast<T*>(functionObject)->RunThread();
		pthread_exit(NULL);
		return retVal;
	}

	void runThreads(std::vector<T*> vecParamObjectsPointers){

		int numThreads, threadId, retValue;
		numThreads = vecParamObjectsPointers.size();

		pthread_t threadsArray[numThreads];

		for(threadId = 0; threadId < numThreads; threadId++) {
			T* parameterObject = vecParamObjectsPointers[threadId];

			retValue = pthread_create(&threadsArray[threadId], NULL, &NumericDerivation::thread_entry, (void*)parameterObject);
			if(retValue != 0)
				throw MSG_COULD_NOT_CREATE_THREAD;
		}

		for(threadId = 0; threadId < numThreads; threadId++) {
			pthread_join(threadsArray[threadId], NULL);
		}
	}
};

#endif /* NUMERICDERIVATION_H_ */
