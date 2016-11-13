#ifndef SL_EPISODIC_COMM_UTILS_H_
#define SL_EPISODIC_COMM_UTILS_H_

#include "SL_user.h"
#include "SL_episodic_communication.h"

extern int maxCommands;
extern int state;
extern int useInitStep;
extern int commandIdx;

void writeStepToSharedMemoryGeneral(int numStepInEpisode);
void episodicSharedMemory(void (*doMotionEpisodeStepFunc)(int, double, int),  int (*isStepOverFunc)(int , double , int ), void (*writeStepToSharedMemoryFunc)(int ), int *simPhysics);
void episodicSharedMemoryWait(void (*doMotionEpisodeStepFunc)(int, double, int),  int (*isStepOverFunc)(int , double , int ), void (*writeStepToSharedMemoryFunc)(int ), int *simPhysics, int waitForMatlab, int holdInPlace);
void episodicSharedMemoryNoMotion(void (*doMotionEpisodeStepFunc)(int, double, int),  int (*isStepOverFunc)(int , double , int ), void (*writeStepToSharedMemoryFunc)(int ), int *simPhysics);

#define MAX_STEP_TIME 10


#endif /* SL_EPISODIC_COMM_UTILS_H_ */
