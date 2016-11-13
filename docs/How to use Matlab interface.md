## Configuration
----------------
You will use functions in `sl/IAS/matlab`. You may need to change the default value of `N_DOFS_SHM`, the degrees of freedom you actually want to control in SL. You can edit this value in the header `SL_user.h` of your robot.
Also, you need to add to your Matlab path the following directories: your robot build folder and `sl/IAS/matlab`.


## 1. SLGetInfoMex
------------------
#### Output
- `N_DOFS` : degrees of freedom of the robot
- `N_DOFS_SHM` : degrees of freedom you actually want to control
- `STEPLIMIT` : maximum number of steps allowed for a trajectory


## 2. SLGetEpisodeMex
----------------------
It is used to 'ask data' to SL. It gets the data from the last SL episode, i.e. states, actions and other variables. Matlab uses it by `SLGetEpisode.m`.

#### Input
- `numStates` : see *4. Notes*

#### Output
- `joints` : joints positions
- `jointsVel` : joints velocities
- `jointsAcc` : joints accelerations
- `jointsDes` : joints desired positions
- `jointsVelDes` : joints desired velocities
- `jointsAccDes` : joints desired accelerations
- `torque` : joints torques
- `cart` 	: joints positions in cartesian space
- `episodeState` : see *4. Notes*
- `numCommand` : see *4. Notes*
- `stepIndex` : **USED ONLY FOR THE TABLE TENNIS TASK**


## 3. SLSendTrajectoryMex
--------------------------
It is used to 'send commands': it tells SL to execute a trajectory or it sends some parameters that SL will use (e.g., the gains of the PD controller). Matlab uses it by `SLInitEpisode.m`, `SLSendTrajectory.m`, `SLSendController.m` and `SLResetEpisode.m`.

### 3.1 SLInitEpisode.m
It initializes an episode. It must be called before `SLSendController.m`.

#### Input
- `initState` : the initial state of the problem (e.g. ball position and velocity)
- `maxCommands` : see *4. Notes*

#### Output
- `trajState` : see *4. Notes*
- `flag` : 1 if everything went well, -1 otherwise

### 3.2 SLSendTrajectory.m
It sends a trajectory to be executed by the robot in SL. Before using it, you must call `SLInitEpisode.m`.

#### Input
- `trajectory` : the trajectory that SL will execute
- `waitTime` : how many seconds the robot will wait to start the trajectory
- `numCommand` : see *4. Notes*
- `maxCommands` : see 4. Notes*
- `stateBuffer` : a buffer used to send additional info (e.g. the max duration of the trajectory)
- `timeOut` : a simple timeout, i.e. the max seconds that Matlab will wait for a response from SL

#### Output
- `reward` : the final cumulative reward of the episode (by default stored in `state(1)`)
- `trajState` : see *4. Notes*
- `flag` : 1 if everything went well, -1 otherwise

### 3.3 SLSendController.m
It sends the parameters of the controller used by SL. Before using it, you must call `SLInitEpisode.m`.

#### Input
- `ctlParams` : the parameters of the controllers (they will be sent with the state buffer)

#### Output
- `reward` : the final cumulative reward of the episode (by default stored in `state(1)`)
- `trajState` : see *4. Notes*
- `flag` : 1 if everything went well, -1 otherwise

### 3.4 SLResetEpisode.m
It resets the internal FSM.

## 4. Notes
-----------
Here are some additional notes. Please see the barrett task `ballonbeam.c` and `BallOnBeam_example.m` for more details.
See `SL_episodic_communication.h` for details about structures and maximum variables sizes.

### 4.1 Main SL variables
- `episodeStep.state[STEPLIMITEPISODE][MAXEPISODESTATES]` : rows indicate the step of the episode, while columns contain some custom variables. For instance you can save the immediate reward and the ball position at each time step

```
   episodeStep.state[numStepInEpisode][0] = calc_reward_ballonbeam();
   episodeStep.state[numStepInEpisode][1] = ballState.x[1];
   episodeStep.state[numStepInEpisode][2] = ballState.xd[1];
```

- `episodeState.state[NUMEPISODICSTATES]` : stores information about the episode (i.e., the executed trajectory). For instance, you can write the initial state (context) and the return of the episode

```
   episodeState.state[0] = ballState.x[1];
   episodeState.state[1] = ballState.xd[1];
   episodeState.state[2] = calc_sum_of_rewards();
```

To summarize, the relevant fixed sizes that you may want to change in `SL_episodic_communication.h` are:

- `STEPLIMITEPISODE` : maximum number of steps of an episode, i.e., maximum length of a trajectory sent from Matlab to SL
- `MAXEPISODESTATES` : maximum number of variables that you can save for each time step
- `NUMEPISODICSTATES` : maximum number of variables that you can save for an episode (trajectory)

### 4.2 Main Matlab variables
- `episodeState(i,j)` corresponds to `episodeStep.state[j][i]` in SL. **MIND THE SWAPPED INDICES!** In `SLGetEpisode.m` you can decide the maximum number of received variables per step by `numStates`.
- `trajState(i)` corresponds to `episodeState.state(i)` in SL.

### 4.3 numCommand
It is an index used by SL to recognize the command received from Matlab. By default:

- `SLInitEpisode.m` : 1
- `SLSendTrajectory.m` : 2
- `SLSendController.m` : 2
- `SLResetEpisode.m` : -1

SL needs to receive commands in the correct order. For instance, it can't receive `SLSendTrajectory` (2) without receiving `SLInitEpisode` (1) first.

### 4.4 maxCommands
It is the maximum number of commands that SL can receive before resetting its FSM after the current command. By default:

- `SLInitEpisode.m` : 2 (after it, SL should receive `SLSendController.m` or `SLSendTrajectory.m`)
- `SLSendTrajectory.m` : 2 (SL must receive `SLInitEpisode.m` first)
- `SLSendController.m` : 2 (SL must receive `SLInitEpisode.m` first)
- `SLResetEpisode.m` : 1

Please notice that you can change both `numCommand` and `maxCommands` according to your needs. If, for instance, you need to exchange some data between Matlab and SL before starting a trajectory, you can increase `maxCommands` in SLInitEpisode and SLSendTrajectory, and use the proper `numCommand` in SLSendTrajectory (2, 3, 4, ...).

