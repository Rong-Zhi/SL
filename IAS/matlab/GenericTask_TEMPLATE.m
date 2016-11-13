% Place this file in you robot folder 
% and change the following paths if necessary
%
% Parameters:
% - COM_MATSTATE : ID of the command
%                  * 0 for no operation
%                  * 1 for goto task
%                  * 2 for kinesthetic teaching
%                  * 3 for sending a trajectory
% - COM_MAXTIME  : max time of the communication 
%                  (useful for kinesthetic teaching)
% - COM_CTL      : controller ID
%                  * -1 for a user-defined controller (e.g., Darias)
%                  * 0 for PD with gravity compensation
%                  * 1 for model-based controller

addpath('../../../build/<ROBOT_FOLDER>/')
addpath('../../IAS/matlab/')

[N_DOFS, N_DOFS_SHM] = SLGetInfoMex;


%% Goto state
SLInitEpisode([]);

initJointState = zeros(1, N_DOFS_SHM); % choose the desired state

COM_MATSTATE = 1;
COM_MAXTIME  = 10.0;
COM_CTL      = 0;
stateBuffer  = [COM_MATSTATE, COM_MAXTIME, COM_CTL];

waitTime    = 0.0;
numCommand  = 2;
maxCommands = 2;
timeOut     = 10000;

[reward, trajState, flag] = SLSendTrajectory(initJointState, waitTime, ... 
    numCommand, maxCommands, stateBuffer, timeOut);

[joints, jointsVel, jointsAcc, jointsDes, jointsVelDes, jointsAccDes, ...
	torque, cart, episodeState, numCommand, stepIndex] = SLGetEpisode();

disp('GOTO DONE!')


%% Follow a trajectory
SLInitEpisode([]);

COM_MATSTATE = 3;
COM_MAXTIME  = 5.0;
COM_CTL      = 0;
stateBuffer  = [COM_MATSTATE, COM_MAXTIME, COM_CTL];

waitTime    = 0.0;
numCommand  = 2;
maxCommands = 2;
timeOut     = 10000;

trajectory = load('<TRAJECTORY>'); % load the trajectory to execute

[reward, trajState, flag] = SLSendTrajectory(trajectory, waitTime, ... 
    numCommand, maxCommands, stateBuffer, timeOut);

[joints, jointsVel, jointsAcc, jointsDes, jointsVelDes, jointsAccDes, ...
	torque, cart, episodeState, numCommand, stepIndex] = SLGetEpisode();

disp('TRAJECTORY DONE!')


%% Reset the robot
SLResetEpisode


%% Kinesthetic teaching
SLInitEpisode([]);

COM_MATSTATE = 2;
COM_MAXTIME  = 5.0;
COM_CTL      = 0;
stateBuffer  = [COM_MATSTATE, COM_MAXTIME, COM_CTL];

waitTime    = 0.0;
numCommand  = 2;
maxCommands = 2;
timeOut     = 10000;

[reward, trajState, flag] = SLSendTrajectory(zeros(1,N_DOFS_SHM), ... 
    waitTime, numCommand, maxCommands, stateBuffer, timeOut);

[joints, jointsVel, jointsAcc, jointsDes, jointsVelDes, jointsAccDes, ...
	torque, cart, episodeState, numCommand, stepIndex] = SLGetEpisode();

disp('KINESTHETIC TEACHING DONE!')
