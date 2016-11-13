% See 'GenericTask_TEMPLATE.m' for more details.

addpath('../../build/barrett/')
addpath('../../IAS/matlab/')

[N_DOFS, N_DOFS_SHM] = SLGetInfoMex;


%% Goto state
SLInitEpisode([]);

initJointState = [0, 1, 0, 2.7, 0, 0, 0]; % beerpong init state

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

waitTime    = 1.0;
numCommand  = 2;
maxCommands = 2;
timeOut     = 10000;

h = load('BeerPong_exampleTrajectory.mat');
trajectory = h.joints;

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
