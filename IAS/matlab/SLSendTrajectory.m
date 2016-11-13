function [ reward, trajState, flag ] = ...
    SLSendTrajectory(trajectory, waitTime, numCommand, maxCommands, stateBuffer, timeOut)

[N_DOFS, N_DOFS_SHM] = SLGetInfoMex;

if (nargin == 1)
    waitTime    = 0.0;
    numCommand  = 2;
    maxCommands = 2;
    stateBuffer = [];
end

if(~exist('timeOut','var'))
    timeOut = 10;
end

[trajState, flag] = SLSendTrajectoryMex(numCommand, ...
    maxCommands, ...
    waitTime, ...
    trajectory, ...
    stateBuffer, ...
    timeOut);

if (flag == 1)
    reward = trajState(1);
else
    reward = -inf;
end

end
