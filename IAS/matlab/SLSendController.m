function [ reward, trajState, flag ] = SLSendController(ctlParams)

[N_DOFS, N_DOFS_SHM] = SLGetInfoMex;

numCommand  = 2;
maxCommands = 2;
waitTime    = 0.0;
trajectory  = zeros(1,N_DOFS_SHM);
timeOut     = 20;
stateBuffer = ctlParams;


[trajState,flag] = SLSendTrajectoryMex(numCommand, ....
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
