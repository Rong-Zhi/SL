function SLResetEpisode()

[N_DOFS, N_DOFS_SHM] = SLGetInfoMex;

numCommand  = -1;
maxCommands = 1;
waitTime    = 0.0;
trajectory  = zeros(1,N_DOFS_SHM);
timeOut     = 10.0;
stateBuffer = [];

[trajState, flag] = SLSendTrajectoryMex(numCommand, ...
    maxCommands, ...
    waitTime, ...
    trajectory, ...
    stateBuffer, ...
    timeOut);

end
