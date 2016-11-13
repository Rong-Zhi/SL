function [trajState, flag] = SLInitEpisode(initState, maxCommands)

[N_DOFS, N_DOFS_SHM] = SLGetInfoMex;

if (nargin == 1)
    maxCommands = 2;
end

numCommand  = 1;
waitTime    = 0.0;
trajectory  = zeros(1,N_DOFS_SHM);
timeOut     = 20; %the original is 20
stateBuffer = initState;

[trajState, flag] = SLSendTrajectoryMex(numCommand, ...
    maxCommands, ...
    waitTime, ...
    trajectory, ...
    stateBuffer, ...
    timeOut);

end