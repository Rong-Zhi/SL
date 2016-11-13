function [joints, jointsVel, jointsAcc, jointsDes, jointsVelDes, jointsAccDes, ...
    torque, cart, episodeState, numCommand, stepIndex] = SLGetEpisode(numStates)

if (nargin == 0)
    numStates = 10;
end

[joints, jointsVel, jointsAcc, jointsDes, jointsVelDes, jointsAccDes, torque, ...
    cart, episodeState, numCommand, stepIndex] = SLGetEpisodeMex(numStates);
