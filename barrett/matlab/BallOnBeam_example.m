% u(t) = Kp * (qd - qt) + Kd * (qdd - qtt);
close all
clear all;
clc;
addpath('/home/sl/build/barrett')
addpath('/home/sl/IAS/matlab')
% parameters initialization
% scalars
Nparams = 2;
Nsamples = 5;
a = 0.000005;
converged = false;
EpisodeCount = 0;
% vectors
gains_temp = [2, 2];
gains = [2,2];
delta_gains = [0.08, 0.08];
initState = [0.3, 0, 0];
% gFD = [0.01, 0.01];
gKp = zeros(100,1);
gKd = zeros(100,1);
gFD =[0,0];
% matrix
Jref_reward = zeros(Nsamples,Nparams);
J_reward = zeros(Nsamples+1,Nparams);
%Jdelta_reward = zeros(Nsamples,Nparams);
Delta_gains = repmat(delta_gains,Nsamples,1);
% initialize the state and J_rewards;
% while loop to search the converged gradients
while ~converged
    EpisodeCount = EpisodeCount + 1;
    gains_temp(1) = gains(1) + delta_gains(1);
    gains_temp(2) = gains(2) + delta_gains(2);
    SLInitEpisode(initState);
    [reward,trajState, flag] = SLSendController(gains);
    reward
    for i = 1:1:Nsamples
        % calculate the gradient for parameter Kp.
        SLInitEpisode(initState);
        [J_reward(i,1),trajState, flag] = SLSendController([gains_temp(1),gains(2)]);
        Jdelta_reward(i,1) = J_reward(i,1) - reward;
        % calculate the gradient for parameter Kd
        SLInitEpisode(initState);
        [J_reward(i,2),trajState, flag] = SLSendController([gains(1),gains_temp(2)]);
        Jdelta_reward(i,2) = J_reward(i,2) - reward;
    end
    %gFD(1) = sum(Jdelta_reward(:,1))/(Nsamples * delta_gains(1));
    %gFD(2) = sum(Jdelta_reward(:,2))/(Nsamples * delta_gains(2));
    gFDl= gFD;
    gFD(1) = pinv(Delta_gains(1:size(Jdelta_reward,1),1)) * Jdelta_reward(:,1);
    gFD(2) = pinv(Delta_gains(1:size(Jdelta_reward,1),2)) * Jdelta_reward(:,2);
    gains_ref = gains;
    gFD
    delta = sqrt(sum(gFDl-gFD).^2)
    if abs(gFD(1)) >= 1000
        a(1) = 0.00005;
    elseif abs(gFD(1)) >= 100
        a(1) = 0.0005;
    elseif abs(gFD(1)) >= 10
        a(1) = 0.005
    elseif abs(gFD(1)) >= 1
        a(1) = 0.05;
    end
    if abs(gFD(2)) >= 1000
        a(2) = 0.00005;
    elseif abs(gFD(2)) >= 100
        a(2) = 0.0005;
    elseif abs(gFD(2)) >= 10
        a(2) = 0.005
    elseif abs(gFD(2)) >= 1
        a(2) = 0.05;
    end
    gains = gains + a .* gFD;
    if (sqrt(gFD(1).^2+gFD(2).^2)<=5)&&(delta <=7)
        converged = true;
    end
    y(EpisodeCount) = gains(1);
    x(EpisodeCount) = gains(2);
    gKp(EpisodeCount) = gFD(1);
    gKd(EpisodeCount) = gFD(2);
    end
figure
plot(gKp);
hold on
plot(gKd,'r--');
title('gradients');
figure
plot(y);
hold on
plot(x,'ro');
title('gains');
%initState = [0.3, 0, 0]; % choose the initial state of your problem
%SLInitEpisode(initState); % initialize the episode
%[reward, trajState, flag] = SLSendController(gains); % send the controller and execute an episode
if flag == -1 % if something was wrong
SLResetEpisode(); % reset the episode
end

%% Plot ball and joints positions
close all;
[joints, jointsVel, jointsAcc, jointsDes, jointsVelDes, jointsAccDes, ...
torque, cart, episodeState, numCommand, stepIndex] = SLGetEpisode(); % get the details of the episode
figure;
plot(episodeState(2,:), 'LineWidth', 2)
xlabel('Time Steps');
ylabel('Ball Position');
figure;
plot(episodeState(3,:), 'LineWidth', 2)
xlabel('Time Steps');
ylabel('Ball Velocity');
figure;
plot(joints', 'LineWidth', 2)
xlabel('Time Steps');
ylabel('Joint Position');