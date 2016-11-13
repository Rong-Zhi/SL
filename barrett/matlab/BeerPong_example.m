clear all;

addpath('../../build/barrett/')
addpath('../../IAS/matlab/')

initState = [0, 0];
[initState, flag] = SLInitEpisode(initState);

[N_DOFS, N_DOFS_SHM] = SLGetInfoMex;

% Load the trajectory to imitate
h = load('BeerPong_exampleTrajectory.mat');
traj = h.joints;

[reward, trajState, flag] = SLSendTrajectory(traj);

if flag == -1 % if something was wrong
    SLResetEpisode(); % reset the episode
end


%% Plot ball positions
close all;

[joints, jointsVel, jointsAcc, jointsDes, jointsVelDes, jointsAccDes, ...
	torque, cart, episodeState, numCommand, stepIndex] = SLGetEpisode(); % get the details of the episode

figure; 
plot(episodeState(1:3,:)', 'LineWidth', 2)
xlabel('Time Steps');
ylabel('Ball Position (in task space)');

figure; 
plot(episodeState(4:6,:)', 'LineWidth', 2)
xlabel('Time Steps');
ylabel('Ball Velocity (in task space)');

figure; 
plot(episodeState(7:9,:)', 'LineWidth', 2)
xlabel('Time Steps');
ylabel('Cup Position (in task space)');
