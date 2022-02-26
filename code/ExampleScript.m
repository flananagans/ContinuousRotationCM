%% Example Script
clear
close all
clc

%% Create chain object with default parameters
c = CRCM();
c.toPlot = false; % whether or not to plot the chain

% store the chain parameters for later
N = c.N;
C = c.C;
alpha = c.alpha;
k = c.k;

%% Initialize chain joint angles
% We can't initialize them to zero because this makes the Jacobian singular
%   so we use an initial condition
theta_init = [0.55, 0, 0.54, 0, -0.51, 0, -1, 0, -1, 0, -1, 0, -0.54]';
theta = zeros(c.N, 1);
if(length(theta_init) > c.N)
   theta = theta_init(1:c.N);
else
   theta(1:length(theta_init)) = theta_init; 
end
c.updateTheta(theta);

%% Generate a trajectory for the desired rotor rotation angle
phis = linspace(0, 2*pi, 100);
[btvec, strain_energy] = c.generateTrajectory(phis);

%% Save the trajectory to a file
% This file can now be loaded into 'PlaybackFullChain.m' to be visualized
save(sprintf('generatedTraj_strainenergy%d.mat', N), ...
            'phis', 'btvec', 'strain_energy', 'N', 'C', 'alpha', 'k');



