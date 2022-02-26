%% Sweep Example
clear
close all
clc

Ns = [9, 13, 15];
Cs = [1.7, 0.7*33/(4*pi), 1.9];
alphas = (pi/180) * [80, 90, 100]; 
k = 15;

allparams = combvec(Ns, Cs, alphas, k);

% Parallelize the sweep using parfor
parfor iter = 1:length(allparams(1, :))
    N = allparams(1, iter);
    C = allparams(2, iter);
    alpha = allparams(3, iter);
    k = allparams(4, iter);
    
    %% Create chain object with desired parameters
    c = CRCM(N, C, alpha, k);
    c.toPlot = false;

    %% Initialize chain joint angles
    % We can't initialize them to zero because this makes the Jacobian singular
    %   so we use an initial condition
    theta_init = [-0.71, 0, -0.13, 0, 0.42, 1, 0, 1.32, 0, 0.9, 0, 0.3]';
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
    parsave(sprintf('generatedTraj_sweep%d.mat', iter), ...
                phis, btvec, strain_energy, N, C, alpha, k);
end

% Function allows for saving within a parallelized for loop
function parsave(fname, phis, btvec, strain_energy, N, C, alpha, k)
  save(fname, 'phis', 'btvec', 'strain_energy', 'N', 'C', 'alpha', 'k');
end

