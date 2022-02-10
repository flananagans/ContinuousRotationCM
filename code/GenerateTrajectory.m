%% Generate the joint trajectory of the chain using the "automatic" method 
% described in the paper

clear
close all
clc

%% TODO
% Energy plots
% Sweep number of links
% Height of links
% Angle of links

toPlot = true; % Do you want to plot the kinematic chain
plotObjs = {};

% Anle phi that we rotate the chain through (this would be angle of the
% crank in our mechanism
phis = linspace(0, 2*pi, 100);

% This script loads the files and sets up the kinematics
% This gives us:
%   g_s - the initial frames for each link (all joint angles theta = 0)
%   w - the axis of rotation for each link
%   q - a point on the axis of rotation for each link
crcdLoadT2;

%% Initialize chain joint angles  
% load('optimizedbtvec.mat')
% theta = btvec(:,1);

% We can't initialize them to zero because this makes the Jacobian singular
%   so we set each joint theta to a small nonzero number
rng('default') %reset random number generator so it's the same every time
theta = rand(13, 1)*0.2 - 0.1;

%% Get the initial configuration -- produces the Jacobian (J) and link objects
% with the frame in link{i}.config
% Run forward kinematics to get joint frames for each link
[J, link] = crcdUpdateKin_v2(g_s, w, q, theta, link);

% Draw it
if(toPlot)
    crcdDraw_v2;
    crcdSetCam;
    zlim([0, 15]);
    crcdDelete_v2;
end

%% Compute the configuration for each angle phi
btvec = zeros(NJ, length(phis)); % Theta trajectory over phi
for phiInd = 1:length(phis)
    phi = phis(phiInd)
    
    % Desired configuration of the rotor for this phi
    g_des = [-cos(2*phi), -sin(2*phi),  0,   0;
             -sin(2*phi),  cos(2*phi),  0,   0;
                       0,           0, -1, 2*C;
                       0,           0,  0,   1];
    
                   
    %% Step 1 - Move chain so current rotor configuration is close to desired
    dt = 0.05; %time step for integration
    if(toPlot)
        plotObjs = struct('hanbase', hanbase, 'han', {han}, 'linkbase', linkbase);
    end
    [theta, link, J] = MoveChainToRotor(g_des, g_s, w, q, theta, link, J, dt, ...
                                            toPlot, plotObjs);
    
    %% Step 2 - Relax the chain to minimize joint angles
    % Rotor is held fixed
    kr = 10; % representative spring constant for each joint
    dt = 0.05; %time step for integration
    if(toPlot)
        plotObjs = struct('hanbase', hanbase, 'han', {han}, 'linkbase', linkbase);
    end
    [theta, link, J] = RelaxChain(g_s, w, q, theta, link, J, kr, dt, ...
                                            toPlot, plotObjs);
    
    %% Step 3 - Move the chain as close as possible to the backbone curve
    % Rotor is held fixed
    kg = 15; % representative spring constant
    dt = 0.05; %time step for integration
    if(toPlot)
        plotObjs = struct('hanbase', hanbase, 'han', {han}, 'linkbase', linkbase);
    end
%     [theta, link, J] = MoveChainToBackbone(phi, g_s, w, q, theta, link, J, kg, dt, C, ...
%                                             toPlot, plotObjs);
    
    btvec(:, phiInd) = theta; % Save the joint thetas for this phi
end

% Save the trajectory to a file
save('generatedTraj_strainenergy.mat', 'btvec');

function [theta, link, J] = MoveChainToRotor(g_des, g_s, w, q, theta, link, J, dt, toPlot, plotObjs)
% This function is Step 1 described in the paper
%
%   Given a desired rotor frame, g_des, this iteratively moves the chain
%   such that the current rotor frame, g_rot, is very close to desired
%
%   This works by creating a desired spatial velocity of the rotor, V_rot, 
%   based on the difference between desired and current frame, converting
%   this to joint velocities using the inverse Jacobian, then integrating
%   the joint velocity for time step dt
    
    if(toPlot)
       hanbase = plotObjs.hanbase;
       han = [plotObjs.han(:)];
       linkbase = plotObjs.linkbase;
    end

    % Get current config of rotor
    g_rot = link{end}.config;
    while(sum(sum(abs(g_des - g_rot))) > 1e-3)
    
        if(toPlot)
            crcdDelete_v2; %han, hanbase used here
        end
        
        % Calculate desired spatial velocity of rotor using the difference
        % in frames
        V_rot = wedge( log_SE3(g_des / g_rot) );
        
        % Convert this to joint velocities using the inverse Jacobian
        %   psuedoinverse because Jacobian is not square
        d_theta = J'/(J*J') * V_rot;
        
        % Integrate d_theta to get new angles
        theta = theta + d_theta*dt;
        
        % get the new configuration
        [J, link] = crcdUpdateKin_v2(g_s, w, q, theta, link);  
        % Get current config of rotor
        g_rot = link{end}.config;
        
        if(toPlot)
            crcdDraw_v2; % linkbase used here
            drawnow;
        end
    end
    
    if(toPlot)
        crcdDelete_v2; %han, hanbase used here
    end
end

function [theta, link, J] = RelaxChain(g_s, w, q, theta, link, J, kr, dt, ...
                                            toPlot, plotObjs)
% This function is Step 2 described in the paper
%
%   Given the current configuration of the chain, 'relax' the chain by
%   moving each joint in the direction that reduces it's joint torque. If
%   we want to relax the chain by other metrics (such as joint strain
%   energy), we can just change the 'd_theta_desired' equation.
%
%   The rotor frame is fixed in this step because this desired joint
%   velocity vector is projected onto the null space of the Jacobian. In
%   other words, the joints are only allowed to move in directions that
%   will produce zero velocity (linear and angular) of the rotor. This may
%   be a good assumption for the real workings of our mechanism because the
%   rotor frame is fixed in space by the position of the crank
 
    if(toPlot)
       hanbase = plotObjs.hanbase;
       han = [plotObjs.han(:)];
       linkbase = plotObjs.linkbase;
    end
    
    theta_prev = zeros(size(theta));
    % Iterate until joint angles converge
    while(sum(sum(abs(theta_prev - theta))) > 1e-3)
        theta_prev = theta;
                
        if(toPlot)
            crcdDelete_v2; %han, hanbase used here
        end
        
        % Calculated desired joint velocities that will reduce joint torque
        % Currently all joints have same stiffness but this can be changed
        % to a dot product if we want to specify a vector of stiffness
        % values for each joint
       
        d_theta_desired = -kr*theta; % joint torque
        %d_theta_desired = -0.5*kr*sign(theta).*(theta.^2); % strain energy
        
        % During this step we want to keep the rotor fixed, so our only
        % allowable movements can come from the null space of the Jacobian,
        % because these are joint movements along axes that do not change
        % the output position or orientation of the rotor
        A = null(J); 
        d_theta = A*A'*d_theta_desired;
        
        % Integrate d_theta to get new angles
        theta = theta + d_theta*dt;
        
        % get the new configuration
        [J, link] = crcdUpdateKin_v2(g_s, w, q, theta, link);
        
        if(toPlot)
            crcdDraw_v2; % linkbase used here
            drawnow;
        end
    end
         
    if(toPlot)
        crcdDelete_v2; %han, hanbase used here
    end
end

function [theta, link, J] = MoveChainToBackbone(phi, g_s, w, q, theta, link, J, kg, dt, C, ...
                                                    toPlot, plotObjs)
% This function is Step 2 described in the paper
%
%   Given the current configuration of the chain, 'relax' the chain by
%   moving each joint in the direction that reduces it's joint torque. If
%   we want to relax the chain by other metrics (such as joint strain
%   energy), we can just change the 'd_theta_desired' equation.
%
%   The rotor frame is fixed in this step because this desired joint
%   velocity vector is projected onto the null space of the Jacobian. In
%   other words, the joints are only allowed to move in directions that
%   will produce zero velocity (linear and angular) of the rotor. This may
%   be a good assumption for the real workings of our mechanism because the
%   rotor frame is fixed in space by the position of the crank

    if(toPlot)
       hanbase = plotObjs.hanbase;
       han = [plotObjs.han(:)];
       linkbase = plotObjs.linkbase;
    end
    
    theta_prev = zeros(size(theta));
    % Iterate until joint angles converge
    while(sum(sum(abs(theta_prev - theta))) > 1e-3)
        theta_prev = theta;
                
        if(toPlot)
            crcdDelete_v2; %han, hanbase used here
        end
        
        % Calculate the partial derivative of the potential function wrt
        % each joint angle
        dP = CalculateDpDtheta(phi, g_s, w, q, theta, link, C);
        
        % Update 'joint stress'
        d_theta = -kg*dP;
        
        % During this step we want to keep the rotor fixed, so our only
        % allowable movements can come from the null space of the Jacobian,
        % because these are joint movements along axes that do not change
        % the output position or orientation of the rotor
        A = null(J); 
        d_theta = A*A'*d_theta;
        
        % Integrate d_theta to get new angles
        theta = theta + d_theta*dt;
        
        % get the new configuration
        [J, link] = crcdUpdateKin_v2(g_s, w, q, theta, link);
        
        if(toPlot)
            crcdDraw_v2; % linkbase used here
            drawnow;
        end 
    end
    
    if(toPlot)
        crcdDelete_v2; %han, hanbase used here
    end
end
    
function dP = CalculateDpDtheta(phi, g_s, w, q, theta, link, C)
% Function to calculate the partial derivative of the potential cost
% function P with respect to each theta. The central difference
% approximation is used

    % Estimate dP/dtheta (derivative of potential function wrt each
    % joint angle using central difference
    dP = zeros(size(theta));
    del_theta = 0.01; % Difference in theta used to estimate derivative
    for ind = 1:length(theta)
        % theta + del_theta
        theta_up = theta;
        theta_up(ind) = theta_up(ind) + del_theta;

        [~, link_up] = crcdUpdateKin_v2(g_s, w, q, theta_up, link);
        P_up = CalculatePotentialFunc(phi, link_up, C);

        % theta - del_theta
        theta_down = theta;
        theta_down(ind) = theta_down(ind) - del_theta;

        [~, link_down] = crcdUpdateKin_v2(g_s, w, q, theta_down, link);
        P_down = CalculatePotentialFunc(phi, link_down, C);

        dP(ind) = (P_up - P_down)/2*del_theta;
    end
end

function P = CalculatePotentialFunc(phi, link, C)
% Helper function to calculate a potential-like function that is used to
% move the chain onto the backbone curve. This calculates the backbone
% curve for this phi, then gets the minimum distance between the curve and
% the ith joint

    %Generate a ton of points along the backbone curve
    s = linspace(0, 1, 250);
    r = zeros(3, length(s)); % [x, y, z]' points along the backbone
    for sInd = 1:length(s)
        [r(:, sInd), R, ~, ~, T, N, B] = crcdBackFunc(phi,s(sInd),0.7,C);
    end
    
    % Get origins of each link
    origs = cellfun(@(x) x.config, link, 'UniformOutput', false);
    origs = cellfun(@(x) x(1:3, end), origs, 'UniformOutput', false);
    
    P = 0;
    for ind = 1:size(origs, 2)
        % Get minimum euclidean distance from this link origin to backbone
        d = min(vecnorm(r - origs{ind}, 2, 1));
        
        % Our function is the sum of the squared minimum distances
        P = P + d^2;
    end
end