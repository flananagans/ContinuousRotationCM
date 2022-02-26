function [traj, strain_energy] = generateTrajectory(obj, phis)
%% Generate the joint trajectory of the chain using the "automatic" method 
% described in the paper given the desired rotor trajectory angles given in
% 'phis'
%
%   Also returns the strain energy calculated for each phi

    %% Compute the configuration for each angle phi
    traj = zeros(length(obj.link), length(phis)); % Theta trajectory over phi
    strain_energy = zeros(1, length(phis));
    for phiInd = 1:length(phis)
        phi = phis(phiInd);
        fprintf('Phi = %0.2f\n', phi);

        % Desired configuration frame transformation of the rotor for this phi
        g_des = [-cos(2*phi), -sin(2*phi),  0,   0;
                 -sin(2*phi),  cos(2*phi),  0,   0;
                           0,           0, -1, 2*obj.C;
                           0,           0,  0,   1];

        %% Step 1 - Move chain so current rotor configuration is close to desired
        obj.moveChainToRotor(g_des);

        obj.camSet = true; % stop resetting camera for every plot
        
        %% Step 2 - Relax the chain to minimize joint angles
        % Rotor is held fixed
        obj.relaxChain();

        %% Step 3 - Move the chain as close as possible to the backbone curve
        % Rotor is held fixed
        kg = 1; % virtual spring stiffness between chain links and backbone
        %obj.moveChainToBackbone(phi, kg);

        traj(:, phiInd) = obj.theta; % Save the joint thetas for this phi
        strain_energy(phiInd) = obj.strain_energy;
    end
end