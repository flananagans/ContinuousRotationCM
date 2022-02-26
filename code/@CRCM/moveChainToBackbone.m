function moveChainToBackbone(obj, phi, kg)
% This function is Step 3 described in the paper
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

    % Iterate until joint angles converge
    d_theta = ones(size(obj.theta));
    while(sum(abs(d_theta)) > 1e-3)

        % Calculate the partial derivative of the potential function wrt
        % each joint angle
        dP = obj.calculateDpDtheta(phi);

        % Update 'joint stress'
        d_theta_desired = -kg*dP;

        % During this step we want to keep the rotor fixed, so our only
        % allowable movements can come from the null space of the Jacobian,
        % because these are joint movements along axes that do not change
        % the output position or orientation of the rotor
        A = null(obj.J); 
        d_theta = A*A'*d_theta_desired;

        % Integrate d_theta to get new angles
        theta = obj.theta + d_theta*obj.dt;

        % get the new configuration (updates J, rotor_frame, link)
        obj.updateTheta(theta);
    end
end