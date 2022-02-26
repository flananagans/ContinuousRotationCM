function moveChainToRotor(obj, g_des)
% This function is Step 1 described in the paper
%
%   Given a desired rotor frame, g_des, this iteratively moves the chain
%   such that the current rotor frame, g_rot, is very close to desired.
%
%   This works by creating a desired spatial velocity of the rotor, V_rot, 
%   based on the difference between desired and current frame, converting
%   this to joint velocities using the inverse Jacobian, then integrating
%   the joint velocity for time step dt
    
    % Iterate until joint angles converge
    d_theta = ones(size(obj.theta));
    while(sum(abs(d_theta)) > 1e-3)

        % Calculate desired spatial velocity of rotor using the difference
        % in frames
        V_rot = CRCM.wedge(CRCM.log_SE3(g_des / obj.rotor_frame));

        % Convert this to joint velocities using the inverse Jacobian
        %   psuedoinverse because Jacobian is not square
        d_theta = obj.J'/(obj.J*obj.J') * V_rot;

        % Integrate d_theta to get new angles
        theta = obj.theta + d_theta*obj.dt;
        % get the new configuration (updates J, rotor_frame, link)
        obj.updateTheta(theta);
    end
end