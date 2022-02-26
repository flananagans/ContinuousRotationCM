function dE = calculateDeDtheta(obj)
% Function to calculate the partial derivative of the total strain energy
% with respect to each joint angle theta.
% The central difference approximation is used

    % Estimate dE/dtheta (derivative of total strain energy wrt each
    % joint angle using central difference
    dE = zeros(size(obj.theta));
    del_theta = 0.01; % Difference in theta used to estimate derivative
    for ind = 1:length(obj.theta)

        % theta + del_theta
        theta_up = obj.theta;
        theta_up(ind) = theta_up(ind) + del_theta;
        E_up = obj.calculateStrainEnergy(theta_up);

        % theta - del_theta
        theta_down = obj.theta;
        theta_down(ind) = theta_down(ind) - del_theta;
        E_down = obj.calculateStrainEnergy(theta_down);

        dE(ind) = (E_up - E_down)/(2*del_theta);
    end
end
