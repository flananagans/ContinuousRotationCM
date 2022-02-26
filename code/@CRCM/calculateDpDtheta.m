function dP = calculateDpDtheta(obj, phi)
% Function to calculate the partial derivative of the potential cost
% function P with respect to each theta. The central difference
% approximation is used.

    % Estimate dP/dtheta (derivative of potential function wrt each
    % joint angle using central difference
    dP = zeros(size(obj.theta));
    del_theta = 0.01; % Difference in theta used to estimate derivative
    cp = obj.copy(); % create copy of object
    cp.toPlot = false;
    for ind = 1:length(obj.theta)
        % theta + del_theta
        theta_up = obj.theta;
        theta_up(ind) = theta_up(ind) + del_theta;
        
        cp.updateTheta(theta_up);
        P_up = calculatePotentialFunc(cp, phi);

        % theta - del_theta
        theta_down = obj.theta;
        theta_down(ind) = theta_down(ind) - del_theta;
        cp.updateTheta(theta_down);
        P_down = calculatePotentialFunc(cp, phi);

        dP(ind) = (P_up - P_down)/(2*del_theta);
    end
end

function P = calculatePotentialFunc(obj, phi)
% Helper function to calculate a potential-like function that is used to
% move the chain onto the backbone curve. This calculates the backbone
% curve for this phi, then gets the minimum distance between the curve and
% the ith joint

    %Generate a ton of points along the backbone curve
    s = linspace(0, 1, 250);
    r = zeros(3, length(s)); % [x, y, z]' points along the backbone
    for sInd = 1:length(s)
        [r(:, sInd), R, ~, ~, T, N, B] = obj.backboneFunc(phi,s(sInd),0.7);
    end

    % Get origins of each link
    origs = cellfun(@(x) x.config, obj.link, 'UniformOutput', false);
    origs = cellfun(@(x) x(1:3, end), origs, 'UniformOutput', false);

    P = 0;
    for ind = 1:size(origs, 2)
        % Get minimum euclidean distance from this link origin to backbone
        d = min(vecnorm(r - origs{ind}, 2, 1));

        % Our function is the sum of the squared minimum distances
        P = P + d^2;
    end
end