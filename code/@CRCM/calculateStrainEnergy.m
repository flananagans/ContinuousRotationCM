function strain_energy = calculateStrainEnergy(obj, theta)
% calculateStrainEnergy
%   Calculates the strain energy within the chain for joint angles theta

    strain_energy = sum(0.5*(obj.k*(theta.^2)));
end