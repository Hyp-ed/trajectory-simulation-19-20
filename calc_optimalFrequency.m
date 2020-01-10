function optimalFrequency = calc_optimalFrequency(velocity,of_coefficients)
% CALC_OPTIMALFREQUENCY	Calculates the optimal frequency for a given velocity.
    optimalFrequency = polyval(of_coefficients, velocity);
end