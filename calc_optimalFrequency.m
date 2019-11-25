function optimalFrequency = calc_optimalFrequency(velocity,of_coefficients)
% CALC_OPTIMALSLIP	Calculates the optimal slip for a given velocity.
    optimalFrequency = polyval(of_coefficients.optimalFrequencyCoefficients, velocity);
end