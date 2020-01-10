function optimalSlips = calc_optimalSlips(velocity,of_coefficients)
% CALC_OPTIMALSLIP	Calculates the optimal slip for a given velocity.
    optimalSlips = polyval(of_coefficients.optimalSlipsCoefficients, velocity);
end