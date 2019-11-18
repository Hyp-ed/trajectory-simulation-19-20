function optimalSlip = calc_optimalSlip(velocity,of_coefficients)
% CALC_OPTIMALSLIP	Calculates the optimal slip for a given velocity.
    optimalSlip = polyval(of_coefficients.optimalSlipCoefficients, velocity);
end