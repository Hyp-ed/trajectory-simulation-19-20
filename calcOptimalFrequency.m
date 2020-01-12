function optimalFrequency = calcOptimalFrequency(velocity,of_coefficients)
% CALCOPTIMALFREQUENCY  Calculates the optimal frequency for a given velocity.
    optimalFrequency = polyval(of_coefficients, velocity);
end