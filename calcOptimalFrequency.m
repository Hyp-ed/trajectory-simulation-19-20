function optimalFrequency = calcOptimalFrequency(velocity, parameters)
% CALCOPTIMALFREQUENCY  Calculates the optimal DSLIM frequency for a given velocity.
    optimalFrequency = polyval(parameters.forceLookupTable.optimalFrequencyCoefficients, velocity);
end