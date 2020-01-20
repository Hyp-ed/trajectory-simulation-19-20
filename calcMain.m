function [velocity, acceleration, distance, phase, frequency, power, powerLoss, powerInput, efficiency, slip, fx] =   ...
    calcMain(parameters, state, i, velocity, acceleration, distance, phase, frequency, power, ...
              powerLoss, powerInput, efficiency, slip, fx)
% CALCMAIN  Calculates trajectory values at each point in time.
% calcMain gets called at each iteration and handles the states of the 
% trajectory via a passed state input argument.
% state = 1 -- Acceleration
% state = 2 -- Deceleration
% state = 3 -- Max Frequency
% @author      Rafael Anderka, Lorenzo Principe

    % Calculate state-specific variables
    switch state
        case 1 % Acceleration
            % Calculations assume optimal frequency
            frequency(i)    = calcOptimalFrequency(velocity(i - 1), parameters);
            fx(i)           = calcFx(frequency(i), velocity(i - 1), parameters);
            phase(i)        = phase(i - 1) + 2 * pi * frequency(i) * parameters.dt;
            powerLoss(i)    = calcPl(frequency(i), velocity(i - 1), parameters);
        case 2 % Deceleration using EmBrakes            
            frequency(i)    = 0;
            fx(i)           = calcBrakingForce(velocity(i-1),parameters);  
            phase(i)        = phase(i - 1);
            powerLoss(i)    = 0;
        case 3 % Max RPM
            frequency(i)    = parameters.maxFrequency;
            fx(i)           = calcFx(frequency(i), velocity(i - 1), parameters);
            phase(i)        = phase(i - 1) + 2 * pi * frequency(i) * parameters.dt;
            powerLoss(i)    = calcPl(frequency(i), velocity(i - 1), parameters);
    end

    % Calculate acceleration
    acceleration(i) = fx(i) / parameters.mass;
    
    % Calculate velocity and distance
    velocity(i) = velocity(i - 1) + parameters.dt * acceleration(i);
    distance(i) = distance(i - 1) + parameters.dt * velocity(i);
    
    % Calculate power and efficiency
    power(i) = fx(i) * velocity(i); % Power output = force * velocity
    powerInput(i) = power(i) + powerLoss(i); % ignoring inertia
    efficiency(i) = power(i) / powerInput(i);
end
