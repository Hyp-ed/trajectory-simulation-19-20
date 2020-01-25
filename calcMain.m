function [velocity, velocitySync, acceleration, distance, phase, frequency, power, powerLoss, powerInput, efficiency, slip, fx, drag, rollFriction] =   ...
    calcMain(parameters, state, i, velocity, velocitySync, acceleration, distance, phase, frequency, power, ...
              powerLoss, powerInput, efficiency, slip, fx, drag, rollFriction)
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
    
    % Calculate air drag
    drag(i) = calcDrag(velocity(i-1),parameters);
    
    % Calculate rolling friction
    rollFriction(i) = calcRollFriction(parameters);
    
    fprintf("%f\n",fx(i));
    % Account for drag and friction to force
    fx(i) = fx(i) - drag(i) - rollFriction(i);
    fprintf("%f after friction\n",fx(i));
    % Calculate acceleration
    acceleration(i) = fx(i) / parameters.mass;
    
    % Calculate velocity and distance
    velocity(i) = velocity(i - 1) + parameters.dt * acceleration(i);
    distance(i) = distance(i - 1) + parameters.dt * velocity(i);
    
    % Calculate power and efficiency
    power(i) = fx(i) * velocity(i); % Power output = force * velocity
    powerInput(i) = power(i) + powerLoss(i); % ignoring inertia
    efficiency(i) = power(i) / powerInput(i);

    % Calculate synchronous speed and slip
    velocitySync(i) = frequency(i) * parameters.limLength / parameters.polePairs;
    slip(i) = (velocitySync(i) - velocity(i)) / velocitySync(i);
    if (isinf(slip(i)))
        slip(i) = 0;
    end
end
