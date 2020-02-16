function state = calcMain(parameters, state, i)
% CALCMAIN  Calculates trajectory values at each point in time.
% calcMain gets called at each iteration and handles the states of the 
% trajectory via a passed state input argument.
% state.mode = 1 -- Acceleration
% state.mode = 2 -- Deceleration
% state.mode = 3 -- Max Frequency
% @author      Rafael Anderka, Lorenzo Principe

    % Calculate state-specific variables
    switch state.mode
        case 1 % Acceleration
            % Calculations assume optimal frequency
            state.frequency(i)      = calcOptimalFrequency(state.velocity(i - 1), parameters);
            state.coilsPowerLoss(i) = calcCoilsPowerLoss(state.current(i - 1), state.resistance(i - 1), state.frequency(i), parameters);
        case 2 % Deceleration using EmBrakes            
            state.frequency(i)      = 0;
            state.coilsPowerLoss(i) = 0;
        case 3 % Max frequency
            state.frequency(i)      = parameters.maxFrequency;
            state.coilsPowerLoss(i) = calcCoilsPowerLoss(state.current(i - 1), state.resistance(i - 1), state.frequency(i), parameters);
    end

    % Calculate temperature and coils resistance
    [state.coilsTemp(i) state.coreTemp(i)] = calcTemperature(parameters, state, i);
    state.resistance(i)     = calcResistance(parameters, state.coilsTemp(i));
    
    % More state-specific variables
    if (state.mode == 1 || state.mode == 3)
        state.voltage(i)        = parameters.maxU;
        state.current(i)        = state.voltage(i) / state.resistance(i);
        if (state.current(i) > parameters.maxI)
            state.current(i) = parameters.maxI;
            state.voltage(i) = state.current(i) * state.resistance(i);
        end
        state.DSLIMForce(i)  = calcFx(state.frequency(i), state.velocity(i - 1), parameters);
        state.brakesForce(i) = 0;
    elseif (state.mode == 2)
        state.voltage(i) = 0;
        state.current(i) = 0;
        state.DSLIMForce(i)  = 0;
        state.brakesForce(i) = calcBrakesForce(state.velocity(i-1), parameters);  
    end

    % Calculate air drag
    state.drag(i) = calcDrag(state.velocity(i-1), parameters);
    
    % Calculate rolling friction
    state.rollFriction(i) = calcRollFriction(parameters);
    
    % Calculate net force
    state.fx(i) = state.DSLIMForce(i) + state.brakesForce(i) + state.drag(i) + state.rollFriction(i);

    % Calculate acceleration
    state.acceleration(i) = state.fx(i) / parameters.mass;
    
    % Calculate velocity, phase and distance
    state.velocity(i) = state.velocity(i - 1) + parameters.dt * state.acceleration(i);
    state.phase(i)    = state.phase(i - 1) + 2 * pi * state.frequency(i) * parameters.dt;
    state.distance(i) = state.distance(i - 1) + parameters.dt * state.velocity(i);
    
    % Calculate power and efficiency
    state.power(i) = state.fx(i) * state.velocity(i); % Power output = force * velocity
    state.powerInput(i) = state.power(i) + state.powerLoss(i); % ignoring inertia
    state.efficiency(i) = state.power(i) / state.powerInput(i);

    % Calculate synchronous speed and slip
    state.velocitySync(i) = state.frequency(i) * parameters.limLength / parameters.polePairs;
    state.slip(i) = (state.velocitySync(i) - state.velocity(i)) / state.velocitySync(i);
    if (isinf(state.slip(i)))
        state.slip(i) = 0;
    end
end
