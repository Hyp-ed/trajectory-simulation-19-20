%% Pod Trajectory Simulation, HypED 2019/20
% This script calculates the trajectory of the pod inside the tube by
% calculating the force from the DSLIM propulsion module. Lookup tables
% generated in COMSOL Multiphysics are used to determine the thrust force
% and power loss for given frequency-velocity-combinations.
%
% NOTE ABOUT TIME STEP (dt):
% For quick estimates acceleration time step of ~0.1s is sufficient. 
% For more accurate results use acceleration time step of 0.05s or smaller.
% @author       Rafael Anderka, Lorenzo Principe

clear; clc;

%% Parameters
% Load parameters from './config.m'
parameters = loadParameters();

% Generate lookup tables and optimal slip coefficients from COMSOL input
parameters.forceLookupTable = generateForceLookupTable();

% Additional parameters
parameters.brakingForce = parameters.mass * parameters.deceleration;  % Total braking force

%% Initialize arrays
%  Create all necessary arrays and initialize with 0s for each time step. 
%  This is computationally faster than extending the arrays after each calculation.
time = 0:parameters.dt:parameters.maxT;     % Create time array with time step dt and maximum time tmax
velocity = zeros(1,length(time));           % Velocity of pod
acceleration = zeros(1,length(time));       % Acceleration of pod
distance = zeros(1,length(time));           % Distance travelled
phase = zeros(1,length(time));              % Current phase of LIM fields
frequency = zeros(1,length(time));          % LIM frequency
power = zeros(1,length(time));              % Power
powerLoss = zeros(1,length(time));          % Power loss
powerInput = zeros(1,length(time));         % Power input
efficiency = zeros(1,length(time));         % Power output / Power input
slip = zeros(1,length(time));               % Slip between LIM field and track
fx = zeros(1,length(time));                 % Net force in direction of track (x) for whole pod

totalStripeCount = floor(parameters.trackLength / parameters.stripeDistance);    % Total number of stripes we will detect
stripeIndices = zeros(1,totalStripeCount);  % Indices at which we detect each stripe

%% Calculation loop
%  This is the main loop of the script, caluclating the relevant values for
%  each point in time. The function calc_main gets called at each iteration 
%  and handles the states of the trajectory internally by passing acceleration "state"
%  variable as the first input argument.
%  state = 1 -- Acceleration
%  state = 2 -- Deceleration
%  state = 3 -- Max frequency

state = 1;         % We start in the acceleration state
stripeCount = 0;   % Initially we have counted 0 stripes

% For each point in time ...
for i = 2:length(time) % Start at i = 2 because values are all init at 1
    %% State transitions
    % If we have exceeded the max. frequency we cap the frequency and recalculate
    if (frequency(i-1) > parameters.maxFrequency)
        state = 3; % Max frequency
        
        % Recalculate previous time = i - 1 to avoid briefly surpassing max frequency
        [velocity, acceleration, distance, phase, frequency, power, powerLoss, powerInput, efficiency, slip, fx] = ...
            calcMain(parameters, state, i, velocity, acceleration, distance, phase, frequency, power, powerLoss, ...
                     powerInput, efficiency, slip, fx);
    end
    
    % If we have reached the maximum allowed acceleration distance we 
    % transition to deceleration
    if (parameters.useMaxAccDistance)
        if distance(i-1) >= (parameters.maxAccDistance)
            state = 2; % Deceleration
        end
    else
        % Calculate braking distance using linear deceleration
        brakingDist = velocity(i-1)^2 / (2 * parameters.deceleration);
        if distance(i-1) >= (parameters.trackLength - brakingDist)
            state = 2; % Deceleration
        end
    end
    
    %% Main calculation
    % Calculate for current time = i
    [velocity, acceleration, distance, phase, frequency, power, powerLoss, powerInput, efficiency, slip, fx] = ...
        calcMain(parameters, state, i, velocity, acceleration, distance, phase, frequency, power, powerLoss, ...
                 powerInput, efficiency, slip, fx);
    
    fprintf("Step: %i, %.2f s, %.2f m, %.2f m/s, %4.0f Hz, %.2f m/s, state: %i\n", i, time(i), distance(i), velocity(i), frequency(i), slip(i), state)
    
    %% Stripes
    if (distance(i) >= (1 + stripeCount) * parameters.stripeDistance)
        stripeIndices(1 + stripeCount) = i;
        stripeCount = stripeCount + 1;
    end
    
    %% Exit conditions
    % Stop when speed is 0m/s or time is up
    if velocity(i) <= 0 || i == length(time)
        % Truncate arrays and create final results structure 
        results = finalizeResults(i, time, distance, velocity, acceleration, phase, frequency, ...
                                 fx, power, powerLoss, powerInput, efficiency, slip);
        % Break from loop
        break;
    end
end

%% Print some results LP Change at the end
% Find max. speed and x force
vMax = max(results.velocity);
vMaxTime = find(results.velocity == vMax) * parameters.dt - parameters.dt;
freqMax = max(results.frequency);
freqMaxTime = find(results.frequency == freqMax) * parameters.dt - parameters.dt;
fxMax = max(results.fx);
fxMin = min(results.fx);
% Let's display some stuff for quick viewing
fprintf('\n--------------------RESULTS--------------------\n');
fprintf('\nDuration of run: %.2f s\n', time(i));
fprintf('\nDistance: %.2f m\n', distance(i));
fprintf('\nTop speed: %.2f m/s i.e. %.2f km/h i.e. %.2f mph at %.2f s\n', vMax(1), vMax(1) * 3.6, vMax(1) * 2.23694, vMaxTime(1));
fprintf('\nMaximum frequency: %3.0f Hz at %.2f s\n', freqMax, freqMaxTime);

%% Plot the trajectory graphs
plotTrajectory(results);