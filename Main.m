%% Pod Trajectory Simulation, HypED 2019/20
% This script calculates the trajectory of the pod inside the tube by
% calculating the force from the DSLIM propulsion module. Lookup tables
% generated in COMSOL Multiphysics are used to determine the thrust force
% and power loss for given frequency-velocity-combinations.
%
% NOTE ABOUT TIME STEP (dt):
% For quick estimates a time step of ~0.1s is sufficient. 
% For more accurate results use a time step of 0.05s or smaller.
% @author       Rafael Anderka, Lorenzo Principe

clear; clc;

%% Parameters
% Load parameters from './config.m'
parameters = loadParameters();

% Generate lookup tables and optimal slip coefficients from COMSOL input
forceLookupTable = generateForceLookupTable();

% Additional parameters
deceleration = 9.81;     % Total braking deceleration

%% Initialize arrays
%  Create all necessary arrays and initialize with 0s for each time step. 
%  This is computationally faster than extending the arrays after each calculation.
time = 0:parameters.dt:parameters.maxT;    % Create time array with time step dt and maximum time tmax
v = zeros(1,length(time));                  % Velocity of pod
a = zeros(1,length(time));                  % Acceleration of pod
distance = zeros(1,length(time));           % Distance travelled
phase = zeros(1,length(time));              % Current phase of Lim fields
frequency = zeros(1,length(time));          % Lim frequency
power = zeros(1,length(time));              % Power
powerLoss = zeros(1,length(time));          % Power loss
powerInput = zeros(1,length(time));         % Power input
efficiency = zeros(1,length(time));         % Power output / Power input
slip = zeros(1,length(time));               % Slip between LIM field and track
fxLIM = zeros(1,length(time));              % Thrust force from a single Halbach wheel
fx = zeros(1,length(time));                 % Net force in direction of track (x) for whole pod

totalStripeCount = floor(parameters.trackLength / parameters.stripeDistance);    % Total number of stripes we will detect
stripeIndices = zeros(1,totalStripeCount); % Indices at which we detect each stripe

%% Calculation loop
%  This is the main loop of the script, caluclating the relevant values for
%  each point in time. The function calc_main gets called at each iteration 
%  and handles the states of the trajectory internally by passing a "state"
%  variable as the first input argument.
%  state = 1 -- Acceleration
%  state = 2 -- Deceleration
%  state = 3 -- Max frequency

state = 1;          % We start in the acceleration state
stripeCount = 0;   % Initially we have counted 0 stripes

% For each point in time ...
for i = 2:length(time) % Start at i = 2 because values are all init at 1
    %% State transitions
    % If we have exceeded the max. RPM we cap the RPM and recalculate
    if (frequency(i-1) > parameters.maxFrequency)
        state = 3; % Max frequency
        
        % Recalculate previous time = i - 1 to avoid briefly surpassing max frequency
        [v,a,distance,phase,frequency,power,powerLoss,powerInput,efficiency,frequency,fxLim,fx] = ...
            calc_main(parameters,state,i-1,v,a,distance,phase,frequency,power,powerLoss,powerInput,efficiency,frequency,fxLim,fx,braking_force,ForceLookupTable);
    
    end
    
    % If we have reached the maximum allowed acceleration distance we 
    % transition to deceleration
    if (parameters.useMaxAccDistance)
        if distance(i-1) >= (parameters.maxAccDistance)
            state = 2; % Deceleration
        end
    else
        % Calculate braking distance using linear deceleration
        brakingDist = v(i-1)^2 / (2 * deceleration);
        if distance(i-1) >= (parameters.trackLength - brakingDist)
            state = 2; % Deceleration
        end
    end
    
    %% Main calculation
    % Calculate for current time = i
    [v,a,distance,phase,frequency,power,powerLoss,powerInput,efficiency,frequency,fxLim,fx] = ...
    calc_main(parameters,state,i,v,a,distance,phase,frequency,power,powerLoss,powerInput,efficiency,frequency,fxLim,fx,braking_force,ForceLookupTable);
    
    fprintf("Step: %i, %.2f s, %.2f m, %.2f m/s, %4.0f RPM, %.2f m/s, state: %i\n", i, time(i), distance(i), v(i), frequency(i) * 60 / (2 * pi), slips(i), state)
    
    %% Stripes
    if (distance(i) >= (1 + stripeCount) * parameters.stripeDistance)
        stripeIndices(1 + stripeCount) = i;
        stripeCount = stripeCount + 1;
    end
    
    %% Exit conditions
    % Stop when speed is 0m/s or time is up
    if v(i) <= 0 || i == length(time)
        % Truncate arrays and create final result structure 
        result = finalizeResults(i, time, distance, v, a, phase, frequency * 60 / (2 * pi), f_thrust_wheel, f_lat_wheel,...
                                 f_x_pod, f_y_pod, power, powerLoss, power_input, efficiency, slips);
        % Break from loop
        break;
    end
end

%% Print some results LP Change at the end
% Find max. speed and x force
v_max = max(result.velocity);
v_max_time = find(result.velocity == v_max) * dt - dt;
max_frequency = max(result.rpm);
f_x_max = max(result.pod_x);
f_x_min = min(result.pod_x);
% Let's display some stuff for quick viewing
fprintf('\n--------------------RESULTS--------------------\n');
fprintf('\nDuration of run: %.2f s\n', time(i));
fprintf('\nDistance: %.2f m\n', distance(i));
fprintf('\nMaximum speed: %.2f m/s at %.2f s\n', v_max(1), v_max_time(1));
fprintf('\nMaximum RPM: %5.0f\n', max_frequency);
fprintf('\nMaximum net thrust force per wheel: %.2f N\n', f_x_max/n_lim);
fprintf('\nMaximum net lateral force per wheel: %.2f N\n', max(f_y_pod)/n_lim);
fprintf('\nPower per motor: %.2f W\n', max(power_input)/n_lim);

%% Plot the trajectory graphs
plotTrajectory(result);