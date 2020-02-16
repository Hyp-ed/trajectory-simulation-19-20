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

clear; clc; close all;
verbose = true;
displayPlots = true;

%% Check for temporary folders
if ~exist('lookupTables/temp','dir')
    mkdir('lookupTables/temp');
end

if ~exist('output','dir')
    mkdir('output');
end

%% Parameters
% Load parameters from './config.m'
parameters = loadParameters();

% Generate lookup tables and optimal slip coefficients from COMSOL input
parameters.forceLookupTable = generateDSLIMLookupTable(displayPlots);

% Generate lookup table for magnetic braking force
brakeTable = xlsread('lookupTables/MAGBRAKES_20-02-04.xlsx');
brakeTable = brakeTable(:,1:2);

% Set up brake force and brake distance interpolants
parameters.brakeForceInterpolant = griddedInterpolant(brakeTable(:,1), brakeTable(:,2));
parameters.brakingDistInterpolant = getBrakingDistInterpolant(parameters, brakeTable);

% Additional parameters
parameters.mechBrakeForce = parameters.mechBrakeFCoef * parameters.mechBrakeNForce;  % Total braking force
parameters.totalStripeCount = floor(parameters.trackLength / parameters.stripeDistance);    % Total number of stripes we will detect

%% Initialize arrays
%  Create all necessary arrays and initialize with 0s for each state.time step. 
%  This is computationally faster than extending the arrays after each calculation.
state.time = 0:parameters.dt:parameters.maxT;           % Create state.time array with state.time step dt and maximum state.time tmax
state.velocity = zeros(1,length(state.time));           % Velocity of pod
state.velocitySync = zeros(1,length(state.time));       % Synchronous velocity of stator field
state.acceleration = zeros(1,length(state.time));       % Acceleration of pod
state.distance = zeros(1,length(state.time));           % Distance travelled
state.phase = zeros(1,length(state.time));              % Current phase of LIM fields
state.frequency = zeros(1,length(state.time));          % LIM frequency
state.power = zeros(1,length(state.time));              % Power
state.powerLoss = zeros(1,length(state.time));          % Power loss
state.powerInput = zeros(1,length(state.time));         % Power input
state.efficiency = zeros(1,length(state.time));         % Power output / Power input
state.slip = zeros(1,length(state.time));               % Slip between LIM field and track
state.fx = zeros(1,length(state.time));                 % Net force in direction of track (x) for whole pod
state.drag = zeros(1,length(state.time));               % Air drag force on pod
state.rollFriction = zeros(1,length(state.time));       % Rolling friction on all sets of suspention wheels
state.stripeIndices = zeros(1,parameters.totalStripeCount);        % Indices at which we detect each stripe

%% Calculation loop
%  This is the main loop of the script, caluclating the relevant values for
%  each point in time. The function calc_main gets called at each iteration 
%  and handles the states of the trajectory internally by passing acceleration "state"
%  variable as the first input argument.
%  state = 1 -- Acceleration
%  state = 2 -- Deceleration
%  state = 3 -- Max frequency

state.mode = 1;         % We start in the acceleration state
state.stripeCount = 0;   % Initially we have counted 0 stripes

% For each point in state.time ...
display('Simulating trajectory')
for i = 2:length(state.time) % Start at i = 2 because values are all init at 1
    %% State transitions
    % If we have exceeded the max. frequency we cap the frequency and recalculate
    if (state.frequency(i - 1) > parameters.maxFrequency)
        state.mode = 3; % Max frequency
        
        % Recalculate previous time = i - 1 to avoid briefly surpassing max frequency
        state = calcMain(parameters, state, i - 1);
    end
    
    % If we have reached the maximum allowed acceleration distance we 
    % transition to deceleration
    if (parameters.useMaxAccDistance)
        if state.distance(i - 1) >= (parameters.maxAccDistance)
            state.mode = 2; % Deceleration
        end
    else
        % Calculate braking distance using linear deceleration
        brakingDist = parameters.brakingDistInterpolant(state.velocity(i - 1));
        if state.distance(i - 1) >= (parameters.trackLength - brakingDist)
            state.mode = 2; % Deceleration
        end
    end
    
    %% Main calculation
    % Calculate for current time = i
    state = calcMain(parameters, state, i);
    
    if verbose
        fprintf("Step: %i, %.2f s, %.2f m, %.2f m/s, %4.0f Hz, %.2f m/s, mode: %i\n", i, state.time(i), state.distance(i), state.velocity(i), state.frequency(i), state.slip(i), state.mode)
    end
    
    %% Stripes
    if (state.distance(i) >= (1 + state.stripeCount) * parameters.stripeDistance)
        state.stripeIndices(1 + state.stripeCount) = i;
        state.stripeCount = state.stripeCount + 1;
    end
    
    %% Exit conditions
    % Stop when speed is 0m/s or state.time is up
    if state.velocity(i) <= 0 || i == length(state.time)
        % Truncate arrays and create final results structure 
        results = finalizeResults(i, state);
        % Break from loop
        break;
    end
end

%% Print some results LP Change at the end
% Find max. speed and x force
[vMax vMaxTime] = max(results.velocity);
vMaxTime = vMaxTime * parameters.dt - parameters.dt;
[freqMax freqMaxTime] = max(results.frequency);
freqMaxTime = freqMaxTime * parameters.dt - parameters.dt;
fxMax = max(results.fx);
fxMin = min(results.fx);

% Let's display some stuff for quick viewing
fprintf('\n--------------------RESULTS--------------------\n');
fprintf('\nDuration of run: %.2f s\n', state.time(i));
fprintf('\nDistance: %.2f m\n', state.distance(i));
fprintf('\nTop speed: %.2f m/s i.e. %.2f km/h i.e. %.2f mph at %.2f s\n', vMax(1), vMax(1) * 3.6, vMax(1) * 2.23694, vMaxTime(1));
fprintf('\nMaximum frequency: %3.0f Hz at %.2f s\n', freqMax, freqMaxTime);

%% Plot the trajectory graphs
if displayPlots
    plotTrajectory(results);
end
