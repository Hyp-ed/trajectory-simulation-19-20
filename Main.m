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
fx_lookup_table = generate_table();


% Additional parameters
deceleration_total = 9.81;     % Total braking deceleration
number_of_stripes = floor(parameters.track_length / parameters.stripe_dist);    % Total number of stripes we will detect

%% Initialize arrays
%  Create all necessary arrays and initialize with 0s for each time step. 
%  This is computationally faster than extending the arrays after each calculation.
time = 0:dt:tmax;                       % Create time array with time step dt and maximum time tmax
v = zeros(1,length(time));              % Velocity of pod
a = zeros(1,length(time));              % Acceleration of pod
distance = zeros(1,length(time));       % Distance travelled
phase = zeros(1,length(time));          % Current phase of Lim fields
frequency = zeros(1,length(time));      % Lim frequency
power = zeros(1,length(time));          % Power
power_loss = zeros(1,length(time));     % Power loss
power_input = zeros(1,length(time));    % Power input
efficiency = zeros(1,length(time));     % Power output / Power input
slips = zeros(1,length(time));          % Slip between LIM field and track
f_thrust_wheel = zeros(1,length(time)); % Thrust force from a single Halbach wheel
f_lat_wheel = zeros(1,length(time));    % Lateral force from a single Halbach wheel   
f_x_pod = zeros(1,length(time));        % Net force in direction of track (x) for whole pod
f_y_pod = zeros(1,length(time));        % Net force in lateral direction (y) for whole pod
stripes = zeros(1,number_of_stripes);   % Indices at which we detect each stripe

%% Calculation loop
%  This is the main loop of the script, caluclating the relevant values for
%  each point in time. The function calc_main gets called at each iteration 
%  and handles the states of the trajectory internally by passing a "state"
%  variable as the first input argument.
%  state = 1 -- Acceleration
%  state = 2 -- Deceleration
%  state = 3 -- Max RPM

state = 1;          % We start in the acceleration state
stripe_count = 0;   % Initially we have counted 0 stripes

% For each point in time ...
for i = 2:length(time) % Start at i = 2 because values are all init at 1
    %% state transitions
    % If we have exceeded the max. RPM we cap the RPM and recalculate
    if (frequency(i-1) * 60 / (2 * pi)) > parameters.mass_rpm
        state = 3; % Max RPM
        
        % Recalculate previous time = i - 1 to avoid briefly surpassing max RPM
        [v,a,distance,phase,frequency,power,power_loss,power_input,efficiency,slips,f_thrust_wheel,f_lat_wheel,f_x_pod,f_y_pod] = ...
        calc_main(state, i - 1, dt, n_lim, n_brake, v, a, distance, phase, frequency, power, power_loss, power_input, efficiency, slips, ...
                  f_thrust_wheel, f_lat_wheel, f_x_pod, f_y_pod, parameters, braking_force, fx_lookup_table, pl_lookup_table, of_coefficients);
    end
    
    % If we have reached the maximum allowed acceleration distance we 
    % transition to deceleration
    if (useMaxAccDistance)
        if distance(i-1) >= (maxAccDistance)
            state = 2; % Deceleration
        end
    else
        % Calculate our 'worst case' braking distance assuming a 100% energy transfer from wheels into translational kinetic energy
        % LP Determine stored energy in Lims that would be translated intro
        %  translational kinetic energy
        kinetic_energy = 0.5 * parameters.mass * v(i-1)^2;
        rotational_kinetic_energy = n_lim * 0.5 * parameters.i * frequency(i-1)^2;
        total_kinetic_energy = kinetic_energy + rotational_kinetic_energy;
        e_tot = kinetic_energy + rotational_kinetic_energy;
        braking_dist = (e_tot / parameters.mass) / (deceleration_total);
        if distance(i-1) >= (parameters.l - braking_dist)
            state = 2; % Deceleration
        end
    end
    
    %% Main calculation
    % Calculate for current time = i
    [v,a,distance,phase,frequency,power,power_loss,power_input,efficiency,slips,f_thrust_wheel,f_lat_wheel,f_x_pod,f_y_pod] = ...
    calc_main(state, i, dt, n_lim, n_brake, v, a, distance, phase, frequency, power, power_loss, power_input, efficiency, slips, ...
              f_thrust_wheel, f_lat_wheel, f_x_pod, f_y_pod, parameters, braking_force, fx_lookup_table, pl_lookup_table, of_coefficients);
    
    fprintf("Step: %i, %.2f s, %.2f m, %.2f m/s, %4.0f RPM, %.2f m/s, state: %i\n", i, time(i), distance(i), v(i), frequency(i) * 60 / (2 * pi), slips(i), state)
    
    %% Stripes
    if (distance(i) >= (1 + stripe_count) * stripe_dist)
        stripes(1 + stripe_count) = i;
        stripe_count = stripe_count + 1;
    end
    
    %% Exit conditions
    % Stop when speed is 0m/s or time is up
    if v(i) <= 0 || i == length(time)
        % Truncate arrays and create final result structure 
        result = finalizeResults(i, time, distance, v, a, phase, frequency * 60 / (2 * pi), f_thrust_wheel, f_lat_wheel,...
                                 f_x_pod, f_y_pod, power, power_loss, power_input, efficiency, slips);
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