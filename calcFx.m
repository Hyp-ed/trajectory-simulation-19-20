function fx = calcFx(frequency, velocity, parameters)
% CALCFX            Calculates thrust force as a function of frequency and velocity by bilinearly interpolating a thrust force lookup table
% Inputs:
%   frequency       DSLIM field frequency
%   velocity        Translational velocity
%   parameters      Script parameters
% Output:
%   fx              Thrust force provided by the full DSLIM module

%% Calculate propulsion force
    % Slip indices (x-axis)
    i_f = frequency / parameters.forceLookupTable.freqStep + 1; % x
    i_f_min = floor(i_f); % x1
    i_f_max = i_f_min + 1; % x2
    
    % Velocity indices (y-axis)
    i_v = velocity / parameters.forceLookupTable.vStep + 1; % y
    i_v_min = floor(i_v); % y1
    i_v_max = i_v_min + 1; % y2
    
    % Bilinear interpolation
    fx1 = (i_f_max - i_f) / (i_f_max - i_f_min) * parameters.forceLookupTable.forces(i_v_min, i_f_min) + (i_f - i_f_min) / (i_f_max - i_f_min) * parameters.forceLookupTable.forces(i_v_min, i_f_max); % Interpolate along x-axis for y1
    fx2 = (i_f_max - i_f) / (i_f_max - i_f_min) * parameters.forceLookupTable.forces(i_v_max, i_f_min) + (i_f - i_f_min) / (i_f_max - i_f_min) * parameters.forceLookupTable.forces(i_v_max, i_f_max); % Interpolate along x-axis for y2
    fx = (i_v_max - i_v) / (i_v_max - i_v_min) * fx1 + (i_v - i_v_min) / (i_v_max - i_v_min) * fx2; % Interpolate along y-axis between (x, y1) and (x, y2)
 
    
end