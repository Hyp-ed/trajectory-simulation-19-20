function fx = calc_fx(frequency,vt,fx_lookup_table)
% CALC_FX           Calculates thrust force as a function of slip and velocity by bilinearly interpolating a thrust force lookup table
% Inputs:
%   slip            Absolute slip
%   vt              Translational Velocity
%   fx_lookup_table A COMSOL-generated thrust force lookup table
% Output:
%   fx              Thrust force per wheel
    
    % Mirror for negative slip
    s = sign(frequency);
    frequency = abs(frequency);
    
    % Slip indices (x-axis)
    i_f = frequency / fx_lookup_table.freq_step + 1; % x
    i_f_min = floor(i_f); % x1
    i_f_max = i_f_min + 1; % x2
    
    % Velocity indices (y-axis)
    i_v = vt / fx_lookup_table.v_step + 1; % y
    i_v_min = floor(i_v); % y1
    i_v_max = i_v_min + 1; % y2
    
    % Bilinear interpolation
    fx1 = (i_f_max - i_f) / (i_f_max - i_f_min) * fx_lookup_table.forces(i_v_min, i_f_min) + (i_f - i_f_min) / (i_f_max - i_f_min) * fx_lookup_table.forces(i_v_min, i_f_max); % Interpolate along x-axis for y1
    fx2 = (i_f_max - i_f) / (i_f_max - i_f_min) * fx_lookup_table.forces(i_v_max, i_f_min) + (i_f - i_f_min) / (i_f_max - i_f_min) * fx_lookup_table.forces(i_v_max, i_f_max); % Interpolate along x-axis for y2
    fx = (i_v_max - i_v) / (i_v_max - i_v_min) * fx1 + (i_v - i_v_min) / (i_v_max - i_v_min) * fx2; % Interpolate along y-axis between (x, y1) and (x, y2)
    fx = fx / 2; % Divide by two since the lookup table is for wheel pairs LP: Probably needs to be removed
    fx = s * fx;
    
end