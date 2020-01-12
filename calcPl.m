function pl = calcPl(frequency, velocity, parameters)
% CALCPL            Calculates power loss as a function of frequency and velocity by bilinearly interpolating a power loss lookup table
% Inputs:
%   frequency       DSLIM field frequency
%   velocity        Translational velocity
%   parameters      Script parameters
% Output:
%   pl              Power loss per wheel

    %{    
    % Frequency indices (x-axis)
    i_f = frequency / parameters.plLookupTable.freqStep + 1; % x
    i_f_min = floor(i_f); % x1
    i_f_max = i_f_min + 1; % x2
    
    % Velocity indices (y-axis)
    i_v = velocity / parameters.plLookupTable.vStep + 1; % y
    i_v_min = floor(i_v); % y1
    i_v_max = i_v_min + 1; % y2
    
    % Bilinear interpolation
    pl1 = (i_f_max - i_f) / (i_f_max - i_f_min) * parameters.plLookupTable.powerLosses(i_v_min, i_f_min) + (i_f - i_f_min) / (i_f_max - i_f_min) * parameters.plLookupTable.powerLosses(i_v_min, i_f_max); % Interpolate along x-axis for y1
    pl2 = (i_f_max - i_f) / (i_f_max - i_f_min) * parameters.plLookupTable.powerLosses(i_v_max, i_f_min) + (i_f - i_f_min) / (i_f_max - i_f_min) * parameters.plLookupTable.powerLosses(i_v_max, i_f_max); % Interpolate along x-axis for y2
    pl = (i_v_max - i_v) / (i_v_max - i_v_min) * pl1 + (i_v - i_v_min) / (i_v_max - i_v_min) * pl2; % Interpolate along y-axis between (x, y1) and (x, y2)
    %}

    pl = 0;
end