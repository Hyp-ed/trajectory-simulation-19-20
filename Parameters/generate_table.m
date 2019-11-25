%% Generates a 3D lookup table that converts COMSOL generated csv look-up table into MATLAB matrix
% @author Lorenzo Principe
clc; clear; close all;

% Input and output file name variables
in_file     = '19-11-17_LIMForceTable_Export.xlsx';
out_table   = 'forceLookupTable.mat';
out_fig     = 'Forces_surfacePlot.fig';


% Import data
data = xlsread(in_file);

% Determine parameters
freq_min    = data(1,1);
freq_max    = data(end,1);
v_min       = data(1,2);
v_max       = data(end,2);

v_step      = data(2,2)-data(1,2);
n_v_steps   = (v_max-v_min)/v_step + 1;
freq_step   = data(n_v_steps+1,1)-data(1,1);
n_freq_steps= (freq_max-freq_min)/freq_step + 1;

% Extract data
frequencies = freq_min:freq_step:freq_max;
velocities  = v_min:v_step:v_max;
forces      = data(1:end,3);

% Reshape data into matrix
forces = reshape(forces,length(velocities),length(frequencies));

% Save Workspace
save(out_table)

% Plot mesh
f = figure('Name','Forces plot','NumberTitle','off');

mesh(frequencies,velocities,forces);
title('Forces look-up table');
xlabel('Frequency (Hz)');
ylabel('Velocity (ms^{-1})');
zlabel('Thrust force (N)');

% Save mesh figure
savefig(f,out_fig)
fprintf("Workspace saves as:\t%s\nFigure saved as:\t%s\n",...
        out_table,out_fig);
