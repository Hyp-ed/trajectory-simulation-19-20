function [] = generate_table()
%% Generates a 3D lookup table that converts COMSOL generated csv look-up table into MATLAB matrix
% @author Lorenzo Principe
clc; clear; close all;

% Input and output file name variable
in_file     = './Parameters/19-11-17_LIMForceTable_Export.xlsx';
out_table   = './Parameters/forceLookupTable.mat';
out_coeff_table = './Parameters/optimalSlipsCoefficients.mat';
out_fig     = './Parameters/Forces_surfacePlot.fig';


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
save(out_table,'forces','v_step','freq_step','velocities','frequencies')

% Plot mesh
f = figure('Name','Forces plot');
ax= axes('Parent',f)

mesh(ax,frequencies,velocities,forces);
title('Forces look-up table');
xlabel('Frequency (Hz)');
ylabel('Velocity (ms^{-1})');
zlabel('Thrust force (N)');

% Save mesh figure
savefig(f,out_fig)
fprintf("Workspace saves as:\t%s\nFigure saved as:\t%s\n",...
        out_table,out_fig);
    
    
%% Determine optimal slip as a function of velocity



opt_frequency   = velocities;
opt_forces      = zeros(length(velocities),1);

for i = 1:length(velocities)
    [~,pos] = max(forces(i,:));
    opt_frequency(i) = frequencies(pos);
    opt_forces(i)    = forces(i,pos);
end

hold on
scatter3(ax,opt_frequency,velocities,opt_forces)
hold off

f2=figure('Name','Optimal Frequency')
scatter(velocities,opt_frequency)


optimalFrequencyCoefficients = polyfit(velocities,opt_frequency,1)

save(out_coeff_table,'optimalFrequencyCoefficients')

fit = polyval(optimalFrequencyCoefficients,velocities);
hold on 
plot(velocities,fit);
hold off

title('Optimal frequency')
xlabel('Velocity (ms^{-1})')
ylabel('Frequency (Hz)')