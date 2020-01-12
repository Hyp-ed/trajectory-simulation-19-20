function forceLookupTable = generateForceLookupTable()
%% Generates a 2D lookup table by converting a COMSOL generated csv look-up table into a MATLAB matrix
% @author Lorenzo Principe
clc; clear; close all;

displayFigures = true;

fprintf("Generating force lookup table ...\n")

% Input and output file name variable
inFile      = './lookupTables/19-11-17_LIMForceTable_Export.xlsx';
outTable    = './lookupTables/temp/forceLookupTable.mat';
outCoeff    = './lookupTables/temp/optimalSlipsCoefficients.mat';
outFig      = './lookupTables/temp/forceLookupTableSurfacePlot.fig';

% Import data
fprintf("- reading input data ...\n")
data = xlsread(inFile);

% Determine parameters
freqMin = data(1, 1);
freqMax = data(end, 1);
vMin    = data(1, 2);
vMax    = data(end, 2);

vStep           = data(2, 2) - data(1, 2);
vStepsCount     = (vMax - vMin) / vStep + 1;
freqStep        = data(vStepsCount + 1, 1) - data(1, 1);
freqStepsCount  = (freqMax - freqMin) / freqStep + 1;

% Extract data
fprintf("- creating MATLAB matrix ...\n")
frequencies = freqMin:freqStep:freqMax;
velocities  = vMin:vStep:vMax;
forces      = data(1:end, 3);

% Reshape data into matrix
forces = reshape(forces, length(velocities), length(frequencies));

% Save force lookup table
save(outTable, 'forces', 'vStep', 'freqStep', 'velocities', 'frequencies');

if displayFigures
    % Plot mesh
    f   = figure('Name', 'Forces plot');
    ax  = axes('Parent', f);

    mesh(ax,frequencies,velocities,forces);
    title('DSLIM 2D thrust force lookup table');
    xlabel('Frequency (Hz)');
    ylabel('Velocity (ms^{-1})');
    zlabel('Thrust force (N)');

    % Save mesh figure
    savefig(f, outFig)
end    
    
%% Determine optimal frequency as a function of velocity
fprintf("- determining optimal frequencies ...\n")
optFrequency    = zeros(1, length(velocities));
optForces       = zeros(length(velocities), 1);

for i = 1:length(velocities)
    [~, pos]        = max(forces(i, :));
    optFrequency(i) = frequencies(pos);
    optForces(i)    = forces(i, pos);
end
if displayFigures
    hold on;
    scatter3(ax, optFrequency, velocities, optForces);
    hold off;

    f2 = figure('Name','Optimal Frequency');
    scatter(velocities, optFrequency);
end

optimalFrequencyCoefficients = polyfit(velocities, optFrequency, 1);

save(outCoeff, 'optimalFrequencyCoefficients');
if displayFigures
    fit = polyval(optimalFrequencyCoefficients, velocities);
    hold on;
    plot(velocities, fit);
    hold off;
    title('Optimal frequency');
    xlabel('Velocity (ms^{-1})');
    ylabel('Frequency (Hz)');
end

forceLookupTable = struct('vStep', vStep, ...
                          'freqStep', freqStep, ...
                          'frequencies', frequencies, ...
                          'velocities', velocities, ...
                          'forces', forces, ...
                          'optimalFrequencyCoefficients', optimalFrequencyCoefficients);