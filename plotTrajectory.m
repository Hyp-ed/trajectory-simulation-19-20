function plotTrajectory(result)
% plotTrajectory   Plots the trajectory graphs from distance and velocity arrays over time
%                  And saves them to 'trajectoryPlot.fig'
% Inputs:
%   time           Base time array
%   distance       Distance over time array
%   velocity       Velocity over time array
% Output: 
% @author          Rafael Anderka
%                  HypED, 03/11/2018
% Modified:        -

    % Set plot size
    plotWidth = 1500;
    plotHeight = 500;

    % Get screen resolution in pixels
    set(0, 'units', 'pixels');          % Set screen units to pixels
    screenRes = get(0, 'screensize');   % Get screen size in pixels
    screenWidth = screenRes(3);         % Width is 3rd entry
    screenHeight = screenRes(4);        % Height is 4th entry

    % Find xy coordinates of plot window to center on screen
    x = (screenWidth - plotWidth) / 2;
    y = (screenHeight - plotHeight) / 2;

    % Create the plots and save them to 'trajectoryPlot.fig'
    figure('position', [x y plotWidth plotHeight]);
    ax1 = subplot(1, 4, 1);
    plot(ax1, result.time, result.distance); axis tight; ylim([0 1300]); title('Displacement vs. Time'); ylabel('Displacement(m)'); xlabel('Time(s)');
    ax2 = subplot(1, 4, 2);
    plot(ax2, result.time, result.velocity); axis tight; ylim([0 90]); title('Velocity vs. Time'); ylabel('Velocity(m/s)'); xlabel('Time(s)');
    ax3 = subplot(1, 4, 3);
    plot(ax3, result.time, result.frequency); axis tight; ylim([0 800]); title('DSLIM Frequency vs. Time'); ylabel('Frequency [Hz]'); xlabel('Time [s]');    
    ax4 = subplot(1, 4, 4);
    plot(ax4, result.time, result.powerInput); axis tight; ylim([0 1400000]); title('Power input vs. Time'); ylabel('Power input(W)'); xlabel('Time(s)');
    savefig('trajectoryPlot');
end