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


    % Get screen resolution in pixels
    set(0, 'units', 'pixels');          % Set screen units to pixels
    screenRes = get(0, 'screensize');   % Get screen size in pixels
    screenWidth = screenRes(3);         % Width is 3rd entry
    screenHeight = screenRes(4);        % Height is 4th entry

    % Set plot size
    plotWidth = screenWidth;
    plotHeight = 700;

    % Find xy coordinates of plot window to center on screen
    x = (screenWidth - plotWidth) / 2;
    y = (screenHeight - plotHeight) / 2;
    

    % Create the plots and save them to 'trajectoryPlot.fig'
    figure('position', [x y plotWidth plotHeight]);
    ax1 = subplot(1, 6, 1);
    plot(ax1, result.time, result.distance); axis tight; ylim([0 450]); title('Displacement vs. Time'); ylabel('Displacement [m]'); xlabel('Time [s]');
    ax2 = subplot(1, 6, 2);
    plot(ax2, result.time, result.velocity); axis tight; ylim([0 50]); title('Velocity vs. Time'); ylabel('Velocity [m/s]'); xlabel('Time [s]');
    ax3 = subplot(1, 6, 3);
    plot(ax3, result.time, result.velocitySync); axis tight; ylim([0 50]); title('DSLIM Sync. Velocity vs. Time'); ylabel('Sync. Velocity [m/s]'); xlabel('Time [s]');
    ax4 = subplot(1, 6, 4);
    plot(ax4, result.time, result.frequency); axis tight; ylim([0 300]); title('DSLIM Frequency vs. Time'); ylabel('Frequency [Hz]'); xlabel('Time [s]');    
    ax5 = subplot(1, 6, 5);
    plot(ax5, result.time, result.slip); axis tight; ylim([0 1.2]); title('Slip vs. Time'); ylabel('Slip'); xlabel('Time [s]');
    ax6 = subplot(1, 6, 6);
    plot(ax6, result.time, result.powerInput); axis tight; ylim([0 20000]); title('Power input vs. Time'); ylabel('Power input [W]'); xlabel('Time [s]');
    savefig('output/trajectoryPlot');
end