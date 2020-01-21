function [ results ] = finalizeResults(maxIndex, time, distance, velocity, velocitySync, acceleration, phase, frequency, fx, power, powerLoss, powerInput, efficiency, slip)
% finalizeResults  Truncates trajectory arrays and creates acceleration results structure
% Inputs:
%   maxIndex        Last used index in results arrays 
%                   (results will be truncated up to this index)
%   time            Time array
%   distance        Distance travelled over time
%   velocity        Velocity of pod over time
%   acceleration    Acceleration of pod over time
%   phase           Phase of the LIM field over time
%   frequency       Frequency of the LIM field over time
%   fx              Total force in x-direction over time
%   power           Power over time
%   powerLoss       Power loss over time
%   powerInput      Power input over time
%   efficiency      Efficiency over time
%   slip            Slips over time
%
% Output:
%   [ results ]     Result structure
% @author           Rafael Anderka
%                   HypED, 03/11/2018

    % Create results structure while truncating each array up to 'maxIndex'
    results = struct;
    results.time            = time(1:maxIndex);
    results.distance        = distance(1:maxIndex);
    results.velocity        = velocity(1: maxIndex);
    results.velocitySync    = velocitySync(1: maxIndex);
    results.acceleration    = acceleration(1:maxIndex);
    results.phase           = phase(1:maxIndex);
    results.frequency       = frequency(1:maxIndex);
    results.fx              = fx(1:maxIndex);
    results.power           = power(1:maxIndex);
    results.powerLoss       = powerLoss(1:maxIndex);
    results.powerInput      = powerInput(1:maxIndex);
    results.efficiency      = efficiency(1:maxIndex);
    results.slip            = slip(1:maxIndex);

end

