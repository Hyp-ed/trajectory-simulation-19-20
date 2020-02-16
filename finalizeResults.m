function results = finalizeResults(maxIndex, state)
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
    results.time            = state.time(1:maxIndex);
    results.distance        = state.distance(1:maxIndex);
    results.velocity        = state.velocity(1: maxIndex);
    results.velocitySync    = state.velocitySync(1: maxIndex);
    results.acceleration    = state.acceleration(1:maxIndex);
    results.phase           = state.phase(1:maxIndex);
    results.frequency       = state.frequency(1:maxIndex);
    results.fx              = state.fx(1:maxIndex);
    results.power           = state.power(1:maxIndex);
    results.powerLoss       = state.powerLoss(1:maxIndex);
    results.powerInput      = state.powerInput(1:maxIndex);
    results.efficiency      = state.efficiency(1:maxIndex);
    results.slip            = state.slip(1:maxIndex);

end

