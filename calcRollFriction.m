function friction= calcRollFriction(parameters)
% CALCROLLFRICTION  Calculates rolling friction of wheels
% Based on: https://www.youtube.com/watch?v=v1FkqZrEPg8
%
% Rolling friction constant in parameters.json is in mm.
% It has to match units of wheel diameter
%
%       
% Inputs:
%   acceleration    Acceleration (surprise)
%   parameters      Script parameters
% Output:
%   friction              overall friction force on pod

%% Temporary fix (currently relying on friction constant 

    % Weight force on each wheel, assuming even weight distribution of pod
    normalForce = ( parameters.mass / parameters.suspWheelsCount ) * 9.81;
    
    friction = normalForce * parameters.rollFrictionConst / ...
        ( 2 * parameters.suspWheelsRadius );
    
%% Velocity dependant factor can be added later

end