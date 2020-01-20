function fx = calcBrakingForce(velocity, parameters)
% CALCBRAKINGFORCE       Calculates braking force as a function of velocity by linearly interpolating a thrust force lookup table
% Inputs:
%   velocity        Translational velocity
%   parameters      Script parameters
% Output:
%   fx              Braking force provided by the combination of magnetic
%                       and mechanical brakes

%% Magnetic braking
% Force determined from lookup talbe linking velocity and braking force
% (incoming from Alejandro)
% Linear interpolation needed
      
%% Mechanical braking
% Need to talk to team

    fx = - parameters.mass * 2 * 9.81;       %temporary fix

%% Account for air drag and rolling friction
    % Add drag
    fx = fx - calcDrag(velocity,parameters);
    % Add rolling friction
    fx = fx - calcRollFriction(velocity,parameters) ;
    
end