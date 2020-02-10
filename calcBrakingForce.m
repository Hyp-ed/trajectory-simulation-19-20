function fx = calcBrakingForce(velocity, parameters)
% CALCBRAKINGFORCE       Calculates braking force as a function of velocity by linearly interpolating a thrust force lookup table
% Inputs:
%   velocity        Translational velocity
%   parameters      Script parameters
% Output:
%   fx              Braking force provided by the combination of magnetic
%                       and mechanical brakes
%
%   Both mechanical and magnetic brakes are applied at the same time

%% Magnetic braking
% Force determined from lookup talbe linking velocity and braking force
% (incoming from Alejandro)
% Linear interpolation needed
      
   mag = - parameters.brakeForceInterpolant(velocity);


%% Mechanical braking
% From talking to Eralp, normal force from mechanical brakes is 6 kN
% Friction coefficient is 
% Assumed to be constant wrt velocity

% Add mechanical braking force
    mech = - parameters.mechBrakeFCoef * parameters.mechBrakeNForce;       
   
    
%% Total braking force
    fx = mech + mag;
end