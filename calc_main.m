function [v,a,distance,phase,frequency,power,powerLoss,powerInput,efficiency,frequency,fxLim,fx] = calc_main(parameters,state,i,v,a,distance,phase,frequency,power,powerLoss,powerInput,efficiency,frequency,fxLim,fx,braking_force,ForceLookupTable)
% CALC_MAIN Calculates trajectory values at each point in time.
% calc_main gets called at each iteration and handles the states of the 
% trajectory via a passed state input argument.
% state = 1 -- Acceleration
% state = 2 -- Deceleration
% state = 3 -- Max RPM
% @author      Andreas Malekos, Rafael Anderka

    % Calculate frequency, LIM thrust force, torque (LP torque to be changed)
    switch state
        case 1 % Acceleration
            % Find optimal frequency and corresponding driving force
            frequency(i) = calc_optimalFrequency(v(i-1), ForceLookupTable.optimalFrequencyCoefficients);
            fx(i) = calc_fx((i), v(i-1), ForceLookupTable);
            
            % Calculate angular velocity and angle of Halbach wheels 
            % LP: frequency? Phase?
            frequency(i) = (frequency(i)+v(i-1))/parameters.ro;
            phase(i) = phase(i-1) + frequency(i) * dt;
            
            % Calculate required angular acceleration and torque 
            % LP needed at all?
            alpha = (frequency(i)-frequency(i-1))/dt;
            torque(i) = alpha * parameters.i;
            
            % Calculate rail heating losses LP Maybe relate this to end
            % effects and rail heating losses from LIMs

            %power_loss(i) = n_lim*calc_pl(frequency(i), v(i-1), pl_lookup_table, parameters);
            powerLoss(i) = 0 % LP For now
                     
        case 2 % Deceleration using EmBrakes
            % Find steady state frequency based on constraints from equations of motion assuming no external motor torque
            frequency(i) = fzero(@(s) (s + v(i-1) + (n_lim*calc_fx(s,v(i-1),ForceLookupTable) - n_brake*braking_force)/parameters.M*dt)/parameters.ro - frequency(i-1) + calc_fx(s,v(i-1),ForceLookupTable)*parameters.ro/parameters.i*dt,[frequency(i-1)-1,frequency(i-1)+1]);

            
            fx(i) = calc_fx(frequency(i), v(i-1), ForceLookupTable);
            frequency(i) = frequency(i-1) - fx(i) * parameters.ro / parameters.i * dt;
            phase(i) = phase(i-1) + frequency(i) * dt;

            powerLoss(i) = n_lim*calc_pl(frequency(i), v(i-1), pl_lookup_table, parameters);
            alpha = (frequency(i)-frequency(i-1))/dt;
        

            
        case 3 % Max RPM
            % Set frequency to max. frequency
            frequency(i) = parameters.m_frequency;
            phase(i) = phase(i-1) + frequency(i) * dt;
            frequency(i) = frequency(i)*parameters.ro - v(i-1);
            fx(i) = calc_fx(frequency(i), v(i-1), ForceLookupTable);
            alpha = (frequency(i)-frequency(i-1))/dt;
            torque(i) = alpha * parameters.i;
            %torque_motor(i) =  torque(i) + f_thrust_wheel(i) * parameters.ro + power_loss(i)/n_lim * frequency(i);
            torque_motor(i) =  torque(i) + fx(i) * parameters.ro;
            powerLoss(i) = n_lim*calc_pl(frequency(i), v(i-1), pl_lookup_table, parameters);
    end
    
  

    
    % Calculate total x forces
    fx(i) = fx(i)*n_lim;
    
    
    
    % Calculate acceleration, velocity and distance
    a(i) = fx(i)/parameters.M;
    
    % Calculate trajectory
    v(i) = v(i-1) + dt*a(i);
    distance(i) = distance(i-1)+dt*v(i);
    
    % Calculate lateral torque, power and efficiency
    power(i) = fx(i)*v(i); % power output = force * velocity
    powerInput(i) = power(i)+powerLoss(i); % ignoring inertia
    
    efficiency(i) = power(i)/powerInput(i);

end
