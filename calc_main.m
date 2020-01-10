function [v,a,distance,theta,frequency,power,power_loss,power_input,efficiency,slips,f_thrust,f_lat_wheel,f_x_pod,f_y_pod] = calc_main(state,i,dt,n_lim,n_brake,v,a,distance,theta,frequency,power,power_loss,power_input,efficiency,slips,f_thrust,f_lat_wheel,f_x_pod,f_y_pod,parameters,braking_force,fx_lookup_table,pl_lookup_table,ct_lookup_table,of_coefficients)
% CALC_MAIN Calculates trajectory values at each point in time.
% calc_main gets called at each iteration and handles the states of the 
% trajectory via a passed state input argument.
% state = 1 -- Acceleration
% state = 2 -- Deceleration
% state = 3 -- Max RPM
% @author      Andreas Malekos, Rafael Anderka

    
    % Calculate slips, LIM thrust force, torque (LP torque to be changed)
    switch state
        case 1 % Acceleration
            % Find optimal slips and corresponding driving force
            slips(i) = calc_optimalSlips(v(i-1), of_coefficients);
            f_thrust(i) = calc_fx(slips(i), v(i-1), fx_lookup_table);
            
            % Calculate angular velocity and angle of Halbach wheels 
            % LP: slips? Phase?
            frequency(i) = (slips(i)+v(i-1))/parameters.ro;
            theta(i) = theta(i-1) + frequency(i) * dt;
            
            % Calculate required angular acceleration and torque 
            % LP needed at all?
            alpha = (frequency(i)-frequency(i-1))/dt;
            torque(i) = alpha * parameters.i;
            
            % Calculate rail heating losses LP Maybe relate this to end
            % effects and rail heating losses from LIMs

            %power_loss(i) = n_lim*calc_pl(slips(i), v(i-1), pl_lookup_table, parameters);
            power_loss(i) = 0 % LP For now
           
         
            
        case 2 % Deceleration using EmBrakes
            % Find steady state slips based on constraints from equations of motion assuming no external motor torque
            slips(i) = fzero(@(s) (s + v(i-1) + (n_lim*calc_fx(s,v(i-1),fx_lookup_table) - n_brake*braking_force)/parameters.M*dt)/parameters.ro - frequency(i-1) + calc_fx(s,v(i-1),fx_lookup_table)*parameters.ro/parameters.i*dt,[slips(i-1)-1,slips(i-1)+1]);

            
            f_thrust(i) = calc_fx(slips(i), v(i-1), fx_lookup_table);
            frequency(i) = frequency(i-1) - f_thrust(i) * parameters.ro / parameters.i * dt;
            theta(i) = theta(i-1) + frequency(i) * dt;

            power_loss(i) = n_lim*calc_pl(slips(i), v(i-1), pl_lookup_table, parameters);
            alpha = (frequency(i)-frequency(i-1))/dt;
        

            
        case 3 % Max RPM
            % Set frequency to max. frequency
            frequency(i) = parameters.m_frequency;
            theta(i) = theta(i-1) + frequency(i) * dt;
            slips(i) = frequency(i)*parameters.ro - v(i-1);
            f_thrust(i) = calc_fx(slips(i), v(i-1), fx_lookup_table);
            alpha = (frequency(i)-frequency(i-1))/dt;
            torque(i) = alpha * parameters.i;
            %torque_motor(i) =  torque(i) + f_thrust_wheel(i) * parameters.ro + power_loss(i)/n_lim * frequency(i);
            torque_motor(i) =  torque(i) + f_thrust(i) * parameters.ro;
            power_loss(i) = n_lim*calc_pl(slips(i), v(i-1), pl_lookup_table, parameters);
    end
    
  

    
    % Calculate total x forces
    f_x_pod(i) = f_thrust(i)*n_lim;
    
    
    
    % Calculate acceleration, velocity and distance
    a(i) = f_x_pod(i)/parameters.M;
    
    % Calculate trajectory
    v(i) = v(i-1) + dt*a(i);
    distance(i) = distance(i-1)+dt*v(i);
    
    % Calculate lateral torque, power and efficiency
    power(i) = f_x_pod(i)*v(i); % power output = force * velocity
    power_input(i) = power(i)+power_loss(i); % ignoring inertia
    
    efficiency(i) = power(i)/power_input(i);

end
