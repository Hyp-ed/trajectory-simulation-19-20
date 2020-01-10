function [v,a,distance,theta,omega,power,power_loss,power_input,efficiency,frequency,f_thrust,f_lat_wheel,f_x_pod,f_y_pod] = calc_main(state,i,dt,n_lim,n_brake,v,a,distance,theta,omega,power,power_loss,power_input,efficiency,frequency,f_thrust,f_lat_wheel,f_x_pod,f_y_pod,Lim_parameters,braking_force,fx_lookup_table,pl_lookup_table,ct_lookup_table,of_coefficients)
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
            frequency(i) = calc_optimalFrequency(v(i-1), of_coefficients);
            f_thrust(i) = calc_fx(frequency(i), v(i-1), fx_lookup_table);
            
            % Calculate angular velocity and angle of Halbach wheels 
            % LP: Frequency? Phase?
            omega(i) = (frequency(i)+v(i-1))/lim_parameters.ro;
            theta(i) = theta(i-1) + omega(i) * dt;
            
            % Calculate required angular acceleration and torque 
            % LP needed at all?
            alpha = (omega(i)-omega(i-1))/dt;
            torque(i) = alpha * lim_parameters.i;
            
            % Calculate rail heating losses LP Maybe relate this to end
            % effects and rail heating losses from LIMs

            %power_loss(i) = n_lim*calc_pl(frequency(i), v(i-1), pl_lookup_table, Lim_parameters);
            power_loss(i) = 0 % LP For now
           
         
            
        case 2 % Deceleration using EmBrakes
            % Find steady state slip based on constraints from equations of motion assuming no external motor torque
            frequency(i) = fzero(@(s) (s + v(i-1) + (n_lim*calc_fx(s,v(i-1),fx_lookup_table) - n_brake*braking_force)/Lim_parameters.M*dt)/Lim_parameters.ro - omega(i-1) + calc_fx(s,v(i-1),fx_lookup_table)*Lim_parameters.ro/Lim_parameters.i*dt,[frequency(i-1)-1,frequency(i-1)+1]);

            
            f_thrust(i) = calc_fx(frequency(i), v(i-1), fx_lookup_table);
            omega(i) = omega(i-1) - f_thrust(i) * lim_parameters.ro / lim_parameters.i * dt;
            theta(i) = theta(i-1) + omega(i) * dt;

            power_loss(i) = n_lim*calc_pl(frequency(i), v(i-1), pl_lookup_table, Lim_parameters);
            alpha = (omega(i)-omega(i-1))/dt;
        

            
        case 3 % Max RPM
            % Set omega to max. omega
            omega(i) = lim_parameters.m_omega;
            theta(i) = theta(i-1) + omega(i) * dt;
            frequency(i) = omega(i)*lim_parameters.ro - v(i-1);
            f_thrust(i) = calc_fx(frequency(i), v(i-1), fx_lookup_table);
            alpha = (omega(i)-omega(i-1))/dt;
            torque(i) = alpha * Lim_parameters.i;
            %torque_motor(i) =  torque(i) + f_thrust_wheel(i) * Lim_parameters.ro + power_loss(i)/n_lim * omega(i);
            torque_motor(i) =  torque(i) + f_thrust(i) * Lim_parameters.ro;
            power_loss(i) = n_lim*calc_pl(frequency(i), v(i-1), pl_lookup_table, Lim_parameters);
    end
    
  

    
    % Calculate total x forces
    f_x_pod(i) = f_thrust(i)*n_lim;
    
    
    
    % Calculate acceleration, velocity and distance
    a(i) = f_x_pod(i)/lim_parameters.M;
    
    % Calculate trajectory
    v(i) = v(i-1) + dt*a(i);
    distance(i) = distance(i-1)+dt*v(i);
    
    % Calculate lateral torque, power and efficiency
    power(i) = f_x_pod(i)*v(i); % power output = force * velocity
    power_input(i) = power(i)+power_loss(i); % ignoring inertia
    
    efficiency(i) = power(i)/power_input(i);

end
