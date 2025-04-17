function [projected_apogee, drag] = calculate_apogee_RK4(current_altitude, velocity, servo_angle, mass, pressure_initial, temp_initial)
    %% RK4 Apogee Prediction
    % This program uses an RK4 algorithm that adjusts for altitude change to
    % predict the apogee of a rocket
    
    % Author: William Teasley
    % Date: 24 March 2024
    % Completed Individually

    %% Set Parameters
    dt = 0.5;
    g = 32.17405; % ft/s^2
    i = 1;
    
    % y = zeros(1, 1000);
    % v = zeros(1, 1000);


    % Sets the initial value of y and v. 
    y(1) = current_altitude;
    v(1) = velocity;
    drag = calculate_drag_new(y(1), v(1), servo_angle, pressure_initial, temp_initial);
    
    Fx = @(y, v) v;
    Fp = @(y, v) -g - (calculate_drag_new(y, v, servo_angle, pressure_initial, temp_initial)/mass);
    
    %% Actual RK4 stuff 
    while v(i) > -10
        kx1 = Fx(y(i), v(i));                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     
        kp1 = Fp(y(i), v(i));
    
        kx2 = Fx(y(i)+0.5*dt*kx1, v(i)+0.5*kp1*dt);
        kp2 = Fp(y(i)+0.5*dt*kx1, v(i)+0.5*kp1*dt);
    
        kx3 = Fx(y(i)+0.5*dt*kx2, v(i)+0.5*kp2*dt);
        kp3 = Fp(y(i)+0.5*dt*kx2, v(i)+0.5*kp2*dt);
    
        kx4 = Fx(y(i)+kx3*dt, v(i)+kp3*dt);
        kp4 = Fp(y(i)+kx3*dt, v(i)+kp3*dt);
    
        y(i+1) = y(i) + (1/6)*(kx1+2*kx2+2*kx3+kx4)*dt;
        v(i+1) = v(i) + (1/6)*(kp1+2*kp2+2*kp3+kp4)*dt;
    
        i = i+1;
    end
    projected_apogee = max(y);
end
    

