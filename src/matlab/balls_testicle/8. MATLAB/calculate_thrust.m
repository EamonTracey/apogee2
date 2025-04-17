function [thrust, motor_mass] = calculate_thrust(motor, t)
%% Derivatives Calculator
%   Author: William Teasley
%
%   Date: 7 March 2025
%
%   This function serves to calculate the thrust and motor mass of the 
%   motor at any given instant
%
%   Inputs:
%       motor_data =    a .csv of the specific motor data containing time,
%                       motor mass, and thrust
%
%   Outputs:
%       thrust =    a variable containing the thrust in lbs at the desired 
%                   time 
%       motor =     a variable containing the mass of the motor in slugs at
%                   the desired time


%% Interpolate Thrust Curve
time = motor.motor_data(:,1);
mass_data = motor.motor_data(:,2);
thrust_data = motor.motor_data(:,3);

if t < 0
    t=0;
end

if t<=time(end) 
    thrust = interp1(time, thrust_data, t);
    motor_mass = ((motor.empty_weight -  motor.full_weight)/(time(end)))*t + motor.full_weight; % interp1(time, mass_data, t);
elseif t>time(end)
    thrust = 0;
    motor_mass = motor.empty_weight;
else
    disp("Problem with t in calculate_thrust")
end


end