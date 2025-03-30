function motor = calculate_motor(motor_name)
%% Motor Generator
%   Author: William Teasley
%
%   Date: 7 March 2025
%
%   This function serves to build the motor of the launch vehicle using
%   manufacturer thrust data
%
%   Inputs:
%       motor_properties =  a struct containing the name of the motor
%                           being used
%
%   Outputs:
%       motor = a struct containing motor data and a function used to 
%               calculate the thrust for a given motor and time
global g

switch motor_name
    case "L1940X"
        motor_data = readmatrix("Thrust Data\L1940_ThrustCurve.csv");
        motor_data = motor_data(3:72, :); % crop data
        motor_data(:,1) = motor_data(:,1) - min(motor_data(:,1)); % Zero the time
        motor_data(:, 2) = motor_data(:, 2)./(32.17405*16); % Convert motor mass from oz to slugs
        motor_data(:, 3) = motor_data(:, 3)*0.224809; % Convert thrust from N to lbf

        motor.motor_data = motor_data;
        motor.full_weight = (136.17/16)/g; % motor.motor_data(1, 2);
        motor.empty_weight = (68.2/16)/g;
        motor.calculate_thrust = @(t) calculate_thrust(motor, t);
    case "balls"
        disp("haha")
end