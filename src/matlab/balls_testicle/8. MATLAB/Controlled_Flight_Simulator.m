%% ACS Controlled Flight Simulator
% This code simulates the powered flight of a rocket with a controlled
% apogee control system (ACS) in order to tune the PI parameters. Let's 
% use the imperial system.

% Property of AKΣ

% Author: William Teasley

% Date: 2 April 2025

clc; clear; close all;
sv = style_values(); % Load in style values for plotting
set(0, 'DefaultAxesFontName', sv.FontName);
addpath('Quaternions');      % include quaternion library
addpath('Quaternions\quaternion_library');
% -------------------------------------------------------------------------

%% PI Parameters
K = 1;
KP = 0.025;
KI = 0.025;
target_apogee = 5100; % ft

global g
g = 32.17405;

%% Values that might change on launch day
% Environment
environment_inish.P0 = 2114.7133; % lbf/ft2
environment_inish.T0 = 55; % °F
environment_inish.u_inf = 24.93333; % ft/s
environment_inish.wind_direction = 0; % wind direction counterclockwise from North, (°)

% Vehicle
vehicle_name = "katie";
vehicle = calculate_vehicle(vehicle_name); 
vehicle.m = (645.92/16)/g; % total vehicle mass with full motor, slugs
vehicle.x_cm = 57/12; % distance from nose to center of mass, ft
vehicle.angle_launch_rail = -5; % launch rail angle, (°)
vehicle.angle_azimuth = 0; % launch rail heading, counterclockwise from North (°)

%% Simulation Parameters
% Environment
environment_inish.wind_profile = "trap";
environment_inish.altitude0_ASL = 692; % altitude of launch rail ASL, ft
environment = calculate_environment(environment_inish, 0);

% Motor
motor_name = "L1940X";
motor = calculate_motor(motor_name);

%% Simulation starts here
mass(1) = vehicle.m + motor.full_weight ;
position(:, 1) = [0; 0; 0]; % xyz position in Global Earth coordinates relative to launch pad, ft
velocity(:, 1) = [0; 0; 0]; % velocity in Global Earth coordinates, ft/s
accel(:, 1) = [0; 0; 0]; % acceleration in Global Earth coordinates, ft/s2
eulers(:, 1) = [0; vehicle.angle_launch_rail*pi/180; vehicle.angle_azimuth*pi/180]; % Euler angles, rad (theta, psi, phi)
angular_vel(:, 1) = [0; 0; 0]; % angular velocity of body frame, rad/s
flap_angle(1) = 0; % drag flap angle, (°)
aoa(1) = 0; % angle of attack, (°)
m(1) = vehicle.m;
thrust(1) = 0;
a(1) = environment.a;
axial_drag(1) = 0;
normal_force(1) = 0;
F_axial(1) = 0;
t_start = 2.5;
Int = 0;

dt = 0.033; % s
t(1) = 0; % s
i = 1;

% Servo Model
max_deployment = dt*45/0.33; % Max speed the the flaps can turn is 45 deg in 0.33s [°/s]
max_flap_angle = 45;
F0 = 10;

x(:,1) = [position(:, 1); velocity(:, 1); eulers(:, 1); angular_vel(:, 1); flap_angle(1); mass(1)]; % initial conditions
dxdt(:,1) = zeros(length(x), 1);


while x(6,i) >= 0
    % Control off before burnout
    if t(i) < t_start % Control off
        x(13, i) = 0;
    else % Control on
        % Calculate predicted apogee
        predicted_apogee(i) = Six_DOF_RK4_Apogee_Prediction_fn(x(:, i), vehicle, motor, environment, t(i));
        fprintf("Predicted Apogee: %.1f ft/s \n", predicted_apogee(i))

        e(i) = predicted_apogee(i) - target_apogee;
        Prop = e(i);
        Int = Int + dt*e(i);
        flap_angle(i) = F0 + (KP*Prop + KI*Int);

        % Delay Model
        if i <= 1
            flap_angle_change(i) = 0;
        else
            flap_angle_change(i) = (flap_angle(i) - flap_angle(i-1));
            if flap_angle_change(i) > max_deployment
                flap_angle(i) = flap_angle(i-1) + max_deployment;
            elseif flap_angle_change(i) < -max_deployment
                flap_angle(i) = flap_angle(i-1) - max_deployment;
            else
                flap_angle(i) = flap_angle(i);
            end
        end

        % Flap angle limits
        if flap_angle(i) > max_flap_angle 
            x(13, i) = max_flap_angle;
        elseif flap_angle(i) < 0 
            x(13, i) = 0;
        else 
            x(13, i) = flap_angle(i);
        end

        flap_angle_change_actual(i) = (flap_angle(i) - flap_angle(i-1));


    end

    % Rocket Dynamics
    ky1 = calculate_derivatives(x(:, i), vehicle, motor, environment, t(i));
    ky2 = calculate_derivatives((x(:, i)+0.5*dt*ky1.vec), vehicle, motor, environment, t(i));
    ky3 = calculate_derivatives((x(:, i)+0.5*dt*ky2.vec), vehicle, motor, environment, t(i));
    ky4 = calculate_derivatives((x(:, i)+ky3.vec*dt), vehicle, motor, environment, t(i));    

    dxdt(:, i) = (1/6)*(ky1.vec+2*ky2.vec+2*ky3.vec+ky4.vec);
    x(:, i+1) = x(:, i) + dxdt(:, i).*dt;

    t(i+1) = t(i) + dt;
    fprintf("Vertical Velocity: %.1f ft/s \n", x(6,i))
    i = i+1;
end

%% Printing and plotting
fprintf("ACS Apogee: %.1f ft \n", max(x(3,:)));

f1=figure(1);
f1.Position = [100,100,800,500]; 
hold on; grid on;
plot(t, x(1,:), 'LineWidth',sv.LineWidth1,'Color',sv.Blue, 'DisplayName', 'X Position');
plot(t, x(2,:), 'LineWidth',sv.LineWidth1,'Color',sv.Orange, 'DisplayName', 'Y Position');
plot(t, x(3,:), 'LineWidth',sv.LineWidth1,'Color',sv.Green, 'DisplayName', 'Z Position');
xlabel('Time [s]'); ylabel('Position [ft]');
title('6DOF Simulated Flight')
legend('Location','northwest')

f2=figure(2);
f2.Position = [100,100,800,500]; 
hold on; grid on;
plot(t, x(4,:), 'LineWidth',sv.LineWidth1,'Color',sv.Blue, 'DisplayName', 'X Velocity');
plot(t, x(5,:), 'LineWidth',sv.LineWidth1,'Color',sv.Orange, 'DisplayName', 'Y Velocity');
plot(t, x(6,:), 'LineWidth',sv.LineWidth1,'Color',sv.Green, 'DisplayName', 'Z Velocity');
xlabel('Time [s]'); ylabel('Velocity [ft/s]');
title('6DOF Simulated Flight')
legend('Location','northwest')

f3=figure(3);
f3.Position = [100,100,800,500]; 
hold on; grid on;
plot(t(1:end-1), dxdt(4,:), 'LineWidth',sv.LineWidth1,'Color',sv.Blue, 'DisplayName', 'X Acceleration');
plot(t(1:end-1), dxdt(5,:), 'LineWidth',sv.LineWidth1,'Color',sv.Orange, 'DisplayName', 'Y Acceleration');
plot(t(1:end-1), dxdt(6,:), 'LineWidth',sv.LineWidth1,'Color',sv.Green, 'DisplayName', 'Z Acceleration');
xlabel('Time [s]'); ylabel('Acceleration [ft/s]');
title('6DOF Simulated Flight')
legend('Location','northwest')

f4=figure(4);
f4.Position = [100,100,800,500]; 
hold on; grid on;
plot(t, x(7,:)*180/pi, 'LineWidth',sv.LineWidth1,'Color',sv.Blue, 'DisplayName', 'Roll, \phi');
plot(t, x(8,:)*180/pi, 'LineWidth',sv.LineWidth1,'Color',sv.Orange, 'DisplayName', 'Pitch, \theta');
plot(t, x(9,:)*180/pi, 'LineWidth',sv.LineWidth1,'Color',sv.Green, 'DisplayName', 'Yaw, \psi');
xlabel('Time [s]'); ylabel('Euler Angles [°]');
title('6DOF Simulated Flight')
legend('Location','northwest')

f5=figure(5);
f5.Position = [100,100,800,500]; 
hold on; grid on;
plot(t, x(10,:)*180/pi, 'LineWidth',sv.LineWidth1,'Color',sv.Blue, 'DisplayName', 'Roll rate, p');
plot(t, x(11,:)*180/pi, 'LineWidth',sv.LineWidth1,'Color',sv.Orange, 'DisplayName', 'Pitch rate, q');
plot(t, x(12,:)*180/pi, 'LineWidth',sv.LineWidth1,'Color',sv.Green, 'DisplayName', 'Yaw rate, r');
xlabel('Time [s]'); ylabel('Angular Velocity [°/s]');
title('6DOF Simulated Flight')
legend('Location','northwest')

f6=figure(6);
f6.Position = [100,100,800,500]; 
hold on; grid on;
plot3(x(1,:),x(2,:),x(3,:), 'LineWidth',sv.LineWidth1,'Color',sv.Blue, 'DisplayName', 'Position');
xlabel('X [ft]'); ylabel('Y [ft]'); zlabel('Z [ft]');
title('6DOF Simulated Flight')
% legend('Location','northwest')
view([4, 2, 1.2]); % axis equal;

f7=figure(7);
f7.Position = [100,100,800,500]; 
hold on; grid on;
plot(t(1:end-1), predicted_apogee, 'LineWidth',sv.LineWidth1,'Color',sv.Blue, 'DisplayName', 'Predicted Apogee');
xlabel('Time [s]'); ylabel('Predicted Apogee [ft]');
title('6DOF Simulated Flight')
legend('Location','northwest')

f8=figure(8);
f8.Position = [100,100,800,500]; 
hold on; grid on;
plot(t, x(13,:), 'LineWidth',sv.LineWidth1,'Color',sv.Blue, 'DisplayName', 'Flap Angle');
xlabel('Time [s]'); ylabel('Flap Angle [°]');
title('6DOF Simulated Flight')
legend('Location','northwest')

f9=figure(9);
f9.Position = [100,100,800,500]; 
hold on; grid on;
plot(t(1:end-1), e, 'LineWidth',sv.LineWidth1,'Color',sv.Blue, 'DisplayName', 'Error');
xlabel('Time [s]'); ylabel('Apogee Error [ft]');
title('6DOF Simulated Flight')
legend('Location','northwest')
% -------------------------------------------------------------------------
