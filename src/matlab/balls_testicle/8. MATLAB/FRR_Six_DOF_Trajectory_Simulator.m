%% 6 DOF Flight Simulator
% This code uses CFD data ran by Alpha Kappa Sigma to comprehensively model 
% all degrees of motion during the flight of a launch vehicle

% Let's use the imperial system

% Author: William Teasley

% Date: 12 September 2024

clc; clear; close all;
sv = style_values(); % Load in style values for plotting
set(0, 'DefaultAxesFontName', sv.FontName);
% -------------------------------------------------------------------------

%% Coordinate System Talk
% Earth Frame (E-frame):
%   X_E  = North
%   Y_E  = West
%   Z_E  = Up
%
% Body Frame (B-frame):
%   X_B  = Along rocket's axis (toward nose)
%   Y_B  = Initially West
%   Z_B  = Initially South
%
% Let’s define Euler angles to describe the orientation of the body frame relative to the Earth frame using 3-2-1 (yaw-pitch-roll) rotation sequence, i.e., rotations about Earth axes in the order:
%   Yaw (psi): Rotation about Z_E (Up)
%   Pitch (theta): Rotation about intermediate Y (after yaw)
%   Roll (phi): Rotation about final X (after yaw and pitch)
%
% This is a Z-Y-X intrinsic rotation sequence.

%% Environmental Conditions
environment_inish.wind_profile = "sharp";
environment_inish.P0 = 2116; % lbf/ft2
environment_inish.T0 = 32.08; % °F
environment_inish.u_inf = 24.9333; % ft/s
environment_inish.wind_direction = 0; % wind direction counterclockwise from North, (°)
environment_inish.altitude0_ASL = 692; % altitude of launch rail ASL, ft
environment = calculate_environment(environment_inish, 0);

%% Launch Vehicle
vehicle_name = "katie";
vehicle = calculate_vehicle(vehicle_name); 

% Values that might change on launch day
global g
g = 32.17405;
vehicle.m = (686.1/16)/g; % total vehicle mass with full motor, slugs
vehicle.x_cm = 56.435/12; % distance from nose to center of mass, ft
vehicle.angle_launch_rail = -5; % launch rail angle, (°)
vehicle.angle_azimuth = 0; % launch rail heading, (°)

%% Rocket Motor
motor_name = "L1940X";
motor = calculate_motor(motor_name);

%% Initial State Conditions
mass(1) = vehicle.m + motor.full_weight ;
position(:, 1) = [0; 0; 0]; % xyz position in Global Earth coordinates relative to launch pad, ft
velocity(:, 1) = [0; 0; 0]; % velocity in Global Earth coordinates, ft/s
accel(:, 1) = [0; 0; 0]; % acceleration in Global Earth coordinates, ft/s2

eulers(:, 1) = [vehicle.angle_launch_rail*pi/180; 0; 0]; % Euler angles, rad (theta, psi, phi)
angular_vel(:, 1) = [0; 0; 0]; % angular velocity of body frame, rad/s
flap_angle(1) = 0; % drag flap angle, (°)
aoa(1) = 0; % angle of attack, (°)

%% Simulate launch with RK4
dt = 0.01; % s
t = dt:dt:20; % s
t_start = 2.5;
t_delay = 2;

% Without ACS
x1(:,1) = [position(:, 1); velocity(:, 1); eulers(:, 1); angular_vel(:, 1); flap_angle(1); mass(1)]; % initial conditions
dxdt1(:,1) = zeros(length(x1), 1);

for i = 1:length(t)-1

    ky1 = calculate_derivatives(x1(:, i), vehicle, motor, environment, t(i));
    ky2 = calculate_derivatives((x1(:, i)+0.5*dt*ky1.vec), vehicle, motor, environment, t(i));
    ky3 = calculate_derivatives((x1(:, i)+0.5*dt*ky2.vec), vehicle, motor, environment, t(i));
    ky4 = calculate_derivatives((x1(:, i)+ky3.vec*dt), vehicle, motor, environment, t(i));    

    dxdt1(:, i) = (1/6)*(ky1.vec+2*ky2.vec+2*ky3.vec+ky4.vec);
    x1(:, i+1) = x1(:, i) + dxdt1(:, i).*dt;
end


% With ACS
x2(:,1) = [position(:, 1); velocity(:, 1); eulers(:, 1); angular_vel(:, 1); flap_angle(1); mass(1)]; % initial conditions
dxdt2(:,1) = zeros(length(x2), 1);

for i = 1:length(t)-1
    if t(i) > (t_start + t_delay) && t(i) < (t_start + t_delay + t_delay)
        x2(13, i) = 45;
    else
        x2(13, i) = 0;
    end

    ky1 = calculate_derivatives(x2(:, i), vehicle, motor, environment, t(i));
    ky2 = calculate_derivatives((x2(:, i)+0.5*dt*ky1.vec), vehicle, motor, environment, t(i));
    ky3 = calculate_derivatives((x2(:, i)+0.5*dt*ky2.vec), vehicle, motor, environment, t(i));
    ky4 = calculate_derivatives((x2(:, i)+ky3.vec*dt), vehicle, motor, environment, t(i));    

    dxdt2(:, i) = (1/6)*(ky1.vec+2*ky2.vec+2*ky3.vec+ky4.vec);
    x2(:, i+1) = x2(:, i) + dxdt2(:, i).*dt;
end

%% Calculate drag
for i = 1:length(t)
    environment1 = calculate_environment(environment_inish, x1(3,i));
    a1 = environment1.a;
    mach1 = x1(6,i)/a1;
    drag1(i) = vehicle.calculate_drag(x1(13,i), 0, mach1);

    environment2 = calculate_environment(environment_inish, x2(3,i));
    a2 = environment2.a;
    mach2 = x2(6,i)/a2;
    drag2(i) = vehicle.calculate_drag(x2(13,i), 0, mach2);
end

%% Compare to real launch data
raw_acs_data =  readtable("Flight Data\Fullscale_Flight_1.csv");
acs_data = truncate_fullscale_flight(raw_acs_data);

% -------------------------------------------------------------------------
%% Printing and plotting
fprintf("Apogee without ACS: %.1f ft \n", max(x1(3,:)))
fprintf("Apogee with ACS: %.1f ft \n", max(x2(3,:)))
fprintf("Total apogee reduction: %.1f ft \n", abs(max(x1(3,:)) - max(x2(3,:))))


f1=figure(1);
f1.Position = [100,100,800,500]; 
hold on; grid on;
plot(t, x1(1,:), 'LineWidth',sv.LineWidth1,'Color',sv.Blue, 'DisplayName', 'X Position');
plot(t, x1(2,:), 'LineWidth',sv.LineWidth1,'Color',sv.Orange, 'DisplayName', 'Y Position');
plot(t, x1(3,:), 'LineWidth',sv.LineWidth1,'Color',sv.Green, 'DisplayName', 'Z Position');
xlabel('Time [s]'); ylabel('Position [ft]');
title('6DOF Simulated Flight')
legend('Location','northwest')

f2=figure(2);
f2.Position = [100,100,800,500]; 
hold on; grid on;
plot(t, x1(4,:), 'LineWidth',sv.LineWidth1,'Color',sv.Blue, 'DisplayName', 'X Velocity');
plot(t, x1(5,:), 'LineWidth',sv.LineWidth1,'Color',sv.Orange, 'DisplayName', 'Y Velocity');
plot(t, x1(6,:), 'LineWidth',sv.LineWidth1,'Color',sv.Green, 'DisplayName', 'Z Velocity');
xlabel('Time [s]'); ylabel('Velocity [ft/s]');
title('6DOF Simulated Flight')
legend('Location','northwest')

f3=figure(3);
f3.Position = [100,100,800,500]; 
hold on; grid on;
plot(t(1:end-1), dxdt2(4,:), 'LineWidth',sv.LineWidth1,'Color',sv.Blue, 'DisplayName', 'X Acceleration');
plot(t(1:end-1), dxdt2(5,:), 'LineWidth',sv.LineWidth1,'Color',sv.Orange, 'DisplayName', 'Y Acceleration');
plot(t(1:end-1), dxdt2(6,:), 'LineWidth',sv.LineWidth1,'Color',sv.Green, 'DisplayName', 'Z Acceleration');
xlabel('Time [s]'); ylabel('Acceleration [ft/s]');
title('6DOF Simulated Flight')
legend('Location','northwest')

f4=figure(4);
f4.Position = [100,100,800,500]; 
hold on; grid on;
plot(t, x1(7,:)*180/pi, 'LineWidth',sv.LineWidth1,'Color',sv.Blue, 'DisplayName', 'Pitch, \theta');
plot(t, x1(8,:)*180/pi, 'LineWidth',sv.LineWidth1,'Color',sv.Orange, 'DisplayName', 'Roll, \phi');
plot(t, x1(9,:)*180/pi, 'LineWidth',sv.LineWidth1,'Color',sv.Green, 'DisplayName', 'Yaw, \psi');
xlabel('Time [s]'); ylabel('Euler Angles [°]');
title('6DOF Simulated Flight')
legend('Location','northwest')

f5=figure(5);
f5.Position = [100,100,800,500]; 
hold on; grid on;
plot(t, x1(10,:), 'LineWidth',sv.LineWidth1,'Color',sv.Blue, 'DisplayName', 'Pitch rate, p');
plot(t, x1(11,:), 'LineWidth',sv.LineWidth1,'Color',sv.Orange, 'DisplayName', 'Roll rate, q');
plot(t, x1(12,:), 'LineWidth',sv.LineWidth1,'Color',sv.Green, 'DisplayName', 'Yaw rate, r');
xlabel('Time [s]'); ylabel('Angular Velocity [°/s]');
title('6DOF Simulated Flight')
legend('Location','northwest')

f6=figure(6);
f6.Position = [100,100,800,500]; 
hold on; grid on;
plot(t, x1(3,:), 'LineWidth',sv.LineWidth1,'Color',sv.Green, 'DisplayName', 'ACS Off');
plot(t, x2(3,:), 'LineWidth',sv.LineWidth1,'Color',sv.Orange, 'DisplayName', 'ACS On');
plot(acs_data.Time, acs_data.Altitude_AGL, 'LineWidth',sv.LineWidth1,'Color',sv.Blue, 'DisplayName', 'Real altitude');
xlabel('Time [s]'); ylabel('Altitude [ft]');
legend('Location','northwest', 'FontSize', sv.FontSize)
set(findall(gcf,'type','text'), 'FontSize', sv.FontSize, 'Color', 'k','FontName', sv.FontName)

f7=figure(7);
f7.Position = [100,100,800,500]; 
hold on; grid on;
plot(t, drag1, 'LineWidth',sv.LineWidth1,'Color',sv.Green, 'DisplayName', 'ACS Off');
plot(t, drag2, 'LineWidth',sv.LineWidth1,'Color',sv.Orange, 'DisplayName', 'ACS On');
xlabel('Time [s]'); ylabel('Drag [lbf]');
legend('Location','northeast', 'FontSize', sv.FontSize)
set(findall(gcf,'type','text'), 'FontSize', sv.FontSize, 'Color', 'k','FontName', sv.FontName)

saveas(f6, "frr_acs_altitude_effect", 'png')
saveas(f7, "frr_acs_drag_effect", 'png')

% -------------------------------------------------------------------------
