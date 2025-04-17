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
global g
g = 32.17405;

environment_inish.wind_profile = "sharp";
environment_inish.P0 = 2114.7133; % lbf/ft2
environment_inish.T0 = 32; % °F
environment_inish.u_inf = 20.5333; % ft/s
environment_inish.wind_direction = 0; % wind direction counterclockwise from North, (°)
environment_inish.altitude0_ASL = 692; % altitude of launch rail ASL, ft
environment = calculate_environment(environment_inish, 0);

%% Launch Vehicle
vehicle_name = "katie";
vehicle = calculate_vehicle(vehicle_name); 

% Values that might change on launch day
vehicle.m = (641.1/16)/g; % total vehicle mass with full motor, slugs
vehicle.x_cm = 55.5/12; % distance from nose to center of mass, ft
vehicle.angle_launch_rail = 7; % launch rail angle, (°)
vehicle.angle_azimuth = 0; % launch rail heading, counterclockwise from North (°)

%% Rocket Motor
motor_name = "L1940X";
motor = calculate_motor(motor_name);

%% Initial State Conditions
mass(1) = 0; % vehicle.m;
position(:, 1) = [0; 0; 0]; % xyz position in Global Earth coordinates relative to launch pad, ft
velocity(:, 1) = [0; 0; 0]; % velocity in Global Earth coordinates, ft/s
accel(:, 1) = [0; 0; 0]; % acceleration in Global Earth coordinates, ft/s2

eulers(:, 1) = [0; vehicle.angle_launch_rail*pi/180; vehicle.angle_azimuth*pi/180]; % Euler angles, rad (phi, theta, psi)
angular_vel(:, 1) = [0; 0; 0]; % angular velocity of body frame, rad/s
flap_angle(1) = 0; % drag flap angle, (°)
aoa(1) = 0; % angle of attack, (°)
m(1) = vehicle.m;
thrust(1) = 0;
a(1) = environment.a;

axial_drag(1) = 0;
normal_force(1) = 0;
F_axial(1) = 0;

%% Simulate launch with RK4
dt = 0.05; % s
t = dt:dt:20; % s
t_start = 4.3667-2;
t_delay = 2;

x(:,1) = [position(:, 1); velocity(:, 1); eulers(:, 1); angular_vel(:, 1); flap_angle(1); mass(1)]; % initial conditions
dxdt(:,1) = zeros(length(x), 1);

for i = 1:length(t)-1
    if t(i) < t_start 
        x(13, i) = 0;
    else
        a = (45 - 22) / (12.89114)^2;
        x(13, i) = 22 + a * (t(i)-t_start).^2;
    end
    % if t(i) > (t_start + t_delay) && t(i) < (t_start + t_delay + t_delay)
    %     x(13, i) = 40;
    % else
    %     x(13, i) = 0;
    % end
    ky1 = calculate_derivatives(x(:, i), vehicle, motor, environment, t(i));
    ky2 = calculate_derivatives((x(:, i)+0.5*dt*ky1.vec), vehicle, motor, environment, t(i));
    ky3 = calculate_derivatives((x(:, i)+0.5*dt*ky2.vec), vehicle, motor, environment, t(i));
    ky4 = calculate_derivatives((x(:, i)+ky3.vec*dt), vehicle, motor, environment, t(i));    

    dxdt(:, i) = (1/6)*(ky1.vec+2*ky2.vec+2*ky3.vec+ky4.vec);
    x(:, i+1) = x(:, i) + dxdt(:, i).*dt;

    % calculate forces for plotting purposes
    a(i+1) = (1/6)*(ky1.a+2*ky2.a+2*ky3.a+ky4.a);
    mach = x(6,i)/a(i+1);
    aoa(i+1) = (1/6)*(ky1.aoa+2*ky2.aoa+2*ky3.aoa+ky4.aoa);
    force_sign = sign(aoa(i+1));

    [axial_drag(i+1), normal] = vehicle.calculate_drag(0, abs(ky4.aoa), mach);
    normal_force(i+1) = force_sign*normal;
    m(i+1) = (1/6)*(ky1.m+2*ky2.m+2*ky3.m+ky4.m);
    thrust(i+1) = (1/6)*(ky1.thrust+2*ky2.thrust+2*ky3.thrust+ky4.thrust);
    F_axial(i+1) = (1/6)*(ky1.F_axial+2*ky2.F_axial+2*ky3.F_axial+ky4.F_axial);
end

% -------------------------------------------------------------------------


%% Compare to real launch data
raw_acs_data =  readtable("Flight Data\ACS\Fullscale_Flight_2.csv");
acs_data = truncate_fullscale_flight(raw_acs_data);

% -------------------------------------------------------------------------
%% Printing and plotting
apogee = max(x(3,:));
fprintf("6DOF Simulated Apogee: %.1f ft \n", apogee)
fprintf("Actual Apogee: %.1f ft \n", acs_data.Apogee);

f1=figure(1);
f1.Position = [100,100,800,500]; 
hold on; grid on;
plot(t, x(1,:), 'LineWidth',sv.LineWidth1,'Color',sv.Blue, 'DisplayName', 'X Position');
plot(t, x(2,:), 'LineWidth',sv.LineWidth1,'Color',sv.Orange, 'DisplayName', 'Y Position');
plot(t, x(3,:), 'LineWidth',sv.LineWidth1,'Color',sv.Green, 'DisplayName', 'Z Position');
plot(acs_data.Time-2, acs_data.Altitude, 'LineWidth',sv.LineWidth1,'Color',sv.Blue, 'DisplayName', 'Real altitude');
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
plot(t, axial_drag, 'LineWidth',sv.LineWidth1,'Color',sv.Orange, 'DisplayName', 'Axial Drag');
xlabel('Time [s]'); ylabel('Axial Drag [lbf]');
title('6DOF Simulated Flight')
legend('Location','northwest')

f7=figure(7);
f7.Position = [100,100,800,500]; 
hold on; grid on;
plot(t, normal_force, 'LineWidth',sv.LineWidth1,'Color',sv.Blue, 'DisplayName', 'Normal Force');
xlabel('Time [s]'); ylabel('Normal Force [lbf]');
title('6DOF Simulated Flight')
legend('Location','northwest')

f8=figure(8);
f8.Position = [100,100,800,500]; 
hold on; grid on;
plot(t, aoa, 'LineWidth',sv.LineWidth1,'Color',sv.Red, 'DisplayName', 'Angle of Attack');
xlabel('Time [s]'); ylabel('Angle of Attack, \alpha [°]');
title('6DOF Simulated Flight')
legend('Location','northwest')

f9=figure(9);
f9.Position = [100,100,800,500]; 
hold on; grid on;
plot(t, m, 'LineWidth',sv.LineWidth1,'Color',sv.Red, 'DisplayName', 'Mass');
xlabel('Time [s]'); ylabel('Mass [slugs]');
title('6DOF Simulated Flight')
legend('Location','northwest')

f10=figure(10);
f10.Position = [100,100,800,500]; 
hold on; grid on;
plot(t, thrust, 'LineWidth',sv.LineWidth1,'Color',sv.Red, 'DisplayName', 'Thrust');
xlabel('Time [s]'); ylabel('Thrust [lbf]');
title('6DOF Simulated Flight')
legend('Location','northwest')

f11=figure(11);
f11.Position = [100,100,800,500]; 
hold on; grid on;
plot(t, a, 'LineWidth',sv.LineWidth1,'Color',sv.Red, 'DisplayName', 'Speed of Sound');
xlabel('Time [s]'); ylabel('Speed of Sound, a [ft/s]');
title('6DOF Simulated Flight')
legend('Location','northwest')

f12=figure(12);
f12.Position = [100,100,800,500]; 
hold on; grid on;
plot(t, F_axial, 'LineWidth',sv.LineWidth1,'Color',sv.Red, 'DisplayName', 'Axial Drag');
xlabel('Time [s]'); ylabel('Drag, [lbf]');
title('6DOF Simulated Flight')
legend('Location','northwest')

f13=figure(13);
f13.Position = [100,100,800,500]; 
hold on; grid on;
plot3(x(1,:),x(2,:),x(3,:), 'LineWidth',sv.LineWidth1,'Color',sv.Blue, 'DisplayName', 'Position');
quiver3(0,0,apogee+50, 0.75*x(1,end)*cosd(environment.wind_direction), 0.75*x(2,end)*sind(environment.wind_direction), 0, 'LineWidth',sv.LineWidth1,'Color',sv.Orange, 'DisplayName', 'Wind')
xlabel('X [ft]'); ylabel('Y [ft]'); zlabel('Z [ft]');
title('6DOF Simulated Flight')
% legend('Location','northwest')
view([4, 2, 1.2]); axis equal;




% -------------------------------------------------------------------------
