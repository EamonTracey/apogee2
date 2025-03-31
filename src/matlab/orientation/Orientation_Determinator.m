%% Orientation Determination
% This code plots ACS Orientation

% Let's use the imperial system

% Author: William Teasley

% Date: 15 January 2025

clc; clear; close all;
sv = style_values(); % Load in style values for plotting
set(0, 'DefaultAxesFontName', sv.FontName);
addpath('Quaternions');      % include quaternion library
addpath('Quaternions\quaternion_library');      % include quaternion library
% -------------------------------------------------------------------------

%% Import and plot sensor data
data =  readtable("Flight Data\ACS\Fullscale_Flight_1.csv");
launch_data = truncate_fullscale_flight(data);
% launch_data = readtable("Flight Data\Testing\pitchtestnumbertwo.csv");

global g
g = 32.1740485564; 

%% Create Data Vectors
AccelerometerBNO085 = [launch_data.Acceleration_X_BNO085, launch_data.Acceleration_Y_BNO085, launch_data.Acceleration_Z_BNO085]; % shouldn't thrust be positive and gravity negative?
MagnetometerBNO085 = [launch_data.Magnetic_X_BNO085, launch_data.Magnetic_Y_BNO085, launch_data.Magnetic_Z_BNO085];
GyroscopeBNO085 = [launch_data.Gyro_X_BNO085, launch_data.Gyro_Y_BNO085, launch_data.Gyro_Z_BNO085];
QuaternionBNO085 = [launch_data.Quaternion_W_BNO085 launch_data.Quaternion_X_BNO085 launch_data.Quaternion_Y_BNO085 launch_data.Quaternion_Z_BNO085];

AccelerometerICM20649 = [launch_data.Acceleration_X_ICM20649, launch_data.Acceleration_Y_ICM20649, launch_data.Acceleration_Z_ICM20649];
GyroscopeICM20649 = [launch_data.Gyro_X_ICM20649, launch_data.Gyro_Y_ICM20649, launch_data.Gyro_Z_ICM20649];

%% Convert Sensor coordinates to Rocket Body coordinates
Quaternion_Rocket_Measured = QuaternionBNO085; % quaternProd(qBNO2Rocket, QuaternionBNO085_Sensor);

%% Calculate Zenith Angle From Quaternions
% f7 = figure(7);
% f7.Position = [100,100,800,500]; 
% hold on; grid on;
% for i = 1:10:round(length(launch_data.Time)/2)
%     plot_quaternion_direction(QuaternionBNO085(i,:),7)
% end

%% Calculate Quaternion using gyro data
quaternion_calculated(1,:) = QuaternionBNO085(1,:);
for i = 1:length(launch_data.Time)-1
    quat = quaternion_calculated(i,:);
    gyro = GyroscopeBNO085(i,:);
    dt = launch_data.Time(i+1) - launch_data.Time(i);

    % quaternion_calculated(i+1,:) = updateQuaternion(quaternion_calculated(i,:), GyroscopeBNO085(i,:), dt);
    quaternion_calculated(i+1,:) = madgwickFilter(quaternion_calculated(i,:), GyroscopeBNO085(i,:), AccelerometerBNO085(i,:), dt, 0.025); % use updateQuaternion, mahonyFilter, or madgwickFilter
end

%% Convert from Rocket Coordinates to Earth Coordinates
euler_measured = quat2eul(Quaternion_Rocket_Measured,'XYZ') * (180/pi);	% use conjugate for sensor frame relative to Earth and convert to degrees.
euler_calculated = quat2eul(quaternion_calculated,'XYZ') * (180/pi);	% use conjugate for sensor frame relative to Earth and convert to degrees.


%% Calculate Zenith Angle
v_up_earth = [0 0 0 1];
zenith_measured = zeros(1, length(launch_data.Time));

for i = 1:length(launch_data.Time)
    v_rot_earth_measured = quatmultiply(QuaternionBNO085(i,:), quatmultiply(v_up_earth, quaternConj(QuaternionBNO085(i,:))));
    zenith_measured(i) = acosd(v_rot_earth_measured(:,4));

    v_rot_earth_calculated = quatmultiply(quaternion_calculated(i,:), quatmultiply(v_up_earth, quaternConj(quaternion_calculated(i,:))));
    zenith_calculated(i) = acosd(v_rot_earth_calculated(:,4));
end




%% Plotting
% f1=figure(1);
% f1.Position = [100,100,800,500]; 
% hold on; grid on;
% plot(launch_data.Time, launch_data.Altitude_AGL, 'LineWidth',sv.LineWidth1,'Color',sv.Red, 'DisplayName', 'Altitude BMP930');
% xlabel('Time [s]'); ylabel('Altitude [ft]');
% title('Flight Data')
% legend('Location','northwest')

f2=figure(2);
f2.Position = [100,100,800,500]; 
hold on; grid on;
plot(launch_data.Time, AccelerometerBNO085(:,1), 'LineWidth',sv.LineWidth1,'Color',sv.Blue, 'DisplayName', 'BNO085 X');
plot(launch_data.Time, AccelerometerBNO085(:,2), 'LineWidth',sv.LineWidth1,'Color',sv.Red, 'DisplayName', 'BNO085 Y');
plot(launch_data.Time, AccelerometerBNO085(:,3), 'LineWidth',sv.LineWidth1,'Color',sv.Purple, 'DisplayName', 'BNO085 Z');
% plot(launch_data.Time, AccelerometerICM20649(:,1), '--', 'LineWidth',sv.LineWidth1,'Color',sv.Blue, 'DisplayName', 'ICM20649 X');
% plot(launch_data.Time, AccelerometerICM20649(:,2), '--', 'LineWidth',sv.LineWidth1,'Color',sv.Red, 'DisplayName', 'ICM20649 Y');
% plot(launch_data.Time, AccelerometerICM20649(:,3), '--', 'LineWidth',sv.LineWidth1,'Color',sv.Purple, 'DisplayName', 'ICM20649 Z');
xlabel('Time [s]'); ylabel('Acceleration [ft/s^2]');
title('Flight Data')
legend('Location','northwest')

f3=figure(3);
f3.Position = [100,100,800,500];
title('Flight Data')
axis(3) = subplot(4,1,1);
hold on; grid on;
plot(launch_data.Time, abs(Quaternion_Rocket_Measured(:,1)), 'LineWidth',sv.LineWidth1,'Color',sv.Blue, 'DisplayName', 'Measured Q_W');
plot(launch_data.Time, abs(quaternion_calculated(:,1)), 'LineWidth',sv.LineWidth1,'Color',sv.Red, 'DisplayName', 'Calculated Q_W');
xlabel('Time [s]'); ylabel('Quaternion');
legend('Location','northeast')
axis(3) = subplot(4,1,2);
hold on; grid on;
plot(launch_data.Time, abs(Quaternion_Rocket_Measured(:,2)), 'LineWidth',sv.LineWidth1,'Color',sv.Blue, 'DisplayName', 'Measured Q_X');
plot(launch_data.Time, abs(quaternion_calculated(:,2)), 'LineWidth',sv.LineWidth1,'Color',sv.Red, 'DisplayName', 'Calculated Q_X');
xlabel('Time [s]'); ylabel('Quaternion');
legend('Location','northeast')
axis(3) = subplot(4,1,3);
hold on; grid on;
plot(launch_data.Time, abs(Quaternion_Rocket_Measured(:,3)), 'LineWidth',sv.LineWidth1,'Color',sv.Blue, 'DisplayName', 'Measured Q_Y');
plot(launch_data.Time, abs(quaternion_calculated(:,3)), 'LineWidth',sv.LineWidth1,'Color',sv.Red, 'DisplayName', 'Calculated Q_Y');
xlabel('Time [s]'); ylabel('Quaternion');
legend('Location','northeast')
axis(3) = subplot(4,1,4);
hold on; grid on;
plot(launch_data.Time, abs(Quaternion_Rocket_Measured(:,4)), 'LineWidth',sv.LineWidth1,'Color',sv.Blue, 'DisplayName', 'Measured Q_Z');
plot(launch_data.Time, abs(quaternion_calculated(:,4)), 'LineWidth',sv.LineWidth1,'Color',sv.Red, 'DisplayName', 'Calculated Q_Z');
xlabel('Time [s]'); ylabel('Quaternion');
legend('Location','northeast')

f4=figure(4);
f4.Position = [100,100,800,500];
title('Flight Data')
axis(4) = subplot(4,1,1);
hold on; grid on;
plot(launch_data.Time, euler_measured(:,1), 'LineWidth',sv.LineWidth1,'Color',sv.Blue, 'DisplayName', 'Measured Roll');
plot(launch_data.Time, euler_calculated(:,1), 'LineWidth',sv.LineWidth1,'Color',sv.Red, 'DisplayName', 'Calculated Roll');
xlabel('Time [s]'); ylabel('Roll Angle, \phi');
legend('Location','northeast')
axis(4) = subplot(4,1,2);
hold on; grid on;
plot(launch_data.Time, euler_measured(:,2), 'LineWidth',sv.LineWidth1,'Color',sv.Blue, 'DisplayName', 'Measured Pitch');
plot(launch_data.Time, euler_calculated(:,2), 'LineWidth',sv.LineWidth1,'Color',sv.Red, 'DisplayName', 'Calculated Pitch');
xlabel('Time [s]'); ylabel('Pitch Angle, \theta');
legend('Location','northeast')
axis(4) = subplot(4,1,3);
hold on; grid on;
plot(launch_data.Time, euler_measured(:,3), 'LineWidth',sv.LineWidth1,'Color',sv.Blue, 'DisplayName', 'Measured Yaw');
plot(launch_data.Time, euler_calculated(:,3), 'LineWidth',sv.LineWidth1,'Color',sv.Red, 'DisplayName', 'Calculated Yaw');
xlabel('Time [s]'); ylabel('Yaw Angle, \psi');
legend('Location','northeast')
axis(4) = subplot(4,1,4);
hold on; grid on;
plot(launch_data.Time, zenith_measured, 'LineWidth',sv.LineWidth1,'Color',sv.Blue, 'DisplayName', 'Measured Zenith');
plot(launch_data.Time, zenith_calculated, 'LineWidth',sv.LineWidth1,'Color',sv.Red, 'DisplayName', 'Calculated Zenith');
xlabel('Time [s]'); ylabel('Zenith Angle, Z');
legend('Location','northeast')

f5=figure(5);
f5.Position = [100,100,800,500];
hold on; grid on;
plot(launch_data.Time, zenith_measured, 'LineWidth',sv.LineWidth1,'Color',sv.Blue, 'DisplayName', 'Measured Zenith Angle');
plot(launch_data.Time, zenith_calculated, 'LineWidth',sv.LineWidth1,'Color',sv.Red, 'DisplayName', 'Calculated Zenith Angle');
xlabel('Time [s]'); ylabel('Zenith Angle, Z');
legend('Location','northeast')

f6=figure(6);
f6.Position = [100,100,800,500];
hold on; grid on;
plot(launch_data.Time, zenith_measured-zenith_calculated, 'LineWidth',sv.LineWidth1,'Color',sv.Blue, 'DisplayName', 'Error');
xlabel('Time [s]'); ylabel('Zenith Angle, Z');
legend('Location','northeast')

% f7=figure(7);
% f7.Position = [100,100,800,500]; 
% hold on; grid on;
% plot(launch_data.Time, vecnorm(AccelerometerBNO085, 2, 2), 'LineWidth',sv.LineWidth1,'Color',sv.Blue, 'DisplayName', 'Average Acceleration');
% xlabel('Time [s]'); ylabel('Acceleration [ft/s^2]');
% title('Flight Data')
% legend('Location','northwest')