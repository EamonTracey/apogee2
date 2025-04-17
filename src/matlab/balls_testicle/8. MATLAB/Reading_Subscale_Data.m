%% Reading ACS Data
% This code rads ACS Subscale Data

% Let's use the imperial system

% Author: William Teasley

% Date: 21 November 2024

clc; clear; close all;
sv = style_values(); % Load in style values for plotting
set(0, 'DefaultAxesFontName', sv.FontName);
% -------------------------------------------------------------------------

%% Start of script
addpath('quaternion_library');      % include quaternion library

%% Import and plot sensor data
data = readtable("ACS Data\ACS 20241124141335.csv");

%% Remove outliers
launch_data = truncate_subscale_flight(data);

Time = data.Time;

% BNO085 Data
AccelerationBNO085X = data.Acceleration_X_BNO085;
AccelerationBNO085Y = data.Acceleration_Z_BNO085;
AccelerationBNO085Z = data.Acceleration_Y_BNO085;

MagneticBNO085X = data.Magnetic_X_BNO085;
MagneticBNO085Y = data.Magnetic_Z_BNO085;
MagneticBNO085Z = data.Magnetic_Y_BNO085;

GyroBNO085X = data.Gyro_X_BNO085;
GyroBNO085Y = data.Gyro_Z_BNO085;
GyroBNO085Z = data.Gyro_Y_BNO085;

% ICM20649 Data (no magnetometer)
AccelerationICM20649X = -1*data.Acceleration_Y_ICM20649;
AccelerationICM20649Y = data.Acceleration_Z_ICM20649;
AccelerationICM20649Z = data.Acceleration_X_ICM20649;

GyroICM20649X = -1*data.Gyro_Y_ICM20649;
GyroICM20649Y = data.Gyro_Z_ICM20649;
GyroICM20649Z = data.Gyro_X_ICM20649;

% Choose which data to use
AccelerometerBNO085 = [AccelerationBNO085X, AccelerationBNO085Y, AccelerationBNO085Z];
MagnetometerBNO085 = [MagneticBNO085X, MagneticBNO085Y, MagneticBNO085Z];
GyroscopeBNO085 = [GyroBNO085X, GyroBNO085Y, GyroBNO085Z];

AccelerometerICM20649 = [AccelerationICM20649X, AccelerationICM20649Y, AccelerationICM20649Z];
GyroscopeICM20649 = [GyroICM20649X, GyroICM20649Y, GyroICM20649Z];

% Plotting
f1=figure(1);
f1.Position = [100,100,800,500]; 
axis(1) = subplot(3,1,1);
hold on; grid on;
plot(Time, GyroBNO085X, 'LineWidth',sv.LineWidth1,'Color',sv.Red, 'DisplayName', 'Gyro X');
plot(Time, GyroBNO085Y, 'LineWidth',sv.LineWidth1,'Color',sv.Green, 'DisplayName', 'Gyro Y');
plot(Time, GyroBNO085Z, 'LineWidth',sv.LineWidth1,'Color',sv.Blue, 'DisplayName', 'Gyro Z');
xlabel('Time [s]'); ylabel('Angular Velocity [rad/s]');
title('Sensor Data')
legend('Location','northwest')

axis(2) = subplot(3,1,2);
hold on; grid on;
plot(Time, AccelerationBNO085X, 'LineWidth',sv.LineWidth1,'Color',sv.Red, 'DisplayName', 'Acceleration X');
plot(Time, AccelerationBNO085Y, 'LineWidth',sv.LineWidth1,'Color',sv.Green, 'DisplayName', 'Acceleration Y');
plot(Time, AccelerationBNO085Z, 'LineWidth',sv.LineWidth1,'Color',sv.Blue, 'DisplayName', 'Acceleration Z');
xlabel('Time [s]'); ylabel('Angular Velocity [rad/s]');
title('Sensor Data')
legend('Location','northwest')


figure('Name', 'Sensor Data');

hold off;
axis(2) = subplot(3,1,2);
hold on;
plot(Time, AccelerationBNO085X, 'r');
plot(Time, AccelerationBNO085Y, 'g');
plot(Time, AccelerationBNO085Z, 'b');
legend('X', 'Y', 'Z');
xlabel('Time (s)');
ylabel('Acceleration (g)');
title('Accelerometer');
hold off;
axis(3) = subplot(3,1,3);
hold on;
plot(Time, MagneticBNO085X, 'r');
plot(Time, MagneticBNO085Y, 'g');
plot(Time, MagneticBNO085Z, 'b');
legend('X', 'Y', 'Z');
xlabel('Time (s)');
ylabel('Flux (G)');
title('Magnetometer');
hold off;
linkaxes(axis, 'x');

%% Process sensor data through algorithm

% AHRS = MadgwickAHRS('SamplePeriod', 1/256, 'Beta', 0.1);
AHRS = MahonyAHRS('SamplePeriod', 1/256, 'Kp', 0.25, 'Ki', 0.01);

quaternion = zeros(length(Time), 4);
for t = 1:length(Time)
    AHRS.Update(GyroscopeBNO085(t,:), AccelerometerICM20649(t,:), MagnetometerBNO085(t,:));	% gyroscope units must be radians
    quaternion(t, :) = AHRS.Quaternion;
end

%% Plot algorithm output as Euler angles
% The first and third Euler angles in the sequence (phi and psi) become
% unreliable when the middle angles of the sequence (theta) approaches ±90
% degrees. This problem commonly referred to as Gimbal Lock.
% See: http://en.wikipedia.org/wiki/Gimbal_lock

euler = quatern2euler(quaternConj(quaternion)) * (180/pi);	% use conjugate for sensor frame relative to Earth and convert to degrees.

figure('Name', 'Euler Angles');
hold on;
plot(Time, euler(:,1), 'r');
plot(Time, euler(:,2), 'g');
plot(Time, euler(:,3), 'b');
title('Euler angles');
xlabel('Time (s)');
ylabel('Angle (deg)');
legend('\phi', '\theta', '\psi');
hold off;

%% End of script