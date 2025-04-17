% ExampleScript.m
%
% This script demonstrates use of the MadgwickAHRS and MahonyAHRS algorithm
% classes with example data. ExampleData.mat contains calibrated gyroscope,
% accelerometer and magnetometer data logged from an AHRS device (x-IMU)
% while it was sequentially rotated from 0 degrees, to +90 degree and then
% to -90 degrees around the X, Y and Z axis.  The script first plots the
% example sensor data, then processes the data through the algorithm and
% plots the output as Euler angles.
%
% Note that the Euler angle plot shows erratic behaviour in phi and psi
% when theta approaches ±90 degrees. This due to a singularity in the Euler
% angle sequence known as 'Gimbal lock'.  This issue does not exist for a
% quaternion or rotation matrix representation.
%
% Date          Author          Notes
% 28/09/2011    SOH Madgwick    Initial release
% 13/04/2012    SOH Madgwick    deg2rad function no longer used
% 06/11/2012    Seb Madgwick    radian to degrees calculation corrected

%% Start of script

addpath('quaternion_library');      % include quaternion library
close all;                          % close all figures
clear;                              % clear all variables
clc;                                % clear the command terminal

%% Import and plot sensor data
data = readtable("ACS Data\ACS Data 3_29_24.csv");
launch_data = truncate_flight(data);

%% Remove outliers
launch_data_clean_struct = rmfield(launch_data, {'State', 'ilaunch', 'iburnout', 'iovershoot', 'iapogee', 'Apogee', 't_apogee', 'iflap', 'TargetApogee'});
launch_data_clean_table = struct2table(launch_data_clean_struct);
launch_data = rmoutliers(launch_data_clean_table, "movmedian", 20);

time = launch_data.Time;
servoPercentage = launch_data.ServoPercentage;
apogeePrediction = launch_data.ApogeePrediction;
altitudeFiltered = launch_data.AltitudeFiltered;
velocityFiltered = launch_data.VelocityFiltered;
accelerationFiltered = launch_data.AccelerationFiltered;

AccelerationUnfilteredX = launch_data.AccelerationUnfilteredX;
AccelerationUnfilteredY = launch_data.AccelerationUnfilteredY;
AccelerationUnfilteredZ = launch_data.AccelerationUnfilteredZ;

MagneticX = launch_data.MagneticX;
MagneticY = launch_data.MagneticY;
MagneticZ = launch_data.MagneticZ;

GyroX = launch_data.GyroX;
GyroY = launch_data.GyroY;
GyroZ = launch_data.GyroZ;

Accelerometer = [AccelerationUnfilteredX, AccelerationUnfilteredY, AccelerationUnfilteredZ];
Magnetometer = [MagneticX, MagneticY, MagneticZ];
Gyroscope = [GyroX, GyroY, GyroZ];

figure('Name', 'Sensor Data');
axis(1) = subplot(3,1,1);
hold on;
plot(time, GyroX, 'r');
plot(time, GyroY, 'g');
plot(time, GyroZ, 'b');
legend('X', 'Y', 'Z');
xlabel('Time (s)');
ylabel('Angular rate (deg/s)');
title('Gyroscope');
hold off;
axis(2) = subplot(3,1,2);
hold on;
plot(time, AccelerationUnfilteredX, 'r');
plot(time, AccelerationUnfilteredY, 'g');
plot(time, AccelerationUnfilteredZ, 'b');
legend('X', 'Y', 'Z');
xlabel('Time (s)');
ylabel('Acceleration (g)');
title('Accelerometer');
hold off;
axis(3) = subplot(3,1,3);
hold on;
plot(time, MagneticX, 'r');
plot(time, MagneticY, 'g');
plot(time, MagneticZ, 'b');
legend('X', 'Y', 'Z');
xlabel('Time (s)');
ylabel('Flux (G)');
title('Magnetometer');
hold off;
linkaxes(axis, 'x');

%% Process sensor data through algorithm

% AHRS = MadgwickAHRS('SamplePeriod', 1/256, 'Beta', 0.1);
AHRS = MahonyAHRS('SamplePeriod', 1/256, 'Kp', 0.5, 'Ki', 0.1);

quaternion = zeros(length(time), 4);
for t = 1:length(time)
    AHRS.Update(Gyroscope(t,:), Accelerometer(t,:), Magnetometer(t,:));	% gyroscope units must be radians
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
plot(time, euler(:,1), 'r');
plot(time, euler(:,2), 'g');
plot(time, euler(:,3), 'b');
title('Euler angles');
xlabel('Time (s)');
ylabel('Angle (deg)');
legend('\phi', '\theta', '\psi');
hold off;

%% End of script