%% Launch Analysis
% This code plots launch data 

% Author: William Teasley
% Date: 30 October 2024
% Completed Individually

clc; clear; close all;
sv = style_values(); % Load in style values for plotting
set(0, 'DefaultAxesFontName', sv.FontName);
% -------------------------------------------------------------------------

%% Load in data
data = readtable("ACS Data\ACS Huntsville.csv");
dataPayload = readtable("Payload Data\BNO055Data_03292024.csv"); 
launch_data = truncate_flight(data);
[time_pitch, pitch] = truncate_payload_data(dataPayload);
clc

%% Convert data cell to variables
time = launch_data.Time;
state = launch_data.State;
servoPercentage = launch_data.ServoPercentage;
apogeePrediction = launch_data.ApogeePrediction;
altitudeFiltered = launch_data.AltitudeFiltered;
velocityFiltered = launch_data.VelocityFiltered;
accelerationFiltered = launch_data.AccelerationFiltered;
accelerationUnfilteredX = launch_data.AccelerationUnfilteredX;
accelerationUnfilteredY = launch_data.AccelerationUnfilteredY;
accelerationUnfilteredZ = launch_data.AccelerationUnfilteredZ;

zenithAngle = launch_data.ZenithAngle;
temp = launch_data.Temp;
apogeeTarget = launch_data.TargetApogee(1); 
t_apogee = launch_data.t_apogee;
g = 32.17; % ft/s2

%% Real Velocity Calculations
% Derivate based on Altitude data
n = length(time);
velocityCalculated1 = zeros(n, 1);
velocityCalculated2 = zeros(n, 1);
for i = 3:n
    velocityCalculated2(i) = (4*altitudeFiltered(i-1) - altitudeFiltered(i-2) - 3*altitudeFiltered(i))/(-2*(time(i) - time(i-1)));
end

velocityCalculated2 = smoothdata(velocityCalculated2, 'gaussian', 20);

% Integrating Acceleration data
velocityCalculatedAccel = zeros(n, 1);
velocityCalculatedAccelUnfiltered = zeros(n, 1);
altitudeCalculatedAccelUnfiltered = zeros(n,1);
for i = 2:n
    velocityCalculatedAccel(i) = velocityCalculatedAccel(i-1) + (accelerationFiltered(i))*(time(i) - time(i-1));
    velocityCalculatedAccelUnfiltered(i) = velocityCalculatedAccelUnfiltered(i-1) + (accelerationUnfilteredZ(i))*(time(i) - time(i-1));
    altitudeCalculatedAccelUnfiltered(i) = altitudeCalculatedAccelUnfiltered(i-1) + (velocityCalculatedAccelUnfiltered(i))*(time(i) - time(i-1));
end

% Interpolate data from March 29 launch
theta = interp1(time_pitch, pitch, time, "spline");
for i = 1:length(theta)
    if abs(theta(i)) > 90
        theta(i) = theta(i-1);
    end
end

vertical_velocity = velocityCalculatedAccel.*cosd(theta);
vertical_velocity2 = velocityCalculatedAccelUnfiltered.*cosd(theta);

% %% Servo Slope
% servo_slope = zeros(1, length(servoPercentage));
% for i = 1:length(servoPercentage)-1
%     servo_slope(i+1) = (servoPercentage(i+1) - servoPercentage(i))/dt ;
% end

%% Apogee Prediction Using Accelerometer Velocity
apogee_prediction_calculated = zeros(1,n);
% for i = 1:n
%     [projected_apogee(i), ~] = calculate_apogee_RK4(current_altitude, velocity, servo_angle, mass, pressure_initial, temp_initial);
% end



%% Print statements
% % [apogee, t_apogee] = max(altitudeFiltered);
% [maxServoSlope, i_servo] = max(servo_slope);
% disp("Target Apogee: " + apogee_target + " ft")
% disp("ACS Altimeter: " + apogee + " ft")
% disp("Theoretical Apogee: " + max(altitudeTheoretical) + " ft")
% disp("Max Servo Slope instructed by PI controller: " + maxServoSlope + " deg/s at " + time(i_servo) + " s")
% disp("Max Possible Servo Slope from testing:  " + 35/0.33 + " deg/s ( = 35 deg / 0.33 s)")
% disp("Flaps deployed at " + time(flap_deployment) + " s at " + altitudeFiltered(flap_deployment) + " ft")

%% Plot
f1=figure(1);
f1.Position = [100,100,800,500]; 
hold on; grid on;
title('ACS Flight')
plot(time, altitudeFiltered, 'LineWidth',sv.LineWidth1,'Color',sv.Color{1}, 'DisplayName', 'Filtered Altitude');
plot(time, altitudeCalculatedAccelUnfiltered, 'LineWidth',sv.LineWidth1,'Color',sv.Color{2}, 'DisplayName', 'Calculated Altitude');
yline(launch_data.Apogee, 'k--', 'LineWidth', sv.LineWidth2, 'DisplayName', 'Apogee');
xline(time(launch_data.ilaunch),'LineWidth',sv.LineWidth2,'Color',sv.Color{2}, 'DisplayName', 'Launch');
xline(time(launch_data.iburnout),'LineWidth',sv.LineWidth2,'Color',sv.Color{3}, 'DisplayName', 'Burnout');
xline(t_apogee,'LineWidth',sv.LineWidth2,'Color',sv.Color{4}, 'DisplayName', 'Actual Apogee');
xlabel('Time [s]'); ylabel('Altitude [ft]')
xlim([0 max(launch_data.Time)]); ylim([0 6000])
legend('Location','southeast', 'Interpreter','latex')
% axis([0 30 4800 apogee_projected(1)*3.281])

f2=figure(2);
f2.Position = [100,100,800,500]; 
hold on; grid on;
title('ACS Flight')
plot(time, velocityFiltered, 'LineWidth',sv.LineWidth1,'Color',sv.Color{1}, 'DisplayName', 'Kalman Filter Velocity');
% plot(time, velocityCalculated2, 'LineWidth',sv.LineWidth1,'Color',sv.Color{3}, 'DisplayName', 'Calculated Velocity, 2nd Order');
% plot(time, accelerationFiltered, 'LineWidth',sv.LineWidth1,'Color',sv.Color{4}, 'DisplayName', 'Kalman Filter Acceleration');
plot(time, accelerationUnfilteredZ, 'LineWidth',sv.LineWidth1,'Color',sv.Color{2}, 'DisplayName', 'Unfiltered Acceleration');
plot(time, velocityCalculatedAccel, 'LineWidth',sv.LineWidth1,'Color',sv.Color{5}, 'DisplayName', 'Calcualted Velocity by Integrating Filtered Acceleration');
plot(time, velocityCalculatedAccelUnfiltered, 'LineWidth',sv.LineWidth1,'Color',sv.Color{3}, 'DisplayName', 'Calcualted Velocity by Integrating Unfiltered Acceleration');
plot(time, vertical_velocity2, 'LineWidth',sv.LineWidth1,'Color',sv.Color{7}, 'DisplayName', 'Vertical Velocity from Raw Accel');
xline(time(launch_data.ilaunch),'LineWidth',sv.LineWidth2,'Color',sv.Color{2}, 'DisplayName', 'Launch');
xline(time(launch_data.iburnout),'LineWidth',sv.LineWidth2,'Color',sv.Color{3}, 'DisplayName', 'Burnout');
xline(t_apogee,'LineWidth',sv.LineWidth2,'Color',sv.Color{4}, 'DisplayName', 'Actual Apogee');
xlabel('Time [s]'); ylabel('Velocity [ft/s]')
xlim([0 max(launch_data.Time)]); % ylim([0 6000])
legend('Location','northeast', 'Interpreter','latex')
% axis([0 30 4800 apogee_projected(1)*3.281])


f3=figure(3);
f3.Position = [100,100,800,500]; 
hold on; grid on;
title('ACS Flight')
plot(time, apogeePrediction, 'LineWidth',sv.LineWidth1,'Color',sv.Color{1}, 'DisplayName', 'On Board Predicted Apogee');
xline(time(launch_data.ilaunch),'LineWidth',sv.LineWidth2,'Color',sv.Color{2}, 'DisplayName', 'Launch');
xline(time(launch_data.iburnout),'LineWidth',sv.LineWidth2,'Color',sv.Color{3}, 'DisplayName', 'Burnout');
xline(t_apogee,'LineWidth',sv.LineWidth2,'Color',sv.Color{4}, 'DisplayName', 'Actual Apogee');
xlabel('Time [s]'); ylabel('Altitude [ft]')
xlim([0 max(launch_data.Time)]); % ylim([0 6000])
legend('Location','northeast', 'Interpreter','latex')

% f2=figure(2);
% plot(time, apogeePrediction, '-', 'LineWidth',linewidth1,'Color',color4);
% % plot(time+dt, analyticalPrediction, '-', 'LineWidth',linewidth,'Color',color6);
% hold on
% % plot(time+dt, analyticalPrediction_with_zenith, '-', 'LineWidth',linewidth,'Color',color1);
% % plot(tt, apogee_projected_theoretical, '-', 'LineWidth',linewidth,'Color',color5);
% yline(apogee_target, 'k-', 'LineWidth', 1.2);
% % yline(apogee, 'Color',color3)
% xline(time(ilaunch),'LineWidth',linewidth2,'Color',color6);
% xline(time(iburnout),'LineWidth',linewidth2,'Color',color1);
% xline(time(t_apogee),'LineWidth',linewidth2,'Color',color2);
% % xline(time(flap_deployment), '--','Color',color6)
% grid on
% legend('Predicted Apogee', 'Target Apogee', 'Launch', 'Burnout', 'Apogee', 'Location', 'southeast','FontName', FontName, 'FontSize', fontSize); % Legend
% xlabel('Time [s]') % Add axis labels
% ylabel('Predicted Apogee [ft]')
% title('ACS Flight')
% set(findall(gcf,'type','text'), 'FontSize', fontSize, 'Color', 'k','FontName', FontName)
% f2.Position = [100,100,800,500]; % this is based on your screen and preference
% 
% % 'RK4 Prediction', 'Prediction with Zenith Angle',
% 
% f3=figure(3);
% plot(time, velocityFiltered, '-', 'LineWidth',linewidth1,'Color',color1);
% hold on
% plot(time, accelerationFiltered, '-', 'LineWidth',linewidth1,'Color',color2);
% plot(tt, velocityTheoretical, '-', 'LineWidth',linewidth1,'Color',color3);
% plot(tt, accelerationTheoretical, '-', 'LineWidth',linewidth1,'Color',color4);
% xline(time(iburnout), 'Color',color5)
% xline(time(bt), 'Color',color6)
% xline(time(flap_deployment), '--','Color',color1)
% xline(time(iovershoot), 'Color',color5)
% xline(time(t_apogee), 'Color',color4)
% grid on
% legend('Measured Filtered Velocity', 'Measured Filtered Acceleration', 'Theoretical Velocity', 'Theoretical Acceleration', 'Burnout Detection', 'Actual Burnout', 'Flap Deployment', 'Overshoot', 'Time of Apogee', 'Location','northeast') % Legend
% xlabel('t, s') % Add axis labels
% ylabel('v(t), ft/s, a(t), ft/s^2')
% title('ACS Flight')
% set(findall(gcf,'type','text'), 'FontSize', fontSize, 'Color', 'k','FontName', FontName)
% f3.Position = [100,100,800,500]; % this is based on your screen and preference
% 
f4=figure(4);
f4.Position = [100,100,800,500]; 
hold on; grid on;
title('ACS Flight')
plot(time, servoPercentage, 'LineWidth',sv.LineWidth1,'Color',sv.Color{1}, 'DisplayName', 'Servo Percentage');
xline(time(launch_data.ilaunch),'LineWidth',sv.LineWidth2,'Color',sv.Color{2}, 'DisplayName', 'Launch');
xline(time(launch_data.iburnout),'LineWidth',sv.LineWidth2,'Color',sv.Color{3}, 'DisplayName', 'Burnout');
xline(t_apogee,'LineWidth',sv.LineWidth2,'Color',sv.Color{4}, 'DisplayName', 'Actual Apogee');
xlabel('Time [s]'); ylabel('Servo Percentage')
xlim([0 max(launch_data.Time)]); % ylim([0 6000])
legend('Location','northeast', 'Interpreter','latex')

% 
% % f5=figure(5);
% % plot(time, temp, 'g-', 'LineWidth', 1.2);
% % grid on
% % legend('Temperature', 'Location','northeast') % Legend
% % xlabel('t, s') % Add axis labels
% % ylabel('Temperature')
% % title('ACS Flight')
% % f5.Position = [100,100,800,500]; % this is based on your screen and preference
% 
if isnan(zenithAngle(1))
    f6=figure(6);
    f6.Position = [100,100,800,500]; 
    hold on; grid on;
    title('ACS Flight')
    plot(time, theta, 'LineWidth',sv.LineWidth1,'Color',sv.Color{1}, 'DisplayName', 'Pitch Angle');
    xline(time(launch_data.ilaunch),'LineWidth',sv.LineWidth2,'Color',sv.Color{2}, 'DisplayName', 'Launch');
    xline(time(launch_data.iburnout),'LineWidth',sv.LineWidth2,'Color',sv.Color{3}, 'DisplayName', 'Burnout');
    xline(t_apogee,'LineWidth',sv.LineWidth2,'Color',sv.Color{4}, 'DisplayName', 'Actual Apogee');
    xlabel('Time [s]'); ylabel('Zenith Angle, \circ')
    xlim([0 max(launch_data.Time)]); % ylim([0 6000])
    legend('Location','northeast', 'Interpreter','latex')
    % axis([0 30 4800 apogee_projected(1)*3.281])

else
    f6=figure(6);
    f6.Position = [100,100,800,500]; 
    hold on; grid on;
    title('ACS Flight')
    plot(time, zenithAngle, 'LineWidth',sv.LineWidth1,'Color',sv.Color{1}, 'DisplayName', 'Zenith Angle');
    xline(time(launch_data.ilaunch),'LineWidth',sv.LineWidth2,'Color',sv.Color{2}, 'DisplayName', 'Launch');
    xline(time(launch_data.iburnout),'LineWidth',sv.LineWidth2,'Color',sv.Color{3}, 'DisplayName', 'Burnout');
    xline(t_apogee,'LineWidth',sv.LineWidth2,'Color',sv.Color{4}, 'DisplayName', 'Actual Apogee');
    legend('Location','northeast') % Legend
    xlabel('Time [s]'); ylabel('Zenith Angle, \circ')
    ylim([-100 100])
end
% 
% f7=figure(7);
% plot(time, dragCFD, 'LineWidth',linewidth1,'Color',color1);
% hold on
% plot(time(bt:end-10), dragMeasured(bt:end-10), 'LineWidth',linewidth1,'Color',color2);
% grid on
% legend('CFD Drag', 'Measured Drag', 'Location','northeast') % Legend
% xlabel('t [s]') % Add axis labels
% ylabel('Drag [lbs]')
% title('ACS Flight')
% f7.Position = [100,100,800,500]; % this is based on your screen and preference
% 
% 
