%% ACS vs. Recovery Flight Analysis
% Fierce battle between ACS and Recovery for the best data. Also extracting
% the scaling factor between ACS's BMP390 and the scoring altimeter,
% Recovery's Blue Raven

% Let's use the imperial system

% Author: William Teasley

% Date: 4 January 2025

clc; clear; close all;
sv = style_values(); % Load in style values for plotting
set(0, 'DefaultAxesFontName', sv.FontName);
% -------------------------------------------------------------------------

%% Import ACS and Recovery data
acs_data =  readtable("Flight Data\ACS\Subscale_Flight_2.csv");
acs_data = truncate_subscale_flight(acs_data);
recovery_data = readtable("Flight Data\Recovery\Subscale_Flight_2.csv");
recovery_data = truncate_subscale_flight_recovery(recovery_data);

%% Constants
g = 32.1740485564; % ft/s2
rho = 0.00232941; % slugs/ft3
P0 = 2063.84; % lbs/ft2
T0 = 15.04;            % Sea-level temperature in °C


%% Interpolate Data
num_points = 10000;
t = linspace(0, acs_data.Time(end), num_points);
acs_altitude_interpolated = interp1(acs_data.Time, acs_data.Altitude_AGL, t);
recovery_altitude_interpolated = interp1(recovery_data.Time, recovery_data.Altitude_AGL, t);
recovery_velocity_interpolated = interp1(recovery_data.Time, recovery_data.Velocity_Up, t);

%% Calculate Dynamic Pressure 
Q = 0.5*rho*recovery_velocity_interpolated.^2;

%% Convert Dynamic Pressure to Altitude Mismatch
L = 0.00356;          % Temperature lapse rate in °F/ft
T_ref = 518.6;        % Reference temperature in Rankine

% Calculatee Pressure from altitude data
T = 59 - L * (acs_altitude_interpolated+692); % Temperature in °F
P = P0 * ((T + 459.7) ./ T_ref).^5.256; % Pressure in lbf/ft²

% Calculate mismatch using total pressure
T = T_ref * ((P+Q) ./ P0).^(1 / 5.256) - 459.7; % Calculate temperature in °F
h = (59 - T) ./ L - 692;                     % Calculate altitude in feet

%% Find Conversion Factor
factor = 0.5*(1.007456593977684 + 1.001322284535345); % recovery_data.Apogee / acs_data.Apogee;
% factor = 1.007456593977684;
% factor = 1.001322284535345;

%% Plotting
f1=figure(1);
f1.Position = [100,100,800,500]; 
hold on; grid on;
plot(acs_data.Time, acs_data.Altitude_AGL, '--', 'LineWidth',sv.LineWidth1,'Color',sv.Red, 'DisplayName', 'ACS Altitude');
yline(acs_data.Apogee, '--', 'LineWidth',sv.LineWidth2,'Color',sv.Red, 'DisplayName', 'ACS Apogee');
xline(acs_data.t_apogee, 'LineWidth',sv.LineWidth2,'Color',sv.Red, 'HandleVisibility', 'off'); 
plot(recovery_data.Time, recovery_data.Altitude_AGL, 'LineWidth',sv.LineWidth1,'Color',sv.Blue, 'DisplayName', 'Recovery Altitude');
yline(recovery_data.Apogee, 'LineWidth',sv.LineWidth2,'Color',sv.Blue, 'DisplayName', 'Recovery Apogee');
plot(acs_data.Time, acs_data.Altitude_AGL*factor, 'LineWidth',sv.LineWidth1,'Color',sv.Red, 'DisplayName', 'Adjusted ACS Altitude');
yline(acs_data.Apogee*factor, 'LineWidth',sv.LineWidth2,'Color',sv.Red, 'DisplayName', 'Adjusted ACS Apogee');
xline(recovery_data.t_apogee, 'LineWidth',sv.LineWidth2,'Color',sv.Blue, 'HandleVisibility', 'off');
% plot(t, h, 'LineWidth',sv.LineWidth1,'Color',sv.Purple, 'DisplayName', 'ACS Altitude Adjusted for Dynamic Pressure');
legend('Location','northwest')
xlabel('Time [s]'); ylabel('Altitude [ft]');
title('Altitude')

f2=figure(2);
f2.Position = [100,100,800,500]; 
hold on; grid on;
plot(t, P, 'LineWidth',sv.LineWidth1,'Color',sv.Red, 'DisplayName', 'Measured Presh');
plot(t, Q, 'LineWidth',sv.LineWidth1,'Color',sv.Blue, 'DisplayName', 'Dynamic Presh');
plot(t, P+Q, 'LineWidth',sv.LineWidth1,'Color',sv.Purple, 'DisplayName', 'Total Presh');
xlabel('Time [s]'); ylabel('Pressure [lbf/ft^2]');
title('Pressure')
legend('Location','northwest')

