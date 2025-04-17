%% 1D RK4 Apogee Prediction
% This program uses an RK4 algorithm that adjusts for altitude change to
% solve the unidimensional ballistic differential equation using CFD data
% in order to predict the apogee of a rocket

% Author: William Teasley
% Date: 3 January 2025
% Completed Individually

clc; clear; close all;
sv = style_values(); % Load in style values for plotting
set(0, 'DefaultAxesFontName', sv.FontName);
% -------------------------------------------------------------------------

%% Initial Values --> CHANGE BEFORE FLIGHT
state.P0 = 2116; % lbf/ft2
state.T0 = 32.08; % F
state.m = 1.1977821; % 1.2007; % 647*0.00194256; % slugs; mass after burnout, so constant

%% Load in CFD Data
state.cfd_data =  readtable("CFD Data\CDR_CFD_2.csv");
state.thrust_data = readtable("Thrust Data\L1940_ThrustCurve.csv");
clc

%% Set Parameters 
dt = 0.01;
state.dt = dt;
state.g = 32.1740485564; % ft/s2

%% Initialize vectors
state.t = 0;
y1(1) = 0; % 1169.62; % Vertical altitude, ft
v1(1) = 0; % 551.5; % Absolute velocity, ft/s
t1(1) = 0;
i = 1;
state.i = i;
state.flap_angle = 0;


%% Calculate trajectory using RK4
Fy = @(y, v) v ;

while v1(i) > -0.001
    ky1 = Fy(y1(i), v1(i));
    kv1 = Fv(y1(i), v1(i), state);

    ky2 = Fy(y1(i)+0.5*dt*ky1, v1(i)+0.5*kv1*dt);
    kv2 = Fv(y1(i)+0.5*dt*ky1, v1(i)+0.5*kv1*dt, state);

    ky3 = Fy(y1(i)+0.5*dt*ky2, v1(i)+0.5*kv2*dt);
    kv3 = Fv(y1(i)+0.5*dt*ky2, v1(i)+0.5*kv2*dt, state);

    ky4 = Fy(y1(i)+ky3*dt, v1(i)+kv3*dt);
    kv4 = Fv(y1(i)+ky3*dt, v1(i)+kv3*dt, state);
    

    y1(i+1) = y1(i) + (1/6)*(ky1+2*ky2+2*ky3+ky4)*dt;
    v1(i+1) = v1(i) + (1/6)*(kv1+2*kv2+2*kv3+kv4)*dt;

    [a1(i+1), drag1(i+1), mach1(i+1)] = Fv(y1(i), v1(i), state);

    t1(i+1) = i*dt;
    state.t(i+1) = t1(i+1);
    state.i = i;
    i = i+1;
end

%% Initialize vectors
state.t = 0;
y2(1) = 0; % 1169.62; % Vertical altitude, ft
v2(1) = 0; % 551.5; % Absolute velocity, ft/s
t2(1) = 0;
i = 1;
state.i = i;
state.flap_angle = 0;

%% Calculate trajectory using RK4
Fy = @(y, v) v ;

while v2(i) > -0.001
    ky1 = Fy(y2(i), v2(i));
    kv1 = Fv(y2(i), v2(i), state);

    ky2 = Fy(y2(i)+0.5*dt*ky1, v2(i)+0.5*kv1*dt);
    kv2 = Fv(y2(i)+0.5*dt*ky1, v2(i)+0.5*kv1*dt, state);

    ky3 = Fy(y2(i)+0.5*dt*ky2, v2(i)+0.5*kv2*dt);
    kv3 = Fv(y2(i)+0.5*dt*ky2, v2(i)+0.5*kv2*dt, state);

    ky4 = Fy(y2(i)+ky3*dt, v2(i)+kv3*dt);
    kv4 = Fv(y2(i)+ky3*dt, v2(i)+kv3*dt, state);
    
    y2(i+1) = y2(i) + (1/6)*(ky1+2*ky2+2*ky3+ky4)*dt;
    v2(i+1) = v2(i) + (1/6)*(kv1+2*kv2+2*kv3+kv4)*dt;

    if t2(i) > 4.6366667 && t2(i) < 6.6366667
        state.flap_angle = 45;
        [a2(i+1), drag2(i+1), mach2(i+1)] = Fv(y2(i), v2(i), state);
    else
        state.flap_angle = 0;
        [a2(i+1), drag2(i+1), mach2(i+1)] = Fv(y2(i), v2(i), state);
    end

    t2(i+1) = i*dt;
    state.t(i+1) = t2(i+1);
    state.i = i;
    i = i+1;
end


%% Analyze Results
apogee = max(y1);
disp("Apogee: " + apogee + " ft")

f1=figure(1);
f1.Position = [100,100,800,500]; 
hold on; grid on;
plot(t1, y1, 'LineWidth',sv.LineWidth1,'Color',sv.Blue, 'DisplayName', 'ACS Off');
plot(t2, y2, 'LineWidth',sv.LineWidth1,'Color',sv.Red, 'DisplayName', 'ACS On');
xlabel('Time [s]'); ylabel('Altitude [ft]');
legend('Location','northwest', 'FontSize', sv.FontSize)
set(findall(gcf,'type','text'), 'FontSize', sv.FontSize, 'Color', 'k','FontName', sv.FontName)

f2=figure(2);
f2.Position = [100,100,800,500]; 
hold on; grid on;
plot(t1, v1, 'LineWidth',sv.LineWidth1,'Color',sv.Blue, 'DisplayName', 'ACS Off');
plot(t2, v2, 'LineWidth',sv.LineWidth1,'Color',sv.Red, 'DisplayName', 'ACS On');
xlabel('Time [s]'); ylabel('Velocity [ft/s]');
legend('Location','northeast', 'FontSize', sv.FontSize)
set(findall(gcf,'type','text'), 'FontSize', sv.FontSize, 'Color', 'k','FontName', sv.FontName)

f3=figure(3);
f3.Position = [100,100,800,500]; 
hold on; grid on;
plot(t1, drag1, 'LineWidth',sv.LineWidth1,'Color',sv.Blue, 'DisplayName', 'ACS Off');
plot(t2, drag2, 'LineWidth',sv.LineWidth1,'Color',sv.Red, 'DisplayName', 'ACS On');
xlabel('Time [s]'); ylabel('Drag [lbf]');
legend('Location','northeast', 'FontSize', sv.FontSize)
set(findall(gcf,'type','text'), 'FontSize', sv.FontSize, 'Color', 'k','FontName', sv.FontName)

f4=figure(4);
f4.Position = [100,100,800,500]; 
hold on; grid on;
plot(t1, mach1, 'LineWidth',sv.LineWidth1,'Color',sv.Blue, 'DisplayName', 'ACS Off');
plot(t2, mach2, 'LineWidth',sv.LineWidth1,'Color',sv.Red, 'DisplayName', 'ACS On');
xlabel('Time [s]'); ylabel('Mach Number');
legend('Location','northeast', 'FontSize', sv.FontSize)
set(findall(gcf,'type','text'), 'FontSize', sv.FontSize, 'Color', 'k','FontName', sv.FontName)

function [a, drag, mach] = Fv(y, v, state)
    %% Load in Data
    g = state.g; % ft/s^2
    t = state.t;
    mass = state.m;
    dt = state.dt;
    i = state.i;
    thrust_data = state.thrust_data;

    %% Crop Thrust Data
    time_raw = thrust_data.Time(1:72);
    motor_mass_raw = thrust_data.MotorMass(1:72)./(g*16); % Convert oz to slugs
    thrust_raw = thrust_data.Thrust(1:72)*0.224809; % Convert N to lbf
    
    %% Interpolate Thrust Curve
    t_thrust = 0:dt:time_raw(end);
    thrust = interp1(time_raw, thrust_raw, t_thrust);
    motor_mass = interp1(time_raw, motor_mass_raw, t_thrust);
    thrust(1) = thrust(2);
    motor_mass(1) = motor_mass(2);
    delay = 2;

    if t <= t_thrust(end)
        if thrust(i) < mass*g
            mass = mass + motor_mass(i) - motor_mass(end)
            a_P = 0;
        else
            mass = mass + motor_mass(i) - motor_mass(end)
            a_P = thrust(i)/mass - g;
        end
        flap_angle = 0;
    elseif t <= t_thrust(end) + delay
        a_P = -g;
        flap_angle = 0;
    else
        a_P = -g;
        flap_angle = state.flap_angle;
    end

    [drag, mach] = calculate_drag_1D(y, v, flap_angle, state);
    a = a_P - drag/mass ;
end





