%% ApogeeCalc1D.m

% Micheline Denn :)
% 19 Febuary 2025

% clear all; clc; close all
% sv = style_values(); % Load in style values for plotting
% set(0, 'DefaultAxesFontName', sv.FontName);

function [apogee, drag] = ApogeeCalc1D(y, v, servo_angle, mass_ab, P0, T0, cfd_data, thrust_data)
% P0 = 2136.00;           % initial pressure, lbf/ft2
% T0 = 75;                % initial temperature, F
% mass_ab = 1.213315217;     % mass after burnout (constant), slugs
% 
% y = 827;             % altitude at burnout, ft
% v = 650;             % velocity at burnout, ft/s
% a_ab = -55;             % acceleration at burnout, ft/s^2

% Function predicts the apogee of the rocket using an RK4 algorithm
    % Inputs:
    %   y           -   current altitude, ft
    %   v           -   current velocity, ft/s
    %   servo_angle -   angle of the servo
    %   mass_ab     -   mass after burnout (constant), slugs
    %   P0          -   initial pressure, lbf/ft^2
    %   T0          -   initial temperature, F
    %   cfd_data    -   imported CFD data
    %   thrust_data -   imported thrust data
    % Outputs:
    %   apogee      -   apogee of rocket
    %   drag        -   drag of rocket

%% Initial Values:
state.P0 = P0;      % Initial pressure inputted into state struct, lbf/ft2
state.T0 = T0;      % Initial temperature inputted into state struct, F
state.m = mass_ab;  % Mass inputted into state struct, slugs

%% Load CFD & Thrust Data:
state.cfd_data =  cfd_data;             % CFD data
state.thrust_data = thrust_data;        % Thrust Curve
clc

%% Set Parameters:
dt = 0.01;                % time step
state.dt = dt;
state.g = 32.1740485564;  % accel. due to gravity, ft/s^2

%% Initialize Vectors:
state.t = 0;                        % time, s
y1(1) = y;                          % initialize vertical altitude vector, ft
v1(1) = v;                          % intitialize absolute velocity, ft/s
t1(1) = 0;
i = 1;                              % initialize iteration
state.i = i;
state.flap_angle = servo_angle;    % servo angle

%% RK4 Trajectory Prediction:
Fy = @(y, v) v ;

while v1(i) > -0.001
%for j = 1:28
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

%% Apogee Prediction
apogee = max(y1);
drag = max(drag1);

end

%% Fv Function:
function [a, drag, mach] = Fv(y, v, state)
    % Inputs:
    %   y       -     altitude, ft
    %   v       -     velocity, ft/s
    %   state   -     state struct
    % Outputs:
    %   a       -     acceleration, ft/s^2
    %   drag    -     drag
    %   mach    -     Mach number

    %% Load in Data:
    g = state.g;                     % accel. due to gravity, ft/s^2
    t = state.t;                     % time
    mass = state.m;                  % mass
    dt = state.dt;                   % time step
    i = state.i;                     % iteration
    thrust_data = state.thrust_data; % Thrust curve data

    %% Crop & Manipulate Thrust Data:
    time_raw = thrust_data.Time(1:72);
    motor_mass_raw = thrust_data.MotorMass(1:72)./(g*16); % Convert oz to slugs
    thrust_raw = thrust_data.Thrust(1:72)*0.224809; % Convert N to lbf
    
    %% Interpolate Thrust Curve:
    t_thrust = 0:dt:time_raw(end);
    thrust = interp1(time_raw, thrust_raw, t_thrust);
    motor_mass = interp1(time_raw, motor_mass_raw, t_thrust);
    thrust(1) = thrust(2);
    motor_mass(1) = motor_mass(2);
    delay = 2;

    if t <= t_thrust(end)
        if thrust(i) < mass*g
            mass = mass - (motor_mass(1) - motor_mass(i));
            a_P = 0;
        else
            mass = mass - (motor_mass(1) - motor_mass(i));
            a_P = thrust(i)/mass - g;
        end
        flap_angle = 0;
    elseif t <= t_thrust(end) + delay
        mass = mass - (motor_mass(1) - motor_mass(end));
        a_P = -g;
        flap_angle = 0;
    else
        mass = mass - (motor_mass(1) - motor_mass(end));
        a_P = -g;
        flap_angle = state.flap_angle;
    end

    [drag, mach] = calculate_drag_1D(y, v, flap_angle, state); % call drag calculation function
    a = a_P - drag/mass ;
end % end of Fv function

%end % end of apogee prediction function

