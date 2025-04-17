%% 6 DOF State Space Flight Simulator
% This code uses the equations from 'State-Space Modeling of a Rocket for 
% Optimal Control System Design' to model all degrees of motion during flight 
% using state space

% Let's use the imperial system

% Author: William Teasley

% Date: 25 December 2024

clc; clear; close all;
sv = style_values(); % Load in style values for plotting
set(0, 'DefaultAxesFontName', sv.FontName);
% -------------------------------------------------------------------------

%% Atmospheric Conditions
g = 32.17; % g in ft/s2
rho = 0.00237717; % air density in kg/m3

%% Physical Constants
rocket.d = 0.5; % diameter of body tube in ft
rocket.S = 0.25*pi*(rocket.d^2); % cross section area in m2
rocket.m = (109/16)/g; % mass of rocket in slugs

rocket.x_cm = 26.871/12; % Distance from nose to center of mass
rocket.x_cp = 35.185/12; % Distance from nose to center of pressure
rocket.Ix = 0.069; % lbft2
rocket.Iy = 0.069;
rocket.Iz = 10.799;

generate_coefficients(rocket)

% Define Longitudinal Dynamics in the given format
% State vector: [u, w, q, theta, z]'
A_long = [ ...
    xu, xw, xq, x_theta, 0; ...
    zu, zw, zq, z_theta, 0; ...
    mu, mw, mq, m_theta, 0; ...
    0,  0,  1,      0,    0; ...
    0,  0,  0,      0,    0  ...
];

B_long = [ ...
    x_eta, x_tau; ...
    z_eta, z_tau; ...
    m_eta, m_tau; ...
    0,      0; ...
    0,      0  ...
];

% Define Lateral Dynamics in the given format
% State vector: [v, p, r, phi, psi]'
A_lat = [ ...
    yb, yp, yr, y_phi, y_psi; ...
    lb, lp, lr, l_phi, l_psi; ...
    nb, np, nr, n_phi, n_psi; ...
    0,  1,  0,      0,     0; ...
    0,  0,  1,      0,     0  ...
];

B_lat = [ ...
    y_xi, y_zeta; ...
    l_xi, l_zeta; ...
    n_xi, n_zeta; ...
    0,      0; ...
    0,      0  ...
];

% Define Control Inputs
u_long = [eta; tau]; % Replace with specific control inputs
u_lat = [xi; zeta]; % Replace with specific control inputs

% Define State-Space Systems
sys_long = ss(A_long, B_long, eye(size(A_long)), zeros(size(B_long)));
sys_lat = ss(A_lat, B_lat, eye(size(A_lat)), zeros(size(B_lat)));

% Example Simulation
% Define initial states for longitudinal and lateral dynamics
x_long_0 = [0; 0; 0; 0; 0]; % Initial longitudinal states
x_lat_0 = [0; 0; 0; 0; 0];  % Initial lateral states

% Time vector for simulation
t = 0:0.01:10; % Simulate for 10 seconds

% Simulate longitudinal and lateral responses
y_long = lsim(sys_long, u_long * ones(size(t)), t, x_long_0);
y_lat = lsim(sys_lat, u_lat * ones(size(t)), t, x_lat_0);

% Plot results
figure;
subplot(2,1,1);
plot(t, y_long);
title('Longitudinal Dynamics Response');
xlabel('Time (s)'); ylabel('States');
legend('u', 'w', 'q', 'theta', 'z');

subplot(2,1,2);
plot(t, y_lat);
title('Lateral Dynamics Response');
xlabel('Time (s)'); ylabel('States');
legend('v', 'p', 'r', 'phi', 'psi');
