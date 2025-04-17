function [theta] = GravTurnFunction(V,t,theta0,dt)
%% Gravity Turn Equation Function
% This program solves the Gravity Turn Equation via a 1st Order Euler
% numerical method and calculates the zenith angle given velocity data.

% Inputs:
    % V      - velocity vector, ft/s
    % t      - time vector, s
    % theta0 - intital zenith angle, degrees
    % dt     - time step
% Outputs:
    % theta  - zenith angle, degrees

%% Parameters
% Gravity
g = 32.2; % ft/s^2

% Time
t0 = t(1); % initial time, s
tF = t(end); % final time, s
time = t0:dt:tF; % initialize time vector from t0 to tF

% Zenith Angle (theta)
theta = theta0*ones(1,length(time)); % initialize theta vector, degrees
dtheta_dt = zeros(1,length(time)); % initialize theta derivative vector

%% 1st Order Euler Method
for i = 1:length(time)-1
    dtheta_dt(i) = -g/V(i) * sind(theta(i));
    theta(i+1) = theta(i)+dt*dtheta_dt(i); % degrees
end





