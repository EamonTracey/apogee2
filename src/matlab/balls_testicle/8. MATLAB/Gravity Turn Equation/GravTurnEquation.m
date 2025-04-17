%% GravTurnEquation.m
% This program solves the Gravity Turn Equation via a 1st Order Euler
% numerical method and calculates the zenith angle given velocity data.

% Author: Micheline Denn
% Date: 23 March 2025

clear
clc

%% Initialize Parameters
% Time
    t0 = 2.26; % s, initial time 
    tF = 18.35; % s, final time
    dt = 0.1; % time step
    t = t0:dt:tF; % initialize time vector from t0 to tF
% Approximate velocity as linear
    v0 = 666.809; % ft/s, initial velocity
    vF = 0; % ft/s, final velocity
    m_v = (v0 - vF) / (t0-tF); % slope
    b_v = vF - m_v*tF; % y-intercept
    V = m_v.*t + b_v; % ft/s, velocity
 % Zenith Angle (theta)
    theta0 = 5; % degrees, initial angle

%% GravTurnFunction
theta = GravTurnFunction(V,t,theta0,dt);

%% Plot
lw = 2; % line width

% Time vs Velocity
subplot(1,3,1)
plot(t,V,'Linewidth',lw);
title('Time vs Velocity');
xlabel('Time, s');
ylabel('Velocity, ft/s');
grid on
set(gca,'fontsize',14)
axis square

% Time vs Zenith Angle
subplot(1,3,2)
plot(t,theta,'r','Linewidth',lw);
title('Time vs Zenith Angle');
xlabel('Time, s');
ylabel('Zenith Angle, degrees');
grid on
set(gca,'fontsize',14)
axis square

% Velocity vs Zenith Angle
subplot(1,3,3)
plot(V,theta,'g','Linewidth',lw);
title('Velocity vs Zenith Angle');
xlabel('Velocity, ft/s');
ylabel('Zenith Angle, degrees');
grid on
set(gca,'fontsize',14)
axis square







%% Archive Code:
% %% Parameters
% % Gravity
% g = 32.2; % m/s^2
% 
% % Time
% t0 = 2.26; % [s] initial time 
% tF = 18.35; % [s] final time
% %tspan = [t0 tF]; % [s] span of time from t0 to tF
% dt = 0.1; % [s] time step
% time = t0:dt:tF; % [s] initialize time vector from t0 to tF
% 
% % Velocity
%     % approximate the velocity as linear
% v0 = 666.809; % m/s, initial velocity
% m_v = (666.809 - 0) / (2.23-18.35); % slope
% b_v = 0 - m_v*18.35; % y-intercept
% V = m_v.*time + b_v; % m/s, velocity
% 
% % Zenith Angle (theta)
% theta0 = 5*(pi/180); % [rad] initial theta
% theta = theta0*ones(1,length(time)); % initialize theta vector
% dtheta_dt = zeros(1,length(time)); % initialize theta derivative vector
% 
% %% 1st Order Euler
% for i=1:length(time)-1
%     dtheta_dt(i) = -g/V(i) * sin(theta(i));
%     theta(i+1) = theta(i)+dt*dtheta_dt(i);
% end
% theta = theta*180/pi;
% 
% plot(time,theta)
% 
