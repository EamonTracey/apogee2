%% 6 DOF Flight Simulator
% This code uses the equations from Aerospace Dynamics with Paul Rumbach 

% Let's use the metric system

% Author: William Teasley

% Date: 21 November 2024

clc; clear; close all;
sv = style_values(); % Load in style values for plotting
set(0, 'DefaultAxesFontName', sv.FontName);
% -------------------------------------------------------------------------
Cm_alpha = 5.978;
rho = 0.00238;
L = 4;
n = 1000;
w = ones(1, n); 

Q = 0.5*rho*V.^2;
M_alpha = Cm_alpha*(Q*S*L/Iy);
M_q = Cm_q*(Q*S*(L^2)/(2*Iy));

CD = @(theta) CD0 + CL(theta)^2 ;

thetadd = @(theta, thetad) M_alpha*(theta+psi) + M_q*thetad;
ax = @(V, theta) 0.5*rho*(CD(theta) *S*V.^2);
ay = @(V, theta) 0.5*rho*(theta*CL_alpha*S*V.^2);