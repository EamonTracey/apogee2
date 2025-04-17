%% Apogee Prediction Model
% 1D apogee prediction model
% v6 update: use fullscale thrust curve

% Author: Peter Verges
% Date: 1/8/2025

clear
clc

% Target Apogee: 5300ft (~1615.44m)

% Drag force parameters
Cd = .497; % drag coefficient
m = 581.68*0.0283495; % post-lauch mass(kg)
g = 9.81; % m/s^2
A = pi*(6.112*0.0254*0.5)^2; % m^2

degrees = 0;
theta = degrees*(pi/180); % zenith angle (radians)

mass_noMotor = 510*0.0283495;

t0 = 0;
tf = 20;

% initial conditions
t = t0;
h0 = 0.5; % starting altitude
hd0 = 0; % starting velocity
h1 = h0; h2 = hd0;

% motor thrust data
load('FullscaleMotor.mat')

launching_time = 2.3090; %sec
launching_mass = MotorMassoz*0.0283495; %kg

% using NASA's Earth Atmosphere model for air density

T0 = 15.04 - 0.00649*h1; % Temperature
P0 = 101.29 * ((T0 + 273.1)/288.08)^5.256; % Pressure

p0 = P0 / (0.2869*(T0 + 273.1)); % Air density

% Numerical integration during launch
i = 2;

while  Times(i) < launching_time

    % T,P,p at each point
    T = 15.04 - 0.00649*h1(i-1);
    P = 101.29 * ((T + 273.1)/288.08)^5.256; 
    p = P / (0.2869*(T + 273.1));

    dt = Times(i)-Times(i-1);
    t(i) = t(i-1) + dt; % time step

    % RK4
    % QUESTION: for mass and thrust, do we use i or i-1?

    k11 = dt*(h2(i-1));
    k21 = dt*((ThrustN(i-1)*cos(theta)-(mass_noMotor + launching_mass(i-1))*g - 0.5*p*Cd*A*(h2(i-1)^2)*cos(theta))*(1/(mass_noMotor + launching_mass(i-1))));
    k12 = dt*(h2(i-1) + 0.5*k21);
    k22 = dt*((ThrustN(i-1)*cos(theta)-(mass_noMotor + launching_mass(i-1))*g - 0.5*p*Cd*A*((h2(i-1)+0.5*k21)^2)*cos(theta))*(1/(mass_noMotor + launching_mass(i-1))));
    k13 = dt*(h2(i-1) + 0.5*k22);
    k23 = dt*((ThrustN(i-1)*cos(theta)-(mass_noMotor + launching_mass(i-1))*g - 0.5*p*Cd*A*((h2(i-1)+0.5*k22)^2)*cos(theta))*(1/(mass_noMotor + launching_mass(i-1))));
    k14 = dt*(h2(i-1) + k23);
    k24 = dt*((ThrustN(i-1)*cos(theta)-(mass_noMotor + launching_mass(i-1))*g - 0.5*p*Cd*A*((h2(i-1)+k23)^2)*cos(theta))*(1/(mass_noMotor + launching_mass(i-1))));

    h1(i) = h1(i-1) + (1/6)*(k11 + 2*(k12 + k13) + k14)*cos(theta);
    h2(i) = h2(i-1) + (1/6)*(k21 + 2*(k22 + k23) + k24);

    i = i+1;
end

% Post-launch numerical integration until the velocity (h2) is <= 0 (i.e. apogee has been reached)

dt = 0.01;

while h2(i-1) > 0

    % T,P,p at each point
    T = 15.04 - 0.00649*h1(i-1);
    P = 101.29 * ((T + 273.1)/288.08)^5.256; 
    p = P / (0.2869*(T + 273.1));

    t(i) = t(i-1) + dt; % time step

    % RK4
    k11 = dt*(h2(i-1));
    k21 = dt*((-m*g - 0.5*p*Cd*A*(h2(i-1)^2)*cos(theta))*(1/m));
    k12 = dt*(h2(i-1) + 0.5*k21);
    k22 = dt*((-m*g - 0.5*p*Cd*A*((h2(i-1)+0.5*k21)^2)*cos(theta))*(1/m));
    k13 = dt*(h2(i-1) + 0.5*k22);
    k23 = dt*((-m*g - 0.5*p*Cd*A*((h2(i-1)+0.5*k22)^2)*cos(theta))*(1/m));
    k14 = dt*(h2(i-1) + k23);
    k24 = dt*((-m*g - 0.5*p*Cd*A*((h2(i-1)+k23)^2)*cos(theta))*(1/m));

    h1(i) = h1(i-1) + (1/6)*(k11 + 2*(k12 + k13) + k14)*cos(theta);
    h2(i) = h2(i-1) + (1/6)*(k21 + 2*(k22 + k23) + k24);

    i = i+1;
end

% Graph numerical integration

disp("Projected Apogee: " + num2str(h1(i-1)*3.28084) + "ft / " + num2str(h1(i-1)) + "m")

figure
plot(t,h1*3.28084,LineWidth=2)
grid on
xlabel("time, t (s)")
ylabel('altitude, h (ft)')
title('Fullscale Apogee Prediction Model - RK4')
set(gca, 'Fontsize', 20)



