%% Wind Gust Calc
% Luke Ricci, Maisy Denn, etc.

% Calculates wind gust vector at every point in flight

% Property of AKS
clear all
clc
close all

% Rocket "constants"
I = 187.59; %lb-ft^2
I = I/32.17; % convert to slug-ft^2
CP2CG = 1.303; % ft
global g
g = 32.17;
gamma = 1.4;
R = 1717;
T = 490; % R

% Load data
data = readtable('Flight Data\ACS\Fullscale_Flight_1.csv');
data = truncate_fullscale_flight(data);
quartdata = load("goodquats.mat");

t = data.Time;
% calculating time step
dt = mean(t(2:end)-t(1:end-1));

q = quartdata.quaternion_calculated; % w,x,y,z
a_rel = [data.Acceleration_X_ICM20649,data.Acceleration_Y_ICM20649,data.Acceleration_Z_ICM20649]'; % x,y,z

%% Global Acceleration

for i = 1:length(t)
    a_glo(i,:) = quat2rotm(q(i,:))*(a_rel(:,i)) + [0;0;-32.17];
end

a_rel = a_rel';

v_glo(1,:) = [0,0,0];
v_rel(1,:) = [0,0,0];
x_glo(1,:) = [0,0,0];
x_rel(1,:) = [0,0,0];

% integrating
for i = 1:length(t)-1
    v_glo(i+1,:) = v_glo(i,:) + 0.5*dt*(a_glo(i,:)+a_glo(i+1,:));
    v_rel(i+1,:) = v_rel(i,:) + 0.5*dt*(a_rel(i,:)+a_rel(i+1,:));

    x_glo(i+1,:) = x_glo(i,:) + 0.5*dt*(v_glo(i,:)+v_glo(i+1,:));
    x_rel(i+1,:) = x_rel(i,:) + 0.5*dt*(v_rel(i,:)+v_rel(i+1,:));
end


%% Orientation
gyro = [data.Gyro_X_BNO085,data.Gyro_Y_BNO085,data.Gyro_Z_BNO085];

% Calculates angular acceleration 'alfa'
cutoff = 5; % filters out oscillations over 5 Hz

alfa(1,:) = [0,0,0]; % initialize

% derivative filter
for i = 2:length(t)
    alfa(i,:) = (cutoff*gyro(i,:) - cutoff*gyro(i-1,:) + alfa(i-1,:))/(1+cutoff*dt);
end

% torque = angular acceleration * moment of inertia (from OR)
tau = alfa.*I;

%  torque = normal force * moment arm, solved for normal force
F = tau/CP2CG;

% broken up into components
F_x = F(:,1);
F_y = F(:,2);

% speed of sound
a = (gamma*R*T)^0.5;

% M = v magnitude divided by speed of sound
v_mag = (v_glo(:,1).^2+v_glo(:,2).^2+v_glo(:,3).^2).^0.5;
M = v_mag/a;

% from data
flap_angle = data.Servo_Angle;

% vehicle = calculate_vehicle('katie');
% [AoA_x, AoA_y] = vehicle.calculate_drag(flap_angle,M,F_x,F_y);
% 
% w_x = v_mag.*tand(AoA_x);
% w_y = v_mag.*tand(AoA_y);
% 
% w_mag = (w_x.^2 + w_y.^2).^0.5;
% 
% w_smooth = smoothdata(w_mag,'gaussian',30);

% alt = data.Altitude_AGL;
% 
% figure(1)
% yyaxis left
% plot(t,w_mag,'Linewidth',1.5)
% ylabel('Wind Speed, {\it w} [ft/s]')
% yyaxis right
% plot(t,alt,'Linewidth',1.5)
% grid on
% xlabel('Time, {\it t} [sec]')
% ylabel('Altitude, a [ft]')
% set(gca,'fontsize',16,'fontname','times new roman')