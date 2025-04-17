%% 6 DOF Flight Simulator
% This code uses the equations from 'State-Space Modeling of a Rocket for 
% Optimal Control System Design' to model all degrees of motion during flight 

% Let's use the imperial system

% Author: William Teasley

% Date: 12 September 2024

clc; clear; close all;
sv = style_values(); % Load in style values for plotting
set(0, 'DefaultAxesFontName', sv.FontName);
% -------------------------------------------------------------------------

%% Atmospheric Conditions
g = 32.17; % g in ft/s2
rho = 0.00237717; % air density in slugs/ft3

%% Physical Constants
d = 0.5; % diameter of body tube in ft
S = 0.25*pi*(d^2); % cross section area in ft2
m = (109/16)/g; % mass of rocket in slugs

x_cm = 26.871/12; % Distance from nose to center of mass
x_cp = 35.185/12; % Distance from nose to center of pressure
Ix = 10.799; % lbft2
Iy = 0.069; % lbft2
Iz = 0.069; % lbft2

T = 130; % Thrust, in lbs

%% Aerodynamic Coefficients
Cmq = -5; % pitch damping derivative w.r.t. pitch rate, -5 to -20 1/rad
Cma = -1.5;  % slope of pitching moment coefficient w.r.t. angle of attack, -1.0 to -2.5 1/rad
Cnr = -0.5; % yaw damping derivative w.r.t. yaw rate, -0.5 to -5.0 1/rad
Cnb = 0; % yaw damping derivative w.r.t. sideslip rate, 0.1 to 1.0 1/rad
Clp = 0; % roll damping derivative, -0.1 to -0.5 1/rad

Cma_dot = 1; % pitch damping derivative w.r.t. angle of attack rate, 0.5 to 2.0 1/rad
Cnb_dot = 0; % slope of yawing moment coefficient w.r.t. angle of sideslip, -0.1 to 0.1 

CA = 0.44; % axial drag coefficient CD, find out from CFD
CN = 0.5; % I think this comes from fin lift forces

%% Initial Values
% Wind profile in Earth coordinates
delta_ue(1) = 0;
delta_ve(1) = 0; % (add wind profile here)
delta_we(1) = 0;
delta_winde(:,1) = [delta_ue(1); delta_ve(1); delta_we(1)];

% Rocket velocity in body coordinates(ft/s)
ub(1) = 0; % x-axis velocity = side slip (0)  
vb(1) = 0; % y-axis velocity = side slip 
wb(1) = 0; % z-axis velocity = longitudinal
Vb(:,1) = [ub(1); vb(1); wb(1)];
VM(1) = sqrt(sumsqr([ub(1), vb(1), wb(1)])); % Total velocity
V_rel_b(1) = sqrt(sumsqr([ub(1), vb(1), wb(1)])); % Total velocity

% Position (ft)
x(1) = 0; % altitude
y(1) = 0; % east
z(1) = 0; % north

% Angular rates (rad/s)
p(1) = 0; % roll rate
q(1) = 0; % pitch rate
r(1) = 0; % yaw rate

% Euler angles (rad)
theta(1) = 0; % pitch angle
phi(1) = 0; % roll angle
psi(1) = 0; % yaw angle

dt = 0.01;
t = 0;
i = 1;
maxit = 1e5;

Fgxe = 0;
Fgye = 0;
Fgze = -m*g;
Fge = [Fgxe; Fgye; Fgze];

LP = 0;
MP = 0;
NP = 0;

%% Transformation Matrices
R_x = [ 1,  0,  0;
        0,  0,  1;
        0, -1,  0];

R_y = [ 0,  0, -1;
        0,  1,  0;
        1,  0,  0];

R_z = [ 0,  1,  0;
       -1,  0,  0;
        0,  0,  1];

%% Simulation
while ub(i) >= 0 && i < maxit    
    %% Aerodynamic forces on rocket in body coordinates
    if abs(VM(i)) < 1e-6
        CNy = 0;
        CNz = 0;
    else
        CNy = CN*vb(i)/V_rel_b(i); % coefficient for normal force in y-axis, 0.5 to 2.0 --> was there a typo in the book?
        CNz = CN*wb(i)/V_rel_b(i); % coefficient for normal force in z-axis, 0.1 to 0.3
    end

    FAxb = -0.5.*rho.*CA.*S.*V_rel_b(i).^2;  % axial drag
    FAyb = -0.5.*rho.*CNy.*S.*V_rel_b(i).^2;  % side slip drag
    FAzb = -0.5.*rho.*CNz.*S.*V_rel_b(i).^2;  % side slip drag
    FAb(:,i) = [FAxb; FAyb; FAzb];

    %% Propulsion Forces on rocket in body coordinates
    % Get thrust profile in future
    FPyb = 0;
    FPzb = 0;
    if t < 1.2
        FPxb = T;
    else
        FPxb = 0;
    end
    FPb = [FPxb; FPyb; FPzb];

    %% Convert gravity forces from Earth coordinates to body coordinates
    % Transfer matrix from body to Earth coordinates based on Euler angles
    Tb2e{i} = [cos(theta(i))*cos(psi(i)), sin(phi(i))*sin(theta(i))*cos(psi(i)) - cos(phi(i))*sin(psi(i)), cos(phi(i))*sin(theta(i))*cos(psi(i)) + sin(phi(i))*sin(psi(i)); ...
            cos(theta(i))*sin(psi(i)), sin(phi(i))*sin(theta(i))*sin(psi(i)) + cos(phi(i))*cos(psi(i)), cos(phi(i))*sin(theta(i))*sin(psi(i)) - sin(phi(i))*cos(psi(i)); ...
            -sin(theta(i)),            sin(phi(i))*cos(theta(i)),                                       cos(phi(i))*cos(theta(i))                                         ] * R_y;
    
    % Transfer matrix from Earth to body coordinates
    Te2b{i} = inv(Tb2e{i});

    % Convert gravity from Earth to body coordinates
    Fgb(:,i) = Te2b{i}*Fge;
    Fgxb = Fgb(1,i);
    Fgyb = Fgb(2,i);
    Fgzb = Fgb(3,i);
    
    %% Calculate angle of attack
    if abs(ub(i)) < 1e-6
        alpha(i) = 0;
        alpha_t(i) = 0;
    else
        alpha(i) = atan(wb(i) / ub(i));
        alpha_t(i) = atan(sqrt(vb(i)^2 + wb(i)^2) / ub(i));
    end

    if abs(V_rel_b(i)) < 1e-6
        beta = 0;               % Sideslip angle (β)
        phia = 0;               % Roll angle (φ)
        Cmref(i) = 0;
        Cnref = 0;
        Cl(i) = 0;
        Cm(i) = 0;
        Cn(i) = 0;

    else
        beta = asin(vb(i) / V_rel_b(i));                             % Sideslip angle (β)
        phia = atan(vb(i) / wb(i));                              % Roll angle (φ)

        % Moment calculations
        Cmref(i) = Cma * alpha(i); % Reference pitching moment coefficient; -0.2 to 0.2
        Cnref = Cnb * beta; % Reference yawing moment coefficient; -0.1 to 0.1

        Cl(i) = (d / (2 * V_rel_b(i))) * (Clp * p(i)); % Roll moment coefficient
        Cm(i) = Cmref(i) + CNz * (x_cm - x_cp) / d + (d / (2 * V_rel_b(i))) * (Cmq + Cma_dot) * q(i); % Pitching moment coefficient
        Cn(i) = Cnref + CNy * (x_cm - x_cp) / d + (d / (2 * V_rel_b(i))) * (Cnr + Cnb_dot) * r(i); % Yawing moment coefficient
    end

    
    LA(i) = 0.5 * rho * V_rel_b(i)^2 * Cl(i) * S * d; % Aerodynamic roll moment
    MA(i) = 0.5 * rho * V_rel_b(i)^2 * Cm(i) * S * d; % Aerodynamic pitching moment
    NA(i) = 0.5 * rho * V_rel_b(i)^2 * Cn(i) * S * d; % Aerodynamic yawing moment

    %% Calculate Derivatives
    udotb = ((FAxb + FPxb + Fgxb)/m) - (q(i)*wb(i) - r(i)*vb(i)); % ft/s2
    vdotb(i) = ((FAyb + FPyb + Fgyb)/m) - (r(i)*ub(i) - p(i)*wb(i)); % ft/s2
    wdotb = ((FAzb + FPzb + Fgzb)/m) - (p(i)*vb(i) - q(i)*ub(i)); % ft/s2

    pdot = (LA(i) + LP - q(i)*r(i)*(Iz - Iy))/Ix; % rad/s2
    qdot = (MA(i) + MP - r(i)*p(i)*(Ix - Iz))/Iy; % rad/s2
    rdot = (NA(i) + NP - p(i)*q(i)*(Iy - Ix))/Iz; % rad/s2

    phi_dot = p(i) + (q(i) * sin(phi(i)) + r(i) * cos(phi(i))) * tan(theta(i)); % Roll angle rate
    theta_dot = q(i) * cos(phi(i)) - r(i) * sin(phi(i));                 % Pitch angle rate
    psi_dot(i) = (q(i) * sin(phi(i)) + r(i) * cos(phi(i))) / cos(theta(i));    % Yaw angle rate

    %% Convert motion to Earth Coordinates
    Ve(:,i) = Tb2e{i}*Vb(:,i); % Velocity vector in earth coordinates
    xdot(i) = Ve(1,i);
    ydot(i) = Ve(2,i);
    zdot(i) = Ve(3,i); % Vertical speed

    %% Update values
    % Angular rates in body reference frame
    p(i+1) = p(i) + pdot*dt; % roll rate
    q(i+1) = q(i) + qdot*dt; % pitch rate
    r(i+1) = r(i) + rdot*dt; % yaw rate

    % Euler angles in Earth reference frame
    phi(i+1) = phi(i) + phi_dot*dt; % roll angle
    theta(i+1) = theta(i) + theta_dot*dt; % pitch angle
    psi(i+1) = psi(i) + psi_dot(i)*dt; % yaw angle

    % Rocket coordinates in Earth reference frame
    x(i+1) = x(i) + xdot(i)*dt; 
    y(i+1) = y(i) + ydot(i)*dt;
    z(i+1) = z(i) + zdot(i)*dt; % Altitude

    % Wind profile in Earth reference frame
    [u_wind_e, v_wind_e, w_wind_e] = wind_profile(z(i+1));
    wind_e = [u_wind_e; v_wind_e; w_wind_e];
    wind_b = Te2b{i}*wind_e;
    % Wind profile in body reference frame 
    ub(i+1) = ub(i) + wind_b(1) + udotb*dt;
    vb(i+1) = vb(i) + wind_b(2) + vdotb(i)*dt;
    wb(i+1) = wb(i) + wind_b(3) + wdotb*dt;
    Vb(:,i+1) = [ub(i+1); vb(i+1); wb(i+1)];

    VM(i+1) = sqrt(sumsqr([ub(i+1), vb(i+1), wb(i+1)]));
    
    V_rel_b(i+1) = norm(Vb(:,i) - wind_b);

    t = t + dt;
    i = i+1;
end

time = linspace(0, t, i);
% -------------------------------------------------------------------------

% Plotting
f1=figure(1);
f1.Position = [100,100,800,500]; 
hold on; grid on;
plot(time, VM, 'LineWidth',sv.LineWidth1,'Color',sv.Blues{1}, 'DisplayName', 'Total Velocity');
plot(time, ub, 'LineWidth',sv.LineWidth1,'Color',sv.Blue, 'DisplayName', 'Axial Velocity');
plot(time(1:end-1), zdot, 'LineWidth',sv.LineWidth1,'Color',sv.Purple, 'DisplayName', 'Vertical Velocity');
plot(time, z, 'LineWidth',sv.LineWidth1,'Color',sv.Red, 'DisplayName', 'Altitude');
xlabel('Time [s]'); ylabel('Velocity [ft/s]');
title('6DOF Simulated Flight')
legend('Location','northwest')

f2=figure(2);
f2.Position = [100,100,800,500]; 
hold on; grid on;
plot(time, phi, 'LineWidth',sv.LineWidth1,'Color',sv.Blue, 'DisplayName', 'Roll');
plot(time, theta, 'LineWidth',sv.LineWidth1,'Color',sv.Red, 'DisplayName', 'Pitch');
plot(time, psi, 'LineWidth',sv.LineWidth1,'Color',sv.Purple, 'DisplayName', 'Yaw');
xlabel('Time [s]'); ylabel('Angle [rad]');
title('6DOF Simulated Flight')
legend('Location','northwest')

f3=figure(3);
f3.Position = [100,100,800,500]; 
hold on; grid on;
plot(time(1:end-1), alpha_t, 'LineWidth',sv.LineWidth1,'Color',sv.Blue, 'DisplayName', 'Angle of Attack');
xlabel('Time [s]'); ylabel('Angle of Attack, \alpha [rad]');
title('6DOF Simulated Flight')
legend('Location','northwest')

f4=figure(4);
f4.Position = [100,100,800,500]; 
hold on; grid on;
plot(time, ub, 'LineWidth',sv.LineWidth1,'Color',sv.Blue, 'DisplayName', 'u');
plot(time, vb, 'LineWidth',sv.LineWidth1,'Color',sv.Red, 'DisplayName', 'v');
plot(time, wb, 'LineWidth',sv.LineWidth1,'Color',sv.Purple, 'DisplayName', 'w');
xlabel('Time [s]'); ylabel('Relative Wind Speed, Body Coordinates [ft/s]');
title('6DOF Simulated Flight')
legend('Location','northwest')

% -------------------------------------------------------------------------

function [ue, ve, we] = wind_profile(altitude)
    if altitude == 0 
        ue = 0;
        ve = 0;
        we = 0;
    else
        ue = 0;
        ve = -0.01;
        we = 0;
    end
end
