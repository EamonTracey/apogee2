function dxdt = calculate_derivatives(x, vehicle, motor, environment, t)
%% Derivatives Calculator
%   Author: William Teasley
%
%   Date: 6 March 2025
%
%   This function serves to calculate the current moments and accelerations
%   on the rocket during simulated fight using CFD Data.
%
%   Inputs:
%       x = the state vector with current position, velocity, flap angle,
%           angle of attack, and Zenith angle of the rocket
%       vehicle =   a struct containing launch vehicle parameters such as 
%       environment =   a function containing atmospheric conditions with 
%                       altitude as an input and density and speed of sound
%                       as outputs
%       t = current flight time, (s)
%
%   Outputs:
%       derivatives =   a state vector containing the resulting the 
%                       accelerations and moments acting on the rocket

%% Grab necessary information
global g

% State
x_earth = x(1:3);
vel_earth = x(4:6);
theta = x(7);
psi = x(8);
phi = x(9);
p = x(10);
q = x(11);
r = x(12);
flap_angle = x(13);
% m = x(14);

% Transformation matrices to convert from Earth to rocket body coordinates
% or vice versa
Te2b = e2b(theta, psi, phi);
Tb2e = b2e(theta, psi, phi);

% Vehicle
Ix = vehicle.Ix;
Iy = vehicle.Iy; 
Iz = vehicle.Iz;
cm = vehicle.x_cm; % distance from nose to center of mass, ft
cp = vehicle.x_cp; % distance from nose to center of pressure, ft
r_body = [cm - cp; 0; 0];
bodyAxisVec_body = [1; 0; 0];
bodyAxisVec_earth = Tb2e*bodyAxisVec_body;

% Motor
[thrust, motor_mass] = motor.calculate_thrust(t);
m = (vehicle.m - motor.full_weight) + motor_mass;

% Environmenmt
altitude = x_earth(3);
environment = calculate_environment(environment.environment_inish, altitude);
wind_earth = environment.wind;
density = environment.density;
density_SL = environment.density_SL;
a = environment.a;
velocityVec_earth = vel_earth+wind_earth;
velocity_mag = norm(velocityVec_earth);
mach_number = velocity_mag/a; 

%% Calculate forces on rocket
% Aerodynamic forces
if norm(vel_earth) < 1e-6 || norm(wind_earth) < 1e-6
    aoa = 0;
    sign_alpha = 1;
    wind_dir_earth = 0;
    perp_wind_dir_earth = 0;
    F_normal_dir = [0; 0; 0];  % No normal force (AoA = 0 or 180°)
else
    % Normalize vectors
    wind_dir_earth = wind_earth/norm(wind_earth);
    perp_wind_dir_earth = [-wind_dir_earth(2); wind_dir_earth(1); 0];

    v_hat = velocityVec_earth / norm(velocityVec_earth);
    b_hat = bodyAxisVec_earth / norm(bodyAxisVec_earth);
    
    % Compute angle magnitude
    dot_val = dot(v_hat, b_hat);
    dot_val = max(min(dot_val, 1), -1);  % Clamp safely
    angle_mag = acosd(dot_val);
    
    % Compute cross product to determine sign
    cross_prod = cross(v_hat, b_hat);
    
    % Determine sign using dot product with reference "up" vector
    dot_val = dot(cross_prod, perp_wind_dir_earth);
    if abs(dot_val) < 1e-6
        sign_alpha = 1;  % Default to +1 or handle explicitly
    else
        sign_alpha = sign(dot_val);
    end
        
    % Apply sign to angle
    aoa = sign_alpha * angle_mag    

    % Find direction of normal force
    if abs(aoa) < 1e-6
        F_normal_dir = [0; 0; 0];  % No normal force (AoA = 0 or 180°)
    else
        F_normal_dir = Te2b*(cross(b_hat, cross(b_hat, v_hat)));  % Direction of normal force on rocket
    end
end

if sign_alpha < 0
    [F_axial, F_normal] = vehicle.calculate_drag(flap_angle, -aoa, mach_number);
    F_axial = F_axial*density/density_SL;
    F_normal = F_normal*density/density_SL;
elseif sign_alpha >= 0
    [F_axial, F_normal] = vehicle.calculate_drag(flap_angle, aoa, mach_number);
    F_axial = F_axial*density/density_SL;
    F_normal = F_normal*density/density_SL;
else 
    disp("Problem with AOA");
end

F_axial_body = [-F_axial; 0; 0];
F_normal_body = F_normal*F_normal_dir;
F_A_body = F_axial_body + F_normal_body;

% Gravity force
g_earth = [0; 0; -g];
g_body = Te2b*g_earth;
F_g_body = m*g_body;

% Propulsive force 
F_P_body = [thrust; 0; 0];

% Total force
F_body = F_A_body + F_P_body + F_g_body;

% Calculate torques on rocket
torque_body = cross(r_body, F_normal_body);

%% Calculate derivatives 
% Rotations in body coordinates 
pdot = (torque_body(1)/Ix)  - q*r*(Iz - Iy)/Ix;
qdot = (torque_body(2)/Iy)  - r*p*(Ix - Iz)/Iy;
rdot = (torque_body(3)/Iz)  - p*q*(Iy - Ix)/Iz;

Omega = [p; q; r];
Omega_dot = [pdot; qdot; rdot];

% Velocity
vel_body = Te2b*vel_earth; 

% Acceleration
accel_body = F_body./m;
accel_earth = Tb2e*(accel_body); %  + 2*cross(Omega, vel_body));

% Euler angles
phi_dot = p + (q * sin(phi) + r * cos(phi)) * tan(theta);    % Roll angle rate
theta_dot = q * cos(phi) - r * sin(phi);                      % Pitch angle rate
psi_dot = (q * sin(phi) + r * cos(phi)) / cos(theta);        % Yaw angle rate

dxdt.vec = [vel_earth; accel_earth; theta_dot; psi_dot; phi_dot; pdot; qdot; rdot; 0; 0];
dxdt.aoa = aoa;
dxdt.m = m;
dxdt.thrust = thrust;
dxdt.a = a;
dxdt.F_axial = F_axial;
end