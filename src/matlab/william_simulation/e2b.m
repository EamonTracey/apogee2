function Te2b = e2b(theta, psi, phi)
%% Earth Coordinates to Rocket Body Transformation
%   Author: William Teasley
%
%   Date: 1 March 2025
%
%   This function serves to convert earth coordinates to rocket body 
%   coordinates assuming the rocket coordinates are:
%       - x is the axial direction of the rocket 
%       - y is north
%       - z is east
%   and Earth coordinates are:
%       - x is west
%       - y is north
%       - z is up 
%
%   Inputs:
%       theta, psi, phi = Euler angles pitch, yaw, and roll
%
%   Outputs:
%       T = earth coordinate to rocket body transformation matrix

% Precompute trigonometric functions
cphi = cos(phi);
sphi = sin(phi);
ctheta = cos(theta);
stheta = sin(theta);
cpsi = cos(psi);
spsi = sin(psi);

% Rotation matrix from Earth to Body using 3-2-1 Euler angles
R_rot = [...
    ctheta * cpsi,                          ctheta * spsi,                          -stheta; 
    sphi * stheta * cpsi - cphi * spsi,    sphi * stheta * spsi + cphi * cpsi,     sphi * ctheta; 
    cphi * stheta * cpsi + sphi * spsi,    cphi * stheta * spsi - sphi * cpsi,     cphi * ctheta];

% Initial static rotation matrix to align Earth frame to initial Body frame
% Earth: X = North, Y = West, Z = Up
% Body: X = Axial (Nose), Y = West, Z = South
R_init = [...
     0,  0,  1; 
     0,  1,  0; 
    -1,  0,  0];

% Total rotation matrix from Earth frame to Body frame
Te2b = R_rot * R_init;

end