function [r,v] = state_vector_from_observe(rho, rhodot, A, Adot, a, adot, theta, phi, H)
% This function calculates the geocentric equatorial position and
% velocity vectors of an object from radar observations of range,
% azimuth, elevation angle and their rates.
% Re - equatorial radius of the earth (km)
% f - earth’s flattening factor
% wE - angular velocity of the earth (rad/s)
% omega - earth’s angular velocity vector (rad/s) in the
% geocentric equatorial frame
% theta - local sidereal time (degrees) of tracking site
% phi - geodetic latitude (degrees) of site
% H - elevation of site (km)
% R - geocentric equatorial position vector (km) of tracking site
% Rdot - inertial velocity (km/s) of site
% rho - slant range of object (km)
% rhodot - range rate (km/s)
% A - azimuth (degrees) of object relative to observation site
% Adot - time rate of change of azimuth (degrees/s)
% a - elevation angle (degrees) of object relative to observation site
% adot - time rate of change of elevation angle (degrees/s)
% dec - topocentric equatorial declination of object (rad)
% decdot - declination rate (rad/s)
% h - hour angle of object (rad)
% RA - topocentric equatorial right ascension of object (rad)
% RAdot - right ascension rate (rad/s)
% Rho - unit vector from site to object
% Rhodot - time rate of change of Rho (1/s)
% r - geocentric equatorial position vector of object (km)
% v - geocentric equatorial velocity vector of object (km)
% --------------------------------------------------------------------
global f Re wE
omega = [0 0 wE];

% Geocentric position vector of site
R = [(Re/sqrt(1-(2*f - f*f)*sin(phi)^2) + H)*cos(phi)*cos(theta), ...
    (Re/sqrt(1-(2*f - f*f)*sin(phi)^2) + H)*cos(phi)*sin(theta), ...
    (Re*(1 - f)^2/sqrt(1-(2*f - f*f)*sin(phi)^2) + H)*sin(phi)];

% Inertial velocity of site
Rdot = cross(omega, R);

% Topocentric declination
dec = asin(cos(phi)*cos(A)*cos(a) + sin(phi)*sin(a));

% Topocentric right ascension
h = acos((cos(phi)*sin(a) - sin(phi)*cos(A)*cos(a))/cos(dec));
if (A > 0) && (A < pi)
    h = 2*pi - h;
end
RA = theta - h;

% Direction cosine unit vector
Rho = [cos(RA)*cos(dec) sin(RA)*cos(dec) sin(dec)];

% Geocentric position vector of object
r = R + rho*Rho;

% Declination rate
decdot = (-Adot*cos(phi)*sin(A)*cos(a) ...
    + adot*(sin(phi)*cos(a) - cos(phi)*cos(A)*sin(a)))/cos(dec);

% Right ascension rate
RAdot = wE ...
    + (Adot*cos(A)*cos(a) - adot*sin(A)*sin(a) ...
    + decdot*sin(A)*cos(a)*tan(dec)) ...
    /(cos(phi)*sin(a) - sin(phi)*cos(A)*cos(a));

% Direction cosine rate vector
Rhodot = [-RAdot*sin(RA)*cos(dec) - decdot*cos(RA)*sin(dec),...
    RAdot*cos(RA)*cos(dec) - decdot*sin(RA)*sin(dec),...
    decdot*cos(dec)];

% Geocentric velocity vector
v = Rdot + rhodot*Rho + rho*Rhodot;
end 


