function environment = calculate_environment(environment_inish, altitude)
%% Environmental Conditions Generator
%   Author: William Teasley
%
%   Date: 7 March 2025
%
%   This function serves to calculate the environmental conditions such as 
%   density, speed of sound, and wind direction for a given altitude
%
%   Inputs:
%       environment_inish = 
%       altitude = 
%
%   Outputs:
%       environment = 

%% Constants
R = 1716;
gamma = 1.4;
L = 0.00356;          % Temperature lapse rate in °F/ft
T_ref = 518.6;        % Reference temperature in Rankine

%% Calculate wind
switch environment_inish.wind_profile
    case "sharp"
        if altitude <= 9.5
            u_inf = 0;
        else
            u_inf = environment_inish.u_inf;
        end
        theta = environment_inish.wind_direction;
        wind = [u_inf*cosd(theta); u_inf*sind(theta); 0];
end

%% Calculate density and speed of sound
P0 = environment_inish.P0; % lbf/ft2
T0 = environment_inish.T0; % °F

% Calculate temperature and pressure from altitude
T = T0 - L * (altitude); % Temperature in °F --> might be a bug
P = P0 * ((T + 459.7) ./ T_ref).^5.256; % Pressure in lbf/ft²
rho = P/(R*(T+459.7));
a = sqrt(gamma*R*(T+459.7));

%% Store values
environment.environment_inish = environment_inish;
environment.density_SL = 0.0023768924; % 0.00237717;
environment.density = rho;
environment.a = a;
environment.wind = wind;
end
