function vehicle = calculate_vehicle(vehicle_name)
%% Vehicle Generator
%   Author: William Teasley
%
%   Date: 6 March 2025
%
%   This function serves to build the launch vehicle based on launch day
%   properties of the vehicle
%
%   Inputs:
%       properties =    a struct containing the properties of the rocket on
%                       the day of the launch
%
%   Outputs:
%       vehicle =   a struct containing 
global g

switch vehicle_name
    case "katie"
        cfd_data = readmatrix("CFD Data\cfd_newtons.csv");
        vehicle.m = (686.1/16)/g; % vehicle mass without motor, slugs
        vehicle.d = 0.5; % diameter of body tube in ft
        vehicle.S = 0.25*pi*(vehicle.d^2); % cross section area in ft2
        vehicle.x_cm = 56.75/12; % distance from nose to center of mass, ft
        vehicle.x_cp = 72.386/12; % distance from nose to center of pressure, ft
        vehicle.Ix = 1.41/g; % lbft2
        vehicle.Iy = 187.29/g; % slug-ft2
        vehicle.Iz = 187.29/g; % slug-ft2
        vehicle.angle_launch_rail = 0; % launch rail angle, (°)
        vehicle.length_launch_rail = 9.5; % launch rail length, (ft)
        vehicle.angle_azimuth = 0; % launch rail heading, (°)
        flap_angle_data = cfd_data(:, 1);
        aoa_data = cfd_data(:, 2);
        mach_number_data = cfd_data(:, 3);
        F_axial_data = cfd_data(:, 4)*0.224809; % convert from N to lbf
        F_normal_data = cfd_data(:, 5)*0.224809; % convert from N to lbf
        
        % Create interpolation functions for axial and normal forces
        AxialInterp = scatteredInterpolant(flap_angle_data, aoa_data, mach_number_data, F_axial_data, 'linear', 'nearest');
        NormalInterp = scatteredInterpolant(flap_angle_data, aoa_data, mach_number_data, F_normal_data, 'linear', 'nearest');

        vehicle.calculate_drag = @(flap_angle, aoa, mach_number) calculate_drag(AxialInterp, NormalInterp, flap_angle, aoa, mach_number);

    case "balls"
        disp("haha")
end




end