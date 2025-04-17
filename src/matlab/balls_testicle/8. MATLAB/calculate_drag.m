function [F_axial, F_normal] = calculate_drag(AxialInterp, NormalInterp, flap_angle, aoa, mach_number)
%% Drag Calculator
%   Author: William Teasley
%
%   Date: 7 March 2025
%
%   This function serves to calculate the axial and normal aerodynamic 
%   force on the rocket for a given flap angle, angle of attack, and mach
%   number from CFD data
%
%   Inputs:
%       cfd_data =  a matrix with columns flap angle, AoA, Mach, axial
%                   drag, and normal force calculated from CFD
%       flap_angle = query flap angle, (°)
%       aoa = query angle of attack, (°)
%       mach_number = query Mach number
%
%   Outputs:
%       F_axial =   interpolated axial force (lbs)
%       F_normal =  interpolated normal force (lbs)


%% Unpack data
% Calculates axial and normal forces using the function inputs as query points
F_axial = AxialInterp(flap_angle, aoa, mach_number);
F_normal = NormalInterp(flap_angle, aoa, mach_number);

% if mach_number>0.7
%     disp("Drag data may be inaccurate - Mach number higher than CFD")
% end
% 
% if flap_angle>45
%     disp("Drag data may be inaccurate - flap angle higher than CFD")
% end
% 
% if aoa>10
%     disp("Drag data may be inaccurate - angle of attack higher than CFD")
% end


end
