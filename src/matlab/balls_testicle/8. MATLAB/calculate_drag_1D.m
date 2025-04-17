function [drag, mach] = calculate_drag_1D(y, v, flap_angle, state)
    %% Constants
    R = 1716;
    gamma = 1.4;
    rho_SL = 0.00237717;

    %% Find Mach number
    L = 0.00356;          % Temperature lapse rate in °F/ft
    T_ref = 518.6;        % Reference temperature in Rankine
    
    % Calculatee Pressure from altitude data
    T = 59 - L * (y+692); % Temperature in °F
    P = state.P0 * ((T + 459.7) ./ T_ref).^5.256; % Pressure in lbf/ft²

    rho = P/(R*(T+459.7));
    a = sqrt(gamma*R*(T+459.7));
    mach = v/a;
    
%     if mach> 0.65
%         mach = 0.65;
%     end

    if flap_angle == 0
        drag = interp1(state.cfd_data.Mach_Number(1:24), state.cfd_data.Drag_Force(1:24), mach)*0.224809*rho/rho_SL;
    elseif flap_angle == 45
        drag = interp1(state.cfd_data.Mach_Number(25:48), state.cfd_data.Drag_Force(25:48), mach)*0.224809*rho/rho_SL;
    else 
        disp("Unable to understand flap angle. We're not there yet")
    end

    