function recovery_data = truncate_fullscale_flight_recovery(data)
%   Inputs:
%       data = matrix of launch data      
%
%   Outputs:
%       state = launch state (ground, launch, burnout, overshoot, apogee)    

%% Find start and end of relevant flight data
    dt = mean(diff(data.Flight_Time__s_));
    freq = 1/dt;
    time_flight = 22; 
    ibeg = 1; 
    iend = round(ibeg + time_flight*freq);

%% Truncate data 
    % Time
    recovery_data.Time = data.Flight_Time__s_(ibeg:iend);
    recovery_data.dt = dt;
    recovery_data.freq = freq;
    recovery_data.time_flight = time_flight;

    % Altitude Data
    recovery_data.Altitude_ASL = data.Baro_Altitude_ASL__feet_(ibeg:iend);
    recovery_data.Altitude_AGL = data.Baro_Altitude_AGL__feet_(ibeg:iend);
    recovery_data.Altitude_Inertial = data.Inertial_Altitude(ibeg:iend);

    % Other stuff
    recovery_data.Zenith = data.Tilt_Angle__deg_(ibeg:iend);
    recovery_data.Velocity_Up = data.Velocity_Up(ibeg:iend);

%% Zero time 
    recovery_data.Time = recovery_data.Time - min(recovery_data.Time);

%% Save Variables in launch_data
    [recovery_data.Apogee, recovery_data.iapogee] = max(recovery_data.Altitude_AGL);
    recovery_data.t_apogee = recovery_data.Time(recovery_data.iapogee);

%% Remove Outliers
    % launc_data = rmoutliers(removevars(launch_data, {'State'}));% , "movmedian", 5);

end