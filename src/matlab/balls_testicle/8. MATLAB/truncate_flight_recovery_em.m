function recovery_data = truncate_flight_recovery_em(data)
%   Inputs:
%       data = matrix of launch data      
%
%   Outputs:
%       state = launch state (ground, launch, burnout, overshoot, apogee)    

%% Find start and end of relevant flight data
    global g time_flight
    dt = mean(diff(data.time));
    freq = 1/dt;
    ibeg = 1; 
    iend = round(3.4*time_flight*freq);

%% Truncate data 
    % Time
    recovery_data.Time = data.time(ibeg:iend);
    recovery_data.dt = dt;
    recovery_data.freq = freq;
    recovery_data.time_flight = time_flight;

    % Altitude Data
    recovery_data.Altitude_ASL = data.altitude(ibeg:iend)*3.280839895;
    recovery_data.Altitude_AGL = data.height(ibeg:iend)*3.280839895;
    recovery_data.Altitude_AGL = recovery_data.Altitude_AGL - recovery_data.Altitude_AGL(1);

    % Other stuff
    recovery_data.Velocity_Up = data.speed(ibeg:iend)*3.280839895;

%% Zero time 
    recovery_data.Time = recovery_data.Time - min(recovery_data.Time);

%% Save Variables in launch_data
    [recovery_data.Apogee, recovery_data.iapogee] = max(recovery_data.Altitude_AGL);
    recovery_data.t_apogee = recovery_data.Time(recovery_data.iapogee);

%% Remove Outliers
    % launc_data = rmoutliers(removevars(launch_data, {'State'}));% , "movmedian", 5);

end