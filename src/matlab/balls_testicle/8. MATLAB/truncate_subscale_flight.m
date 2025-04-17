function launch_data = truncate_subscale_flight(data)
%   Inputs:
%       data = matrix of launch data      
%
%   Outputs:
%       state = launch state (ground, launch, burnout, overshoot, apogee)    

%% Find start and end of relevant flight data
    n = length(data.Time);
    counter = 1;
    dt = mean(diff(data.Time(1:end-2)));
    freq = 1/dt;

    altitude_offset = mean(data.Altitude_BMP390(1:round(10*freq)));
    data.Altitude_AGL = data.Altitude_BMP390-altitude_offset;

    for i = 10:n
        if data.Altitude_AGL(i) > 5 && counter == 1
            ilaunch = i;
            counter = 0;
        end
    end

    time_flight = 18; 
    ibeg = round(ilaunch-2.1*freq);
    iend = round(ibeg + time_flight*freq);

%% Truncate data 
    % Time
    launch_data.Time = data.Time(ibeg:iend);
    launch_data.dt = dt;
    launch_data.freq = freq;
    launch_data.time_flight = time_flight;

    % BMP390 Data
    launch_data.Altitude_AGL = data.Altitude_AGL(ibeg:iend)*3.280839895;
    launch_data.Altitude_ASL = data.Altitude_BMP390(ibeg:iend)*3.280839895;
    launch_data.Temperature_BMP390 = data.Temperature_BMP390(ibeg:iend);

    % BNO085 Data
    launch_data.Acceleration_X_BNO085 = data.Acceleration_X_BNO085(ibeg:iend)*3.280839895;
    launch_data.Acceleration_Y_BNO085 = data.Acceleration_Y_BNO085(ibeg:iend)*3.280839895;
    launch_data.Acceleration_Z_BNO085 = data.Acceleration_Z_BNO085(ibeg:iend)*3.280839895;
    
    launch_data.Magnetic_X_BNO085 = data.Magnetic_X_BNO085(ibeg:iend);
    launch_data.Magnetic_Y_BNO085 = data.Magnetic_Y_BNO085(ibeg:iend);
    launch_data.Magnetic_Z_BNO085 = data.Magnetic_Z_BNO085(ibeg:iend);
    
    launch_data.Gyro_X_BNO085 = data.Gyro_X_BNO085(ibeg:iend);
    launch_data.Gyro_Y_BNO085 = data.Gyro_Y_BNO085(ibeg:iend);
    launch_data.Gyro_Z_BNO085 = data.Gyro_Z_BNO085(ibeg:iend);

    launch_data.Quaternion_W_BNO085 = data.Quaternion_W_BNO085(ibeg:iend);
    launch_data.Quaternion_X_BNO085 = data.Quaternion_X_BNO085(ibeg:iend);
    launch_data.Quaternion_Y_BNO085 = data.Quaternion_Y_BNO085(ibeg:iend);
    launch_data.Quaternion_Z_BNO085 = data.Quaternion_Z_BNO085(ibeg:iend);
    
    % ICM20649 Data (no magnetometer)
    launch_data.Acceleration_X_ICM20649 = data.Acceleration_X_ICM20649(ibeg:iend)*3.280839895;
    launch_data.Acceleration_Y_ICM20649 = data.Acceleration_Y_ICM20649(ibeg:iend)*3.280839895;
    launch_data.Acceleration_Z_ICM20649 = data.Acceleration_Z_ICM20649(ibeg:iend)*3.280839895;
    
    launch_data.Gyro_X_ICM20649 = data.Gyro_X_ICM20649(ibeg:iend);
    launch_data.Gyro_Y_ICM20649 = data.Gyro_Y_ICM20649(ibeg:iend);
    launch_data.Gyro_Z_ICM20649 = data.Gyro_Z_ICM20649(ibeg:iend);

%% Zero time 
    launch_data.Time = launch_data.Time - min(launch_data.Time);
    launch_data.Altitude_AGL = launch_data.Altitude_AGL - mean(launch_data.Altitude_AGL(1:round(1.5*freq)));

%% Save Variables in launch_data
    [launch_data.Apogee, launch_data.iapogee] = max(launch_data.Altitude_AGL);
    launch_data.t_apogee = launch_data.Time(launch_data.iapogee);

%% Remove Outliers
    % launc_data = rmoutliers(removevars(launch_data, {'State'}));% , "movmedian", 5);

end