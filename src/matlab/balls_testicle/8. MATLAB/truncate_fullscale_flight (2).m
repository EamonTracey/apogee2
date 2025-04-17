function launch_data = truncate_fullscale_flight(data)
%   Inputs:
%       data = matrix of launch data      
%
%   Outputs:
%       launch_data = struct including truncated flight info starting   

%% Find start and end of relevant flight data
    global g time_flight
    dt = mean(diff(data.Time(1:end-2)));
    freq = 1/dt;

    n = length(data.Stage);    
    counter = 1;
    truth = 1;

    for i = 1:n
        if abs(data.Acceleration_Z_ICM20649(i))>8*g && truth == 1
            ilaunch = i;
            truth = 0;
        end

        if data.Stage(i) =="GROUND"
            % ilaunch = i+1
        elseif data.Stage(i) == "BURN"
            iburnout = i+1;
        elseif data.Stage(i) == "COAST"
            iovershoot = i+1;
            iapogee = i+1;
        elseif data.Stage(i) == "OVERSHOOT"
            iapogee= i+1;
        end

        if data.Servo_Angle(i) == 0
            iflap = i+1;
        end


        % Find Apogee while attempting to ignore pressure increase caused
        % by ejection charge

        if i > 50 && i<(n-50)
            w = 40; % window size
            if (data.Stage(i) == "COAST" || data.Stage(i) == "OVERSHOOT") && counter == 1 && data.Velocity_Z(i) >= -10 && data.Velocity_Z(i) <= 10
                [apogee, iapogee_actual]  = max(data.Altitude_BMP390(i-w:i+w));
                counter = 0;
                iapogee_actual = iapogee_actual+i-w;
            end
        end
    end

    time_foreplay = 2;
    % time_flight = 22;
    ibeg = round(ilaunch-time_foreplay*freq);
    iend = round(ibeg + time_flight*freq);

%% Truncate data 
    % Time
    launch_data.Time = data.Time(ibeg:iend);
    launch_data.dt = dt;
    launch_data.freq = freq;
    launch_data.time_flight = time_flight;
    launch_data.Stage = data.Stage(ibeg:iend);

    % BMP390 Data
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

    % Servo Angle
    launch_data.Servo_Angle = data.Servo_Angle(ibeg:iend);

    % Calculated velocity Data
    launch_data.Velocity_X = data.Velocity_X(ibeg:iend);
    launch_data.Velocity_Y = data.Velocity_Y(ibeg:iend);
    launch_data.Velocity_Z = data.Velocity_Z(ibeg:iend);

%% Zero time and altitude
    launch_data.Time = launch_data.Time - min(launch_data.Time);
    launch_data.Altitude_AGL = launch_data.Altitude_ASL - mean(launch_data.Altitude_ASL(1:round(1.5*freq)));

%% Save Variables in launch_data
    launch_data.Apogee = apogee*3.280839895 - mean(launch_data.Altitude_ASL(1:round(1.5*freq)));
    launch_data.iapogee = iapogee_actual-ibeg;
    launch_data.t_apogee = launch_data.Time(launch_data.iapogee);

    launch_data.ilaunch = ilaunch-ibeg;
    launch_data.iburnout = iburnout-ibeg;
    launch_data.iovershoot = iovershoot-ibeg;
    launch_data.iflap = iflap-ibeg;
end