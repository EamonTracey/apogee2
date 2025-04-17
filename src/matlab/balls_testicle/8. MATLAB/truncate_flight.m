function launch_data = truncate_flight(data)
%   Inputs:
%       data = matrix of launch data      
%
%   Outputs:
%       state = launch state (ground, launch, burnout, overshoot, apogee)    

%% Find start and end of relevant flight data
    n = length(data.State);
    state = zeros(1, n);
    ilaunch = 0;
    iburnout = 0;
    iovershoot = 0;
    iapogee = 0;
    iflap = 0;
    counter = 1;

    for i = 1:n
        if data.State(i) =="State.GROUND"
            state(i) = "GROUND";
            ilaunch = i+1;
        elseif data.State(i) == "State.LAUNCHED"
            state(i) = "LAUNCHING";
            iburnout = i+1;
        elseif data.State(i) == "State.BURNOUT"
            state(i) = "COASTING";
            iovershoot = i+1;
            iapogee = i+1;
        elseif data.State(i) == "State.OVERSHOOT"
            state(i) = "OVERSHOOTING";
            iapogee= i+1;
        elseif data.State(i) == "State.APOGEE"
            state(i) = "APOGEE";
        end

        if data.FlapAngle(i) == 0
            iflap = i+1;
        end


        if i > 50 && i<(n-50)
                    % disp(i)

            w = 20; % window size
            if (data.State(i) == "State.BURNOUT" || data.State(i) == "State.OVERSHOOT") && data.VelocityFiltered(i) >= -10 && data.VelocityFiltered(i) <= 10 && counter == 1
            [apogee, iapogee_actual]  = max(data.AltitudeFiltered(i-w:i+w));
            counter = 0;
            iapogee_actual = iapogee_actual+i-w-1;
            end
        end
    end
    
    k = 10;
    g = 75; 
    ibeg = ilaunch-g;
    iend = iapogee+k;

%% Truncate data
    launch_data.Time = data.Time(ibeg:iend);
    launch_data.State = data.State(ibeg:iend);
    launch_data.ApogeePrediction = data.ApogeePrediction(ibeg:iend);
    launch_data.ServoPercentage = data.FlapAngle(ibeg:iend);
    launch_data.AltitudeFiltered = data.AltitudeFiltered(ibeg:iend);
    launch_data.AltitudeUnfiltered = data.Altitude(ibeg:iend);
    launch_data.VelocityFiltered = data.VelocityFiltered(ibeg:iend);
    launch_data.AccelerationFiltered = data.AccelerationFiltered(ibeg:iend);
    launch_data.AccelerationUnfilteredX = data.AccelerationX(ibeg:iend);
    launch_data.AccelerationUnfilteredY = data.AccelerationY(ibeg:iend);
    launch_data.AccelerationUnfilteredZ = data.AccelerationZ(ibeg:iend);
    launch_data.ZenithAngle = data.EulerAngle2(ibeg:iend);
    launch_data.Temp = data.Temperature(ibeg:iend);
    
    launch_data.MagneticX = data.MagneticX(ibeg:iend);    
    launch_data.MagneticY = data.MagneticY(ibeg:iend);    
    launch_data.MagneticZ = data.MagneticZ(ibeg:iend);    
    launch_data.GyroX = data.GyroX(ibeg:iend);    
    launch_data.GyroY = data.GyroY(ibeg:iend);    
    launch_data.GyroZ = data.GyroZ(ibeg:iend);

%% Zero time 
    launch_data.Time = launch_data.Time - min(launch_data.Time);

%% Save Variables in launch_data
    launch_data.ilaunch = ilaunch-ibeg+1;
    launch_data.iburnout = iburnout-ibeg+1;
    launch_data.iovershoot = iovershoot-ibeg+1;
    launch_data.iapogee = iapogee-ibeg+1;
    launch_data.Apogee = apogee;
    launch_data.t_apogee = launch_data.Time(iapogee_actual-ibeg+1);
    launch_data.iflap = iflap;
    launch_data.TargetApogee = data.TargetApogee(1);

%% Remove Outliers
%     launch_data = rmoutliers(removevars(launch_data, {'State'}));% , "movmedian", 5);

end