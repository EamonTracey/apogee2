%% ACSModel.m

clear all; close all; clc;

% Micheline Denn :)
% 19 Feburary 2025

%% Initial Values:
P0 = 2136.00;           % initial pressure, lbf/ft2
T0 = 75;                % initial temperature, F
mass = 1.213315217;     % mass after burnout (constant), slugs

y_ab = 827;             % altitude at burnout, ft
v_ab = 650;             % velocity at burnout, ft/s
a_ab = -55;             % acceleration at burnout, ft/s^2

%% Set Parameters:
avgFreq = 32;
dt = 1/avgFreq; % time, step, average time between data points

%% Constants:
g = 32.17405;                       % ft/s^2
rho = 0.002247;                     % density, slug/ft**3
apogee_target = 5200;               % target apogee, ft
max_servo_deployment = 35/0.33;     % Maximum servo rotation, deg/s
max_servo_angle = 40;
min_servo_angle = 0; 

%% PID Parameters:
K = 0.01;
Kp = 25;
Ki = 5;
Kd = 0;

%% Load Data
cfd_data =  readtable("CDR_CFD_2.csv");
thrust_data = readtable("L1940_ThrustCurve.csv");
clc

%% ACS Model:
% Initial Conditions
    y(1) = y_ab;            % altitude vector; begins at altitude of burnout, ft
    v(1) = v_ab;            % velocity vector; begins at velocity of burnout, ft/s
    a(1) = a_ab;            % acceleration vector; begins at accleration of burnout, ft/s^2
    servo_angle(1) = 0;     % servo_angle vector; begins at initial position of 0, degrees    
% Initial Apogee Prediction
    % returns initial apogee (ft) and initial drag
    [apogee_predict(1), drag(1)] = ApogeeCalc1D(y(1), v(1), servo_angle(1), mass, P0, T0, cfd_data, thrust_data);
    e(1) = 0; % initialize error; e = apogee_predict - apogee_target
% Initialize PID
    prop(1) = e(1);     % Proportional
    der(1) = 0;         % Derivative
    int(1) = 0;         % Integral

% Model
j = 1; % initialize iteration
while v(j) > -10
    [apogee_predict(j+1), drag(j+1)] = ApogeeCalc1D(y(j), v(j), servo_angle(j), mass, P0, T0, cfd_data, thrust_data);
        % Predicts the next apogee and the next drag with the current altitude, velocity, and servo angle
     
     % Error
        e(j+1) = apogee_predict(j) - apogee_target;
     
     % PID
        prop(j+1) = e(j+1);
        der(j+1) = (e(j+1) - e(j)) / dt;
        int(j+1) = int(j) + (e(j+1) + e(j)) * dt/2;

     % Feedback Loop - Adjust servo angle depending on error
        u(j+1) = K * (Kd*der(j)) + Kp*prop(j) + Ki*int(j);
        servo_angle(j+1) = servo_angle(j) + dt*u(j+1);

%      Delay model (From William's code)
%           Servo cannot actuate faster than 35 deg in 0.33 s
%           --> which means max slope of servo angle plot is 106.06 deg/s
        servo_change(j+1) = (servo_angle(j+1) - servo_angle(j))/dt ;
        if servo_change(j+1) >= max_servo_deployment
            servo_angle(j+1) = servo_angle(j) + max_servo_deployment*dt ;
        elseif servo_change(j+1) <= -1*max_servo_deployment
             servo_angle(j+1) = servo_angle(j) - max_servo_deployment*dt ;
        end

        if servo_angle(j+1) >= max_servo_angle || servo_angle(j+1) <= min_servo_angle
              servo_angle(j+1) = servo_angle(j);
        end
    
    % Recalculate Values
        a(j+1) = -g - drag(j)/mass;
        v(j+1) = v(j) + dt*a(j);
        y(j+1) = y(j) + v(j)*dt + 0.5*randi([-1 1]);

        j = j+1;
        t(j) = dt*(j-2);

end % end of while loop
    

% %     j = 1;
% %     while v{i}(j) > -10
% %         %[apogee_prediction{i}(j+1), drag{i}(j+1)] = calculate_apogee_RK4(y{i}(j), v{i}(j), servo_angle{i}(j), mass, pressure_initial, temp_initial);
% %         [apogee_prediction{i}(1), drag{i}(1)] = ApogeeCalc1D(y{i}(1), v{i}(1), servo_angle{i}(1), mass, pressure_initial, temp_initial);
% %         switch mode
% %             case 1
% %                 e{i}(j+1) = apogee_prediction{i}(j) - apogee_target;
% % 
% %                 Prop(j+1) = e{i}(j+1);
% %                 Der(j+1) = (e{i}(j+1)-e{i}(j))/dt;
% %                 Int(j+1) = Int(j) + ((e{i}(j+1) + e{i}(j))*dt/2);
% % 
% %                 u(j+1) = K*(Kd*Der(j) + Kp*Prop(j) + Ki*Int(j));
% %                 servo_angle{i}(j+1) = servo_angle{i}(j) + dt*u(j+1);    
% % 
% %                 %% Delay model
% %                 %   Servo cannot actuate faster than 35 deg in 0.33 s
% %                 %   --> which means max slope of servo angle plot is 106.06 deg/s
% %                 servo_change(j+1) = (servo_angle{i}(j+1) - servo_angle{i}(j))/dt ;
% %                 if servo_change(j+1) >= max_servo_deployment
% %                     servo_angle{i}(j+1) = servo_angle{i}(j) + max_servo_deployment*dt ;
% %                 elseif servo_change(j+1) <= -1*max_servo_deployment
% %                     servo_angle{i}(j+1) = servo_angle{i}(j) - max_servo_deployment*dt ;
% %                 end
% % 
% %                 if servo_angle{i}(j+1) >= max_servo_angle || servo_angle{i}(j+1) <= min_servo_angle
% %                     servo_angle{i}(j+1) = servo_angle{i}(j);
% %                 end
% %             case 2
% %                 servo_angle{i}(j+1) = 0;
% %         end
% % 
% %         a{i}(j+1) = -g - drag{i}(j)/mass ;
% %         v{i}(j+1) = v{i}(j) + dt*a{i}(j);
% %         y{i}(j+1) = y{i}(j) + v{i}(j)*dt + 0.5*randi([-1 1]);
% %         j = j+1;
% %         t{i}(j) = dt*(j-2);
% % 
% %     end
% %     
% %     [apogee(i), t_apogee(i)] = max(y{i});
% %     disp("Target Apogee: " + apogee_target(1) + " ft")
% %     disp("Actual predicted apogee: " + apogee(i) + " ft")
% % 
% % end
% 
% 
% % disp("Average apogee: " + apogees_avg + " ft")
% % disp("Range: " + apogees_range + " ft")
% % disp("Standard Deviation: " + apogees_std + " ft")
% 
% %% Style values
% linewidth = 1;
% pointSize = 20;
% circleSize = 7.5; 
% fontSize = 14;
% color1 = '#A2142F'; % red 
% color2 =  '#0072BD'; % blue 
% color3 = '#7E2F8E'; % purple
% color4 = '#77AC30' ; % green
% color5 = 'k'; % black
% color6 = '#D95319'; % orange
% 
% %% Plot
% f1=figure(1);
% yline(apogee_target, 'k-', 'LineWidth', 1.2);
% hold on
% for i = 1:length(Vinishes)
%     plot(t{i}, apogee_prediction{i}, 'b-', 'LineWidth',linewidth,'Color',color6);
%     hold on
%     yline(apogee(i), '-', 'LineWidth', 1.2, 'Color',color4);
%     xline(t{i}(t_apogee(i)))
% end
% grid on
% legend('Target Apogee', 'Projected Apogee', 'Actual Apogee', 'FontSize', fontSize, 'Location', 'northeast'); % Legend
% xlabel('t, s') % Add axis labels
% ylabel('y(t), m')
% title('ACS Flight')
% f1.Position = [100,100,800,500]; 
% % axis([0 30 4800 apogee_projected(1)*3.281])
% 
% f2=figure(2);
% % plot(t, y, 'b-', 'LineWidth', 1.2);
% yline(apogee_target, 'k-', 'LineWidth', 1.2, 'DisplayName', 'Target Apogee');
% xline(t{i}(t_apogee(i)), 'DisplayName', 'Time of Apogee')
% hold on
% for i = 1:length(Vinishes)
%     plot(t{i}, y{i},'LineWidth', 1.2, 'DisplayName', 'Trajectory');
%     hold on
% end
% grid on
% legend('Location','southeast') % Legend
% xlabel('t, s') % Add axis labels
% ylabel('y(t), ft')
% title('ACS Flight')
% f2.Position = [100,100,800,500]; % this is based on your screen and preference
% 
% f3=figure(3);
% for i = 1:length(Vinishes)
%     plot(t{i}, v{i}, 'r--', t{i}, a{i}, 'g-', 'LineWidth', 1.2);
%     hold on
% end
% grid on
% legend('Velocity', 'Acceleration', 'Location','northeast') % Legend
% xlabel('t, s') % Add axis labels
% ylabel('v(t), m/s, a(t), m/s^2')
% title('ACS Flight')
% f3.Position = [100,100,800,500]; % this is based on your screen and preference
% 
% f4=figure(4);
% for i = 1:length(Vinishes)
%     plot(t{i}, servo_angle{i}, 'g-', 'LineWidth', 1.2);
%     hold on
% end
% grid on
% legend('Servo Angle', 'Location','northeast') % Legend
% xlabel('t, s') % Add axis labels
% ylabel('Servo Angle')
% title('ACS Flight')
% f4.Position = [100,100,800,500]; % this is based on your screen and preference
% 
% f5=figure(5);
% for i = 1:length(Vinishes)
%     plot(t{i}, e{i}, 'r--', 'LineWidth', 1.2);
%     hold on
% end
% grid on
% legend('Error', 'Location','northeast') % Legend
% xlabel('t, s') % Add axis labels
% ylabel('Error, m')
% title('ACS Flight')
% f5.Position = [100,100,800,500]; % this is based on your screen and preference
% 
% f6=figure(6);
% for i = 1:length(Vinishes)
%     plot(t{i}, drag{i}, '-', 'LineWidth',linewidth,'Color',color6);
%     hold on
% end
% grid on
% legend('Drag', 'FontSize', fontSize, 'Location', 'southeast'); % Legend
% xlabel('t, s') % Add axis labels
% ylabel('D(t), Lbf')
% title('ACS Flight')
% f6.Position = [100,100,800,500]; 
% 
% % close all
% 
% % f7=figure(7);
% % plot(apogees, '^', 'LineWidth',linewidth,'Color',color2);
% % hold on 
% % yline(apogee_target, 'k-', 'LineWidth', 1.2);
% % yline(apogees_avg, 'LineWidth',linewidth,'Color',color1);
% % grid on
% % legend('Apogee', 'Target Apogee', 'Average Apogee',  'FontSize', fontSize, 'Location', 'southeast'); % Legend
% % xlabel('Test Run') % Add axis labels
% % ylabel('Apogee, ft')
% % title('ACS Flight')
% % f7.Position = [100,100,800,500]; 
% 
% 
% 
function [a, drag, mach] = Fv(y, v, state)
    %% Load in Data
    g = state.g; % ft/s^2
    t = state.t;
    mass = state.m;
    dt = state.dt;
    i = state.i;
    thrust_data = state.thrust_data;

    %% Crop Thrust Data
    time_raw = thrust_data.Time(1:72);
    motor_mass_raw = thrust_data.MotorMass(1:72)./(g*16); % Convert oz to slugs
    thrust_raw = thrust_data.Thrust(1:72)*0.224809; % Convert N to lbf
    
    %% Interpolate Thrust Curve
    t_thrust = 0:dt:time_raw(end);
    thrust = interp1(time_raw, thrust_raw, t_thrust);
    motor_mass = interp1(time_raw, motor_mass_raw, t_thrust);
    thrust(1) = thrust(2);
    motor_mass(1) = motor_mass(2);
    delay = 2;

    if t <= t_thrust(end)
        if thrust(i) < mass*g
            mass = mass - (motor_mass(1) - motor_mass(i));
            a_P = 0;
        else
            mass = mass - (motor_mass(1) - motor_mass(i));
            a_P = thrust(i)/mass - g;
        end
        flap_angle = 0;
    elseif t <= t_thrust(end) + delay
        mass = mass - (motor_mass(1) - motor_mass(end));
        a_P = -g;
        flap_angle = 0;
    else
        mass = mass - (motor_mass(1) - motor_mass(end));
        a_P = -g;
        flap_angle = state.flap_angle;
    end

    [drag, mach] = calculate_drag_1D(y, v, flap_angle, state);
    a = a_P - drag/mass ;
end






