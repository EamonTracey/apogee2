%% New ACS Numeric Model
% PID Forward Loop Model for an air braking system

% Author: William Teasley
% Date: 6 April 2023
% Completed Individually

clc; clear; close all;

%% Initial Values --> CHANGE BEFORE FLIGHT
pressure_initial = 2136.00; % lbf/ft2
temp_initial = 75; % F
mass = 1.213315217; % slugs; mass after burnout, so constant

%% Set Parameters
% tmin = 0; % Start time
% tmax = 25; % End time
avgFreq = 32;
dt = 1/avgFreq; % Average time between data points
% t = tmin:dt:tmax;
% n = length(t);

%% Constants
g = 32.17405; % ft/s^2
rho = 0.002247; % slug/ft**3
apogee_target = 5200; % ft
max_servo_deployment = 35/0.33 ; % Maximum servo rotation, deg/s
max_servo_angle = 40;
min_servo_angle = 0; 

%% Modes
mode = 1; % 1 = control ON | 2 = control OFF

%% PID Parameters
K = 0.01;
Kp = 25;
Ki = 5;
Kd = 0;

%% Control loop model
% % Preallocate vector
% y = zeros(1, n);
% v = zeros(1, n);
% a = zeros(1, n);
% drag = zeros(1, n);
% servo_angle = zeros(1, n);
% apogee_prediction = zeros(1, n);
% e = zeros(1, n);
% Prop = zeros(1, n);
% Der = zeros(1, n);
% Int = zeros(1, n);
% u = zeros(1, n);

Vinishes = 650; % 550:50:650;


for i = 1:length(Vinishes)
    % Initial Conditions
    y{i}(1) = 827; % altitude at burnout, ft
    v{i}(1) = Vinishes(i); % burnout velocity, ft/s
    a{i}(1) = -55; % burnout decceleration, ft/s^2
    servo_angle{i}(1) = 0;
    [apogee_prediction{i}(1), drag{i}(1)] = calculate_apogee_RK4(y{i}(1), v{i}(1), servo_angle{i}(1), mass, pressure_initial, temp_initial);
    e{i}(1) = 0; % apogee_prediction(1) - apogee_target;
    Prop(1) = e{i}(1);
    Der(1) = 0;
    Int(1) = 0;

    j = 1;
    while v{i}(j) > -10
        [apogee_prediction{i}(j+1), drag{i}(j+1)] = calculate_apogee_RK4(y{i}(j), v{i}(j), servo_angle{i}(j), mass, pressure_initial, temp_initial);

        switch mode
            case 1
                e{i}(j+1) = apogee_prediction{i}(j) - apogee_target;

                Prop(j+1) = e{i}(j+1);
                Der(j+1) = (e{i}(j+1)-e{i}(j))/dt;
                Int(j+1) = Int(j) + ((e{i}(j+1) + e{i}(j))*dt/2);

                u(j+1) = K*(Kd*Der(j) + Kp*Prop(j) + Ki*Int(j));
                servo_angle{i}(j+1) = servo_angle{i}(j) + dt*u(j+1);    

                %% Delay model
                %   Servo cannot actuate faster than 35 deg in 0.33 s
                %   --> which means max slope of servo angle plot is 106.06 deg/s
                servo_change(j+1) = (servo_angle{i}(j+1) - servo_angle{i}(j))/dt ;
                if servo_change(j+1) >= max_servo_deployment
                    servo_angle{i}(j+1) = servo_angle{i}(j) + max_servo_deployment*dt ;
                elseif servo_change(j+1) <= -1*max_servo_deployment
                    servo_angle{i}(j+1) = servo_angle{i}(j) - max_servo_deployment*dt ;
                end

                if servo_angle{i}(j+1) >= max_servo_angle || servo_angle{i}(j+1) <= min_servo_angle
                    servo_angle{i}(j+1) = servo_angle{i}(j);
                end
            case 2
                servo_angle{i}(j+1) = 0;
        end

        a{i}(j+1) = -g - drag{i}(j)/mass ;
        v{i}(j+1) = v{i}(j) + dt*a{i}(j);
        y{i}(j+1) = y{i}(j) + v{i}(j)*dt + 0.5*randi([-1 1]);
        j = j+1;
        t{i}(j) = dt*(j-2);

    end
    
    [apogee(i), t_apogee(i)] = max(y{i});
    disp("Target Apogee: " + apogee_target(1) + " ft")
    disp("Actual predicted apogee: " + apogee(i) + " ft")

end


% disp("Average apogee: " + apogees_avg + " ft")
% disp("Range: " + apogees_range + " ft")
% disp("Standard Deviation: " + apogees_std + " ft")

%% Style values
linewidth = 1;
pointSize = 20;
circleSize = 7.5; 
fontSize = 14;
color1 = '#A2142F'; % red 
color2 =  '#0072BD'; % blue 
color3 = '#7E2F8E'; % purple
color4 = '#77AC30' ; % green
color5 = 'k'; % black
color6 = '#D95319'; % orange

%% Plot
f1=figure(1);
yline(apogee_target, 'k-', 'LineWidth', 1.2);
hold on
for i = 1:length(Vinishes)
    plot(t{i}, apogee_prediction{i}, 'b-', 'LineWidth',linewidth,'Color',color6);
    hold on
    yline(apogee(i), '-', 'LineWidth', 1.2, 'Color',color4);
    xline(t{i}(t_apogee(i)))
end
grid on
legend('Target Apogee', 'Projected Apogee', 'Actual Apogee', 'FontSize', fontSize, 'Location', 'northeast'); % Legend
xlabel('t, s') % Add axis labels
ylabel('y(t), m')
title('ACS Flight')
f1.Position = [100,100,800,500]; 
% axis([0 30 4800 apogee_projected(1)*3.281])

f2=figure(2);
% plot(t, y, 'b-', 'LineWidth', 1.2);
yline(apogee_target, 'k-', 'LineWidth', 1.2, 'DisplayName', 'Target Apogee');
xline(t{i}(t_apogee(i)), 'DisplayName', 'Time of Apogee')
hold on
for i = 1:length(Vinishes)
    plot(t{i}, y{i},'LineWidth', 1.2, 'DisplayName', 'Trajectory');
    hold on
end
grid on
legend('Location','southeast') % Legend
xlabel('t, s') % Add axis labels
ylabel('y(t), ft')
title('ACS Flight')
f2.Position = [100,100,800,500]; % this is based on your screen and preference

f3=figure(3);
for i = 1:length(Vinishes)
    plot(t{i}, v{i}, 'r--', t{i}, a{i}, 'g-', 'LineWidth', 1.2);
    hold on
end
grid on
legend('Velocity', 'Acceleration', 'Location','northeast') % Legend
xlabel('t, s') % Add axis labels
ylabel('v(t), m/s, a(t), m/s^2')
title('ACS Flight')
f3.Position = [100,100,800,500]; % this is based on your screen and preference

f4=figure(4);
for i = 1:length(Vinishes)
    plot(t{i}, servo_angle{i}, 'g-', 'LineWidth', 1.2);
    hold on
end
grid on
legend('Servo Angle', 'Location','northeast') % Legend
xlabel('t, s') % Add axis labels
ylabel('Servo Angle')
title('ACS Flight')
f4.Position = [100,100,800,500]; % this is based on your screen and preference

f5=figure(5);
for i = 1:length(Vinishes)
    plot(t{i}, e{i}, 'r--', 'LineWidth', 1.2);
    hold on
end
grid on
legend('Error', 'Location','northeast') % Legend
xlabel('t, s') % Add axis labels
ylabel('Error, m')
title('ACS Flight')
f5.Position = [100,100,800,500]; % this is based on your screen and preference

f6=figure(6);
for i = 1:length(Vinishes)
    plot(t{i}, drag{i}, '-', 'LineWidth',linewidth,'Color',color6);
    hold on
end
grid on
legend('Drag', 'FontSize', fontSize, 'Location', 'southeast'); % Legend
xlabel('t, s') % Add axis labels
ylabel('D(t), Lbf')
title('ACS Flight')
f6.Position = [100,100,800,500]; 

% close all

% f7=figure(7);
% plot(apogees, '^', 'LineWidth',linewidth,'Color',color2);
% hold on 
% yline(apogee_target, 'k-', 'LineWidth', 1.2);
% yline(apogees_avg, 'LineWidth',linewidth,'Color',color1);
% grid on
% legend('Apogee', 'Target Apogee', 'Average Apogee',  'FontSize', fontSize, 'Location', 'southeast'); % Legend
% xlabel('Test Run') % Add axis labels
% ylabel('Apogee, ft')
% title('ACS Flight')
% f7.Position = [100,100,800,500]; 

