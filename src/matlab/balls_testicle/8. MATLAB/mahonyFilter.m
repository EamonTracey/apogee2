function [q, error] = mahonyFilter(q, gyro, accel, dt, Kp, Ki)
% mahonyFilter: Updates quaternion using Mahony filter
% Inputs:
%   q     - current quaternion [qw qx qy qz]'
%   gyro  - gyro measurements [gx gy gz] in rad/s
%   accel - accelerometer [ax ay az] in m/s^2
%   dt    - timestep in seconds
%   Kp    - proportional gain
%   Ki    - integral gain
% Output:
%   q     - updated quaternion [qw qx qy qz]'
global g

    % Ensure q is a column vector
    q = q(:)';

    persistent eInt
    if isempty(eInt)
        eInt = [0 0 0];
    end

    % Normalize accelerometer measurement
    if norm(accel) == 0
        return; % avoid division by zero
    end

    % Check if acceleration is 10% above g
    if norm(accel) > 1.1*g
        gyro_corrected = gyro;
    else
        accel = accel / norm(accel);
    
        % Estimated gravity direction from current quaternion
        R = quat2rotm(q); % 3x3 rotation matrix from body to inertial
        v = R(:,3)         % third column is "down" direction in body frame
    
        % Error is the cross product between estimated and measured gravity
        error = cross(v, accel)
    
        % Integrate error
        eInt = eInt + error * dt
    
        % Apply PI correction to gyro
        gyro_corrected = gyro + Kp * error + Ki * eInt;
    end

    % Quaternion update with corrected gyro
    omega = [0 gyro_corrected(:)'];
    q_dot = 0.5 * quatmultiply(q, omega);
    q = q + q_dot * dt;
    q = q / norm(q);
end