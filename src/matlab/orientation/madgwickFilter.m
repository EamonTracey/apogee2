function [q, error] = madgwickFilter(q, gyro, accel, dt, beta)
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
        omega = [0 gyro_corrected(:)'];
        dq = 0.5 * quatmultiply(q, omega);
    else
        accel = accel / norm(accel);
    
        % Quaternion elements
        qw = q(1); qx = q(2); qy = q(3); qz = q(4);
    
        % Estimated gravity direction from current quaternion
        g_dir = [2*(qx*qz - qw*qy);
             2*(qw*qx + qy*qz);
             qw^2 - qx^2 - qy^2 + qz^2];
    
        % Gradient descent error (cross product between measured and estimated gravity)
        error = cross(g_dir, accel);

        % Convert error into quaternion derivative (scaled)
        dq = 0.5 * quatmultiply(q, [0 gyro(:)']);
        correction = [0, error];
        dq = dq - beta * correction;
    end

    % Quaternion update with corrected gyro
    q = q + dq * dt;
    q = q / norm(q);
end