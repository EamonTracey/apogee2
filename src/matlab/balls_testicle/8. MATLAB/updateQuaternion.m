function q_new = updateQuaternion(q, gyro, dt)
% -------------------------------------------------------------------------
% updateQuaternion: Propagates quaternion using gyroscope data
% Inputs:
%   q    - current quaternion [qw qx qy qz]
%   gyro - angular velocity vector [rad/s] in body frame [gx gy gz]
%   dt   - timestep [s]
% Output:
%   q_new - updated quaternion [qw qx qy qz]

    % Ensure q is a column vector
    q = q(:)'; 
    
    % Angular velocity quaternion (pure quaternion)
    omega = [0 gyro(:)'];

    % Quaternion multiplication (q_dot = 0.5 * q * omega)
    q_dot = 0.5 * quatmultiply(q, omega);

    % Integrate quaternion
    q_new = q + q_dot * dt;

    % Normalize to prevent drift
    q_new = q_new / norm(q_new);
end

