function predicted_apogee = Six_DOF_RK4_Apogee_Prediction_fn(state, vehicle, motor, environment, t)
%% Apogee Prediction
% This code uses the current state of the rocket to predict its projected 
% apogee assuming the flap angle remains the same.

% Property of AKΣ

% Author: William Teasley

% Date: 2 April 2025

% -------------------------------------------------------------------------
x(:,1) = state; % initial conditions
dxdt(:,1) = zeros(length(x), 1);
i = 1;
dt = 0.025;

if t<0
    t = dt;
end

while x(6,i) > -1 
    ky1 = calculate_derivatives(x(:, i), vehicle, motor, environment, t);
    ky2 = calculate_derivatives((x(:, i)+0.5*dt*ky1.vec), vehicle, motor, environment, t);
    ky3 = calculate_derivatives((x(:, i)+0.5*dt*ky2.vec), vehicle, motor, environment, t);
    ky4 = calculate_derivatives((x(:, i)+ky3.vec*dt), vehicle, motor, environment, t);    

    dxdt(:, i) = (1/6)*(ky1.vec+2*ky2.vec+2*ky3.vec+ky4.vec);
    x(:, i+1) = x(:, i) + dxdt(:, i).*dt;
    i = i+1;
    t = t + dt;
end

predicted_apogee = max(x(3, :));
% -------------------------------------------------------------------------