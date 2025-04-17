wind_dir = [0; 1; 0]
wind_dir_perp = [-wind_dir(2); wind_dir(1); 0]


rotation_axis_dir = [0; 1; 0];
% rightVector_body = Tb2e*rightVector_body;
bodyAxisVec_earth = [0; 0; 1];
velocityVec_earth = [-ef10; 0; 200];

v_hat = velocityVec_earth / norm(velocityVec_earth)
b_hat = bodyAxisVec_earth / norm(bodyAxisVec_earth)

% Compute angle (magnitude) in radians
angle_mag = acosd(dot(v_hat, b_hat))

% Compute cross product to determine sign
cross_prod = cross(v_hat, b_hat)

% Determine sign using dot product with reference "up" vector
sign_alpha = sign(dot(cross_prod, rightVector_body));

% Apply sign to angle
aoa = sign_alpha * angle_mag