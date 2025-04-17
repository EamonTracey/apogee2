function plot_quaternion_direction(q, fig)
    sv = style_values(); % Load in style values for plotting

    % q should be a 4-element vector: [w, x, y, z]
    
    % Normalize the quaternion
    q = q / norm(q);

    % Extract components
    w = q(1);
    x = q(2);
    y = q(3);
    z = q(4);

    % Define the original vector (zenith)
    v = [0; 0; 1];

    % Compute the rotated vector using quaternion rotation
    % Formula: v_rot = q * v * q_conj
    q_vec = [x; y; z];
    t = 2 * cross(q_vec, v);
    v_rot = v + w * t + cross(q_vec, t);

    % Plot the original and rotated vectors
    f = figure(fig);
    f.Position = [100,100,800,500]; 
    quiver3(0, 0, 0, 0, 0, 1, 'k', 'LineWidth', 2, 'MaxHeadSize', 0.5); % Original zenith (black)
    hold on;
    quiver3(0, 0, 0, v_rot(1), v_rot(2), v_rot(3), 'r', 'LineWidth', 2, 'MaxHeadSize', 0.5); % Rotated vector (red)

    % Set plot limits and labels
    axis equal;
    grid on;
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    title('Quaternion Rotation of Zenith Vector');
    legend('Original Zenith', 'Rotated Direction');
    view(3);
end
