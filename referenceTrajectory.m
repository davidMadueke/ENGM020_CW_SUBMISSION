function desired_state = referenceTrajectory(t)
    % Define the reference trajectory based on time

    % Constants
    takeoff_duration = 2.0;  % Time duration for takeoff (seconds)
    hover_duration = 5.0;    % Time duration for steady flight (seconds)
    landing_duration = 2.0;  % Time duration for landing (seconds)

    % Takeoff maneuver
    if t <= takeoff_duration
        % Linearly interpolate position for takeoff from (0, 0, 0) to (0, 0, 3)
        desired_position = [0; 0; 3 * (t / takeoff_duration)];
        desired_velocity = [0; 0; 3 / takeoff_duration];
        % EulerAngles and angular rates are initialized to zeros for simplicity
        desired_angles = [0; 0; 0];
        desired_angular_rates = [0; 0; 0];
    % Waypoint Navigation (Steady Flight)
    elseif t <= takeoff_duration + hover_duration
        % Maintain a constant altitude at (0, 0, 3) going to (5,0,3)
        desired_position = [5 * ((t-takeoff_duration) / hover_duration); 0; 3];
        desired_velocity = [0; 0; 0];
        % EulerAngles and angular rates are initialized to zeros for simplicity
        desired_angles = [0; 0; 0];
        desired_angular_rates = [0; 0; 0];
    % Vertical Landing Maneuver
    elseif t <= takeoff_duration + hover_duration + landing_duration
        % Linearly interpolate position for landing from (5, 0, 3) to (5, 0, 0)
        desired_position = [5; 0; 3 - 3 * ((t - takeoff_duration - hover_duration) / landing_duration)];
        desired_velocity = [0; 0; -3 / landing_duration];
        % EulerAngles and angular rates are initialized to zeros for simplicity
        desired_angles = [0; 0; 0];
        desired_angular_rates = [0; 0; 0];
    else
        % Quadcopter is on the ground 
        desired_position = [0; 0; 0];
        desired_velocity = [0; 0; 0];
        % EulerAngles and angular rates are initialized to zeros for simplicity
        desired_angles = [0; 0; 0];
        desired_angular_rates = [0; 0; 0];
    end

    % Combine position, velocity, Euler angles, and angular rates into the state vector
    % In the form
    % [z z_dot psi psi_dot x x_dot phi phi_dot y y_dot theta theta_dot]^T
    [x,y,z] = deal(desired_position(1), desired_position(2), desired_position(3));
    [x_dot, y_dot, z_dot] = deal(desired_velocity(1), desired_velocity(2), desired_velocity(3));
    
    [psi, phi, theta] = deal(desired_angles(1), desired_angles(2), desired_angles(3));
    [psi_dot, phi_dot, theta_dot] = deal(desired_angular_rates(1), desired_angular_rates(2), desired_angular_rates(3));

    desired_state = [z z_dot psi psi_dot x x_dot phi phi_dot y y_dot theta theta_dot]';
end

