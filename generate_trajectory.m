function [etad, etad_dot, etad_ddot] = generate_trajectory(t, trajectory_type, dt)
    % generate_trajectory Generates various trajectories and their derivatives.
    %
    %   [etad, etad_dot, etad_ddot] = generate_trajectory(t, trajectory_type, dt)
    %
    %   Inputs:
    %       t               - Time vector.
    %       trajectory_type - String specifying the trajectory type ('8', 'm', 'circle').
    %       dt              - Time step (unused in this implementation).
    %
    %   Outputs:
    %       etad            - Desired trajectory (6 x length(t)).
    %       etad_dot        - Desired velocity (6 x length(t)).
    %       etad_ddot       - Desired acceleration (6 x length(t)).

    etad = zeros(6, length(t));
    etad_dot = zeros(6, length(t));
    etad_ddot = zeros(6, length(t));

    switch lower(trajectory_type)
        case '8' % 8-shape (lemniscate)
            a = 5; % Scale factor
            omega = 0.1; % Frequency
            for i = 1:length(t)
                denom = 1 + sin(omega * t(i))^2;
                % Desired position
                etad(:, i) = [a * cos(omega * t(i)) / denom;
                              a * sin(omega * t(i)) * cos(omega * t(i)) / denom;
                              2 - 2 * cos(omega * t(i));
                              0; 0; 0];
                % Desired velocity (first derivatives)
                denom_deriv = 2 * omega * sin(omega * t(i)) * cos(omega * t(i));
                etad_dot(:, i) = [-a * omega * sin(omega * t(i)) / denom - ...
                                  a * cos(omega * t(i)) * denom_deriv / denom^2;
                                  a * omega * (cos(omega * t(i))^2 - sin(omega * t(i))^2) / denom - ...
                                  a * sin(omega * t(i)) * cos(omega * t(i)) * denom_deriv / denom^2;
                                  2 * omega * sin(omega * t(i));
                                  0; 0; 0];
                % Desired acceleration (second derivatives)
                denom_ddot = 2 * omega^2 * (cos(omega * t(i))^2 - sin(omega * t(i))^2);
                etad_ddot(:, i) = [-a * omega^2 * cos(omega * t(i)) / denom - ...
                                   2 * a * omega * sin(omega * t(i)) * (-denom_deriv) / denom^2 - ...
                                   a * cos(omega * t(i)) * (denom_ddot / denom^2 - 2 * denom_deriv^2 / denom^3);
                                   -a * omega^2 * 2 * sin(omega * t(i)) * cos(omega * t(i)) / denom - ...
                                   a * omega * (cos(omega * t(i))^2 - sin(omega * t(i))^2) * (-denom_deriv) / denom^2 - ...
                                   a * sin(omega * t(i)) * cos(omega * t(i)) * (denom_ddot / denom^2 - 2 * denom_deriv^2 / denom^3);
                                   2 * omega^2 * cos(omega * t(i));
                                   0; 0; 0];
                % Yaw based on line of sight
                etad(6, i) = atan2(etad_dot(2, i), etad_dot(1, i));
            end

        case 'm' % M-shape
            amplitude = 3; % Amplitude
            frequency = 0.1; % Frequency
            vertical_offset = 2; % Vertical offset
            for i = 1:length(t)
                % Parametric equations for the M-shape
                etad(:, i) = [amplitude * sin(frequency * t(i));
                              amplitude * sin(2 * frequency * t(i));
                              vertical_offset - amplitude * cos(frequency * t(i));
                              0; 0; 0];
                % Desired velocity
                etad_dot(:, i) = [amplitude * frequency * cos(frequency * t(i));
                                  2 * amplitude * frequency * cos(2 * frequency * t(i));
                                  amplitude * frequency * sin(frequency * t(i));
                                  0; 0; 0];
                % Desired acceleration
                etad_ddot(:, i) = [-amplitude * frequency^2 * sin(frequency * t(i));
                                   -4 * amplitude * frequency^2 * sin(2 * frequency * t(i));
                                   amplitude * frequency^2 * cos(frequency * t(i));
                                   0; 0; 0];
                % Yaw based on line of sight
                etad(6, i) = atan2(etad_dot(2, i), etad_dot(1, i));
            end

        case 'circle' % Circular trajectory
            for i = 1:length(t)
                etad(:, i) = [2 * sin(0.1 * t(i));
                              2 - 2 * cos(0.1 * t(i));
                              2 - 2 * cos(0.1 * t(i));
                              (pi/6) * sin(0.1 * t(i));
                              (-pi/4) * sin(0.1 * t(i));
                              (pi/3) * sin(0.1 * t(i))];
                etad_dot(:, i) = [0.2 * cos(0.1 * t(i));
                                  0.2 * sin(0.1 * t(i));
                                  0.2 * sin(0.1 * t(i));
                                  0.1 * (pi/6) * cos(0.1 * t(i));
                                  -0.1 * (pi/4) * cos(0.1 * t(i));
                                  0.1 * (pi/3) * cos(0.1 * t(i))];
                etad_ddot(:, i) = [-0.02 * sin(0.1 * t(i));
                                   0.02 * cos(0.1 * t(i));
                                   0.02 * cos(0.1 * t(i));
                                   -0.01 * (pi/6) * sin(0.1 * t(i));
                                   0.01 * (pi/4) * sin(0.1 * t(i));
                                   -0.01 * (pi/3) * sin(0.1 * t(i))];
                % Yaw based on line of sight
                etad(6, i) = atan2(etad_dot(2, i), etad_dot(1, i));
            end

        otherwise
            error('Invalid trajectory type. Choose ''8'', ''m'', or ''circle''.');
    end
end
