function [etad, etad_dot, etad_ddot] = generate_trajectory(t, trajectory_type, dt)
% generate_trajectory Generates various trajectories and their derivatives.
%
%   [etad, etad_dot, etad_ddot] = generate_trajectory(t, trajectory_type, dt)
%
%   Inputs:
%       t               - Time vector.
%       trajectory_type - String specifying the trajectory type ('S', '8', 'T', 'circle').
%       dt              - Time step.
%
%   Outputs:
%       etad            - Desired trajectory (6 x length(t)).
%       etad_dot        - Desired velocity (6 x length(t)).
%       etad_ddot       - Desired acceleration (6 x length(t)).

    etad = zeros(6, length(t));
    etad_dot = zeros(6, length(t));
    etad_ddot = zeros(6, length(t));

    switch lower(trajectory_type)
        case 's' % S-shape
           for i = 1:length(t) 
            fi = 0.01;
             etad(:,i) = [4*sin(2*fi*t(i));4-4*cos(fi*t(i));(2-2*cos(fi*t(i)));0;0;0];
             etad_dot = [4*2*fi*cos(2*fi*t(i));4*fi*sin(fi*t(i));2*fi*sin(fi*t(i));0;0;0];
             etad_ddot = [-4*4*fi*fi*sin(2*fi*t(i));4*fi*fi*cos(fi*t(i));2*fi*fi*cos(fi*t(i));0;0;0];
             etad(6,i) = atan2(etad_dot(2),etad_dot(1)); % based on LoS
           end


        case '8' % 8-shape (lemniscate)
            a = 5; % Scale factor for the 8-shape
            omega = 0.1; % Frequency of the 8-shape
            for i = 1:length(t)
                etad(:, i) = [a * cos(omega * t(i)) / (1 + sin(omega * t(i))^2);
                              a * sin(omega * t(i)) * cos(omega * t(i)) / (1 + sin(omega * t(i))^2);
                              2 - 2 * cos(omega * t(i));
                              0; 0; 0];
                etad_dot = [5 * 0.1 * cos(0.1 * t(i));-5 * 0.1 * sin(0.1 * t(i));2 * 0.1 * sin(0.1 * t(i));0;0;0];
                etad_ddot = [-5 * 0.1 * 0.1 * sin(0.1 * t(i));-5 * 0.1 * 0.1 * cos(0.1 * t(i));2 * 0.1 * 0.1 * cos(0.1 * t(i));0;0;0];

                
            end
       case 'm' 
            for i = 1:length(t)
       
            amplitude = 3; % 
            frequency = 0.1; % 
            vertical_offset = 2; 

        % Parametric equations for the M-shape
            etad(:, i) = [amplitude * sin(frequency * t(i));
                      amplitude * sin(2 * frequency * t(i));
                      vertical_offset - amplitude * cos(frequency * t(i));
                      0; 0; 0];

        
            etad_dot = [amplitude * frequency * cos(frequency * t(i));
                          2 * amplitude * frequency * cos(2 * frequency * t(i));
                          amplitude * frequency * sin(frequency * t(i));
                          0; 0; 0];

            etad_ddot = [-amplitude * frequency^2 * sin(frequency * t(i));
                           -4 * amplitude * frequency^2 * sin(2 * frequency * t(i));
                           amplitude * frequency^2 * cos(frequency * t(i));
                           0; 0; 0];

            etad(6, i) = atan2(etad_dot(2), etad_dot(1)); 
            end 
            

        case 'circle'
            for i = 1:length(t)
                 etad(:,i) = [2*sin(0.1*t(i));2-2*cos(0.1*t(i));2-2*cos(0.1*t(i));0*pi/6*sin(0.1*t(i));-0*pi/4*sin(0.1*t(i));0*pi/3*sin(0.1*t(i))] ;
                 etad_dot = [0.2*cos(0.1*t(i));0.2*sin(0.1*t(i));0.2*sin(0.1*t(i));0*0.1*pi/6*cos(0.1*t(i));-0*0.1*pi/4*cos(0.1*t(i));0*0.1*pi/3*cos(0.1*t(i))];
                 etad_ddot = [-0.2*0.1*sin(0.1*t(i));0.2*0.1*cos(0.1*t(i));0.2*0.1*cos(0.1*t(i));-0*0.1*0.1*pi/6*sin(0.1*t(i));+0*0.1*0.1*pi/4*sin(0.1*t(i));-0*0.1*0.1*pi/3*sin(0.1*t(i))];
                 etad(6,i) = atan2(etad_dot(2),etad_dot(1)); % based on LoS
            end
        otherwise
            error('Invalid trajectory type. Choose ''S'', ''8'', ''T'', or ''circle''.');
    end
end