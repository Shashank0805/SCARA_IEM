% SCARA Robot Animation in MATLAB

% Arm lengths
L1 = 0.3;  % First link length (m)
L2 = 0.25; % Second link length (m)

% Time steps
frames = 100;
theta1_vals = linspace(-pi/4, pi/4, frames);  % Joint 1 motion
theta2_vals = linspace(pi/4, -pi/4, frames);  % Joint 2 motion

% Set up figure
figure;
axis equal;
axis([-0.6, 0.6, -0.6, 0.6]);
grid on;
title('SCARA Robot Animation');
xlabel('X (m)');
ylabel('Y (m)');
hold on;

% Trail history
trail_x = [];
trail_y = [];

for i = 1:frames
    theta1 = theta1_vals(i);
    theta2 = theta2_vals(i);

    % Joint positions
    x1 = L1 * cos(theta1);
    y1 = L1 * sin(theta1);
    x2 = x1 + L2 * cos(theta1 + theta2);
    y2 = y1 + L2 * sin(theta1 + theta2);

    % Clear previous arm
    cla;

    % Draw robot arm
    plot([0, x1, x2], [0, y1, y2], 'bo-', 'LineWidth', 4);

    % Store and plot trail
    trail_x(end+1) = x2;
    trail_y(end+1) = y2;
    plot(trail_x, trail_y, 'r-', 'LineWidth', 1.5);

    % Pause to create animation effect
    pause(0.05);
end
