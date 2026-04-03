%% Sliding Mode Control (SMC) of a Simple Pendulum
% Description: This script simulates a simple pendulum under Sliding Mode 
% Control. It demonstrates the use of a switching control law (sign function) 
% to maintain stability despite damping/uncertainties.
clear; clc; close all;
%% 1. Parameter Initialization
% Physical constants of the pendulum system
params.Mass = 1;
params.Lenght = 1;
params.g = 9.81;
params.b = -1; % Active damping/friction coefficient
params.I = params.Mass * params.Lenght^2; % Moment of Inertia (I = mL^2)

% SMC Design Parameters
params.Slope = 5;  % Sliding surface slope (lambda)
params.K = 10;     % Switching gain (must be large enough to overcome disturbances)

% Simulation setup
x0 = [1; 0];       % Initial conditions: [Angle (rad); Angular Velocity (rad/s)]
tspan = [0 10];    % Time vector for simulation

%% 2. Numerical Integration
% Note: Using sign(S) often makes the system 'stiff' due to high-frequency 
% switching (chattering). If ode45 fails, switch to ode23s or ode15s.
[t, x] = ode45(@(t,x) pendulum_smc(t, x, params), tspan, x0);

theta = x(:,1);
omega = x(:,2);

%% 3. Post-Processing: Data Logging
% Pre-allocate memory for efficiency
S = zeros(length(t), 1);
u = zeros(length(t), 1);

for i = 1:length(t)
    x1 = theta(i);
    x2 = omega(i);
    
    % Re-calculating surface and control for plotting
    S(i) = params.Slope*x1 + x2;
    
    % Equivalent control: Cancels nominal nonlinear dynamics
    u_eq = params.b*x2 + params.Mass*params.g*params.Lenght*sin(x1) - params.I*params.Slope*x2;
    
    % Switching control: Provides robustness via high-frequency switching
    u_sw = -params.I * params.K * sign(S(i));
    
    u(i) = u_eq + u_sw;
end

%% 4. Visualization of Results
figure('Name', 'SMC Pendulum Analysis', 'Color', 'w');

% Top Left: Trajectory Tracking
subplot(2,2,1)
plot(t, theta, 'LineWidth', 2)
xlabel('Time (s)'); ylabel('\theta (rad)');
title('Pendulum Angle vs Time'); grid on;

% Top Right: Phase Portrait
subplot(2,2,2)
plot(theta, omega, 'LineWidth', 2)
xlabel('\theta (rad)'); ylabel('\omega (rad/s)');
title('Phase Portrait (\theta vs \omega)'); grid on;

% Bottom Left: Control Effort
subplot(2,2,3)
plot(t, u, 'LineWidth', 2)
xlabel('Time (s)'); ylabel('Control Input (Torque)');
title('Control Input (Note Chattering)'); grid on;

% Bottom Right: Sliding Surface Convergence
subplot(2,2,4)
plot(t, S, 'LineWidth', 2)
xlabel('Time (s)'); ylabel('Sliding Surface S');
title('Sliding Surface S \rightarrow 0'); grid on;

%% 5. SMC Control Law and System Dynamics Function
function dxdt = pendulum_smc(~, x, params)
    % State Assignment
    x1 = x(1); % Theta (Position)
    x2 = x(2); % Theta_dot (Velocity)
    
    % Local variables for readability
    M = params.Mass; L = params.Lenght; g = params.g;
    b = params.b; I = params.I; Slope = params.Slope; K = params.K;

    %% --- SMC Controller Logic ---
    % Define the sliding surface S = lambda*e + e_dot
    % Here, target position is 0, so error e = x1
    S = Slope*x1 + x2;
    
    % Equivalent Control (Model-Based)
    u_eq = b*x2 + M*g*L*sin(x1) - I*Slope*x2;
    
    % Switching Control (Robust Part)
    % Uses the signum function which leads to the "Chattering" effect
    u_sw = -I * K * sign(S);
    
    % Total Control Torque
    u = u_eq + u_sw;

    %% --- Plant Dynamics ---
    dx1 = x2;
    % State-space representation of the second-order pendulum equation
    % I*dd_theta + b*d_theta + mgl*sin(theta) = u
    dx2 = -(b/I)*x2 - (M*g*L/I)*sin(x1) + (1/I)*u;
    
    dxdt = [dx1; dx2];
end