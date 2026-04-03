%% SCARA Robot Sliding Mode Control (SMC) Simulation
% Description: This script simulates a 3-DOF SCARA robot using Sliding Mode 
% Control to reach a desired joint configuration. It includes the non-linear 
% dynamics, reaching law implementation, and visualization of results.
clear; clc; close all;

%% 1. Simulation Parameters & Initial Conditions
tspan = [0 5];             % Simulation duration (seconds)
x0 = [0; 0; 0; 0; 0; 0];    % Initial state: [q1; q2; d3; dq1; dq2; dd3]

% Desired Setpoints (Target positions for the 3 joints)
qd = [pi/2; pi/4; 0.2];     % [Theta1 (rad); Theta2 (rad); d3 (m)]
dqd = [0; 0; 0];            % Desired velocity (zero for setpoint tracking)

% ODE Solver options for high precision
options = odeset('RelTol', 1e-4, 'AbsTol', 1e-4);

%% 2. Run Numerical Integration
% We use ode15s as robot dynamics can be "stiff" due to control discontinuities
[t, x] = ode15s(@(t,x) robot_dynamics(t, x, qd, dqd), tspan, x0, options);

%% 3. Post-Processing: Data Extraction & Logging
q1 = x(:, 1);  dq1 = x(:, 4);  % Joint 1 (Revolute)
q2 = x(:, 2);  dq2 = x(:, 5);  % Joint 2 (Revolute)
d3 = x(:, 3);  dd3 = x(:, 6);  % Joint 3 (Prismatic)

A = diag([5, 5, 5]); % Sliding surface slope (lambda) - must match function below
S_history = zeros(length(t), 3);
u_history = zeros(length(t), 3); 

% Re-calculate control inputs (u) and sliding surfaces (S) for plotting
for i = 1:length(t)
    q_i = [q1(i); q2(i); d3(i)];
    dq_i = [dq1(i); dq2(i); dd3(i)];
    
    e_i = q_i - qd;
    e_dot_i = dq_i - dqd;
    
    % S = lambda*e + e_dot
    S_history(i, :) = (A * e_i + e_dot_i)';
  
    % Extract torque/force values from the dynamics function
    [~, u_val] = robot_dynamics(t(i), x(i,:)', qd, dqd);
    u_history(i, :) = u_val';
end

%% 4. Visualization
% Figure 1: Convergence to the Sliding Surface
figure('Name', 'Sliding Surfaces');
plot(t, S_history(:,1), 'r', t, S_history(:,2), 'b', t, S_history(:,3), 'k', 'LineWidth', 1.5);
title('Sliding Surface Convergence (S \rightarrow 0)');
xlabel('Time (s)'); ylabel('S value');
legend('S_1', 'S_2', 'S_3');
grid on;

% Figure 2: Trajectory Tracking (Position)
figure('Name', 'Joint Positions');
plot(t, q1, 'r', t, q2, 'b', t, d3, 'k', 'LineWidth', 1.5);
hold on;
yline(qd(1), 'r--', 'Target \theta_1');
yline(qd(2), 'b--', 'Target \theta_2');
yline(qd(3), 'k--', 'Target d_3');
title('Joint Position vs Time');
xlabel('Time (s)'); ylabel('Position (rad or m)');
legend('\theta_1', '\theta_2', 'd_3');
grid on;

% Figure 3: Computed Control Effort
figure('Name', 'Control Inputs');
plot(t, u_history(:,1), 'r', t, u_history(:,2), 'b', t, u_history(:,3), 'k', 'LineWidth', 1.5);
title('Control Input (Torque/Force) vs Time');
xlabel('Time (s)'); ylabel('Control Effort (Nm/N)');
legend('\tau_1', '\tau_2', 'F_3');
grid on;

%% 5. Robot Dynamics & SMC Controller Function
function [dxdt, u] = robot_dynamics(~, x, qd, dqd)
    % State decomposition
    q  = x(1:3);    % Current positions
    dq = x(4:6);    % Current velocities
    
    theta2 = q(2);  % Local variable for mass matrix calculations
    
    % Tracking Error
    e = q - qd;
    e_dot = dq - dqd;
    
    %% --- Dynamic Model of the SCARA Robot ---
    % Inertia Matrix M(q)
    M_matrix = [8.42179 + 0.009*cos(theta2), 5.6086 + 0.0045*cos(theta2), 0;
               (1/4)*(5.1064 + 0.018*cos(theta2)), 1.2766, 0;
               0, 0, 0.0025];
               
    % Gravity Vector G(q)
    G_matrix = [0; 0; 0.04905];
                
    % Coriolis and Centrifugal Matrix C(q, dq)
    C_matrix = [1.0 - 0.0045*sin(theta2*dq(1)), (-0.0045*sin(theta2*dq(1)))-(0.0045*sin(theta2*dq(2))), 0;
                0.0045*sin(theta2*dq(1)), 1, 0;
                0, 0, 1];
                
    % Combine Non-linear terms
    CG = C_matrix * dq + G_matrix;
    Minv = inv(M_matrix); 
    fx = -Minv * CG; % Drift characteristics
        
    %% --- Sliding Mode Controller Design ---
    % Define Sliding Surface: S = A*e + e_dot
    % A (Lambda) determines the speed of convergence once on the surface
    A = diag([5, 5, 5]); 
    S = A * e + e_dot; 
    
    % Beta Gain: Ensures the "Reaching Condition" S_dot * S < 0 is met
    % Includes the compensation for system dynamics (rho) + a constant (beta0)
    rho = abs( M_matrix * (A * e_dot + fx) ); 
    beta0 = [5; 5; 0.1];  
    beta_x = rho + beta0;
    
    % Use a Saturation Approximation to reduce Chattering
    % Instead of sign(S), we use S / (|S| + epsilon)
    epsilon = 0.05;
    sgn_approx = S ./ (abs(S) + epsilon); 
    
    % Control Law: u = -Beta * sat(S/epsilon)
    u = -beta_x .* sgn_approx; 
    
    %% --- Equation of Motion ---
    % acceleration = M^-1 * (u - C*dq - G)
    ddq = M_matrix \ (u - CG);
    
    % Output state derivative
    dxdt = [dq; ddq];
end