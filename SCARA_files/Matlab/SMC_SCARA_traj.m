%% SCARA Cartesian Trajectory Tracking (Circular Path)
% Description: This script implements a Sliding Mode Controller to make the 
% SCARA end-effector follow a circular path in the XY plane. It utilizes 
% Inverse Kinematics for position and a Jacobian for velocity mapping.
clear; clc; close all;
%% 1. Simulation & Robot Geometry
tspan = [0 5]; 
tf = tspan(2);
x0 = zeros(6,1);
options = odeset('RelTol', 1e-4, 'AbsTol', 1e-4);

% Link Lengths (m)
L1 = 0.5;
L2 = 0.4;

% Circle Parameters
xc = 0.2;       % Center X
yc = 0.3;       % Center Y
r_circle = 0.5; % Radius

% Run ODE solver using the trajectory-aware dynamics function
[t, x] = ode23(@(t,x) robot_dynamics(t, x, tf, L1, L2, xc, yc, r_circle), tspan, x0, options);

%% 2. Data Processing & Cartesian Mapping
A = diag([5, 5, 5]); 
S_history = zeros(length(t), 3); 
u_history = zeros(length(t), 3);
X_cartesian = zeros(length(t), 2); % Stores actual [X, Y] of end-effector

for i = 1:length(t)
    % Get desired joint states for current time step
    [qd, dqd] = get_desired_trajectory(t(i), tf, L1, L2, xc, yc, r_circle);
    
    e = x(i, 1:3)' - qd; 
    edot = x(i, 4:6)' - dqd;
    S_history(i, :) = (A * e + edot)';
    
    % Re-calculate control inputs for visualization
    [~, u_val] = robot_dynamics(t(i), x(i,:)', tf, L1, L2, xc, yc, r_circle);
    u_history(i, :) = u_val';
    
    % Forward Kinematics: Convert joint angles to XY coordinates
    theta1 = x(i, 1); theta2 = x(i, 2);
    X_cartesian(i, 1) = L1*cos(theta1) + L2*cos(theta1+theta2);
    X_cartesian(i, 2) = L1*sin(theta1) + L2*sin(theta1+theta2);
end

%% 3. Visualization Setup
set(0, 'DefaultTextInterpreter', 'latex');
set(0, 'DefaultLegendInterpreter', 'latex');
set(0, 'DefaultAxesTickLabelInterpreter', 'latex');
fSize = 12; lw = 1.8;

figNames = {'Fig1_Sliding_Clean', 'Fig2_Position_Clean', 'Fig3_Control_Clean', 'Fig4_Cartesian_Path'};
dataCell = {S_history, x(:,1:3), u_history, X_cartesian};
yLabs = {'Sliding Surface $S(t)$', 'Position (rad/m)', 'Control Input $u$', 'Y Position (m)'};

%% 4. Plotting Loop
for k = 1:4
    hFig = figure('Units', 'inches', 'Position', [1+k, 1+k, 6, 4.5], 'Color', 'w');
    ax = axes('LineWidth', 1.2, 'FontSize', fSize, 'Box', 'off', 'TickDir', 'out');
    hold on; grid on;
    
    if k == 4
        % Cartesian Path Plot: Comparing Ideal Circle vs. Actual Robot Path
        theta_ideal = linspace(0, 2*pi, 100);
        x_ideal = xc + r_circle * cos(theta_ideal);
        y_ideal = yc + r_circle * sin(theta_ideal);
        
        plot(x_ideal, y_ideal, 'r--', 'LineWidth', lw); % Target Path
        plot(X_cartesian(:,1), X_cartesian(:,2), 'k', 'LineWidth', lw); % Robot Path
        
        xlabel('X Position (m)');
        axis equal; % Important for viewing a true circle
        lgd = legend('Desired Path', 'Actual Path', 'Location', 'best');
    else
        % Time-series plots for S, q, and u
        plot(t, dataCell{k}, 'LineWidth', lw);
        xlabel('Time $t$ (s)');
        if k == 1
            lgd = legend('$S_1$', '$S_2$', '$S_3$', 'Location', 'best'); 
        elseif k == 2
            lgd = legend('$\theta_1$', '$\theta_2$', '$d_3$', 'Location', 'best');
        elseif k == 3
            lgd = legend('$\tau_1$', '$\tau_2$', '$F_3$', 'Location', 'best'); 
        end
    end
    ylabel(yLabs{k});
    lgd.BackgroundAlpha = 0.5;
    exportgraphics(hFig, [figNames{k}, '.png'], 'Resolution', 600);
end

%% 5. Trajectory Generation (Inverse Kinematics)
function [qd, dqd] = get_desired_trajectory(t, tf, L1, L2, xc, yc, r)
    t = min(t, tf);
    
    % Use a quintic-like polynomial to smoothly parameterize the circle angle p
    p = (6*pi*t^2)/(tf^2) - (4*pi*t^3)/(tf^3);
    dp = (12*pi*t)/(tf^2) - (12*pi*t^2)/(tf^3);
    
    % Cartesian Coordinates
    xd = xc + r * cos(p);
    yd = yc + r * sin(p);
    dxd = -r * sin(p) * dp;
    dyd = r * cos(p) * dp;
    
    % Inverse Kinematics for 2-Link Arm
    D = (xd^2 + yd^2 - L1^2 - L2^2) / (2 * L1 * L2);
    D = max(min(D, 1), -1); % Numerical clamping to avoid imaginary roots
    
    theta2d = atan2(sqrt(1 - D^2), D); % Elbow-up configuration
    theta1d = atan2(yd, xd) - atan2(L2 * sin(theta2d), L1 + L2 * D);
    
    % Jacobian Matrix calculation for velocity mapping (dq = J \ dx)
    J11 = -L1 * sin(theta1d) - L2 * sin(theta1d + theta2d);
    J12 = -L2 * sin(theta1d + theta2d);
    J21 = L1 * cos(theta1d) + L2 * cos(theta1d + theta2d);
    J22 = L2 * cos(theta1d + theta2d);
    J = [J11, J12; J21, J22];
    
    % Singularity Robustness Check
    if rcond(J) < 1e-6
        dq12 = [0; 0]; 
    else
        dq12 = J \ [dxd; dyd];
    end
    
    qd = [theta1d; theta2d; 0.2];
    dqd = [dq12(1); dq12(2); 0];
end

%% 6. Robot Dynamics & SMC Controller
function [dxdt, u] = robot_dynamics(t, x, tf, L1, L2, xc, yc, r)
    % Update target state based on current time
    [qd, dqd] = get_desired_trajectory(t, tf, L1, L2, xc, yc, r);
    
    q = x(1:3); dq = x(4:6);
    e = q - qd; edot = dq - dqd;
    
    % SCARA Dynamic Matrices
    M = [8.42179+0.009*cos(x(2)), 5.6086+0.0045*cos(x(2)), 0; 
        (5.1064+0.018*cos(x(2)))/4, 1.2766, 0; 
         0, 0, 0.0025];
    G = [0; 0; 0.04905];
    C = [1.0-0.0045*sin(x(2)*x(4)), -0.0045*sin(x(2)*x(4))-0.0045*sin(x(2)*x(5)), 0; 
         0.0045*sin(x(2)*x(4)), 1, 0; 
         0, 0, 1];
    CG = C*dq + G;
    
    % SMC Reaching Law logic
    S = diag([5, 5, 5])*e + edot;
    rho = abs(M * (diag([5, 5, 5])*edot + (-M\CG)));
    u = -(rho + [5; 5; 0.1]) .* sign(S);
    
    dxdt = [dq; M\(u - CG)];
end