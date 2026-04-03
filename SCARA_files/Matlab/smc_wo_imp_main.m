%% SCARA SMC with Impulse Disturbance
% Description: This script simulates the robustness of a Sliding Mode Controller
% for a 3-DOF SCARA robot. It includes a scripted disturbance at t=2.0s to 
% visualize how the controller recovers and maintains stability.

clear; clc; close all;

%% 1. Simulation Setup
tspan = [0 5]; 
x0 = zeros(6,1);              % [q1; q2; d3; dq1; dq2; dd3]
qd = [pi/2; pi/4; 0.2];       % Target Setpoint
dqd = [0; 0; 0];              % Target Velocity

% ode23s is preferred here due to the discontinuous 'sign' function and 
% the sudden impulse disturbance, both of which introduce stiffness.
options = odeset('RelTol', 1e-4, 'AbsTol', 1e-4);
[t, x] = ode23s(@(t,x) robot_dynamics(t, x, qd, dqd), tspan, x0, options);

%% 2. Data Processing & Control Logging
A = diag([5, 5, 5]); % Sliding surface slope (Lambda)
S_history = zeros(length(t), 3); 
u_history = zeros(length(t), 3);

for i = 1:length(t)
    e = x(i, 1:3)' - qd; 
    edot = x(i, 4:6)' - dqd;
    
    % Track the sliding surface values over time
    S_history(i, :) = (A * e + edot)';
    
    % Re-calculate u to visualize the control effort response to the disturbance
    [~, u_val] = robot_dynamics(t(i), x(i,:)', qd, dqd);
    u_history(i, :) = u_val';
end

%% 3. Global Plot Styling (LaTeX)
set(0, 'DefaultTextInterpreter', 'latex');
set(0, 'DefaultLegendInterpreter', 'latex');
set(0, 'DefaultAxesTickLabelInterpreter', 'latex');
fSize = 12; lw = 1.8;

%% 4. Automated Plotting Loop
figNames = {'Fig1_Sliding_Clean', 'Fig2_Position_Clean', 'Fig3_Control_Clean'};
dataCell = {S_history, x(:,1:3), u_history};
yLabs = {'Sliding Surface $S(t)$', 'Position (rad/m)', 'Control Input $u$ (Nm/N)'};

for k = 1:3
    hFig = figure('Units', 'inches', 'Position', [1+k, 1+k, 6, 4.5], 'Color', 'w');
    ax = axes('LineWidth', 1.2, 'FontSize', fSize, 'Box', 'off', 'TickDir', 'out');
    hold on; grid on;
    
    % Plot Data
    plot(t, dataCell{k}, 'LineWidth', lw);
    
    xlabel('Time $t$ (s)');
    ylabel(yLabs{k});
    
    % Legends based on data type
    if k == 1
        lgd = legend('$S_1$', '$S_2$', '$S_3$', 'Location', 'southeast'); 
    elseif k == 2
        lgd = legend('$\theta_1$', '$\theta_2$', '$d_3$', 'Location', 'best');
    else
        lgd = legend('$\tau_1$', '$\tau_2$', '$F_3$', 'Location', 'best'); 
    end
    lgd.BackgroundAlpha = 0.5;
    
    % --- Graphical Annotations (Axes Arrows) ---
    drawnow; 
    pos = get(ax, 'Position'); 
    annotation('arrow', [pos(1), pos(1)+pos(3)+0.04], [pos(2), pos(2)], 'LineWidth', 1.5);
    annotation('arrow', [pos(1), pos(1)], [pos(2), pos(2)+pos(4)+0.04], 'LineWidth', 1.5);
    
    % Export for documentation
    exportgraphics(hFig, [figNames{k}, '.png'], 'Resolution', 600);
end

%% 5. Robot Dynamics with Disturbance Injection
function [dxdt, u] = robot_dynamics(t, x, qd, dqd)
    % State decomposition
    q = x(1:3); dq = x(4:6);
    e = q - qd; edot = dq - dqd;
    
    % Dynamic Matrices (Mass, Coriolis, Gravity)
    M = [8.42179+0.009*cos(x(2)), 5.6086+0.0045*cos(x(2)), 0; 
         (5.1064+0.018*cos(x(2)))/4, 1.2766, 0; 
         0, 0, 0.0025];
    G = [0; 0; 0.04905];
    C = [1.0-0.0045*sin(x(2)*x(4)), -0.0045*sin(x(2)*x(4))-0.0045*sin(x(2)*x(5)), 0; 
         0.0045*sin(x(2)*x(4)), 1, 0; 
         0, 0, 1];
         
    CG = C*dq + G;
    
    % --- SMC Controller Law ---
    % Sliding Surface: S = Lambda*e + edot
    S = diag([5, 5, 5])*e + edot;
    
    % Nonlinear compensation gain (rho)
    rho = abs(M * (diag([5, 5, 5])*edot + (-M\CG)));
    
    % Control Input: u = -(rho + beta)*sgn(S)
    u = -(rho + [5; 5; 0.1]) .* sign(S);
    
    % INjecting disturbance here
    % This simulates an external impact or sudden load change
    dist = [0; 0; 0];
    if t >= 2.0 && t <= 2.05
        % Applying a momentary 30 Nm/N torque/force pulse
        dist = [30; 30; 5]; 
    end
    
    % Acceleration: ddq = M^-1 * (u - C*dq - G + Disturbance)
    ddq = M\(u - CG + dist);
    dxdt = [dq; ddq];
end