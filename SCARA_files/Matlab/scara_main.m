%% SCARA SMC Simulation 
% Description: High-fidelity simulation and automated plotting of a 
% 3-DOF SCARA robot using Sliding Mode Control. 
clear; clc; close all;

%% --- 1. Simulation Setup ---
tspan = [0 5]; 
x0 = zeros(6,1);              % Initial states [q1, q2, d3, dq1, dq2, dd3]
qd = [pi/2; pi/4; 0.2];       % Target Joint Configuration
dqd = [0; 0; 0];              % Target Velocities

% ode23s is an adaptive step solver ideal for "stiff" SMC systems
options = odeset('RelTol', 1e-4, 'AbsTol', 1e-4);
[t, x] = ode23s(@(t,x) robot_dynamics(t, x, qd, dqd), tspan, x0, options);

%% --- 2. Post-Simulation Data Processing ---
lambda = 5; % Sliding surface slope constant
A = diag([lambda, lambda, lambda]); 
S_history = zeros(length(t), 3); 
u_history = zeros(length(t), 3); 

for i = 1:length(t)
    % Extract error and error derivative for current time step
    e = x(i, 1:3)' - qd; 
    edot = x(i, 4:6)' - dqd;
    
    % Calculate Sliding Surface: S = lambda*e + edot
    S_history(i, :) = (A * e + edot)';
    
    % Re-run dynamics function to capture the control input 'u' used at each step
    [~, u_val] = robot_dynamics(t(i), x(i,:)', qd, dqd);
    u_history(i, :) = u_val';
end

%% --- 3. Global Visualization Settings ---
% Enabling LaTeX for all text elements for professional publication quality
set(0, 'DefaultTextInterpreter', 'latex');
set(0, 'DefaultLegendInterpreter', 'latex');
set(0, 'DefaultAxesTickLabelInterpreter', 'latex');

fSize = 12; % Font size for labels
lw = 1.8;   % Line width for plots

%% --- 4. Automated Plotting & Export Loop ---
figNames = {'Fig1_Sliding_Clean', 'Fig2_Position_Clean', 'Fig3_Control_Clean'};
dataCell = {S_history, x(:,1:3), u_history};
yLabs = {'Sliding Surface $S(t)$', 'Position (rad/m)', 'Control Input $u$ (Nm/N)'};

for k = 1:3
    % Create figure with specific physical dimensions (6x4.5 inches)
    hFig = figure('Units', 'inches', 'Position', [1+k, 1+k, 6, 4.5], 'Color', 'w');
    
    % Set up axes: Remove box and use outward-facing ticks for "Clean" look
    ax = axes('LineWidth', 1.2, 'FontSize', fSize, 'Box', 'off', 'TickDir', 'out');
    hold on; grid on;
    
    % Plot current dataset
    plot(t, dataCell{k}, 'LineWidth', lw);
    
    xlabel('Time $t$ (s)');
    ylabel(yLabs{k});
    
    % Dynamic Legend Labeling
    if k == 1
        lgd = legend('$S_1$', '$S_2$', '$S_3$', 'Location', 'southeast'); 
    elseif k == 2
        lgd = legend('$\theta_1$', '$\theta_2$', '$d_3$', 'Location', 'best');
    else
        lgd = legend('$\tau_1$', '$\tau_2$', '$F_3$', 'Location', 'best'); 
    end
    lgd.BackgroundAlpha = 0.5; % Semi-transparent legend
    
    %% --- Aesthetic Annotations (Axis Arrows) ---
    drawnow; % Required to ensure get(ax, 'Position') returns correct rendered values
    pos = get(ax, 'Position'); % [left bottom width height]
    
    % Draw arrows at the ends of the X and Y axes
    annotation('arrow', [pos(1), pos(1)+pos(3)+0.04], [pos(2), pos(2)], 'LineWidth', 1.5);
    annotation('arrow', [pos(1), pos(1)], [pos(2), pos(2)+pos(4)+0.04], 'LineWidth', 1.5);
    
    % Export to high-resolution PNG for reports
    exportgraphics(hFig, [figNames{k}, '.png'], 'Resolution', 600);
end

%% --- 5. Robot Dynamics & Controller Definition ---
function [dxdt, u] = robot_dynamics(~, x, qd, dqd)
    % State: x = [q1; q2; d3; dq1; dq2; dd3]
    q = x(1:3); 
    dq = x(4:6);
    
    % Error vectors
    e = q - qd; 
    edot = dq - dqd;
    
    % Mass/Inertia Matrix M(q)
    M = [8.42179+0.009*cos(x(2)), 5.6086+0.0045*cos(x(2)), 0; 
         (5.1064+0.018*cos(x(2)))/4, 1.2766, 0; 
         0, 0, 0.0025];
    
    % Gravity Vector G(q)
    G = [0; 0; 0.04905];
    
    % Coriolis/Centrifugal Matrix C(q, dq)
    C = [1.0-0.0045*sin(x(2)*x(4)), -0.0045*sin(x(2)*x(4))-0.0045*sin(x(2)*x(5)), 0; 
         0.0045*sin(x(2)*x(4)), 1, 0; 
         0, 0, 1];
    
    CG = C*dq + G;
    
    % Define the Sliding Surface
    lambda_val = 5;
    S = diag([lambda_val, lambda_val, lambda_val])*e + edot;
    
    %% --- SMC Control Law ---
    % rho represents the bound on uncertainties/disturbances
    % beta_0 = [5; 5; 0.1] is the reaching gain
    rho = abs(M * (diag([lambda_val, lambda_val, lambda_val])*edot + (-M\CG)));
    
    % u = - (rho + beta_0) * sign(S)
    % Note: Using sign(S) here; for hardware, consider using sat(S/epsilon)
    u = -(rho + [5; 5; 0.1]) .* sign(S);
    
    % System Dynamics: ddq = M^-1 * (u - C*dq - G)
    ddq = M\(u - CG);
    dxdt = [dq; ddq];
end