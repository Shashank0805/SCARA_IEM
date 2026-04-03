clear; clc; close all; 

% --- System Definition (SCARA Robot Model) ---
A = [0, 0, 1, 0; 
     0, 0, 0, 1; 
     0, 0.000102771, 0.0008673, 0.000321434; 
     0, -0.000133507, -0.00122245, -0.000322415];
B = [0, 0; 0, 0; 1, 0; 0, 1];
C = [1, 0, 0, 0; 0, 1, 0, 0]; 
D = [0, 0; 0, 0];

% --- Diverse Weights Selection ---
% We use logspace to get 15 values ranging from 0.1 to 1000
% This shows a much wider variety of control behaviors than 1:15
q_weights = logspace(-1, 3, 15); 
sim_time = 8; 

% --- Figure Setup ---
fig = figure('Color', 'w', 'Units', 'inches', 'Position', [2, 2, 9, 6]);
hold on;

% Color map to make the 15 lines look distinct (from blue to red)
colors = jet(length(q_weights));

% --- LQR Iteration Loop ---
for k = 1:length(q_weights)
    Q_val = q_weights(k);
    R_val = 1; % Keeping R constant as the baseline
    
    Q_curr = Q_val * eye(4);
    R_curr = R_val * eye(2);
    
    % Compute LQR Gain
    K = lqr(A, B, Q_curr, R_curr);
    
    % Closed-Loop System
    sys_cl = ss(A - B*K, B, C, D);
    
    % Calculate Step Response
    [y, t] = step(sys_cl, sim_time); 
    
    % Plot with distinct colors
    plot(t, y(:,1,1), 'LineWidth', 1.6, 'Color', colors(k,:), ...
         'DisplayName', sprintf('Q/R Ratio: %.2f', Q_val/R_val));
end

% --- Labeling ---
title('\bf{SCARA Robot: LQR Control Sensitivity Analysis}', 'FontSize', 14);
%subtitle('Comparison of 15 Iterations using Logarithmic Weight Scaling');
xlabel('Time (seconds)', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Joint 1 Position (rad/m)', 'FontSize', 12, 'FontWeight', 'bold');

% Formatting
grid on;
ax = gca;
ax.GridLineStyle = '--';
ax.GridAlpha = 0.3;
ax.FontSize = 10;

% Legend styling
lgd = legend('Location', 'northeastoutside', 'FontSize', 8);
title(lgd, 'Weighting Ratio');

% --- High-Resolution Export ---
exportgraphics(fig, 'SCARA_Diverse_LQR_600DPI.png', 'Resolution', 600);

fprintf('High-resolution diverse plot saved successfully.\n');
hold off;