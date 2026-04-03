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
q_weights = logspace(-1, 3, 15); 
sim_time = 8; 

% --- Figure Setup ---
fig = figure('Color', 'w', 'Units', 'inches', 'Position', [2, 2, 9, 6]);
hold on;

% Color map
colors = jet(length(q_weights));

% --- LQR Iteration Loop ---
for k = 1:length(q_weights)
    Q_val = q_weights(k);
    R_val = 1; 
    
    Q_curr = Q_val * eye(4);
    R_curr = R_val * eye(2);
    
    K = lqr(A, B, Q_curr, R_curr);
    sys_cl = ss(A - B*K, B, C, D);
    [y, t] = step(sys_cl, sim_time); 
    
    plot(t, y(:,1,1), 'LineWidth', 1.6, 'Color', colors(k,:), ...
         'DisplayName', sprintf('Q/R Ratio: %.2f', Q_val/R_val));
end

% --- Professional Labeling ---
title('\bf{SCARA Robot: LQR Control Sensitivity Analysis}', 'FontSize', 14);
subtitle('Comparison of 15 Iterations');
xlabel('Time (seconds)', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Joint 1 Position (rad/m)', 'FontSize', 12, 'FontWeight', 'bold');

% Formatting
grid on;
ax = gca;
ax.GridLineStyle = '--';
ax.GridAlpha = 0.3;
ax.FontSize = 10;
ax.Box = 'off'; % Removes the top and right box lines for arrow style

% Legend styling
lgd = legend('Location', 'northeastoutside', 'FontSize', 8);
title(lgd, 'Weighting Ratio');

% --- ADD AXIS ARROWS ---
% Ensure the plot is fully rendered so positions are accurate
drawnow; 

% Get axes position in normalized figure units
pos = ax.Position; 

% X-axis arrow (Bottom-left to Bottom-right)
annotation('arrow', [pos(1), pos(1) + pos(3) + 0.03], [pos(2), pos(2)], ...
    'LineWidth', 1.5, 'HeadStyle', 'vback2', 'HeadWidth', 8, 'HeadLength', 8);

% Y-axis arrow (Bottom-left to Top-left)
annotation('arrow', [pos(1), pos(1)], [pos(2), pos(2) + pos(4) + 0.03], ...
    'LineWidth', 1.5, 'HeadStyle', 'vback2', 'HeadWidth', 8, 'HeadLength', 8);

% --- High-Resolution Export ---
exportgraphics(fig, 'SCARA_LQR_With_Arrows_600DPI.png', 'Resolution', 600);

fprintf('High-resolution plot with arrows saved successfully.\n');
hold off;