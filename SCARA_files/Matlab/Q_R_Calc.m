%% Pole Placement vs. LQR Reverse Engineering
% Project: Control Systems - State Space Analysis
% Author: Shashank Parth Sinha
% Description: This script designs a controller for a 4th-order MIMO system 
% using Pole Placement (based on performance specs) and then searches for 
% LQR weighting matrices (Q and R) that yield a similar gain matrix K.

clear; clc; close all;

%% 1. System Definition (Open Loop)
% State Transition Matrix A and Input Matrix B
A = [0, 0, 1, 0; 
     0, 0, 0, 1; 
     0, 0.000102771, 0.0008673, 0.000321434; 
     0, -0.000133507, -0.00122245, -0.000322415];

B = [0, 0; 
     0, 0; 
     1, 0; 
     0, 1];

fprintf('\nOpen Loop Eigenvalues:\n');
disp(eig(A)); % Checks stability (Roots in RHP/LHP)

%% 2. Symbolic Analysis of the Closed-Loop System
% Building the symbolic matrix A - BK to see the structure of the gain influence
fprintf('\nFormation of symbolic A-BK \n');
syms k11 k12 k13 k14 k21 k22 k23 k24 s
K_sym = [k11 k12 k13 k14;
         k21 k22 k23 k24];

Acl_sym = A - B*K_sym; % Resultant state matrix with full-state feedback
fprintf('\nSymbolic A-BK Matrix:\n');
disp(Acl_sym);

% Characteristic Equation: det(sI - (A-BK)) = 0
char_eq_sym = det(s*eye(4) - Acl_sym);
char_poly_sym = expand(char_eq_sym);
fprintf('\nSymbolic Characteristic Equation:\n');
disp(char_poly_sym);

%% 3. Performance Specifications & Target Pole Calculation
% Designing for a second-order behavior (dominant poles)
zeta = 0.7;       % Damping ratio (targets approx 5% overshoot)
Ts   = 2;         % Desired settling time (sec)
wn = 4/(zeta*Ts); % Natural frequency calculation based on 2% criterion

fprintf('\nDesired Performance Specs:\n');
fprintf('Chosen zeta = %.2f\n', zeta);
fprintf('Desired Settling Time = %.2f sec\n', Ts);
fprintf('Computed Natural Frequency wn = %.4f\n', wn);

% Target 4th order polynomial: Using two pairs of the same dominant poles
% for a balanced, high-order response.
desired_poly = conv([1 2*zeta*wn wn^2], [1 2*zeta*wn wn^2]);
fprintf('\nDesired 4th Order Characteristic Polynomial:\n');
disp(desired_poly);

desired_poles = roots(desired_poly);
fprintf('\nDesired Poles for Pole Placement:\n');
disp(desired_poles);

%% 4. Controller Design: Pole Placement (Ackermann/Place)
fprintf('\nSolving for K using Pole Placement\n');
K_place = place(A, B, desired_poles); % Computes Gain matrix K
fprintf('\nGain Matrix K from Pole Placement:\n');
disp(K_place);

% Verify placement by checking eigenvalues of (A-BK)
Acl_place = A - B*K_place;
fprintf('\nClosed Loop Poles (Verified):\n');
disp(eig(Acl_place));

%% 5. LQR Approximation Loop (Reverse Engineering Q and R)
% Logic: Iterate through scaling factors to find which Q/R ratio matches 
% the K matrix obtained from manual pole placement.
fprintf('\nSearching Q and R to Match K...\n');
best_error = inf;

for q_scale = 1:200
    for r_scale = 1:50
        % Defining trial weighting matrices
        Q_try = q_scale * eye(4); % Penalizes state error
        R_try = r_scale * eye(2); % Penalizes control effort (energy)
        
        % Solving the Algebraic Riccati Equation (ARE)
        [K_lqr, ~, ~] = lqr(A, B, Q_try, R_try);
        
        % Calculate Frobenius Norm difference between K matrices
        error = norm(K_lqr - K_place);
        
        if error < best_error
            best_error = error;
            Q_best = Q_try;
            R_best = R_try;
            K_best = K_lqr;
        end
    end
end

fprintf('\nBest Approximated Q:\n'); disp(Q_best);
fprintf('\nBest Approximated R:\n'); disp(R_best);
fprintf('\nLQR Gain K (Optimized Approximation):\n'); disp(K_best);

%% 6. Final Comparison & Step Response
Acl_best = A - B*K_best;
C = [1 0 0 0; 0 1 0 0]; % Monitoring positions x1 and x2
D = zeros(2, 2);

% Creating State-Space Systems
sys_place = ss(Acl_place, B, C, D);
sys_lqr   = ss(Acl_best, B, C, D);

% Visualization
figure('Name', 'Pole Placement Response', 'Color', 'w');
step(sys_place, 5);
title('Step Response: Pole Placement ($\zeta=0.7, T_s=2$)');
grid on;

figure('Name', 'LQR Response', 'Color', 'w');
step(sys_lqr, 5);
title('Step Response: Approximated LQR (Optimal Cost)');
grid on;