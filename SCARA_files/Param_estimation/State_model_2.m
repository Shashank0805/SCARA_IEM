
% Given parameters
R  = 2.0;        % Ohm - Armature resistance
L  = 0.0015;     % H    - Armature inductance 
Kt = 0.10;       % Nm/A - Torque constant
Kb = 0.095;      % V·s/rad - Back EMF constant 
J  = 0.01;       % kg·m^2 - Rotor inertia
B  = 0.001;      % N·m·s/rad - Damping coefficient

% State-space matrices
A = [-R/L  -Kb/L;
      Kt/J -B/J];
Bv = [1/L; 0];
C = [0 1];   % output = speed
D = 0;

% Create state-space system
sys_ss = ss(A, Bv, C, D);

% Convert to transfer function
[b, a] = ss2tf(A, Bv, C, D);

% Display transfer function coefficients
disp('Transfer Function Coefficients:');
disp('Numerator (b):');
disp(b);
disp('Denominator (a):');
disp(a);

% Plot pole-zero map
figure;
pzmap(sys_ss);
title('Pole-Zero Map');
grid on;

% Check poles
poles = eig(sys_ss);
disp('Poles of the system:');
disp(poles);