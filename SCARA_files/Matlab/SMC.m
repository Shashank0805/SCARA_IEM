%% Sliding Mode Control for Simple Pendulum
clear; clc; close all;

%% 1. System Parameters
params.M = 1;          % mass (kg)
params.L = 1;          % length (m)
params.g = 9.81;       % gravity (m/s^2)
params.b = -1;         % damping (as assumed in paper)
params.I = params.M*params.L^2;

%% 2. Controller Parameters
params.c = 5;          % sliding surface slope
params.K = 10;         % switching gain
params.epsilon = 0.05; % boundary layer thickness

%% 3. Initial Conditions
x0 = [1; 0];           % [theta; omega]

%% 4. Simulation Time
tspan = [0 10];

%% 5. Simulation
[t,x] = ode45(@(t,x) pendulum_smc(t,x,params), tspan, x0);

theta = x(:,1);
omega = x(:,2);

%% 6. Recompute control and sliding surface for plotting
S = zeros(length(t),1);
u = zeros(length(t),1);

for i = 1:length(t)
    
    x1 = theta(i);
    x2 = omega(i);
    
    M = params.M; L = params.L; g = params.g;
    b = params.b; I = params.I;
    c = params.c; K = params.K; eps = params.epsilon;
    
    % Sliding surface
    S(i) = c*x1 + x2;
    
    % Equivalent control
    u_eq = b*x2 + M*g*L*sin(x1) - I*c*x2;
    
    % Saturation function
    sat = S(i)/(abs(S(i))+eps);
    
    % Switching control
    u_sw = -I*K*sat;
    
    % Total control
    u(i) = u_eq + u_sw;
end

%% 7. Plot Results

figure('Name','Sliding Mode Control Results','Position',[200 200 1000 700])

subplot(2,2,1)
plot(t,theta,'LineWidth',2)
xlabel('Time (s)')
ylabel('\theta (rad)')
title('Pendulum Angle')
grid on

subplot(2,2,2)
plot(t,omega,'LineWidth',2)
xlabel('Time (s)')
ylabel('\omega (rad/s)')
title('Angular Velocity')
grid on

subplot(2,2,3)
plot(theta,omega,'LineWidth',2)
xlabel('\theta (rad)')
ylabel('\omega (rad/s)')
title('Phase Portrait')
grid on

subplot(2,2,4)
plot(t,u,'LineWidth',2)
xlabel('Time (s)')
ylabel('Control Torque')
title('Control Input')
grid on

figure
plot(t,S,'LineWidth',2)
xlabel('Time (s)')
ylabel('Sliding Surface S')
title('Sliding Surface Convergence')
grid on

%% ============================================
%% Pendulum Dynamics with SMC Controller
%% ============================================

function dxdt = pendulum_smc(~,x,params)

x1 = x(1);   % theta
x2 = x(2);   % omega

M = params.M; L = params.L; g = params.g;
b = params.b; I = params.I;
c = params.c; K = params.K; eps = params.epsilon;

%% Sliding Surface
S = c*x1 + x2;

%% Equivalent Control
u_eq = b*x2 + M*g*L*sin(x1) - I*c*x2;

%% Boundary layer saturation
sat = sign(S);

%% Switching Control
u_sw = -I*K*sat;

%% Total Control
u = u_eq + u_sw;

%% System Dynamics
dx1 = x2;
dx2 = -(b/I)*x2 - (M*g*L/I)*sin(x1) + (1/I)*u;

dxdt = [dx1; dx2];

end