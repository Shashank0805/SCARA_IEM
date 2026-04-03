function [A,B,C,D] = dcmotor_model_simple(par, Ts)
% Simplified Grey-box model for DS3235 Servo Motor
% par = [J, b, K, R]
% State: omega (angular speed)
% Input: u = voltage
% Output: y = omega (rad/s)

J = par(1);   % Inertia
b = par(2);   % Viscous friction
K = par(3);   % Torque constant
R = par(4);   % Armature resistance

% Differential equation:
% J*dω/dt + b*ω = (K/R)*u - (K^2/R)*ω
%
% => dω/dt = [-(b + K^2/R)/J]*ω + (K/(J*R))*u

A = -(b + (K^2)/R)/J;
B = K/(J*R);
C = 1;
D = 0;
end
