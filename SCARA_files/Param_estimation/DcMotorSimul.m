close all;
clear all;
clc;
 
 
% define the constants
J=0.1
b=0.5
Kt=1
L=1
R=5
Ke=1
L=1
 
% define the system matrices
A=[0 1 0; 0 -b/J Kt/J; 0 -Ke/L -R/L]
B=[0; 0; 1/L]
W=[0; -1/J; 0]
C=[0 1 0]
 
% combine the control input and disturbance matrices in a single matrix
Btotal=[B W]
 
% define the state-space model
sys = ss(A,Btotal,C,[])
 
% discretization constant
dt = 0.01;
% final simulation time
Tfinal= 5;
% discretized time
discretization_time = 0:dt:Tfinal;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% system response to the step control input 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 
% control input
figure(1)
control_step = ones(size(discretization_time))';
plot(discretization_time,control_step,'LineWidth',2)
xlabel('Time')
ylabel('Control input (voltage)')
title('Control Input Used for Simulation')
 
% disturbance 
disturbance = zeros(size(discretization_time))';
 
% control input and disturbance
U_step=[control_step, disturbance];
 
% simulate the system
Ycontrol_step=lsim(sys,U_step,discretization_time);
 
% plot the system output 
figure(2)
plot(discretization_time,Ycontrol_step,'LineWidth',2)
xlabel('Time')
ylabel('System output (angular velocity)')
title('System response to the step control input')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% system response to the sinusoidal control input 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 
% control input
figure(3)
control_sin = (4+sin(5*discretization_time))';
plot(discretization_time,control_sin,'LineWidth',2)
xlabel('Time')
ylabel('Control input (voltage)')
title('Control Input Used for Simulation')
 
% disturbance 
disturbance = zeros(size(discretization_time))';
 
% control input and disturbance
U_sin=[control_sin, disturbance];
 
% simulate the system
Ycontrol_sin=lsim(sys,U_sin,discretization_time);
 
% plot the system output 
figure(4)
plot(discretization_time,Ycontrol_sin,'LineWidth',2)
xlabel('Time')
ylabel('System output (angular velocity)')
title('System response to the sinusoidal control input')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% system response to non zero inital conditions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
X0=[1; 1; 1];
Yinitial=initial(sys,X0,discretization_time);
% plot the system output 
figure(5)
plot(discretization_time,Yinitial,'LineWidth',2)
xlabel('Time')
ylabel('System output (angular velocity)')
title('System response to the initial condition')
