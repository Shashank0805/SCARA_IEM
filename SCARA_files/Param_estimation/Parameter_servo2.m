

clc; clear; close all;

% === Load Experimental Data ===
data = readtable('Parameter_Estimation.xlsx');


t = data.Time_ms_ / 1000;     % convert ms → seconds
u = data.Voltage_V_;          % input voltage
y = data.Speed;               % speed output


Ts = mean(diff(t));

% Build identification dataset
z = iddata(y, u, Ts, 'InputName', 'Voltage', 'OutputName', 'Speed');

% === Define Initial Parameters ===
% [ R, L, Kt, Kb, J, B ]
Parameters = [2, 0.01, 0.1, 0.1, 0.01, 0.001];
ParameterNames = {'R','L','Kt','Kb','J','B'};

% === Grey-box Model (continuous-time) ===
FileName = 'my_dcmotor_m';   % must be available on MATLAB path
sys0 = idgrey(FileName, Parameters, 'c', {}, 0, 'PName', ParameterNames);

% === Parameter Estimation ===
opt = greyestOptions('Display','on');
sys_est = greyest(z, sys0, opt);

% === Show Estimated Parameters ===
disp('=== Estimated Motor Parameters ===')
param_values = getpvec(sys_est);
for i = 1:length(ParameterNames)
    fprintf('%s = %.6f\n', ParameterNames{i}, param_values(i));
end
