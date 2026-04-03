%% servo_validation_full.m
% Validation of identified servo model against experimental data (5V & 6V)

clc; clear; close all;

%% Settings
filename = 'Modified_data_without_4th_quad_data.xlsx';
sheets = {'5V','6V'};   % validate at these voltages
Ts = 0.01;              % resample step [s]

% Identified model parameters
Ksys = 7.09;   % rad/s per V (slope)
tau  = 3.31;   % s (time constant)
bias = 18.6;   % rad/s (intercept)
sys_tf = tf(Ksys,[tau 1]);

%% Loop through chosen sheets
for i = 1:numel(sheets)
    sheet = sheets{i};

    % Load sheet data
    T = readtable(filename,'Sheet',sheet);
    t = T.("Time_inS_");
    u = T.("Voltage_V_");
    y = T.("SpeedInRadians");

    % Clean
    t = t(:); u = u(:); y = y(:);
    valid = ~isnan(t) & ~isnan(u) & ~isnan(y);
    t = t(valid); u = u(valid); y = y(valid);

    % Resample to uniform grid
    t_uniform = (t(1):Ts:t(end))';
    u_uniform = interp1(t,u,t_uniform,'previous');
    y_uniform = interp1(t,y,t_uniform,'linear');

    % Smooth experimental speed
    y_smooth = smoothdata(y_uniform,'movmean',50);

    % Simulate model response + add bias
    y_model = lsim(sys_tf,u_uniform,t_uniform) + bias;

    % Plot comparison
    figure; hold on; grid on;
    plot(t_uniform,y_smooth,'b','LineWidth',1.5,'DisplayName',['Experimental Data (' sheet ')']);
    plot(t_uniform,y_model,'r--','LineWidth',1.5,'DisplayName','Model Output + Bias');
    xlabel('Time (s)'); ylabel('Speed (rad/s)');
    title(sprintf('Validation at %.2f V Input',mean(u)));
    legend;
end
