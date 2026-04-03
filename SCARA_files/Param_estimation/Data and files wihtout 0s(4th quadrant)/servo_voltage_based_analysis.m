clc; clear; close all;

%% Settings
filename = 'Modified_data_without_4th_quad_data.xlsx';
sheets = {'3V','4V','5V','6V'};
steady_fraction = 0.2;
min_points_steady = 50;
use_smoothing = true;
smooth_window = 5;

%% Containers
V_list = []; W_list = []; Tau_list = [];
results = struct([]);

for i = 1:numel(sheets)
    T = readtable(filename,'Sheet',sheets{i});

    % Columns (update names if needed)
    t = T.("Time_inS_");       % time [s]
    u = T.("Voltage_V_");      % supply voltage [V]
    y = T.("SpeedInRadians");  % speed [rad/s]

    % Clean
    t = t(:); u = u(:); y = y(:);
    valid = ~isnan(t) & ~isnan(u) & ~isnan(y);
    t = t(valid); u = u(valid); y = y(valid);
    [t, idx] = sort(t); u=u(idx); y=y(idx);

    if use_smoothing && numel(y) > smooth_window
        y_s = smoothdata(y,'movmedian',smooth_window);
    else
        y_s = y;
    end

    % Outlier rejection
    isOut = isoutlier(y_s,'median','ThresholdFactor',3);
    t(isOut)=[]; u(isOut)=[]; y_s(isOut)=[];

    % Steady-state
    N = numel(t);
    start_idx = max(1, round((1-steady_fraction)*N));
    seg_idx = start_idx:N;
    if numel(seg_idx) < min_points_steady
        start_idx = max(1, N-min_points_steady+1);
        seg_idx = start_idx:N;
    end
    steady_u = median(u(seg_idx));
    steady_w = median(y_s(seg_idx));

    V_list(end+1,1) = steady_u;
    W_list(end+1,1) = steady_w;

    %% Exponential fit for tau
    w_inf = steady_w;
    w0 = y_s(1);
    t0 = t(1);

    fun = @(p) sum((y_s - (w_inf + (w0-w_inf)*exp(-(t-t0)/p))).^2);
    tau0 = (t(end)-t(1))/5;
    tau = fminsearch(fun, tau0);

    Tau_list(end+1,1) = tau;

    results(i).sheet = sheets{i};
    results(i).voltage = steady_u;
    results(i).w_inf = steady_w;
    results(i).tau = tau;

    % Plot fit
    figure('Name',['Exponential fit - ' sheets{i}]); hold on; grid on;
    plot(t,y_s,'b','DisplayName','Data');
    plot(t,w_inf + (w0-w_inf)*exp(-(t-t0)/tau),'r--','DisplayName','Exp fit');
    xlabel('Time (s)'); ylabel('Speed (rad/s)');
    title(sprintf('%s: tau=%.3f s, w_inf=%.2f rad/s',sheets{i},tau,w_inf));
    legend;
end

%% Static mapping (linear regression)
X = [V_list ones(size(V_list))];
theta = X\W_list;
a = theta(1); b = theta(2);  % slope & intercept
yfit = X*theta;
SSres = sum((W_list - yfit).^2);
SStot = sum((W_list - mean(W_list)).^2);
R2 = 1 - SSres/SStot;

fprintf('\n=== Static mapping ===\n');
fprintf('W_ss ≈ %.4f * V + %.4f  [rad/s], R^2=%.3f\n', a, b, R2);

fprintf('\n=== Per-sheet results ===\n');
for i=1:numel(results)
    fprintf('%s : V=%.2f V, W_inf=%.2f rad/s, tau=%.3f s\n',...
        results(i).sheet, results(i).voltage, results(i).w_inf, results(i).tau);
end

figure('Name','Steady speed vs Voltage'); hold on; grid on;
scatter(V_list,W_list,80,'filled'); plot(V_list,yfit,'-r','LineWidth',1.5);
xlabel('Voltage (V)'); ylabel('Steady Speed (rad/s)');
title('Static mapping (steady speed vs supply voltage)');

%% Effective transfer function (use reliable sheets 5V & 6V)
Ksys = a;                       % slope from regression
tau_eff = mean(Tau_list(3:4));  % average tau from 5V and 6V
bias = b;                       % intercept (rad/s)

sys_tf = tf(Ksys,[tau_eff 1]);

disp('=== Identified Model ===');
disp('Dynamic part (no bias):');
sys_tf

fprintf('\nFull identified input-output relation:\n');
fprintf('ω(s) = (%.3f / (%.3f s + 1)) * V(s) + %.3f\n', Ksys, tau_eff, bias);

[A,B,C,D] = ssdata(sys_tf);
fprintf('\nState-space matrices (dynamic part only):\n');
A, B, C, D


%% Validation (5V and 6V)
Ts = 0.01;
valSheets = {'5V','6V'};

for k = 1:numel(valSheets)
    T = readtable(filename,'Sheet',valSheets{k});
    t = T.("Time_inS_"); 
    u = T.("Voltage_V_"); 
    y = T.("SpeedInRadians");

    t = t(:); u = u(:); y = y(:);
    valid = ~isnan(t) & ~isnan(u) & ~isnan(y);
    t = t(valid); u = u(valid); y = y(valid);

    t_uni = (t(1):Ts:t(end))';
    u_uni = interp1(t,u,t_uni,'previous');
    y_uni = interp1(t,y,t_uni,'linear');
    y_smooth = smoothdata(y_uni,'movmean',50);

    % Simulate model and add bias
    y_model = lsim(sys_tf,u_uni,t_uni) + bias;

    figure; hold on; grid on;
    plot(t_uni,y_smooth,'b','LineWidth',1.5,'DisplayName',['Experimental ' valSheets{k}]);
    plot(t_uni,y_model,'r--','LineWidth',1.5,'DisplayName','Model Output');
    xlabel('Time (s)'); ylabel('Speed (rad/s)');
    title(['Validation at ' valSheets{k}]);
    legend;
end
