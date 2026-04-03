% Filename and sheet setup
filename = 'Modified_data_without_4th_quad_data.xlsx';
sheets = {'3V', '4V', '5V', '6V'};
Ts = 0.12;  % Uniform sampling time in seconds

% Initialize storage
data_id_all = cell(length(sheets), 1);
Y_all = [];
U_all = [];

fprintf('--- Resampling and iddata Creation ---\n');

for i = 1:length(sheets)
    % Read table from sheet
    raw = readtable(filename, 'Sheet', sheets{i});
    
    % Extract signals
    t_orig     = raw.Time_inS_;
    u_orig     = raw.Voltage_V_;
    pos_orig   = raw.PositionInRad;
    speed_orig = raw.SpeedInRadians;
    
    % Create uniform time vector
    t_uniform = t_orig(1):Ts:t_orig(end);
    
    % Interpolate signals
    u_interp     = interp1(t_orig, u_orig, t_uniform, 'linear');
    pos_interp   = interp1(t_orig, pos_orig, t_uniform, 'linear');
    speed_interp = interp1(t_orig, speed_orig, t_uniform, 'linear');
    
    % Create iddata object
    y = [pos_interp', speed_interp'];
    u = u_interp';
    data_id = iddata(y, u, Ts);
    
    % Store individual iddata
    data_id_all{i} = data_id;
    
    % Accumulate for merged dataset
    Y_all = [Y_all; y];
    U_all = [U_all; u];
    
    % Display stats
    fprintf('Sheet %-3s → Samples: %d | Duration: %.2f s\n', ...
        sheets{i}, length(t_uniform), t_uniform(end) - t_uniform(1));
end

% Create merged iddata object manually
merged_data = iddata(Y_all, U_all, Ts);


fprintf('\nAll sheets resampled and merged manually.\n');
fprintf('Total samples in merged dataset: %d\n', length(merged_data.OutputData));
fprintf('Sampling time used: %.3f seconds\n', merged_data.Ts);
for i = 1:length(data_id_all)
    fprintf('Sheet %s → Ts = %.3f s\n', sheets{i}, data_id_all{i}.Ts);
end