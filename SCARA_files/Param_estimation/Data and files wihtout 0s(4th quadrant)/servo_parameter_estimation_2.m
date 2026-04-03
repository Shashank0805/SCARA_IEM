%% servo_parameter_estimation.m
% Full pipeline: resample → merge → grey‐box model → parameter estimation

%% 1. Setup & Resample All Sheets
filename = 'Modified_data_without_4th_quad_data.xlsx';
sheets   = {'3V','4V','5V','6V'};
Ts       = 0.12;          % desired uniform sampling time (s)

Y_all = [];
U_all = [];

fprintf('--- Resampling & Merging Data ---\n');
for i = 1:numel(sheets)
    raw       = readtable(filename,'Sheet',sheets{i});
    t_orig    = raw.Time_inS_;
    u_orig    = raw.Voltage_V_;
    pos_orig  = raw.Position_deg_;
    spd_orig  = raw.Speed_deg_s__corrected_;
    
    t_unif    = t_orig(1):Ts:t_orig(end);
    u_i       = interp1(t_orig,  u_orig,  t_unif,'linear');
    pos_i     = interp1(t_orig,  pos_orig,t_unif,'linear');
    spd_i     = interp1(t_orig,  spd_orig,t_unif,'linear');
    
    Y_all     = [Y_all; [pos_i' spd_i']];
    U_all     = [U_all; u_i'];
    
    fprintf(' %s → %4d samples @ %.2f s\n', sheets{i}, numel(t_unif), t_unif(end)-t_unif(1));
end

merged_data = iddata(Y_all, U_all, Ts);
fprintf('\nMerged dataset: %d samples @ Ts=%.3f s\n\n', ...
        length(merged_data.OutputData), merged_data.Ts);


%% 2. Define the Grey‐Box Model
% Make sure dc_motor_model.m (with [A,B,C,D] = dc_motor_model(par,Ts,varargin)) 
% lives in the same folder.

init_par = [0.01, 0.1, 0.01, 1, 0.01];  % [J, b, K, R, L]
model = idgrey('dc_motor_model', init_par, 'd', [], Ts);

% Enforce positive lower bounds
for k = 1:5
    model.Structure.Parameters(k).Minimum = 1e-5;
end


%% 3. Estimate Parameters with Proper Options
% Build the options object via constructor
opt = greyestOptions( ...
      'Display', 'on', ...             % Show estimation progress
      'EnforceStability', true, ...    % Ensure the estimated model is stable
      'OutputWeight', eye(2));         % 2x2 identity matrix for weighting
      
fprintf('🔍 Running greyest estimation...\n');
estimated_model = greyest(merged_data, model, opt);

fprintf('\n📊 Optimized Parameters:\n');
% To get a clean table of parameters, it's better to access them this way:
getpvec(estimated_model)

%% 4. Quick Validation
fprintf('\n📈 Compare fit:\n');
compare(merged_data, estimated_model);