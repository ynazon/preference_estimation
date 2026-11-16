clear 
clc

%% Exploratory Aim
% Process exploratory aim data for AFO Emulator project

% dataFolder = 'E:\AFO Emulator\Data\Data\'; 
dataFolder = 'C:\Users\nazon\OneDrive\Documents\Walter Reed\Preference Project\Exploratory Aim Data\Exploratory Aim\';
% dataFolder = '/Volumes/STORE N GO/AFO Emulator/Data/';
subject = 'AFOWR01';

% load([dataFolder, subject, '\Exploratory Aim\GUI\AFOWR01_EA_Training.mat'])
% training_coordinate_id = coordinate_id;
% training_ids = -IDs(:,1:2);

% filePattern = fullfile([dataFolder, subject, '/Exploratory Aim/GUI/', subject, '_EA_*']); % match your file names
filePattern = fullfile([dataFolder, subject, '\GUI\', subject, '_EA_*']); % match your file names
files = dir(filePattern);

EA = struct();
for k = 1:length(files)
    fileName = files(k).name;
    filePath = fullfile(files(k).folder, fileName);
    fprintf('Processing %s\n', fileName);

    gui_data = load(filePath);
    EA(k).coordinate_id = gui_data.coordinate_id;
    EA(k).stiffness = gui_data.IDs(:,1);
    EA(k).NA = gui_data.IDs(:,2);
    EA(k).final_stiffness = gui_data.IDs(end,1);
    EA(k).final_NA = gui_data.IDs(end,2);
    if (strcmp(subject, 'AFOWR01') && k == 5) 
        EA(k).stiffness = -EA(k).stiffness;
        EA(k).final_stiffness = -EA(k).final_stiffness;
    end
    
end

filePattern = fullfile([dataFolder, subject, '\Humotech\', subject, '_EA_*']); % match your file names
files = dir(filePattern);

start_S = -2.4;
start_NA = -7.5;
trigger_threshold = 5; 
time_threshold = 6000; 

for k = 1:length(files)
    fileName = files(k).name;
    filePath = fullfile(files(k).folder, fileName);
    fprintf('Processing %s\n', fileName);

    ht_data = load(filePath);

    timeCol = 92;
    triggerCol = 87;
    minSpacing = 100; 
    
    time = ht_data.data(:, timeCol);
    trigger = ht_data.data(:, triggerCol);
    
    % 1. Find candidate triggers where trigger < 5
    candidateIdx = find(trigger < 5);
    
    % 2. Filter triggers to remove any within 10 samples of each other
    if isempty(candidateIdx)
        error('No trigger points found below threshold.');
    end
    
    keepIdx = candidateIdx(1);
    for i = 2:length(candidateIdx)
        if candidateIdx(i) - keepIdx(end) > minSpacing
            keepIdx(end+1) = candidateIdx(i);
        end
    end
    
    triggerIdx = keepIdx;
    
    % 3. Handle number of triggers
    if length(triggerIdx) == 2
        % ✅ Special case:
        % Start at FIRST trigger, end at LAST trigger
        startIdx = triggerIdx(1);
        stopIdx  = triggerIdx(2);
        keepMask = false(size(ht_data.data,1),1);
        keepMask(startIdx:stopIdx) = true;
    
    else
        % ✅ Keep only odd-numbered trials (start → stop)
        keepMask = false(size(ht_data.data,1),1);
        for i = 1:2:length(triggerIdx)
            startIdx = triggerIdx(i);
            if (i+1) <= length(triggerIdx)
                stopIdx  = triggerIdx(i+1);
            else
                stopIdx  = size(ht_data.data,1);
            end
            keepMask(startIdx:stopIdx) = true;
        end
    end
    
    % 4. Apply mask
    ht_data.data_cut = ht_data.data(keepMask, :);
    
    % 5. Make time continuous across cuts
    cutIdx = find(diff(keepMask) == -1);  % trial end indices
    offset = zeros(size(time));
    cumRemoved = 0;
    
    for i = 1:length(cutIdx)
        % Find next start after this cut (if exists)
        nextStart = find(triggerIdx > cutIdx(i), 1, 'first');
        if ~isempty(nextStart) && nextStart <= length(triggerIdx)
            nextStartIdx = triggerIdx(nextStart);
            timeRemoved = time(nextStartIdx) - time(cutIdx(i)+1);
            cumRemoved = cumRemoved + timeRemoved;
            offset(time > time(cutIdx(i))) = cumRemoved;
        end
    end
    
    % ✅ Adjust time for continuity
    ht_data.data_cut(:, timeCol) = ht_data.data_cut(:, timeCol) - offset(keepMask);
    ht_data.data = ht_data.data_cut;

    % trigger = min(find(ht_data.data(:,87) < trigger_threshold));
    EA(k).time = ht_data.data(:,92);
    EA(k).stiffness_ht = ht_data.data(:,61);
    EA(k).NA_ht = ht_data.data(:,14);
    EA(k).final_stiffness_ht = ht_data.data(end,61);
    EA(k).final_NA_ht = ht_data.data(end,14);

    % Find when either stiffness or NA changes
    stiffness = EA(k).stiffness_ht;
    NA = EA(k).NA_ht;
    time = EA(k).time;
    
    % A new setting occurs whenever either parameter changes
    settingChange = [true; ...
        diff(stiffness) ~= 0 | diff(NA) ~= 0];
    
    % Indices where each new parameter setting begins
    settingStartIdx = find(settingChange);
    
    % End index of each setting
    settingEndIdx = [settingStartIdx(2:end)-1; length(time)];
    
    % Calculate duration of each parameter setting
    settingDuration = time(settingEndIdx) - time(settingStartIdx);
    
    % Store individual setting durations
    EA(k).setting_duration = settingDuration;
    
    % Average time spent at each parameter setting
    EA(k).avg_time_per_setting = mean(settingDuration);

    EA(k).std_S = std(EA(k).stiffness_ht(length(EA(k).stiffness_ht)-time_threshold:end));
    EA(k).std_NA = std(EA(k).NA_ht(length(EA(k).NA_ht)-time_threshold:end));

    % Logical vector: true whenever BOTH variables are within tolerance
    inTol = abs(EA(k).stiffness_ht - EA(k).final_stiffness_ht) < 2*EA(k).std_S & ...
            abs(EA(k).NA_ht - EA(k).final_NA_ht) < 2*EA(k).std_NA;
    
    % Find transitions from outside to inside
    crossings = find(diff([0; inTol]) == 1);
    
    % The last crossing is the convergence point
    lastCrossing = crossings(end);
    EA(k).convergence_time = EA(k).time(lastCrossing) - EA(k).time(1);

    figure;
    lineColor = [78/255 0 138/255];

    % Stiffness subplot
    subplot(2,1,1)
    plot(EA(k).time - EA(k).time(1), EA(k).stiffness_ht, 'Color', lineColor , 'LineWidth', 1.5);
    hold on;

    % Final stiffness ± std
    yline(EA(k).final_stiffness_ht + 2 * EA(k).std_S, 'r--', 'LineWidth', 1.5);
    yline(EA(k).final_stiffness_ht - 2 * EA(k).std_S, 'r--', 'LineWidth', 1.5);

    % Convergence time
    xline(EA(k).convergence_time, 'k', 'LineWidth', 1.5);

    xlabel('Time (s)');
    ylabel('Stiffness (Nm/deg)');
    % title(sprintf('EA Trial %d', k));
    grid on;

    % Neutral Angle subplot
    subplot(2,1,2)
    plot(EA(k).time- EA(k).time(1), EA(k).NA_ht, 'Color', lineColor, 'LineWidth', 1.5);
    hold on;

    % Final NA ± std
    yline(EA(k).final_NA_ht + 2 * EA(k).std_NA, 'r--', 'LineWidth', 1.5);
    yline(EA(k).final_NA_ht - 2 * EA(k).std_NA, 'r--', 'LineWidth', 1.5);

    % Convergence time
    xline(EA(k).convergence_time, 'k', 'LineWidth', 1.5);

    xlabel('Time (s)');
    ylabel('Neutral Angle (deg)');
    grid on;

    % Link x-axes so zooming/panning affects both
    linkaxes(findall(gcf,'Type','axes'),'x');
end

average_stiffness = mean([EA.final_stiffness])
std_stiffness = std2([EA.final_stiffness])
average_NA = mean([EA.final_NA])
std_NA = std2([EA.final_NA])
average_convergence_time = mean([EA.convergence_time])
std_convergence_time = std2([EA.convergence_time])
average_time_per_setting = mean([EA.avg_time_per_setting])
std_time_per_setting  = std2([EA.avg_time_per_setting])




%% Plot

if strcmp(subject, 'AFOWR01')
    actual = [8.9, 2.8];
elseif strcmp(subject, 'AFOWR02')
    actual = [2.02, 1.7];
end

stiffness = [EA.final_stiffness];

emulated = [-stiffness; EA.final_NA]';

% --- Compute mean of emulated values ---
emu_mean = mean(emulated);

% --- Compute percent difference (element-wise) ---
percent_diff = abs((emu_mean - actual) ./ actual) * 100;

% --- Compute covariance and confidence ellipse ---
covMat = cov(emulated);
[eigVec, eigVal] = eig(covMat);

% 95% confidence scaling factor (chi-square for 2 DOF)
chi2_val = chi2inv(0.95, 2);
theta = linspace(0, 2*pi, 100);

% Ellipse in unit circle form
ellipse_coords = [cos(theta); sin(theta)] .* sqrt(chi2_val);

% Transform to ellipse using eigen decomposition
ellipse_transformed = eigVec * sqrt(eigVal) * ellipse_coords + emu_mean';

% --- Define colors ---
% pointColor = [117/255 88/255 131/255];
pointColor = [78/255 0 138/255];
ellipseColor = [117/255 88/255 131/255];

% --- Plot everything ---
figure;
hold on;

% Confidence ellipse (filled area)
hEllipse = fill(ellipse_transformed(1,:), ellipse_transformed(2,:), ellipseColor, ...
    'FaceAlpha', 0.25, 'EdgeColor', 'none', 'DisplayName','95% Confidence Ellipse');

% Emulated points (larger)
hEmu = scatter(emulated(:,1), emulated(:,2), 400, pointColor, 'filled', 'DisplayName','Emulated Values');

% Mean of emulated values (even larger diamond)
hMean = scatter(emu_mean(1), emu_mean(2), 500, 'd', 'MarkerFaceColor', 'w', ...
    'MarkerEdgeColor', pointColor, 'LineWidth', 1.5, 'DisplayName','Emulated Mean');


% Actual value (larger black pentagon)
hActual = scatter(actual(1), actual(2), 500, 'k', '^', 'filled', 'DisplayName','Actual Value');

xlim([0,10])
ylim([-5,20])
% --- Labels and legend ---
xlabel('Stiffness (Nm/deg)', 'FontSize', 36, 'FontWeight', 'bold');
ylabel('Neutral Angle (deg)', 'FontSize', 36, 'FontWeight', 'bold');

% --- Dummy handles for legend (legend-only, larger markers) ---
hEllipse_leg = patch(NaN, NaN, ellipseColor, ...
    'FaceAlpha', 0.25, 'EdgeColor', 'none');

hEmu_leg = scatter(NaN, NaN, 400, 'MarkerEdgeColor', pointColor, 'LineWidth', 10);
hMean_leg = plot(NaN, NaN,  'd', 'MarkerSize', 15, ...
    'MarkerFaceColor', 'w', 'MarkerEdgeColor', pointColor, 'LineWidth', 1.5);
hActual_leg = scatter(NaN, NaN, 500, 'k', '^', 'MarkerEdgeColor', 'k', 'LineWidth', 8);

% --- Legend ---
legend([hEllipse_leg, hEmu_leg, hMean_leg, hActual_leg], ...
       {'95% Confidence Ellipse', 'Emulated Values', 'Emulated Mean', 'Prescribed Value'}, ...
       'FontSize', 36, 'Location', 'best');

% %% Plot dummy confidence ellipse
% emulated = [2 4;
%             3.9 2.62;
%             3.18 0.38;
%             0.82 0.38;
%             0.1 2.62];
% 
% % --- Compute mean of emulated values ---
% emu_mean = mean(emulated);
% 
% % --- Compute percent difference (element-wise) ---
% percent_diff = abs((emu_mean - actual) ./ actual) * 100;
% 
% % --- Compute covariance and confidence ellipse ---
% covMat = cov(emulated);
% [eigVec, eigVal] = eig(covMat);
% 
% % 95% confidence scaling factor (chi-square for 2 DOF)
% chi2_val = chi2inv(0.95, 2);
% theta = linspace(0, 2*pi, 100);
% 
% % Ellipse in unit circle form
% ellipse_coords = [cos(theta); sin(theta)] .* sqrt(chi2_val);
% 
% % Transform to ellipse using eigen decomposition
% ellipse_transformed = eigVec * sqrt(eigVal) * ellipse_coords + emu_mean';
% 
% % --- Define colors ---
% % pointColor = [117/255 88/255 131/255];
% pointColor = [78/255 0 138/255];
% ellipseColor = [117/255 88/255 131/255];
% 
% % --- Plot everything ---
% figure;
% hold on;
% 
% % Confidence ellipse (filled area)
% hEllipse = fill(ellipse_transformed(1,:), ellipse_transformed(2,:), ellipseColor, ...
%     'FaceAlpha', 0.25, 'EdgeColor', 'black', 'DisplayName','95% Confidence Ellipse');
% 
% % Emulated points (larger)
% hEmu = scatter(emulated(:,1), emulated(:,2), 400, pointColor, 'filled', 'DisplayName','Emulated Values');
% 
% % Mean of emulated values (even larger diamond)
% hMean = scatter(emu_mean(1), emu_mean(2), 500, 'd', 'MarkerFaceColor', 'w', ...
%     'MarkerEdgeColor', pointColor, 'LineWidth', 1.5, 'DisplayName','Emulated Mean');
% 
% 
% % Actual value (larger black pentagon)
% % hActual = scatter(actual(1), actual(2), 500, 'k', '^', 'filled', 'DisplayName','Actual Value');
% 
% % xlim([0,10])
% % ylim([-5,20])
% % --- Labels and legend ---
% xlabel('Stiffness (Nm/deg)', 'FontSize', 36, 'FontWeight', 'bold');
% ylabel('Neutral Angle (deg)', 'FontSize', 36, 'FontWeight', 'bold');
% 
% % --- Dummy handles for legend (legend-only, larger markers) ---
% hEllipse_leg = patch(NaN, NaN, ellipseColor, ...
%     'FaceAlpha', 0.25, 'EdgeColor', 'none');
% 
% hEmu_leg = scatter(NaN, NaN, 400, 'MarkerEdgeColor', pointColor, 'LineWidth', 10);
% hMean_leg = plot(NaN, NaN,  'd', 'MarkerSize', 15, ...
%     'MarkerFaceColor', 'w', 'MarkerEdgeColor', pointColor, 'LineWidth', 1.5);
% hActual_leg = scatter(NaN, NaN, 500, 'k', '^', 'MarkerEdgeColor', 'k', 'LineWidth', 8);
% 
% % --- Legend ---
% legend([hEllipse_leg, hEmu_leg, hMean_leg], ...
%        {'95% Confidence Ellipse', 'Data Points', 'Mean'}, ...
%        'FontSize', 36, 'Location', 'best');


