function [HT_output,GUI_output] = HumotechSegment(subject,affected_side)

dataFolder = 'C:\Users\nazon\OneDrive\Documents\Walter Reed\Preference Project\Exploratory Aim Data\Exploratory Aim\';
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
    EA(k).path = gui_data.points;
    EA(k).final_stiffness = gui_data.IDs(end,1);
    EA(k).final_NA = gui_data.IDs(end,2);

    if (strcmp(subject, 'AFOWR01') && k == 5) 
        EA(k).stiffness = -EA(k).stiffness;
        EA(k).final_stiffness = -EA(k).final_stiffness;
    end

end

filePattern = fullfile([dataFolder, subject, '\Humotech\', subject, '_EA_*']); % match your file names
files = dir(filePattern);

trigger_threshold = 5; 

for k = 1:length(files)
    fileName = files(k).name;
    filePath = fullfile(files(k).folder, fileName);
    fprintf('Processing %s\n', fileName);

    ht_data = load(filePath);

    timeidx = 92;
    triggeridx = 87;
    NeutralAngleidx = 14;
    Stiffnessidx = 61;
    ExoAnkleidx = 71;

    if affected_side == 'L'
        FPidx = 88; % left force plate index
    else
        FPidx = 89; % right force plate index
    end
    minSpacing = 100; 

    time = ht_data.data(:, timeidx);
    trigger = ht_data.data(:, triggeridx);

    % 1. Find candidate triggers where trigger < 5
    candidateIdx = find(trigger < trigger_threshold);

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
    ht_data.data_cut(:, timeidx) = ht_data.data_cut(:, timeidx) - offset(keepMask);
    ht_data.data = ht_data.data_cut;

    HT_time_vector = ht_data.data(:,timeidx)-ht_data.data(1,timeidx);
    
    HC_upper_threshold = 75;
    HC_lower_threshold = 20;
    FP_slope_threshold = 3.75;
    minDist = 100; 

    % Filter Signal
    fs = 1/mean(diff(HT_time_vector));
    fc = 10;
    [b,a] = butter(2,fc/(fs/2));
    GRF_signal = ht_data.data(1:end,FPidx);
    filtered_GRF = filter(b,a,GRF_signal);
    
    HC = find(ht_data.data(1:end-1,FPidx) > HC_lower_threshold & ht_data.data(1:end-1,FPidx) < HC_upper_threshold & diff(ht_data.data(:,FPidx)) > FP_slope_threshold) + 1;
    HC = HC([true; diff(HC) > minDist]);
    TO = find(ht_data.data(1:end-1,FPidx) > HC_lower_threshold & ht_data.data(1:end-1,FPidx) < HC_upper_threshold & diff(ht_data.data(:,FPidx)) < -FP_slope_threshold) + 1;
    TO = TO([true; diff(TO) > minDist]);

    % HC = find(filtered_GRF(1:end-1) > HC_lower_threshold & filtered_GRF(1:end-1) < HC_upper_threshold & diff(filtered_GRF) > FP_slope_threshold) + 1;
    % HC = HC([true; diff(HC) > minDist]);
    % TO = find(filtered_GRF(1:end-1) > HC_lower_threshold & filtered_GRF(1:end-1) < HC_upper_threshold & diff(filtered_GRF) < -FP_slope_threshold) + 1;
    % TO = TO([true; diff(TO) > minDist]);

    if HC(1) > TO(1)
        HC = HC(1:end-1);
        TO = TO(2:end);
    end

    for i = 1:length(HC)-1
        HT_time{i} = HT_time_vector(HC(i):HC(i+1),1);
        HT_stiffness{i} = ht_data.data(HC(i):HC(i+1),Stiffnessidx);
        HT_neutral_angle{i} = ht_data.data(HC(i):HC(i+1),NeutralAngleidx);
        HT_ankle_angle{i} = ht_data.data(HC(i):HC(i+1),ExoAnkleidx);
        HT_force_plate{i} = ht_data.data(HC(i):HC(i+1),FPidx);
    end

    % for i = 1:length(TO)-1
    %     HT_time{i} = HT_time_vector(TO(i):TO(i+1),1);
    %     HT_stiffness{i} = ht_data.data(TO(i):TO(i+1),Stiffnessidx);
    %     HT_neutral_angle{i} = ht_data.data(TO(i):TO(i+1),NeutralAngleidx);
    %     HT_ankle_angle{i} = ht_data.data(TO(i):TO(i+1),ExoAnkleidx);
    %     HT_force_plate{i} = ht_data.data(TO(i):TO(i+1),FPidx);
    % end

    % for i = 1:length(TO)-1
    %     HT_time{i} = HT_time_vector(HC(i):TO(i),1);
    %     HT_stiffness{i} = ht_data.data(HC(i):TO(i),Stiffnessidx);
    %     HT_neutral_angle{i} = ht_data.data(HC(i):TO(i),NeutralAngleidx);
    %     HT_ankle_angle{i} = ht_data.data(HC(i):TO(i),ExoAnkleidx);
    %     HT_force_plate{i} = ht_data.data(HC(i):TO(i),FPidx);
    % end

end

GUI_output = EA;

HT_output = struct('HT_time',HT_time,...
    'HT_stiffness',HT_stiffness,...
    'HT_neutral_angle',HT_neutral_angle,...
    'HT_ankle_angle',HT_ankle_angle,...
    'HT_force_plate',HT_force_plate);

end