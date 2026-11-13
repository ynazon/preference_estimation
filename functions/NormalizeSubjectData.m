function normalized_data = NormalizeSubjectData(segmented_data,gui_data,normalization_method,output_format)

% normalized_data = struct();
% left_seg_fields = fieldnames(segmented_data(1).left);
% right_seg_fields = fieldnames(segmented_data(1).right);
% 
% [left_normalization_vector,right_normalization_vector] = ...
%     CreateNormalizationScalingVector(exo_side);

% Normalize human & humotech data
% Process left side data

normalized_data = table();
segmented_data_names = segmented_data.Properties.VariableNames;

[~,TrialIdx] = find(strcmp(segmented_data_names,'Trial_Number'));
[~,SubjIdx] = find(strcmp(segmented_data_names,'Subject'));

for k=1:length(segmented_data_names)

    if strcmp(segmented_data_names(k),'HT_stiffness')
        norm_method = 'stiffness';
    elseif strcmp(segmented_data_names(k),'HT_neutral_angle')
        norm_method = 'na';
    elseif strcmp(segmented_data_names(k),'Subject') || ...
           strcmp(segmented_data_names(k),'Exo_Side') || ...
           strcmp(segmented_data_names(k),'Trial_Number')
        norm_method = 'none';
    else
        norm_method = normalization_method;
    end
        
    normalized_data{:,k} = ...
        normalize_input(segmented_data.(segmented_data_names{k}),...
        gui_data,norm_method,segmented_data_names{k},...
        segmented_data.(segmented_data_names{TrialIdx}),...
        segmented_data.(segmented_data_names{SubjIdx}),output_format);

    %     % Normalize data
    %     for j=1:length(segmented_data(k).left)
    %         normalized_data(k).left(j).(left_seg_fields{i}) = ...
    %             normalize_input(segmented_data(k).left(j).(left_seg_fields{i}),...
    %             gui_data(k).final_stiffness,gui_data(k).final_NA,norm_method,left_seg_fields{i},...
    %             left_normalization_vector);
    %     end
    % end
end
normalized_data.Properties.VariableNames = segmented_data_names;

end


function scaled_signal = normalize_input(X,gui,normalization_method,signal_name,trial_number_vector,subject_vector,output_format)
    if strcmp(normalization_method,'min_max')
        if strcmp(signal_name,'Time')
            for i=1:length(X)
                scaled_signal{i} = (X{i} - min(X{i})) / (max(X{i}) - min(X{i}));
            end
            scaled_signal = scaled_signal';
        else
            [min_value,max_value] = min_max_norm(X);
            for i=1:length(X)
                scaled_signal{i} = (X{i} - min_value)/(max_value - min_value);
            end
            scaled_signal = scaled_signal';
        end

    elseif strcmp(normalization_method,'none') % Perform no normalization
        if strcmp(signal_name,'Time')
            for i=1:length(X)
                scaled_signal{i} = (X{i} - min(X{i})) / (max(X{i}) - min(X{i}));
            end
            scaled_signal = scaled_signal';
        else
            scaled_signal = X;
        end

    elseif strcmp(normalization_method,'stiffness')
        max_k = 10;
        min_k = 0;
        for i=1:length(trial_number_vector)
            final_k = gui.final_stiffness(...
                (trial_number_vector(i) == gui.Trial_Number) &...
                (subject_vector(i) == gui.Subject));
            mass = gui.subj_mass(...
                (trial_number_vector(i) == gui.Trial_Number) &...
                (subject_vector(i) == gui.Subject));
            scaled_final_k = (final_k - min_k) / (max_k - min_k);
            scaled_current_k = (X(i) - min_k)/(max_k - min_k);
            if strcmp(output_format,'delta') 
                scaled_signal(i) = (X(i) - final_k)/mass; % in delta k
                % scaled_signal(i) = scaled_current_k - scaled_final_k; % in delta k
            elseif strcmp(output_format,'percent') 
                scaled_signal(i) = (scaled_current_k/scaled_final_k)*100; % in percent k
            end
        end
        scaled_signal = scaled_signal';

    elseif strcmp(normalization_method,'na')
        max_na = 20;
        min_na = -5;
        for i=1:length(trial_number_vector)
            final_na = gui.final_NA(...
                (trial_number_vector(i) == gui.Trial_Number) &...
                (subject_vector(i) == gui.Subject));
            mass = gui.subj_mass(...
                (trial_number_vector(i) == gui.Trial_Number) &...
                (subject_vector(i) == gui.Subject));
            scaled_final_na = (final_na - min_na) / (max_na - min_na);
            scaled_current_na = (X(i) - min_na)/(max_na - min_na);
            if strcmp(output_format,'delta') 
                scaled_signal(i) = (X(i) - final_na)/mass; % in delta k
                % scaled_signal(i) = scaled_current_na - scaled_final_na; % in delta k
            elseif strcmp(output_format,'percent') 
                scaled_signal(i) = (scaled_current_na/scaled_final_na)*100; % in percent k
            end
        end
        scaled_signal = scaled_signal';
    
    elseif strcmp(normalization_method,'custom')
        scaling_factor = 0.2;

        if strcmp(joint,'ankle')
            min_value = ankle_ROM - ankle_ROM*scaling_factor;
            max_value = ankle_ROM + ankle_ROM*scaling_factor;
        elseif strcmp(joint,'knee')
            min_value = knee_ROM - knee_ROM*scaling_factor;
            max_value = knee_ROM + knee_ROM*scaling_factor;
        elseif strcmp(joint,'hip')
            min_value = hip_ROM - hip_ROM*scaling_factor;
            max_value = hip_ROM + hip_ROM*scaling_factor;
        else
            disp('Incorrect Joint')
        end
        scaled_signal = (X - min_value)/(max_value - min_value); 
        
    else % use z-score normalization
        if strcmp(signal_name,'Time')
            for i=1:length(X)
                scaled_signal{i} = (X{i} - min(X{i})) / (max(X{i}) - min(X{i}));
            end
        else
            scaled_signal = normalize(X);
        end
    end
end

function [global_min,global_max] = min_max_norm(X)
    for i=1:length(X)
        local_min(i) = min(X{i});
        local_max(i) = max(X{i});
    end

    global_min = min(local_min);
    global_max = max(local_max);
end

function zscore_norm(X)
end