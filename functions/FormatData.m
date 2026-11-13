function exo_side_tbl = FormatData(segmented_data,feature_set,subject,affected_side)

% Define fieldnames
left_fieldnames = fieldnames(segmented_data(1).left);
right_fieldnames = fieldnames(segmented_data(1).right);
segmented_fieldnames = fieldnames(segmented_data);

if length(left_fieldnames) > length(right_fieldnames)
    ExoSideIdx = 1;
    exo_fieldnames = left_fieldnames;
else
    ExoSideIdx = 2;
    exo_fieldnames = right_fieldnames;
end

% Initialization
StiffnessIdx = find(strcmp('HT_stiffness',exo_fieldnames)); % index of exo stiffness
NAIdx = find(strcmp('HT_neutral_angle',exo_fieldnames)); % index of exo neutral angle
exo_side_tbl = table();

for i=1:length(segmented_data)
        
        % Make exo table strings
        subject_length_vector = segmented_data(i).(segmented_fieldnames{ExoSideIdx});
        subject_array = categorical(MakeStringArray(subject_length_vector,subject));
        affected_side_array = categorical(MakeStringArray(subject_length_vector,affected_side));
        trial_array = categorical(MakeStringArray(subject_length_vector,num2str(i)));
        
        stiffness = [];
        na = [];
        for j=1:length(subject_length_vector)
            stiffness(j) = segmented_data(i).(segmented_fieldnames{ExoSideIdx})(j).(exo_fieldnames{StiffnessIdx})(end);
            na(j) = segmented_data(i).(segmented_fieldnames{ExoSideIdx})(j).(exo_fieldnames{NAIdx})(end);
        end

        output_tbl = table(stiffness',na',subject_array,affected_side_array,trial_array);
        output_tbl.Properties.VariableNames = {exo_fieldnames{StiffnessIdx},exo_fieldnames{NAIdx},'Subject','Exo_Side','Trial_Number'};
        
        if strcmp(feature_set,'footshank')
            removal_fields = {'Hip','Knee','Pelvis','Thigh'};
            FullRemovalIdx = [];
            for w=1:length(removal_fields)
                rmIdx = find(strncmp(exo_fieldnames,removal_fields{w},length(removal_fields{w})));
                FullRemovalIdx = [FullRemovalIdx,rmIdx'];
            end
            final_feature_set = rmfield(segmented_data(i).(segmented_fieldnames{ExoSideIdx}),exo_fieldnames([FullRemovalIdx,StiffnessIdx,NAIdx]));
        elseif strcmp(feature_set,'exo')
            keeper_fields = {'HT','Time'}; % These are kept fields
            FullRemovalIdx = find(~strncmp(exo_fieldnames,keeper_fields{1},length(keeper_fields{1})) ...
                == ~strncmp(exo_fieldnames,keeper_fields{2},length(keeper_fields{2})));
            final_feature_set = rmfield(segmented_data(i).(segmented_fieldnames{ExoSideIdx}),exo_fieldnames([FullRemovalIdx',StiffnessIdx,NAIdx]));
        elseif strcmp(feature_set,'human')
            removal_fields = {'HT'};
            FullRemovalIdx = [];
            for w=1:length(removal_fields)
                rmIdx = find(strncmp(exo_fieldnames,removal_fields{w},length(removal_fields{w})));
                FullRemovalIdx = [FullRemovalIdx,rmIdx'];
            end
            final_feature_set = rmfield(segmented_data(i).(segmented_fieldnames{ExoSideIdx}),exo_fieldnames([FullRemovalIdx,StiffnessIdx,NAIdx]));
        elseif strcmp(feature_set,'ankle')
            removal_fields = {'Hip','Knee','Pelvis','Thigh','Shank','Foot','HT'};
            FullRemovalIdx = [];
            for w=1:length(removal_fields)
                rmIdx = find(strncmp(exo_fieldnames,removal_fields{w},length(removal_fields{w})));
                FullRemovalIdx = [FullRemovalIdx,rmIdx'];
            end
            final_feature_set = rmfield(segmented_data(i).(segmented_fieldnames{ExoSideIdx}),exo_fieldnames([FullRemovalIdx,StiffnessIdx,NAIdx]));
        else
            final_feature_set = rmfield(segmented_data(i).(segmented_fieldnames{ExoSideIdx}),exo_fieldnames([StiffnessIdx,NAIdx]));
        end

        feature_tbl = struct2table(final_feature_set);
        T = [output_tbl,feature_tbl];
        exo_side_tbl = [exo_side_tbl;T];

end

% Remove NaN's from table
exo_side_tbl = rmmissing(exo_side_tbl);

end

function output_string = MakeStringArray(data_length_vector,input_string)
    output_string = strings(length(data_length_vector),1);
    for i=1:length(output_string)
        output_string(i) = input_string;
    end
end
