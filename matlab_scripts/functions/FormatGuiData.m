function gui_tbl = FormatGuiData(gui_data,subject,affected_side)

% Define fieldnames
gui_fieldnames = fieldnames(gui_data);
NAidx = find(strcmp(gui_fieldnames,'final_NA'));
Kidx = find(strcmp(gui_fieldnames,'final_stiffness'));
SubjMassIdx = find(strcmp(gui_fieldnames,'subj_mass'));
SubjHeightIdx = find(strcmp(gui_fieldnames,'subj_height'));

gui_tbl = table();
gui_tbl{:,1} = [gui_data.(gui_fieldnames{Kidx})]';
gui_tbl{:,2} = [gui_data.(gui_fieldnames{NAidx})]';
subject_length_vector = gui_tbl{:,1};
subject_array = categorical(MakeStringArray(subject_length_vector,subject));
affected_side_array = categorical(MakeStringArray(subject_length_vector,affected_side));
trial_array = categorical(1:length(subject_length_vector));
gui_tbl{:,3} = subject_array;
gui_tbl{:,4} = affected_side_array;
gui_tbl{:,5} = trial_array';
gui_tbl{:,6} = [gui_data.(gui_fieldnames{SubjMassIdx})]';
gui_tbl{:,7} = [gui_data.(gui_fieldnames{SubjHeightIdx})]';

gui_tbl.Properties.VariableNames = ...
    {gui_fieldnames{Kidx},gui_fieldnames{NAidx},...
    'Subject','Exo_Side','Trial_Number',...
    gui_fieldnames{SubjMassIdx},gui_fieldnames{SubjHeightIdx}};

% Remove NaN's from table
gui_tbl = rmmissing(gui_tbl);

end
% 
function output_string = MakeStringArray(data_length_vector,input_string)
output_string = strings(length(data_length_vector),1);
for i=1:length(output_string)
    output_string(i) = input_string;
end
end
