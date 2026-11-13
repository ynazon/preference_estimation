% Process AFO data for ML
clc
close all
clear all

subjects = {'AFOWR01','AFOWR02','AFOWR03'};
subject_mass = [1.7,1.88,1.78]; % in m
subject_height = [98.3,72.2,70]; % in kg
% subjects = {'AFOWR01','AFOWR02'};
% subjects = {'AFOWR01'};
exo_side = {'L','L','L'};
normalization_method = 'min_max'; % Options: 'min_max', 'none', 'custom', 'zscore';
output_scaling_method = 'delta'; % Options: 'percent', 'delta'
feature_set = 'footshank'; % Options: 'full', 'footshank', 'exo', 'human', 'ankle'
datafile_location = 'C:\Users\nazon\OneDrive\Documents\Walter Reed\Preference Project\Exploratory Aim Data\Exploratory Aim\subjects';

% Initialize tables
full_segmented_table = table();
full_gui_table = table();

for i=1:length(subjects)
    segmented_data = SegmentSubjectData(subjects{i},exo_side{i},datafile_location);
    gui = GUI_data_extraction(subjects{i},subject_mass(i),subject_height(i),datafile_location);
    segmented_tbl = FormatData(segmented_data,feature_set,subjects{i},exo_side{i});
    gui_tbl = FormatGuiData(gui,subjects{i},exo_side{i});
    full_segmented_table = [full_segmented_table; segmented_tbl];
    full_gui_table = [full_gui_table; gui_tbl];
end

normalized_data = NormalizeSubjectData(full_segmented_table,full_gui_table,normalization_method,output_scaling_method);
interpoled_data = InterpolateData(normalized_data);
extracted_features = FeatureExtraction(normalized_data);

disp('finished processing data')
