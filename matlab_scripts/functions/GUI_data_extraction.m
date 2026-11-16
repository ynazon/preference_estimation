function gui = GUI_data_extraction(subject,subject_mass,subject_height,dataFolder)
% gui = GUI_data_extraction(subject)
% This function exports GUI data from the experiment
% Inputs:
%   subject - This is a string (ex: 'AFOWR01')
%
% Outputs:
%   gui - This is a struct. This struct contains information from the GUI
%   collected during the experiment. The exported information is the setup
%   of the axes (coordinate id), stiffness of the exo (stiffness), neutral
%   angle (NA), the path of the chosen points (path), the final stiffness
%   chosen (final_stiffness), and the final neutral angle chosen
%   (final_NA).


% find file location
% dataFolder = 'C:\Users\nazon\OneDrive\Documents\Walter Reed\Preference Project\Exploratory Aim Data\Exploratory Aim\';
filePattern = fullfile([dataFolder, '\', subject, '\GUI\', subject, '_EA_*']); % match your file names
files = dir(filePattern);

% create data struct
EA = struct();

% define trials to skip
if strcmp(subject,'AFOWR01')
    FileStartIdx = 2;
else
    FileStartIdx = 1;
end
SaveIdx = 1;

for k = FileStartIdx:length(files)
    fileName = files(k).name;
    filePath = fullfile(files(k).folder, fileName);
    fprintf('GUI Data Processing %s\n', fileName);

    gui_data = load(filePath);

    % Retrieve data 
    EA(SaveIdx).coordinate_id = gui_data.coordinate_id;
    EA(SaveIdx).stiffness = [gui_data.IDs(1,1),-gui_data.IDs(2:end,1)'];
    EA(SaveIdx).NA = gui_data.IDs(:,2);
    EA(SaveIdx).path = gui_data.points;
    EA(SaveIdx).final_stiffness = -gui_data.IDs(end,1);
    EA(SaveIdx).final_NA = gui_data.IDs(end,2);
    EA(SaveIdx).subj_mass = subject_mass;
    EA(SaveIdx).subj_height = subject_height;

    if (strcmp(subject, 'AFOWR01') && k == 5) 
        EA(SaveIdx).stiffness = -EA(SaveIdx).stiffness;
        EA(SaveIdx).final_stiffness = -EA(SaveIdx).final_stiffness;
    end

    SaveIdx = SaveIdx + 1;

end

gui = EA;

end