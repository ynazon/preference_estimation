function cut_data = SegmentSubjectData(subject,affected_side,filepath)
% cut_data = SegmentSubjectData(subject,affected_side)
% Inputs: 
%   subject - This is a string (ex: 'AFOWR01')
%   affected_side - This is a string. The options are 'L' or 'R' for left 
%                   and right. This is the side that the exo was worn on
%                   
% Outputs:
%   left,right - This is a struct with human segmented human data. This segmented
%   data includes time, inverse kinematics (IK) for the hip-knee-ankle,
%   inverse dynamics (ID) for the hip-knee-ankle, vertical GRF's, and ankle
%   joint power. Depending on the affected side chosen (e.g. L or R) the
%   struct will also include segmented Humotech information. The Humotech
%   info provided is controller stiffness (HT_stiffness), controller
%   neutral angle (HT_neutral_angle), exo encoder readings
%   (HT_ankle_angle), and GRF information from the exo (HT_force_plate).

%% Initialization
% Import OpenSim functions
import org.opensim.modeling.*

% Define File path
% filepath = 'C:\Users\nazon\OneDrive\Documents\Walter Reed\Preference Project\Exploratory Aim Data\Exploratory Aim\subjects';
filePattern = fullfile([filepath, '\', subject, '\Humotech\', subject, '_EA_*']); % match your file names
files = dir(filePattern); 

% OpenSim indices
% IK indices
OSTimeidx = 1;
LeftHipFlexIKidx = 16;
LeftHipAddIKidx = 17;
LeftHipRotIKidx = 18;
LeftKneeIKidx = 19;
LeftAnkleIKidx = 21;

RightHipFlexIKidx = 8;
RightHipAddIKidx = 9;
RightHipRotIKidx = 10;
RightKneeIKidx = 11;
RightAnkleIKidx = 13;

% ID indices
LeftHipFlexIDidx = 11;
LeftHipAddIDidx = 12;
LeftHipRotIDidx = 13;
LeftKneeIDidx = 19;
LeftAnkleIDidx = 28;

RightHipFlexIDidx = 8;
RightHipAddIDidx = 9;
RightHipRotIDidx = 10;
RightKneeIDidx = 17;
RightAnkleIDidx = 27;

% JP indices
LeftAnkleJPidx = 2;
RightAnkleJPidx = 3;

% BK indices
PelvisXBKidx = 2;
PelvisYBKidx = 3;
PelvisZBKidx = 4;
PelvisRollXBKidx = 5;
PelvisPitchYBKidx = 6;
PelvisYawZBKidx = 7;

LeftThighXBKidx = 44;
LeftThighYBKidx = 45;
LeftThighZBKidx = 46;
LeftThighRollXBKidx = 47;
LeftThighPitchYBKidx = 48;
LeftThighYawZBKidx = 49;
LeftShankXBKidx = 50;
LeftShankYBKidx = 51;
LeftShankZBKidx = 52;
LeftShankRollXBKidx = 53;
LeftShankPitchYBKidx = 54;
LeftShankYawZBKidx = 55;
LeftFootXBKidx = 68;
LeftFootYBKidx = 69;
LeftFootZBKidx = 70;
LeftFootRollXBKidx = 71;
LeftFootPitchYBKidx = 72;
LeftFootYawZBKidx = 73;

RightThighXBKidx = 8;
RightThighYBKidx = 9;
RightThighZBKidx = 10;
RightThighRollXBKidx = 11;
RightThighPitchYBKidx = 12;
RightThighYawZBKidx = 13;
RightShankXBKidx = 14;
RightShankYBKidx = 15;
RightShankZBKidx = 16;
RightShankRollXBKidx = 17;
RightShankPitchYBKidx = 18;
RightShankYawZBKidx = 19;
RightFootXBKidx = 29;
RightFootYBKidx = 30;
RightFootZBKidx = 31;
RightFootRollXBKidx = 32;
RightFootPitchYBKidx = 33;
RightFootYawZBKidx = 34;

% Load humotech data
fileName = files(1).name;
filePath = fullfile(files(1).folder, fileName);
ht_data = load(filePath);

% Humotech indices
NeutralAngleidx = find(strcmp('Angle Shift (deg)',ht_data.signalNames));
Stiffnessidx = find(strcmp('Coeff5',ht_data.signalNames));
ExoAnkleidx = find(strcmp('Angle (deg)+tare',ht_data.signalNames));
EncoderVelidx = find(strcmp('Encoder Velocity (rad/sec)',ht_data.signalNames));
NetTorqueidx = find(strcmp('Net Torque (Nm)',ht_data.signalNames));
triggeridx = find(strcmp('Trigger_Button',ht_data.signalNames));
timeidx = find(strcmp('Time',ht_data.signalNames));

if affected_side == 'L'
    FPidx = find(strcmp('FP_Left',ht_data.signalNames)); % left force plate index
else
    FPidx = find(strcmp('FP_Right',ht_data.signalNames)); % right force plate index
end

% humotech trigger variables
minSpacing = 100;
trigger_threshold = 1;

% heel strike segmentation variables
HC_upper_threshold = 75;
HC_lower_threshold = 20;
minDist = 1000;

% make struct
cut_data = struct();

% define trials to skip
if strcmp(subject,'AFOWR01')
    FileStartIdx = 2;
else
    FileStartIdx = 1;
end
SaveIdx = 1;

%% Segmentation
for k = FileStartIdx:length(files)
    
    % Load data
    if strcmp(subject,'AFOWR01')
        IK1 = Osim.readMOT([filepath '\' subject '\IK\' subject '_EA_000' num2str(k) '_IK.mot']);
        ID1 = Osim.readSTO([filepath '\' subject '\ID\' subject '_EA_000' num2str(k) '_ID.sto']);
        BKPos1 = Osim.readSTO([filepath '\' subject '\BK\' subject '_EA_000' num2str(k) '_BodyKinematics_pos_global.sto']);
        BKVel1 = Osim.readSTO([filepath '\' subject '\BK\' subject '_EA_000' num2str(k) '_BodyKinematics_vel_global.sto']);
        BKAcc1 = Osim.readSTO([filepath '\' subject '\BK\' subject '_EA_000' num2str(k) '_BodyKinematics_acc_global.sto']);
        GRF1 = Osim.readMOT([filepath '\' subject '\ExtLoads\' subject '_EA_000' num2str(k) '_GRF.mot']);
        load([filepath '\' subject '\Power\' subject '_EA_000' num2str(k) '_FootPower.mat']);
        JP1 = FootPowerTable;
    else
        IK1 = Osim.readMOT([filepath '\' subject '\IK\' subject '_EA_' num2str(k) '_IK.mot']);
        ID1 = Osim.readSTO([filepath '\' subject '\ID\' subject '_EA_' num2str(k) '_ID.sto']);
        BKPos1 = Osim.readSTO([filepath '\' subject '\BK\' subject '_EA_' num2str(k) '_BodyKinematics_pos_global.sto']);
        BKVel1 = Osim.readSTO([filepath '\' subject '\BK\' subject '_EA_' num2str(k) '_BodyKinematics_vel_global.sto']);
        BKAcc1 = Osim.readSTO([filepath '\' subject '\BK\' subject '_EA_' num2str(k) '_BodyKinematics_acc_global.sto']);
        GRF1 = Osim.readMOT([filepath '\' subject '\ExtLoads\' subject '_EA_' num2str(k) '_GRF.mot']);
        load([filepath '\' subject '\Power\' subject '_EA_' num2str(k) '_FootPower.mat']);
        JP1 = FootPowerTable;
    end

    fileName = files(k).name;
    filePath = fullfile(files(k).folder, fileName);
    fprintf('Segment Data Processing %s\n', fileName);
    
    % Load humotech data
    ht_data = load(filePath);

    % Define time and trigger signals 
    time = ht_data.data(:, timeidx);
    trigger = ht_data.data(:, triggeridx);

    % 1. Find candidate triggers where trigger exceeds threshold
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

    elseif length(triggerIdx) == 1
        startIdx = triggerIdx(1);
        stopIdx  = length(trigger);
        keepMask = false(size(ht_data.data,1),1);
        keepMask(startIdx:stopIdx) = true;

    else
        % ✅ Keep only odd-numbered trials (start → stop)
        keepMask = false(size(ht_data.data,1),1);
        %%%%%%%%%%%
        startIdx = triggerIdx(end-1);
        stopIdx  = triggerIdx(end);
        %%%%%%%%%
        % for i = 1:2:length(triggerIdx)
        %     startIdx = triggerIdx(i);
        %     if (i+1) <= length(triggerIdx)
        %         stopIdx  = triggerIdx(i+1);
        %     else
        %         stopIdx  = size(ht_data.data,1);
        %     end
        %     keepMask(startIdx:stopIdx) = true;
        % end
        %%%%%%%%%%%%
        keepMask(startIdx:stopIdx) = true;
        %%%%%%%%%%%
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

    % Adjust time for continuity
    ht_data.data_cut(:, timeidx) = ht_data.data_cut(:, timeidx) - offset(keepMask);
    ht_data.data = ht_data.data_cut;
    
    % Find start time for segmentation
    tstart = IK1.Header(1);
    tstart_idx = find(round(IK1.Header,3) == round(tstart,3));
    GRF1 = GRF1(find(round(GRF1.Header,3) == round(tstart,3),1):end,:);
    GRF_time_vector = GRF1{:,1};
    HT_time_vector = ht_data.data(:,timeidx)-ht_data.data(1,timeidx);

    % Interpolate Data to match GRF sampling frequency
    HT1 = interp1(HT_time_vector, ht_data.data(tstart_idx:end,...
        [Stiffnessidx,NeutralAngleidx,ExoAnkleidx,FPidx, ...
        EncoderVelidx,NetTorqueidx]), GRF_time_vector, 'linear');
    IK1 = interp1(IK1{tstart_idx:end,1}, IK1{tstart_idx:end,:}, GRF_time_vector, 'linear');
    ID1 = interp1(ID1{tstart_idx:end,1}, ID1{tstart_idx:end,:}, GRF_time_vector, 'linear');
    JP1 = interp1(JP1{tstart_idx:end,1}, JP1{tstart_idx:end,:}, GRF_time_vector, 'linear'); 
    BKPos1 = interp1(BKPos1{tstart_idx:end,1}, BKPos1{tstart_idx:end,:}, GRF_time_vector, 'linear');
    BKVel1 = interp1(BKVel1{tstart_idx:end,1}, BKVel1{tstart_idx:end,:}, GRF_time_vector, 'linear');
    BKAcc1 = interp1(BKAcc1{tstart_idx:end,1}, BKAcc1{tstart_idx:end,:}, GRF_time_vector, 'linear');
 
    % Heel strike and toe off detection
    HC_L = find(GRF1.FPLeft_vy(2:end) > HC_lower_threshold & GRF1.FPLeft_vy(2:end) < HC_upper_threshold & diff(GRF1.FPLeft_vy) > 0 ) + 1;
    HC_L = HC_L([true; diff(HC_L) > minDist]);
    TO_L = find(GRF1.FPLeft_vy(2:end) > HC_lower_threshold & GRF1.FPLeft_vy(2:end) < HC_upper_threshold & diff(GRF1.FPLeft_vy) < 0 ) + 1;
    TO_L = TO_L([true; diff(TO_L) > minDist]);
    HC_R = find(GRF1.FPRight_vy(2:end) > HC_lower_threshold & GRF1.FPRight_vy(2:end) < HC_upper_threshold & diff(GRF1.FPRight_vy) > 0 ) + 1;
    HC_R = HC_R([true; diff(HC_R) > minDist]);
    TO_R = find(GRF1.FPRight_vy(2:end) > HC_lower_threshold & GRF1.FPRight_vy(2:end) < HC_upper_threshold & diff(GRF1.FPRight_vy) < 0 ) + 1;
    TO_R = TO_R([true; diff(TO_R) > minDist]);
    
    if HC_L(1) > TO_L(1)
        HC_L = HC_L(1:end-1);
        TO_L = TO_L(2:end);
    end
    
    if HC_R(1) > TO_R(1)
        HC_R = HC_R(1:end-1);
        TO_R = TO_R(2:end);
    end

    % Segment data
    if affected_side == 'L'
    % Clear data
    time_L = {};
    Hip_Flex_IK_L = {};
    Hip_Add_IK_L = {};
    Hip_Rot_IK_L = {};
    Knee_IK_L = {};
    Ankle_IK_L = {};

    Hip_Flex_ID_L = {};
    Hip_Add_ID_L = {};
    Hip_Rot_ID_L = {};
    Knee_ID_L = {};
    Ankle_ID_L = {};

    GRF_L = {};
    Ankle_JP_L = {};

    % BK Pos
    Pos_PelvisX_BK = {};
    Pos_PelvisY_BK = {};
    Pos_PelvisZ_BK = {};
    Pos_PelvisRollX_BK = {};
    Pos_PelvisPitchY_BK = {};
    Pos_PelvisYawZ_BK = {};

    Pos_ThighX_BK_L = {};
    Pos_ThighY_BK_L = {};
    Pos_ThighZ_BK_L = {};
    Pos_ThighRollX_BK_L = {};
    Pos_ThighPitchY_BK_L = {};
    Pos_ThighYawZ_BK_L = {};

    Pos_ShankX_BK_L = {};
    Pos_ShankY_BK_L = {};
    Pos_ShankZ_BK_L = {};
    Pos_ShankRollX_BK_L = {};
    Pos_ShankPitchY_BK_L = {};
    Pos_ShankYawZ_BK_L = {};

    Pos_FootX_BK_L = {};
    Pos_FootY_BK_L = {};
    Pos_FootZ_BK_L = {};
    Pos_FootRollX_BK_L = {};
    Pos_FootPitchY_BK_L = {};
    Pos_FootYawZ_BK_L = {};
    
    % BK Vel
    Vel_PelvisX_BK = {};
    Vel_PelvisY_BK = {};
    Vel_PelvisZ_BK = {};
    Vel_PelvisRollX_BK = {};
    Vel_PelvisPitchY_BK = {};
    Vel_PelvisYawZ_BK = {};

    Vel_ThighX_BK_L = {};
    Vel_ThighY_BK_L = {};
    Vel_ThighZ_BK_L = {};
    Vel_ThighRollX_BK_L = {};
    Vel_ThighPitchY_BK_L = {};
    Vel_ThighYawZ_BK_L = {};

    Vel_ShankX_BK_L = {};
    Vel_ShankY_BK_L = {};
    Vel_ShankZ_BK_L = {};
    Vel_ShankRollX_BK_L = {};
    Vel_ShankPitchY_BK_L = {};
    Vel_ShankYawZ_BK_L = {};

    Vel_FootX_BK_L = {};
    Vel_FootY_BK_L = {};
    Vel_FootZ_BK_L = {};
    Vel_FootRollX_BK_L = {};
    Vel_FootPitchY_BK_L = {};
    Vel_FootYawZ_BK_L = {};

    % BK Acc
    Acc_PelvisX_BK = {};
    Acc_PelvisY_BK = {};
    Acc_PelvisZ_BK = {};
    Acc_PelvisRollX_BK = {};
    Acc_PelvisPitchY_BK = {};
    Acc_PelvisYawZ_BK = {};

    Acc_ThighX_BK_L = {};
    Acc_ThighY_BK_L = {};
    Acc_ThighZ_BK_L = {};
    Acc_ThighRollX_BK_L = {};
    Acc_ThighPitchY_BK_L = {};
    Acc_ThighYawZ_BK_L = {};

    Acc_ShankX_BK_L = {};
    Acc_ShankY_BK_L = {};
    Acc_ShankZ_BK_L = {};
    Acc_ShankRollX_BK_L = {};
    Acc_ShankPitchY_BK_L = {};
    Acc_ShankYawZ_BK_L = {};

    Acc_FootX_BK_L = {};
    Acc_FootY_BK_L = {};
    Acc_FootZ_BK_L = {};
    Acc_FootRollX_BK_L = {};
    Acc_FootPitchY_BK_L = {};
    Acc_FootYawZ_BK_L = {};

    % Right Side
    time_R = {};

    Hip_Flex_IK_R = {};
    Hip_Add_IK_R = {};
    Hip_Rot_IK_R = {};
    Knee_IK_R = {};
    Ankle_IK_R = {};

    Hip_Flex_ID_R = {};
    Hip_Add_ID_R = {};
    Hip_Rot_ID_R = {};
    Knee_ID_R = {};
    Ankle_ID_R = {};

    GRF_R = {};
    Ankle_JP_R = {};

    % BK Pos
    Pos_ThighX_BK_R = {};
    Pos_ThighY_BK_R = {};
    Pos_ThighZ_BK_R = {};
    Pos_ThighRollX_BK_R = {};
    Pos_ThighPitchY_BK_R = {};
    Pos_ThighYawZ_BK_R = {};

    Pos_ShankX_BK_R = {};
    Pos_ShankY_BK_R = {};
    Pos_ShankZ_BK_R = {};
    Pos_ShankRollX_BK_R = {};
    Pos_ShankPitchY_BK_R = {};
    Pos_ShankYawZ_BK_R = {};

    Pos_FootX_BK_R = {};
    Pos_FootY_BK_R = {};
    Pos_FootZ_BK_R = {};
    Pos_FootRollX_BK_R = {};
    Pos_FootPitchY_BK_R = {};
    Pos_FootYawZ_BK_R = {};

    % BK Vel
    Vel_ThighX_BK_R = {};
    Vel_ThighY_BK_R = {};
    Vel_ThighZ_BK_R = {};
    Vel_ThighRollX_BK_R = {};
    Vel_ThighPitchY_BK_R = {};
    Vel_ThighYawZ_BK_R = {};

    Vel_ShankX_BK_R = {};
    Vel_ShankY_BK_R = {};
    Vel_ShankZ_BK_R = {};
    Vel_ShankRollX_BK_R = {};
    Vel_ShankPitchY_BK_R = {};
    Vel_ShankYawZ_BK_R = {};

    Vel_FootX_BK_R = {};
    Vel_FootY_BK_R = {};
    Vel_FootZ_BK_R = {};
    Vel_FootRollX_BK_R = {};
    Vel_FootPitchY_BK_R = {};
    Vel_FootYawZ_BK_R = {};

    % BK Acc
    Acc_ThighX_BK_R = {};
    Acc_ThighY_BK_R = {};
    Acc_ThighZ_BK_R = {};
    Acc_ThighRollX_BK_R = {};
    Acc_ThighPitchY_BK_R = {};
    Acc_ThighYawZ_BK_R = {};

    Acc_ShankX_BK_R = {};
    Acc_ShankY_BK_R = {};
    Acc_ShankZ_BK_R = {};
    Acc_ShankRollX_BK_R = {};
    Acc_ShankPitchY_BK_R = {};
    Acc_ShankYawZ_BK_R = {};

    Acc_FootX_BK_R = {};
    Acc_FootY_BK_R = {};
    Acc_FootZ_BK_R = {};
    Acc_FootRollX_BK_R = {};
    Acc_FootPitchY_BK_R = {};
    Acc_FootYawZ_BK_R = {};

    % Humotech Data
    HT_stiffness = {};
    HT_neutral_angle = {};
    HT_ankle_angle = {};
    HT_force_plate = {};
    HT_encoder_velocity = {};
    HT_exo_torque = {};
        
        for i = 1:length(HC_L)-1
            time_L{i} = IK1(HC_L(i):HC_L(i+1),OSTimeidx);
            Hip_Flex_IK_L{i} = IK1(HC_L(i):HC_L(i+1),LeftHipFlexIKidx);
            Hip_Add_IK_L{i} = IK1(HC_L(i):HC_L(i+1),LeftHipAddIKidx);
            Hip_Rot_IK_L{i} = IK1(HC_L(i):HC_L(i+1),LeftHipRotIKidx);
            Knee_IK_L{i} = IK1(HC_L(i):HC_L(i+1),LeftKneeIKidx);
            Ankle_IK_L{i} = IK1(HC_L(i):HC_L(i+1),LeftAnkleIKidx);

            Hip_Flex_ID_L{i} = ID1(HC_L(i):HC_L(i+1),LeftHipFlexIDidx);
            Hip_Add_ID_L{i} = ID1(HC_L(i):HC_L(i+1),LeftHipAddIDidx);
            Hip_Rot_ID_L{i} = ID1(HC_L(i):HC_L(i+1),LeftHipRotIDidx);
            Knee_ID_L{i} = ID1(HC_L(i):HC_L(i+1),LeftKneeIDidx);
            Ankle_ID_L{i} = ID1(HC_L(i):HC_L(i+1),LeftAnkleIDidx);

            GRF_L{i} = GRF1.FPLeft_vy(HC_L(i):HC_L(i+1));
            Ankle_JP_L{i} = JP1(HC_L(i):HC_L(i+1),LeftAnkleJPidx);

            Pos_PelvisX_BK{i} = BKPos1(HC_L(i):HC_L(i+1),PelvisXBKidx);
            Pos_PelvisY_BK{i} = BKPos1(HC_L(i):HC_L(i+1),PelvisYBKidx);
            Pos_PelvisZ_BK{i} = BKPos1(HC_L(i):HC_L(i+1),PelvisZBKidx);
            Pos_PelvisRollX_BK{i} = BKPos1(HC_L(i):HC_L(i+1),PelvisRollXBKidx);
            Pos_PelvisPitchY_BK{i} = BKPos1(HC_L(i):HC_L(i+1),PelvisPitchYBKidx);
            Pos_PelvisYawZ_BK{i} = BKPos1(HC_L(i):HC_L(i+1),PelvisYawZBKidx);

            Pos_ThighX_BK_L{i} = BKPos1(HC_L(i):HC_L(i+1),LeftThighXBKidx);
            Pos_ThighY_BK_L{i} = BKPos1(HC_L(i):HC_L(i+1),LeftThighYBKidx);
            Pos_ThighZ_BK_L{i} = BKPos1(HC_L(i):HC_L(i+1),LeftThighZBKidx);
            Pos_ThighRollX_BK_L{i} = BKPos1(HC_L(i):HC_L(i+1),LeftThighRollXBKidx);
            Pos_ThighPitchY_BK_L{i} = BKPos1(HC_L(i):HC_L(i+1),LeftThighPitchYBKidx);
            Pos_ThighYawZ_BK_L{i} = BKPos1(HC_L(i):HC_L(i+1),LeftThighYawZBKidx);

            Pos_ShankX_BK_L{i} = BKPos1(HC_L(i):HC_L(i+1),LeftShankXBKidx);
            Pos_ShankY_BK_L{i} = BKPos1(HC_L(i):HC_L(i+1),LeftShankYBKidx);
            Pos_ShankZ_BK_L{i} = BKPos1(HC_L(i):HC_L(i+1),LeftShankZBKidx);
            Pos_ShankRollX_BK_L{i} = BKPos1(HC_L(i):HC_L(i+1),LeftShankRollXBKidx);
            Pos_ShankPitchY_BK_L{i} = BKPos1(HC_L(i):HC_L(i+1),LeftShankPitchYBKidx);
            Pos_ShankYawZ_BK_L{i} = BKPos1(HC_L(i):HC_L(i+1),LeftShankYawZBKidx);

            Pos_FootX_BK_L{i} = BKPos1(HC_L(i):HC_L(i+1),LeftFootXBKidx);
            Pos_FootY_BK_L{i} = BKPos1(HC_L(i):HC_L(i+1),LeftFootYBKidx);
            Pos_FootZ_BK_L{i} = BKPos1(HC_L(i):HC_L(i+1),LeftFootZBKidx);
            Pos_FootRollX_BK_L{i} = BKPos1(HC_L(i):HC_L(i+1),LeftFootRollXBKidx);
            Pos_FootPitchY_BK_L{i} = BKPos1(HC_L(i):HC_L(i+1),LeftFootPitchYBKidx);
            Pos_FootYawZ_BK_L{i} = BKPos1(HC_L(i):HC_L(i+1),LeftFootYawZBKidx);

            Vel_PelvisX_BK{i} = BKVel1(HC_L(i):HC_L(i+1),PelvisXBKidx);
            Vel_PelvisY_BK{i} = BKVel1(HC_L(i):HC_L(i+1),PelvisYBKidx);
            Vel_PelvisZ_BK{i} = BKVel1(HC_L(i):HC_L(i+1),PelvisZBKidx);
            Vel_PelvisRollX_BK{i} = BKVel1(HC_L(i):HC_L(i+1),PelvisRollXBKidx);
            Vel_PelvisPitchY_BK{i} = BKVel1(HC_L(i):HC_L(i+1),PelvisPitchYBKidx);
            Vel_PelvisYawZ_BK{i} = BKVel1(HC_L(i):HC_L(i+1),PelvisYawZBKidx);

            Vel_ThighX_BK_L{i} = BKVel1(HC_L(i):HC_L(i+1),LeftThighXBKidx);
            Vel_ThighY_BK_L{i} = BKVel1(HC_L(i):HC_L(i+1),LeftThighYBKidx);
            Vel_ThighZ_BK_L{i} = BKVel1(HC_L(i):HC_L(i+1),LeftThighZBKidx);
            Vel_ThighRollX_BK_L{i} = BKVel1(HC_L(i):HC_L(i+1),LeftThighRollXBKidx);
            Vel_ThighPitchY_BK_L{i} = BKVel1(HC_L(i):HC_L(i+1),LeftThighPitchYBKidx);
            Vel_ThighYawZ_BK_L{i} = BKVel1(HC_L(i):HC_L(i+1),LeftThighYawZBKidx);

            Vel_ShankX_BK_L{i} = BKVel1(HC_L(i):HC_L(i+1),LeftShankXBKidx);
            Vel_ShankY_BK_L{i} = BKVel1(HC_L(i):HC_L(i+1),LeftShankYBKidx);
            Vel_ShankZ_BK_L{i} = BKVel1(HC_L(i):HC_L(i+1),LeftShankZBKidx);
            Vel_ShankRollX_BK_L{i} = BKVel1(HC_L(i):HC_L(i+1),LeftShankRollXBKidx);
            Vel_ShankPitchY_BK_L{i} = BKVel1(HC_L(i):HC_L(i+1),LeftShankPitchYBKidx);
            Vel_ShankYawZ_BK_L{i} = BKVel1(HC_L(i):HC_L(i+1),LeftShankYawZBKidx);

            Vel_FootX_BK_L{i} = BKVel1(HC_L(i):HC_L(i+1),LeftFootXBKidx);
            Vel_FootY_BK_L{i} = BKVel1(HC_L(i):HC_L(i+1),LeftFootYBKidx);
            Vel_FootZ_BK_L{i} = BKVel1(HC_L(i):HC_L(i+1),LeftFootZBKidx);
            Vel_FootRollX_BK_L{i} = BKVel1(HC_L(i):HC_L(i+1),LeftFootRollXBKidx);
            Vel_FootPitchY_BK_L{i} = BKVel1(HC_L(i):HC_L(i+1),LeftFootPitchYBKidx);
            Vel_FootYawZ_BK_L{i} = BKVel1(HC_L(i):HC_L(i+1),LeftFootYawZBKidx);

            Acc_PelvisX_BK{i} = BKAcc1(HC_L(i):HC_L(i+1),PelvisXBKidx);
            Acc_PelvisY_BK{i} = BKAcc1(HC_L(i):HC_L(i+1),PelvisYBKidx);
            Acc_PelvisZ_BK{i} = BKAcc1(HC_L(i):HC_L(i+1),PelvisZBKidx);
            Acc_PelvisRollX_BK{i} = BKAcc1(HC_L(i):HC_L(i+1),PelvisRollXBKidx);
            Acc_PelvisPitchY_BK{i} = BKAcc1(HC_L(i):HC_L(i+1),PelvisPitchYBKidx);
            Acc_PelvisYawZ_BK{i} = BKAcc1(HC_L(i):HC_L(i+1),PelvisYawZBKidx);

            Acc_ThighX_BK_L{i} = BKAcc1(HC_L(i):HC_L(i+1),LeftThighXBKidx);
            Acc_ThighY_BK_L{i} = BKAcc1(HC_L(i):HC_L(i+1),LeftThighYBKidx);
            Acc_ThighZ_BK_L{i} = BKAcc1(HC_L(i):HC_L(i+1),LeftThighZBKidx);
            Acc_ThighRollX_BK_L{i} = BKAcc1(HC_L(i):HC_L(i+1),LeftThighRollXBKidx);
            Acc_ThighPitchY_BK_L{i} = BKAcc1(HC_L(i):HC_L(i+1),LeftThighPitchYBKidx);
            Acc_ThighYawZ_BK_L{i} = BKAcc1(HC_L(i):HC_L(i+1),LeftThighYawZBKidx);

            Acc_ShankX_BK_L{i} = BKAcc1(HC_L(i):HC_L(i+1),LeftShankXBKidx);
            Acc_ShankY_BK_L{i} = BKAcc1(HC_L(i):HC_L(i+1),LeftShankYBKidx);
            Acc_ShankZ_BK_L{i} = BKAcc1(HC_L(i):HC_L(i+1),LeftShankZBKidx);
            Acc_ShankRollX_BK_L{i} = BKAcc1(HC_L(i):HC_L(i+1),LeftShankRollXBKidx);
            Acc_ShankPitchY_BK_L{i} = BKAcc1(HC_L(i):HC_L(i+1),LeftShankPitchYBKidx);
            Acc_ShankYawZ_BK_L{i} = BKAcc1(HC_L(i):HC_L(i+1),LeftShankYawZBKidx);

            Acc_FootX_BK_L{i} = BKAcc1(HC_L(i):HC_L(i+1),LeftFootXBKidx);
            Acc_FootY_BK_L{i} = BKAcc1(HC_L(i):HC_L(i+1),LeftFootYBKidx);
            Acc_FootZ_BK_L{i} = BKAcc1(HC_L(i):HC_L(i+1),LeftFootZBKidx);
            Acc_FootRollX_BK_L{i} = BKAcc1(HC_L(i):HC_L(i+1),LeftFootRollXBKidx);
            Acc_FootPitchY_BK_L{i} = BKAcc1(HC_L(i):HC_L(i+1),LeftFootPitchYBKidx);
            Acc_FootYawZ_BK_L{i} = BKAcc1(HC_L(i):HC_L(i+1),LeftFootYawZBKidx);

            HT_stiffness{i} = -HT1(HC_L(i):HC_L(i+1),1);
            HT_neutral_angle{i} = -HT1(HC_L(i):HC_L(i+1),2);
            HT_ankle_angle{i} = -HT1(HC_L(i):HC_L(i+1),3);
            HT_force_plate{i} = HT1(HC_L(i):HC_L(i+1),4);
            HT_encoder_velocity{i} = -HT1(HC_L(i):HC_L(i+1),5);
            HT_exo_torque{i} = HT1(HC_L(i):HC_L(i+1),6);
        end
        
        for i = 1:length(HC_R)-1
            time_R{i} = IK1(HC_R(i):HC_R(i+1),OSTimeidx);
            Hip_Flex_IK_R{i} = IK1(HC_R(i):HC_R(i+1),RightHipFlexIKidx);
            Hip_Add_IK_R{i} = IK1(HC_R(i):HC_R(i+1),RightHipAddIKidx);
            Hip_Rot_IK_R{i} = IK1(HC_R(i):HC_R(i+1),RightHipRotIKidx);
            Knee_IK_R{i} = IK1(HC_R(i):HC_R(i+1),RightKneeIKidx);
            Ankle_IK_R{i} = IK1(HC_R(i):HC_R(i+1),RightAnkleIKidx);

            Hip_Flex_ID_R{i} = ID1(HC_R(i):HC_R(i+1),RightHipFlexIDidx);
            Hip_Add_ID_R{i} = ID1(HC_R(i):HC_R(i+1),RightHipAddIDidx);
            Hip_Rot_ID_R{i} = ID1(HC_R(i):HC_R(i+1),RightHipRotIDidx);
            Knee_ID_R{i} = ID1(HC_R(i):HC_R(i+1),RightKneeIDidx);
            Ankle_ID_R{i} = ID1(HC_R(i):HC_R(i+1),RightAnkleIDidx);

            GRF_R{i} = GRF1.FPRight_vy(HC_R(i):HC_R(i+1));
            Ankle_JP_R{i} = JP1(HC_R(i):HC_R(i+1),RightAnkleJPidx);

            Pos_ThighX_BK_R{i} = BKPos1(HC_R(i):HC_R(i+1),RightThighXBKidx);
            Pos_ThighY_BK_R{i} = BKPos1(HC_R(i):HC_R(i+1),RightThighYBKidx);
            Pos_ThighZ_BK_R{i} = BKPos1(HC_R(i):HC_R(i+1),RightThighZBKidx);
            Pos_ThighRollX_BK_R{i} = BKPos1(HC_R(i):HC_R(i+1),RightThighRollXBKidx);
            Pos_ThighPitchY_BK_R{i} = BKPos1(HC_R(i):HC_R(i+1),RightThighPitchYBKidx);
            Pos_ThighYawZ_BK_R{i} = BKPos1(HC_R(i):HC_R(i+1),RightThighYawZBKidx);

            Pos_ShankX_BK_R{i} = BKPos1(HC_R(i):HC_R(i+1),RightShankXBKidx);
            Pos_ShankY_BK_R{i} = BKPos1(HC_R(i):HC_R(i+1),RightShankYBKidx);
            Pos_ShankZ_BK_R{i} = BKPos1(HC_R(i):HC_R(i+1),RightShankZBKidx);
            Pos_ShankRollX_BK_R{i} = BKPos1(HC_R(i):HC_R(i+1),RightShankRollXBKidx);
            Pos_ShankPitchY_BK_R{i} = BKPos1(HC_R(i):HC_R(i+1),RightShankPitchYBKidx);
            Pos_ShankYawZ_BK_R{i} = BKPos1(HC_R(i):HC_R(i+1),RightShankYawZBKidx);

            Pos_FootX_BK_R{i} = BKPos1(HC_R(i):HC_R(i+1),RightFootXBKidx);
            Pos_FootY_BK_R{i} = BKPos1(HC_R(i):HC_R(i+1),RightFootYBKidx);
            Pos_FootZ_BK_R{i} = BKPos1(HC_R(i):HC_R(i+1),RightFootZBKidx);
            Pos_FootRollX_BK_R{i} = BKPos1(HC_R(i):HC_R(i+1),RightFootRollXBKidx);
            Pos_FootPitchY_BK_R{i} = BKPos1(HC_R(i):HC_R(i+1),RightFootPitchYBKidx);
            Pos_FootYawZ_BK_R{i} = BKPos1(HC_R(i):HC_R(i+1),RightFootYawZBKidx);

            Vel_ThighX_BK_R{i} = BKVel1(HC_R(i):HC_R(i+1),RightThighXBKidx);
            Vel_ThighY_BK_R{i} = BKVel1(HC_R(i):HC_R(i+1),RightThighYBKidx);
            Vel_ThighZ_BK_R{i} = BKVel1(HC_R(i):HC_R(i+1),RightThighZBKidx);
            Vel_ThighRollX_BK_R{i} = BKVel1(HC_R(i):HC_R(i+1),RightThighRollXBKidx);
            Vel_ThighPitchY_BK_R{i} = BKVel1(HC_R(i):HC_R(i+1),RightThighPitchYBKidx);
            Vel_ThighYawZ_BK_R{i} = BKVel1(HC_R(i):HC_R(i+1),RightThighYawZBKidx);

            Vel_ShankX_BK_R{i} = BKVel1(HC_R(i):HC_R(i+1),RightShankXBKidx);
            Vel_ShankY_BK_R{i} = BKVel1(HC_R(i):HC_R(i+1),RightShankYBKidx);
            Vel_ShankZ_BK_R{i} = BKVel1(HC_R(i):HC_R(i+1),RightShankZBKidx);
            Vel_ShankRollX_BK_R{i} = BKVel1(HC_R(i):HC_R(i+1),RightShankRollXBKidx);
            Vel_ShankPitchY_BK_R{i} = BKVel1(HC_R(i):HC_R(i+1),RightShankPitchYBKidx);
            Vel_ShankYawZ_BK_R{i} = BKVel1(HC_R(i):HC_R(i+1),RightShankYawZBKidx);

            Vel_FootX_BK_R{i} = BKVel1(HC_R(i):HC_R(i+1),RightFootXBKidx);
            Vel_FootY_BK_R{i} = BKVel1(HC_R(i):HC_R(i+1),RightFootYBKidx);
            Vel_FootZ_BK_R{i} = BKVel1(HC_R(i):HC_R(i+1),RightFootZBKidx);
            Vel_FootRollX_BK_R{i} = BKVel1(HC_R(i):HC_R(i+1),RightFootRollXBKidx);
            Vel_FootPitchY_BK_R{i} = BKVel1(HC_R(i):HC_R(i+1),RightFootPitchYBKidx);
            Vel_FootYawZ_BK_R{i} = BKVel1(HC_R(i):HC_R(i+1),RightFootYawZBKidx);

            Acc_ThighX_BK_R{i} = BKAcc1(HC_R(i):HC_R(i+1),RightThighXBKidx);
            Acc_ThighY_BK_R{i} = BKAcc1(HC_R(i):HC_R(i+1),RightThighYBKidx);
            Acc_ThighZ_BK_R{i} = BKAcc1(HC_R(i):HC_R(i+1),RightThighZBKidx);
            Acc_ThighRollX_BK_R{i} = BKAcc1(HC_R(i):HC_R(i+1),RightThighRollXBKidx);
            Acc_ThighPitchY_BK_R{i} = BKAcc1(HC_R(i):HC_R(i+1),RightThighPitchYBKidx);
            Acc_ThighYawZ_BK_R{i} = BKAcc1(HC_R(i):HC_R(i+1),RightThighYawZBKidx);

            Acc_ShankX_BK_R{i} = BKAcc1(HC_R(i):HC_R(i+1),RightShankXBKidx);
            Acc_ShankY_BK_R{i} = BKAcc1(HC_R(i):HC_R(i+1),RightShankYBKidx);
            Acc_ShankZ_BK_R{i} = BKAcc1(HC_R(i):HC_R(i+1),RightShankZBKidx);
            Acc_ShankRollX_BK_R{i} = BKAcc1(HC_R(i):HC_R(i+1),RightShankRollXBKidx);
            Acc_ShankPitchY_BK_R{i} = BKAcc1(HC_R(i):HC_R(i+1),RightShankPitchYBKidx);
            Acc_ShankYawZ_BK_R{i} = BKAcc1(HC_R(i):HC_R(i+1),RightShankYawZBKidx);

            Acc_FootX_BK_R{i} = BKAcc1(HC_R(i):HC_R(i+1),RightFootXBKidx);
            Acc_FootY_BK_R{i} = BKAcc1(HC_R(i):HC_R(i+1),RightFootYBKidx);
            Acc_FootZ_BK_R{i} = BKAcc1(HC_R(i):HC_R(i+1),RightFootZBKidx);
            Acc_FootRollX_BK_R{i} = BKAcc1(HC_R(i):HC_R(i+1),RightFootRollXBKidx);
            Acc_FootPitchY_BK_R{i} = BKAcc1(HC_R(i):HC_R(i+1),RightFootPitchYBKidx);
            Acc_FootYawZ_BK_R{i} = BKAcc1(HC_R(i):HC_R(i+1),RightFootYawZBKidx);
        end
    
    else
    % Clear data
    time_L = {};
    Hip_Flex_IK_L = {};
    Hip_Add_IK_L = {};
    Hip_Rot_IK_L = {};
    Knee_IK_L = {};
    Ankle_IK_L = {};

    Hip_Flex_ID_L = {};
    Hip_Add_ID_L = {};
    Hip_Rot_ID_L = {};
    Knee_ID_L = {};
    Ankle_ID_L = {};

    GRF_L = {};
    Ankle_JP_L = {};

    % BK Pos
    Pos_PelvisX_BK = {};
    Pos_PelvisY_BK = {};
    Pos_PelvisZ_BK = {};
    Pos_PelvisRollX_BK = {};
    Pos_PelvisPitchY_BK = {};
    Pos_PelvisYawZ_BK = {};

    Pos_ThighX_BK_L = {};
    Pos_ThighY_BK_L = {};
    Pos_ThighZ_BK_L = {};
    Pos_ThighRollX_BK_L = {};
    Pos_ThighPitchY_BK_L = {};
    Pos_ThighYawZ_BK_L = {};

    Pos_ShankX_BK_L = {};
    Pos_ShankY_BK_L = {};
    Pos_ShankZ_BK_L = {};
    Pos_ShankRollX_BK_L = {};
    Pos_ShankPitchY_BK_L = {};
    Pos_ShankYawZ_BK_L = {};

    Pos_FootX_BK_L = {};
    Pos_FootY_BK_L = {};
    Pos_FootZ_BK_L = {};
    Pos_FootRollX_BK_L = {};
    Pos_FootPitchY_BK_L = {};
    Pos_FootYawZ_BK_L = {};
    
    % BK Vel
    Vel_PelvisX_BK = {};
    Vel_PelvisY_BK = {};
    Vel_PelvisZ_BK = {};
    Vel_PelvisRollX_BK = {};
    Vel_PelvisPitchY_BK = {};
    Vel_PelvisYawZ_BK = {};

    Vel_ThighX_BK_L = {};
    Vel_ThighY_BK_L = {};
    Vel_ThighZ_BK_L = {};
    Vel_ThighRollX_BK_L = {};
    Vel_ThighPitchY_BK_L = {};
    Vel_ThighYawZ_BK_L = {};

    Vel_ShankX_BK_L = {};
    Vel_ShankY_BK_L = {};
    Vel_ShankZ_BK_L = {};
    Vel_ShankRollX_BK_L = {};
    Vel_ShankPitchY_BK_L = {};
    Vel_ShankYawZ_BK_L = {};

    Vel_FootX_BK_L = {};
    Vel_FootY_BK_L = {};
    Vel_FootZ_BK_L = {};
    Vel_FootRollX_BK_L = {};
    Vel_FootPitchY_BK_L = {};
    Vel_FootYawZ_BK_L = {};

    % BK Acc
    Acc_PelvisX_BK = {};
    Acc_PelvisY_BK = {};
    Acc_PelvisZ_BK = {};
    Acc_PelvisRollX_BK = {};
    Acc_PelvisPitchY_BK = {};
    Acc_PelvisYawZ_BK = {};

    Acc_ThighX_BK_L = {};
    Acc_ThighY_BK_L = {};
    Acc_ThighZ_BK_L = {};
    Acc_ThighRollX_BK_L = {};
    Acc_ThighPitchY_BK_L = {};
    Acc_ThighYawZ_BK_L = {};

    Acc_ShankX_BK_L = {};
    Acc_ShankY_BK_L = {};
    Acc_ShankZ_BK_L = {};
    Acc_ShankRollX_BK_L = {};
    Acc_ShankPitchY_BK_L = {};
    Acc_ShankYawZ_BK_L = {};

    Acc_FootX_BK_L = {};
    Acc_FootY_BK_L = {};
    Acc_FootZ_BK_L = {};
    Acc_FootRollX_BK_L = {};
    Acc_FootPitchY_BK_L = {};
    Acc_FootYawZ_BK_L = {};

    % Right Side
    time_R = {};

    Hip_Flex_IK_R = {};
    Hip_Add_IK_R = {};
    Hip_Rot_IK_R = {};
    Knee_IK_R = {};
    Ankle_IK_R = {};

    Hip_Flex_ID_R = {};
    Hip_Add_ID_R = {};
    Hip_Rot_ID_R = {};
    Knee_ID_R = {};
    Ankle_ID_R = {};

    GRF_R = {};
    Ankle_JP_R = {};

    % BK Pos
    Pos_ThighX_BK_R = {};
    Pos_ThighY_BK_R = {};
    Pos_ThighZ_BK_R = {};
    Pos_ThighRollX_BK_R = {};
    Pos_ThighPitchY_BK_R = {};
    Pos_ThighYawZ_BK_R = {};

    Pos_ShankX_BK_R = {};
    Pos_ShankY_BK_R = {};
    Pos_ShankZ_BK_R = {};
    Pos_ShankRollX_BK_R = {};
    Pos_ShankPitchY_BK_R = {};
    Pos_ShankYawZ_BK_R = {};

    Pos_FootX_BK_R = {};
    Pos_FootY_BK_R = {};
    Pos_FootZ_BK_R = {};
    Pos_FootRollX_BK_R = {};
    Pos_FootPitchY_BK_R = {};
    Pos_FootYawZ_BK_R = {};

    % BK Vel
    Vel_ThighX_BK_R = {};
    Vel_ThighY_BK_R = {};
    Vel_ThighZ_BK_R = {};
    Vel_ThighRollX_BK_R = {};
    Vel_ThighPitchY_BK_R = {};
    Vel_ThighYawZ_BK_R = {};

    Vel_ShankX_BK_R = {};
    Vel_ShankY_BK_R = {};
    Vel_ShankZ_BK_R = {};
    Vel_ShankRollX_BK_R = {};
    Vel_ShankPitchY_BK_R = {};
    Vel_ShankYawZ_BK_R = {};

    Vel_FootX_BK_R = {};
    Vel_FootY_BK_R = {};
    Vel_FootZ_BK_R = {};
    Vel_FootRollX_BK_R = {};
    Vel_FootPitchY_BK_R = {};
    Vel_FootYawZ_BK_R = {};

    % BK Acc
    Acc_ThighX_BK_R = {};
    Acc_ThighY_BK_R = {};
    Acc_ThighZ_BK_R = {};
    Acc_ThighRollX_BK_R = {};
    Acc_ThighPitchY_BK_R = {};
    Acc_ThighYawZ_BK_R = {};

    Acc_ShankX_BK_R = {};
    Acc_ShankY_BK_R = {};
    Acc_ShankZ_BK_R = {};
    Acc_ShankRollX_BK_R = {};
    Acc_ShankPitchY_BK_R = {};
    Acc_ShankYawZ_BK_R = {};

    Acc_FootX_BK_R = {};
    Acc_FootY_BK_R = {};
    Acc_FootZ_BK_R = {};
    Acc_FootRollX_BK_R = {};
    Acc_FootPitchY_BK_R = {};
    Acc_FootYawZ_BK_R = {};

    % Humotech Data
    HT_stiffness = {};
    HT_neutral_angle = {};
    HT_ankle_angle = {};
    HT_force_plate = {};
    HT_encoder_velocity = {};
    HT_exo_torque = {};
        
        for i = 1:length(HC_L)-1
            time_L{i} = IK1(HC_L(i):HC_L(i+1),OSTimeidx);
            Hip_Flex_IK_L{i} = IK1(HC_L(i):HC_L(i+1),LeftHipFlexIKidx);
            Hip_Add_IK_L{i} = IK1(HC_L(i):HC_L(i+1),LeftHipAddIKidx);
            Hip_Rot_IK_L{i} = IK1(HC_L(i):HC_L(i+1),LeftHipRotIKidx);
            Knee_IK_L{i} = IK1(HC_L(i):HC_L(i+1),LeftKneeIKidx);
            Ankle_IK_L{i} = IK1(HC_L(i):HC_L(i+1),LeftAnkleIKidx);

            Hip_Flex_ID_L{i} = ID1(HC_L(i):HC_L(i+1),LeftHipFlexIDidx);
            Hip_Add_ID_L{i} = ID1(HC_L(i):HC_L(i+1),LeftHipAddIDidx);
            Hip_Rot_ID_L{i} = ID1(HC_L(i):HC_L(i+1),LeftHipRotIDidx);
            Knee_ID_L{i} = ID1(HC_L(i):HC_L(i+1),LeftKneeIDidx);
            Ankle_ID_L{i} = ID1(HC_L(i):HC_L(i+1),LeftAnkleIDidx);

            GRF_L{i} = GRF1.FPLeft_vy(HC_L(i):HC_L(i+1));
            Ankle_JP_L{i} = JP1(HC_L(i):HC_L(i+1),LeftAnkleJPidx);

            Pos_PelvisX_BK{i} = BKPos1(HC_L(i):HC_L(i+1),PelvisXBKidx);
            Pos_PelvisY_BK{i} = BKPos1(HC_L(i):HC_L(i+1),PelvisYBKidx);
            Pos_PelvisZ_BK{i} = BKPos1(HC_L(i):HC_L(i+1),PelvisZBKidx);
            Pos_PelvisRollX_BK{i} = BKPos1(HC_L(i):HC_L(i+1),PelvisRollXBKidx);
            Pos_PelvisPitchY_BK{i} = BKPos1(HC_L(i):HC_L(i+1),PelvisPitchYBKidx);
            Pos_PelvisYawZ_BK{i} = BKPos1(HC_L(i):HC_L(i+1),PelvisYawZBKidx);

            Pos_ThighX_BK_L{i} = BKPos1(HC_L(i):HC_L(i+1),LeftThighXBKidx);
            Pos_ThighY_BK_L{i} = BKPos1(HC_L(i):HC_L(i+1),LeftThighYBKidx);
            Pos_ThighZ_BK_L{i} = BKPos1(HC_L(i):HC_L(i+1),LeftThighZBKidx);
            Pos_ThighRollX_BK_L{i} = BKPos1(HC_L(i):HC_L(i+1),LeftThighRollXBKidx);
            Pos_ThighPitchY_BK_L{i} = BKPos1(HC_L(i):HC_L(i+1),LeftThighPitchYBKidx);
            Pos_ThighYawZ_BK_L{i} = BKPos1(HC_L(i):HC_L(i+1),LeftThighYawZBKidx);

            Pos_ShankX_BK_L{i} = BKPos1(HC_L(i):HC_L(i+1),LeftShankXBKidx);
            Pos_ShankY_BK_L{i} = BKPos1(HC_L(i):HC_L(i+1),LeftShankYBKidx);
            Pos_ShankZ_BK_L{i} = BKPos1(HC_L(i):HC_L(i+1),LeftShankZBKidx);
            Pos_ShankRollX_BK_L{i} = BKPos1(HC_L(i):HC_L(i+1),LeftShankRollXBKidx);
            Pos_ShankPitchY_BK_L{i} = BKPos1(HC_L(i):HC_L(i+1),LeftShankPitchYBKidx);
            Pos_ShankYawZ_BK_L{i} = BKPos1(HC_L(i):HC_L(i+1),LeftShankYawZBKidx);

            Pos_FootX_BK_L{i} = BKPos1(HC_L(i):HC_L(i+1),LeftFootXBKidx);
            Pos_FootY_BK_L{i} = BKPos1(HC_L(i):HC_L(i+1),LeftFootYBKidx);
            Pos_FootZ_BK_L{i} = BKPos1(HC_L(i):HC_L(i+1),LeftFootZBKidx);
            Pos_FootRollX_BK_L{i} = BKPos1(HC_L(i):HC_L(i+1),LeftFootRollXBKidx);
            Pos_FootPitchY_BK_L{i} = BKPos1(HC_L(i):HC_L(i+1),LeftFootPitchYBKidx);
            Pos_FootYawZ_BK_L{i} = BKPos1(HC_L(i):HC_L(i+1),LeftFootYawZBKidx);

            Vel_PelvisX_BK{i} = BKVel1(HC_L(i):HC_L(i+1),PelvisXBKidx);
            Vel_PelvisY_BK{i} = BKVel1(HC_L(i):HC_L(i+1),PelvisYBKidx);
            Vel_PelvisZ_BK{i} = BKVel1(HC_L(i):HC_L(i+1),PelvisZBKidx);
            Vel_PelvisRollX_BK{i} = BKVel1(HC_L(i):HC_L(i+1),PelvisRollXBKidx);
            Vel_PelvisPitchY_BK{i} = BKVel1(HC_L(i):HC_L(i+1),PelvisPitchYBKidx);
            Vel_PelvisYawZ_BK{i} = BKVel1(HC_L(i):HC_L(i+1),PelvisYawZBKidx);

            Vel_ThighX_BK_L{i} = BKVel1(HC_L(i):HC_L(i+1),LeftThighXBKidx);
            Vel_ThighY_BK_L{i} = BKVel1(HC_L(i):HC_L(i+1),LeftThighYBKidx);
            Vel_ThighZ_BK_L{i} = BKVel1(HC_L(i):HC_L(i+1),LeftThighZBKidx);
            Vel_ThighRollX_BK_L{i} = BKVel1(HC_L(i):HC_L(i+1),LeftThighRollXBKidx);
            Vel_ThighPitchY_BK_L{i} = BKVel1(HC_L(i):HC_L(i+1),LeftThighPitchYBKidx);
            Vel_ThighYawZ_BK_L{i} = BKVel1(HC_L(i):HC_L(i+1),LeftThighYawZBKidx);

            Vel_ShankX_BK_L{i} = BKVel1(HC_L(i):HC_L(i+1),LeftShankXBKidx);
            Vel_ShankY_BK_L{i} = BKVel1(HC_L(i):HC_L(i+1),LeftShankYBKidx);
            Vel_ShankZ_BK_L{i} = BKVel1(HC_L(i):HC_L(i+1),LeftShankZBKidx);
            Vel_ShankRollX_BK_L{i} = BKVel1(HC_L(i):HC_L(i+1),LeftShankRollXBKidx);
            Vel_ShankPitchY_BK_L{i} = BKVel1(HC_L(i):HC_L(i+1),LeftShankPitchYBKidx);
            Vel_ShankYawZ_BK_L{i} = BKVel1(HC_L(i):HC_L(i+1),LeftShankYawZBKidx);

            Vel_FootX_BK_L{i} = BKVel1(HC_L(i):HC_L(i+1),LeftFootXBKidx);
            Vel_FootY_BK_L{i} = BKVel1(HC_L(i):HC_L(i+1),LeftFootYBKidx);
            Vel_FootZ_BK_L{i} = BKVel1(HC_L(i):HC_L(i+1),LeftFootZBKidx);
            Vel_FootRollX_BK_L{i} = BKVel1(HC_L(i):HC_L(i+1),LeftFootRollXBKidx);
            Vel_FootPitchY_BK_L{i} = BKVel1(HC_L(i):HC_L(i+1),LeftFootPitchYBKidx);
            Vel_FootYawZ_BK_L{i} = BKVel1(HC_L(i):HC_L(i+1),LeftFootYawZBKidx);

            Acc_PelvisX_BK{i} = BKAcc1(HC_L(i):HC_L(i+1),PelvisXBKidx);
            Acc_PelvisY_BK{i} = BKAcc1(HC_L(i):HC_L(i+1),PelvisYBKidx);
            Acc_PelvisZ_BK{i} = BKAcc1(HC_L(i):HC_L(i+1),PelvisZBKidx);
            Acc_PelvisRollX_BK{i} = BKAcc1(HC_L(i):HC_L(i+1),PelvisRollXBKidx);
            Acc_PelvisPitchY_BK{i} = BKAcc1(HC_L(i):HC_L(i+1),PelvisPitchYBKidx);
            Acc_PelvisYawZ_BK{i} = BKAcc1(HC_L(i):HC_L(i+1),PelvisYawZBKidx);

            Acc_ThighX_BK_L{i} = BKAcc1(HC_L(i):HC_L(i+1),LeftThighXBKidx);
            Acc_ThighY_BK_L{i} = BKAcc1(HC_L(i):HC_L(i+1),LeftThighYBKidx);
            Acc_ThighZ_BK_L{i} = BKAcc1(HC_L(i):HC_L(i+1),LeftThighZBKidx);
            Acc_ThighRollX_BK_L{i} = BKAcc1(HC_L(i):HC_L(i+1),LeftThighRollXBKidx);
            Acc_ThighPitchY_BK_L{i} = BKAcc1(HC_L(i):HC_L(i+1),LeftThighPitchYBKidx);
            Acc_ThighYawZ_BK_L{i} = BKAcc1(HC_L(i):HC_L(i+1),LeftThighYawZBKidx);

            Acc_ShankX_BK_L{i} = BKAcc1(HC_L(i):HC_L(i+1),LeftShankXBKidx);
            Acc_ShankY_BK_L{i} = BKAcc1(HC_L(i):HC_L(i+1),LeftShankYBKidx);
            Acc_ShankZ_BK_L{i} = BKAcc1(HC_L(i):HC_L(i+1),LeftShankZBKidx);
            Acc_ShankRollX_BK_L{i} = BKAcc1(HC_L(i):HC_L(i+1),LeftShankRollXBKidx);
            Acc_ShankPitchY_BK_L{i} = BKAcc1(HC_L(i):HC_L(i+1),LeftShankPitchYBKidx);
            Acc_ShankYawZ_BK_L{i} = BKAcc1(HC_L(i):HC_L(i+1),LeftShankYawZBKidx);

            Acc_FootX_BK_L{i} = BKAcc1(HC_L(i):HC_L(i+1),LeftFootXBKidx);
            Acc_FootY_BK_L{i} = BKAcc1(HC_L(i):HC_L(i+1),LeftFootYBKidx);
            Acc_FootZ_BK_L{i} = BKAcc1(HC_L(i):HC_L(i+1),LeftFootZBKidx);
            Acc_FootRollX_BK_L{i} = BKAcc1(HC_L(i):HC_L(i+1),LeftFootRollXBKidx);
            Acc_FootPitchY_BK_L{i} = BKAcc1(HC_L(i):HC_L(i+1),LeftFootPitchYBKidx);
            Acc_FootYawZ_BK_L{i} = BKAcc1(HC_L(i):HC_L(i+1),LeftFootYawZBKidx);
        end

        for i = 1:length(HC_R)-1
            time_R{i} = IK1(HC_R(i):HC_R(i+1),OSTimeidx);
            Hip_Flex_IK_R{i} = IK1(HC_R(i):HC_R(i+1),RightHipFlexIKidx);
            Hip_Add_IK_R{i} = IK1(HC_R(i):HC_R(i+1),RightHipAddIKidx);
            Hip_Rot_IK_R{i} = IK1(HC_R(i):HC_R(i+1),RightHipRotIKidx);
            Knee_IK_R{i} = IK1(HC_R(i):HC_R(i+1),RightKneeIKidx);
            Ankle_IK_R{i} = IK1(HC_R(i):HC_R(i+1),RightAnkleIKidx);

            Hip_Flex_ID_R{i} = ID1(HC_R(i):HC_R(i+1),RightHipFlexIDidx);
            Hip_Add_ID_R{i} = ID1(HC_R(i):HC_R(i+1),RightHipAddIDidx);
            Hip_Rot_ID_R{i} = ID1(HC_R(i):HC_R(i+1),RightHipRotIDidx);
            Knee_ID_R{i} = ID1(HC_R(i):HC_R(i+1),RightKneeIDidx);
            Ankle_ID_R{i} = ID1(HC_R(i):HC_R(i+1),RightAnkleIDidx);

            GRF_R{i} = GRF1.FPRight_vy(HC_R(i):HC_R(i+1));
            Ankle_JP_R{i} = JP1(HC_R(i):HC_R(i+1),RightAnkleJPidx);

            Pos_ThighX_BK_R{i} = BKPos1(HC_R(i):HC_R(i+1),RightThighXBKidx);
            Pos_ThighY_BK_R{i} = BKPos1(HC_R(i):HC_R(i+1),RightThighYBKidx);
            Pos_ThighZ_BK_R{i} = BKPos1(HC_R(i):HC_R(i+1),RightThighZBKidx);
            Pos_ThighRollX_BK_R{i} = BKPos1(HC_R(i):HC_R(i+1),RightThighRollXBKidx);
            Pos_ThighPitchY_BK_R{i} = BKPos1(HC_R(i):HC_R(i+1),RightThighPitchYBKidx);
            Pos_ThighYawZ_BK_R{i} = BKPos1(HC_R(i):HC_R(i+1),RightThighYawZBKidx);

            Pos_ShankX_BK_R{i} = BKPos1(HC_R(i):HC_R(i+1),RightShankXBKidx);
            Pos_ShankY_BK_R{i} = BKPos1(HC_R(i):HC_R(i+1),RightShankYBKidx);
            Pos_ShankZ_BK_R{i} = BKPos1(HC_R(i):HC_R(i+1),RightShankZBKidx);
            Pos_ShankRollX_BK_R{i} = BKPos1(HC_R(i):HC_R(i+1),RightShankRollXBKidx);
            Pos_ShankPitchY_BK_R{i} = BKPos1(HC_R(i):HC_R(i+1),RightShankPitchYBKidx);
            Pos_ShankYawZ_BK_R{i} = BKPos1(HC_R(i):HC_R(i+1),RightShankYawZBKidx);

            Pos_FootX_BK_R{i} = BKPos1(HC_R(i):HC_R(i+1),RightFootXBKidx);
            Pos_FootY_BK_R{i} = BKPos1(HC_R(i):HC_R(i+1),RightFootYBKidx);
            Pos_FootZ_BK_R{i} = BKPos1(HC_R(i):HC_R(i+1),RightFootZBKidx);
            Pos_FootRollX_BK_R{i} = BKPos1(HC_R(i):HC_R(i+1),RightFootRollXBKidx);
            Pos_FootPitchY_BK_R{i} = BKPos1(HC_R(i):HC_R(i+1),RightFootPitchYBKidx);
            Pos_FootYawZ_BK_R{i} = BKPos1(HC_R(i):HC_R(i+1),RightFootYawZBKidx);

            Vel_ThighX_BK_R{i} = BKVel1(HC_R(i):HC_R(i+1),RightThighXBKidx);
            Vel_ThighY_BK_R{i} = BKVel1(HC_R(i):HC_R(i+1),RightThighYBKidx);
            Vel_ThighZ_BK_R{i} = BKVel1(HC_R(i):HC_R(i+1),RightThighZBKidx);
            Vel_ThighRollX_BK_R{i} = BKVel1(HC_R(i):HC_R(i+1),RightThighRollXBKidx);
            Vel_ThighPitchY_BK_R{i} = BKVel1(HC_R(i):HC_R(i+1),RightThighPitchYBKidx);
            Vel_ThighYawZ_BK_R{i} = BKVel1(HC_R(i):HC_R(i+1),RightThighYawZBKidx);

            Vel_ShankX_BK_R{i} = BKVel1(HC_R(i):HC_R(i+1),RightShankXBKidx);
            Vel_ShankY_BK_R{i} = BKVel1(HC_R(i):HC_R(i+1),RightShankYBKidx);
            Vel_ShankZ_BK_R{i} = BKVel1(HC_R(i):HC_R(i+1),RightShankZBKidx);
            Vel_ShankRollX_BK_R{i} = BKVel1(HC_R(i):HC_R(i+1),RightShankRollXBKidx);
            Vel_ShankPitchY_BK_R{i} = BKVel1(HC_R(i):HC_R(i+1),RightShankPitchYBKidx);
            Vel_ShankYawZ_BK_R{i} = BKVel1(HC_R(i):HC_R(i+1),RightShankYawZBKidx);

            Vel_FootX_BK_R{i} = BKVel1(HC_R(i):HC_R(i+1),RightFootXBKidx);
            Vel_FootY_BK_R{i} = BKVel1(HC_R(i):HC_R(i+1),RightFootYBKidx);
            Vel_FootZ_BK_R{i} = BKVel1(HC_R(i):HC_R(i+1),RightFootZBKidx);
            Vel_FootRollX_BK_R{i} = BKVel1(HC_R(i):HC_R(i+1),RightFootRollXBKidx);
            Vel_FootPitchY_BK_R{i} = BKVel1(HC_R(i):HC_R(i+1),RightFootPitchYBKidx);
            Vel_FootYawZ_BK_R{i} = BKVel1(HC_R(i):HC_R(i+1),RightFootYawZBKidx);

            Acc_ThighX_BK_R{i} = BKAcc1(HC_R(i):HC_R(i+1),RightThighXBKidx);
            Acc_ThighY_BK_R{i} = BKAcc1(HC_R(i):HC_R(i+1),RightThighYBKidx);
            Acc_ThighZ_BK_R{i} = BKAcc1(HC_R(i):HC_R(i+1),RightThighZBKidx);
            Acc_ThighRollX_BK_R{i} = BKAcc1(HC_R(i):HC_R(i+1),RightThighRollXBKidx);
            Acc_ThighPitchY_BK_R{i} = BKAcc1(HC_R(i):HC_R(i+1),RightThighPitchYBKidx);
            Acc_ThighYawZ_BK_R{i} = BKAcc1(HC_R(i):HC_R(i+1),RightThighYawZBKidx);

            Acc_ShankX_BK_R{i} = BKAcc1(HC_R(i):HC_R(i+1),RightShankXBKidx);
            Acc_ShankY_BK_R{i} = BKAcc1(HC_R(i):HC_R(i+1),RightShankYBKidx);
            Acc_ShankZ_BK_R{i} = BKAcc1(HC_R(i):HC_R(i+1),RightShankZBKidx);
            Acc_ShankRollX_BK_R{i} = BKAcc1(HC_R(i):HC_R(i+1),RightShankRollXBKidx);
            Acc_ShankPitchY_BK_R{i} = BKAcc1(HC_R(i):HC_R(i+1),RightShankPitchYBKidx);
            Acc_ShankYawZ_BK_R{i} = BKAcc1(HC_R(i):HC_R(i+1),RightShankYawZBKidx);

            Acc_FootX_BK_R{i} = BKAcc1(HC_R(i):HC_R(i+1),RightFootXBKidx);
            Acc_FootY_BK_R{i} = BKAcc1(HC_R(i):HC_R(i+1),RightFootYBKidx);
            Acc_FootZ_BK_R{i} = BKAcc1(HC_R(i):HC_R(i+1),RightFootZBKidx);
            Acc_FootRollX_BK_R{i} = BKAcc1(HC_R(i):HC_R(i+1),RightFootRollXBKidx);
            Acc_FootPitchY_BK_R{i} = BKAcc1(HC_R(i):HC_R(i+1),RightFootPitchYBKidx);
            Acc_FootYawZ_BK_R{i} = BKAcc1(HC_R(i):HC_R(i+1),RightFootYawZBKidx);

            HT_stiffness{i} = -HT1(HC_R(i):HC_R(i+1),1);
            HT_neutral_angle{i} = -HT1(HC_R(i):HC_R(i+1),2);
            HT_ankle_angle{i} = -HT1(HC_R(i):HC_R(i+1),3);
            HT_force_plate{i} = HT1(HC_R(i):HC_R(i+1),4);
            HT_encoder_velocity{i} = -HT1(HC_R(i):HC_R(i+1),5);
            HT_exo_torque{i} = HT1(HC_R(i):HC_R(i+1),6);
        end
    end

    %% Save Data
    if affected_side == 'L'
        left = struct('Time', time_L, ...
            'Hip_Flex_IK', Hip_Flex_IK_L, 'Hip_Add_IK', Hip_Add_IK_L, ...
            'Hip_Rot_IK', Hip_Rot_IK_L,'Knee_IK', Knee_IK_L, 'Ankle_IK', Ankle_IK_L, ...
            'Hip_Flex_ID', Hip_Flex_ID_L, 'Hip_Add_ID', Hip_Add_ID_L, ...
            'Hip_Rot_ID', Hip_Rot_ID_L,'Knee_ID', Knee_ID_L, 'Ankle_ID', Ankle_ID_L, ...
            'GRF', GRF_L, 'Ankle_JP', Ankle_JP_L,'Pelvis_X_Pos',Pos_PelvisX_BK, ...
            'Pelvis_Y_Pos',Pos_PelvisY_BK,'Pelvis_Z_Pos',Pos_PelvisZ_BK,'Pelvis_RollX_Pos',Pos_PelvisRollX_BK, ...
            'Pelvis_PitchY_Pos',Pos_PelvisPitchY_BK,'Pelvis_YawZ_Pos',Pos_PelvisYawZ_BK, ...
            'Thigh_X_Pos',Pos_ThighX_BK_L,'Thigh_Y_Pos',Pos_ThighY_BK_L,'Thigh_Z_Pos',Pos_ThighZ_BK_L, ...
            'Thigh_RollX_Pos',Pos_ThighRollX_BK_L,'Thigh_PitchY_Pos',Pos_ThighPitchY_BK_L, ...
            'Thigh_YawZ_Pos',Pos_ThighYawZ_BK_L,'Shank_X_Pos',Pos_ShankX_BK_L, ...
            'Shank_Y_Pos',Pos_ShankY_BK_L,'Shank_Z_Pos',Pos_ShankZ_BK_L,'Shank_RollX_Pos',Pos_ShankRollX_BK_L, ...
            'Shank_PitchY_Pos',Pos_ShankPitchY_BK_L,'Shank_YawZ_Pos',Pos_ShankYawZ_BK_L, ...
            'Foot_X_Pos',Pos_FootX_BK_L,'Foot_Y_Pos',Pos_FootY_BK_L,'Foot_Z_Pos',Pos_FootZ_BK_L, ...
            'Foot_RollX_Pos',Pos_FootRollX_BK_L,'Foot_PitchY_Pos',Pos_FootPitchY_BK_L,'Foot_YawZ_Pos',Pos_FootYawZ_BK_L, ...
            'Pelvis_X_Vel',Vel_PelvisX_BK, ...
            'Pelvis_Y_Vel',Vel_PelvisY_BK,'Pelvis_Z_Vel',Vel_PelvisZ_BK,'Pelvis_RollX_Vel',Vel_PelvisRollX_BK, ...
            'Pelvis_PitchY_Vel',Vel_PelvisPitchY_BK,'Pelvis_YawZ_Vel',Vel_PelvisYawZ_BK, ...
            'Thigh_X_Vel',Vel_ThighX_BK_L,'Thigh_Y_Vel',Vel_ThighY_BK_L,'Thigh_Z_Vel',Vel_ThighZ_BK_L, ...
            'Thigh_RollX_Vel',Vel_ThighRollX_BK_L,'Thigh_PitchY_Vel',Vel_ThighPitchY_BK_L, ...
            'Thigh_YawZ_Vel',Vel_ThighYawZ_BK_L,'Shank_X_Vel',Vel_ShankX_BK_L, ...
            'Shank_Y_Vel',Vel_ShankY_BK_L,'Shank_Z_Vel',Vel_ShankZ_BK_L,'Shank_RollX_Vel',Vel_ShankRollX_BK_L, ...
            'Shank_PitchY_Vel',Vel_ShankPitchY_BK_L,'Shank_YawZ_Vel',Vel_ShankYawZ_BK_L, ...
            'Foot_X_Vel',Vel_FootX_BK_L,'Foot_Y_Vel',Vel_FootY_BK_L,'Foot_Z_Vel',Vel_FootZ_BK_L, ...
            'Foot_RollX_Vel',Vel_FootRollX_BK_L,'Foot_PitchY_Vel',Vel_FootPitchY_BK_L,'Foot_YawZ_Vel',Vel_FootYawZ_BK_L, ...
            'Pelvis_X_Acc',Acc_PelvisX_BK, ...
            'Pelvis_Y_Acc',Acc_PelvisY_BK,'Pelvis_Z_Acc',Acc_PelvisZ_BK,'Pelvis_RollX_Acc',Acc_PelvisRollX_BK, ...
            'Pelvis_PitchY_Acc',Acc_PelvisPitchY_BK,'Pelvis_YawZ_Acc',Acc_PelvisYawZ_BK, ...
            'Thigh_X_Acc',Acc_ThighX_BK_L,'Thigh_Y_Acc',Acc_ThighY_BK_L,'Thigh_Z_Acc',Acc_ThighZ_BK_L, ...
            'Thigh_RollX_Acc',Acc_ThighRollX_BK_L,'Thigh_PitchY_Acc',Acc_ThighPitchY_BK_L, ...
            'Thigh_YawZ_Acc',Acc_ThighYawZ_BK_L,'Shank_X_Acc',Acc_ShankX_BK_L, ...
            'Shank_Y_Acc',Acc_ShankY_BK_L,'Shank_Z_Acc',Acc_ShankZ_BK_L,'Shank_RollX_Acc',Acc_ShankRollX_BK_L, ...
            'Shank_PitchY_Acc',Acc_ShankPitchY_BK_L,'Shank_YawZ_Acc',Acc_ShankYawZ_BK_L, ...
            'Foot_X_Acc',Acc_FootX_BK_L,'Foot_Y_Acc',Acc_FootY_BK_L,'Foot_Z_Acc',Acc_FootZ_BK_L, ...
            'Foot_RollX_Acc',Acc_FootRollX_BK_L,'Foot_PitchY_Acc',Acc_FootPitchY_BK_L,'Foot_YawZ_Acc',Acc_FootYawZ_BK_L, ...
            'HT_stiffness',HT_stiffness,'HT_neutral_angle',HT_neutral_angle, ...
            'HT_ankle_angle',HT_ankle_angle,'HT_force_plate',HT_force_plate,...
            'HT_encoder_velocity',HT_encoder_velocity,'HT_exo_torque',HT_exo_torque);
        right = struct('Time', time_R, ...
            'Hip_Flex_IK', Hip_Flex_IK_R, 'Hip_Add_IK', Hip_Add_IK_R, ...
            'Hip_Rot_IK', Hip_Rot_IK_R,'Knee_IK', Knee_IK_R, 'Ankle_IK', Ankle_IK_R, ...
            'Hip_Flex_ID', Hip_Flex_ID_R, 'Hip_Add_ID', Hip_Add_ID_R, ...
            'Hip_Rot_ID', Hip_Rot_ID_R,'Knee_ID', Knee_ID_R, 'Ankle_ID', Ankle_ID_R, ...
            'GRF', GRF_R, 'Ankle_JP', Ankle_JP_R,'Thigh_X_Pos',Pos_ThighX_BK_R,...
            'Thigh_Y_Pos',Pos_ThighY_BK_R,'Thigh_Z_Pos',Pos_ThighZ_BK_R, ...
            'Thigh_RollX_Pos',Pos_ThighRollX_BK_R,'Thigh_PitchY_Pos',Pos_ThighPitchY_BK_R, ...
            'Thigh_YawZ_Pos',Pos_ThighYawZ_BK_R,'Shank_X_Pos',Pos_ShankX_BK_R, ...
            'Shank_Y_Pos',Pos_ShankY_BK_R,'Shank_Z_Pos',Pos_ShankZ_BK_R,'Shank_RollX_Pos',Pos_ShankRollX_BK_R, ...
            'Shank_PitchY_Pos',Pos_ShankPitchY_BK_R,'Shank_YawZ_Pos',Pos_ShankYawZ_BK_R, ...
            'Foot_X_Pos',Pos_FootX_BK_R,'Foot_Y_Pos',Pos_FootY_BK_R,'Foot_Z_Pos',Pos_FootZ_BK_R, ...
            'Foot_RollX_Pos',Pos_FootRollX_BK_R,'Foot_PitchY_Pos',Pos_FootPitchY_BK_R,'Foot_YawZ_Pos',Pos_FootYawZ_BK_R,...
            'Thigh_X_Vel',Vel_ThighX_BK_R,'Thigh_Y_Vel',Vel_ThighY_BK_R,'Thigh_Z_Vel',Vel_ThighZ_BK_R, ...
            'Thigh_RollX_Vel',Vel_ThighRollX_BK_R,'Thigh_PitchY_Vel',Vel_ThighPitchY_BK_R, ...
            'Thigh_YawZ_Vel',Vel_ThighYawZ_BK_R,'Shank_X_Vel',Vel_ShankX_BK_R, ...
            'Shank_Y_Vel',Vel_ShankY_BK_R,'Shank_Z_Vel',Vel_ShankZ_BK_R,'Shank_RollX_Vel',Vel_ShankRollX_BK_R, ...
            'Shank_PitchY_Vel',Vel_ShankPitchY_BK_R,'Shank_YawZ_Vel',Vel_ShankYawZ_BK_R, ...
            'Foot_X_Vel',Vel_FootX_BK_R,'Foot_Y_Vel',Vel_FootY_BK_R,'Foot_Z_Vel',Vel_FootZ_BK_R, ...
            'Foot_RollX_Vel',Vel_FootRollX_BK_R,'Foot_PitchY_Vel',Vel_FootPitchY_BK_R,'Foot_YawZ_Vel',Vel_FootYawZ_BK_R, ...
            'Thigh_X_Acc',Acc_ThighX_BK_R,'Thigh_Y_Acc',Acc_ThighY_BK_R,'Thigh_Z_Acc',Acc_ThighZ_BK_R, ...
            'Thigh_RollX_Acc',Acc_ThighRollX_BK_R,'Thigh_PitchY_Acc',Acc_ThighPitchY_BK_R, ...
            'Thigh_YawZ_Acc',Acc_ThighYawZ_BK_R,'Shank_X_Acc',Acc_ShankX_BK_R, ...
            'Shank_Y_Acc',Acc_ShankY_BK_R,'Shank_Z_Acc',Acc_ShankZ_BK_R,'Shank_RollX_Acc',Acc_ShankRollX_BK_R, ...
            'Shank_PitchY_Acc',Acc_ShankPitchY_BK_R,'Shank_YawZ_Acc',Acc_ShankYawZ_BK_R, ...
            'Foot_X_Acc',Acc_FootX_BK_R,'Foot_Y_Acc',Acc_FootY_BK_R,'Foot_Z_Acc',Acc_FootZ_BK_R, ...
            'Foot_RollX_Acc',Acc_FootRollX_BK_R,'Foot_PitchY_Acc',Acc_FootPitchY_BK_R,'Foot_YawZ_Acc',Acc_FootYawZ_BK_R);
    else
        left = struct('Time', time_L, ...
            'Hip_Flex_IK', Hip_Flex_IK_L, 'Hip_Add_IK', Hip_Add_IK_L, ...
            'Hip_Rot_IK', Hip_Rot_IK_L,'Knee_IK', Knee_IK_L, 'Ankle_IK', Ankle_IK_L, ...
            'Hip_Flex_ID', Hip_Flex_ID_L, 'Hip_Add_ID', Hip_Add_ID_L, ...
            'Hip_Rot_ID', Hip_Rot_ID_L,'Knee_ID', Knee_ID_L, 'Ankle_ID', Ankle_ID_L, ...
            'GRF', GRF_L, 'Ankle_JP', Ankle_JP_L,'Pelvis_X_Pos',Pos_PelvisX_BK, ...
            'Pelvis_Y_Pos',Pos_PelvisY_BK,'Pelvis_Z_Pos',Pos_PelvisZ_BK,'Pelvis_RollX_Pos',Pos_PelvisRollX_BK, ...
            'Pelvis_PitchY_Pos',Pos_PelvisPitchY_BK,'Pelvis_YawZ_Pos',Pos_PelvisYawZ_BK, ...
            'Thigh_X_Pos',Pos_ThighX_BK_L,'Thigh_Y_Pos',Pos_ThighY_BK_L,'Thigh_Z_Pos',Pos_ThighZ_BK_L, ...
            'Thigh_RollX_Pos',Pos_ThighRollX_BK_L,'Thigh_PitchY_Pos',Pos_ThighPitchY_BK_L, ...
            'Thigh_YawZ_Pos',Pos_ThighYawZ_BK_L,'Shank_X_Pos',Pos_ShankX_BK_L, ...
            'Shank_Y_Pos',Pos_ShankY_BK_L,'Shank_Z_Pos',Pos_ShankZ_BK_L,'Shank_RollX_Pos',Pos_ShankRollX_BK_L, ...
            'Shank_PitchY_Pos',Pos_ShankPitchY_BK_L,'Shank_YawZ_Pos',Pos_ShankYawZ_BK_L, ...
            'Foot_X_Pos',Pos_FootX_BK_L,'Foot_Y_Pos',Pos_FootY_BK_L,'Foot_Z_Pos',Pos_FootZ_BK_L, ...
            'Foot_RollX_Pos',Pos_FootRollX_BK_L,'Foot_PitchY_Pos',Pos_FootPitchY_BK_L,'Foot_YawZ_Pos',Pos_FootYawZ_BK_L, ...
            'Pelvis_X_Vel',Vel_PelvisX_BK, ...
            'Pelvis_Y_Vel',Vel_PelvisY_BK,'Pelvis_Z_Vel',Vel_PelvisZ_BK,'Pelvis_RollX_Vel',Vel_PelvisRollX_BK, ...
            'Pelvis_PitchY_Vel',Vel_PelvisPitchY_BK,'Pelvis_YawZ_Vel',Vel_PelvisYawZ_BK, ...
            'Thigh_X_Vel',Vel_ThighX_BK_L,'Thigh_Y_Vel',Vel_ThighY_BK_L,'Thigh_Z_Vel',Vel_ThighZ_BK_L, ...
            'Thigh_RollX_Vel',Vel_ThighRollX_BK_L,'Thigh_PitchY_Vel',Vel_ThighPitchY_BK_L, ...
            'Thigh_YawZ_Vel',Vel_ThighYawZ_BK_L,'Shank_X_Vel',Vel_ShankX_BK_L, ...
            'Shank_Y_Vel',Vel_ShankY_BK_L,'Shank_Z_Vel',Vel_ShankZ_BK_L,'Shank_RollX_Vel',Vel_ShankRollX_BK_L, ...
            'Shank_PitchY_Vel',Vel_ShankPitchY_BK_L,'Shank_YawZ_Vel',Vel_ShankYawZ_BK_L, ...
            'Foot_X_Vel',Vel_FootX_BK_L,'Foot_Y_Vel',Vel_FootY_BK_L,'Foot_Z_Vel',Vel_FootZ_BK_L, ...
            'Foot_RollX_Vel',Vel_FootRollX_BK_L,'Foot_PitchY_Vel',Vel_FootPitchY_BK_L,'Foot_YawZ_Vel',Vel_FootYawZ_BK_L, ...
            'Pelvis_X_Acc',Acc_PelvisX_BK, ...
            'Pelvis_Y_Acc',Acc_PelvisY_BK,'Pelvis_Z_Acc',Acc_PelvisZ_BK,'Pelvis_RollX_Acc',Acc_PelvisRollX_BK, ...
            'Pelvis_PitchY_Acc',Acc_PelvisPitchY_BK,'Pelvis_YawZ_Acc',Acc_PelvisYawZ_BK, ...
            'Thigh_X_Acc',Acc_ThighX_BK_L,'Thigh_Y_Acc',Acc_ThighY_BK_L,'Thigh_Z_Acc',Acc_ThighZ_BK_L, ...
            'Thigh_RollX_Acc',Acc_ThighRollX_BK_L,'Thigh_PitchY_Acc',Acc_ThighPitchY_BK_L, ...
            'Thigh_YawZ_Acc',Acc_ThighYawZ_BK_L,'Shank_X_Acc',Acc_ShankX_BK_L, ...
            'Shank_Y_Acc',Acc_ShankY_BK_L,'Shank_Z_Acc',Acc_ShankZ_BK_L,'Shank_RollX_Acc',Acc_ShankRollX_BK_L, ...
            'Shank_PitchY_Acc',Acc_ShankPitchY_BK_L,'Shank_YawZ_Acc',Acc_ShankYawZ_BK_L, ...
            'Foot_X_Acc',Acc_FootX_BK_L,'Foot_Y_Acc',Acc_FootY_BK_L,'Foot_Z_Acc',Acc_FootZ_BK_L, ...
            'Foot_RollX_Acc',Acc_FootRollX_BK_L,'Foot_PitchY_Acc',Acc_FootPitchY_BK_L,'Foot_YawZ_Acc',Acc_FootYawZ_BK_L);
        right = struct('Time', time_R, ...
            'Hip_Flex_IK', Hip_Flex_IK_R, 'Hip_Add_IK', Hip_Add_IK_R, ...
            'Hip_Rot_IK', Hip_Rot_IK_R,'Knee_IK', Knee_IK_R, 'Ankle_IK', Ankle_IK_R, ...
            'Hip_Flex_ID', Hip_Flex_ID_R, 'Hip_Add_ID', Hip_Add_ID_R, ...
            'Hip_Rot_ID', Hip_Rot_ID_R,'Knee_ID', Knee_ID_R, 'Ankle_ID', Ankle_ID_R, ...
            'GRF', GRF_R, 'Ankle_JP', Ankle_JP_R,'Thigh_X_Pos',Pos_ThighX_BK_R,...
            'Thigh_Y_Pos',Pos_ThighY_BK_R,'Thigh_Z_Pos',Pos_ThighZ_BK_R, ...
            'Thigh_RollX_Pos',Pos_ThighRollX_BK_R,'Thigh_PitchY_Pos',Pos_ThighPitchY_BK_R, ...
            'Thigh_YawZ_Pos',Pos_ThighYawZ_BK_R,'Shank_X_Pos',Pos_ShankX_BK_R, ...
            'Shank_Y_Pos',Pos_ShankY_BK_R,'Shank_Z_Pos',Pos_ShankZ_BK_R,'Shank_RollX_Pos',Pos_ShankRollX_BK_R, ...
            'Shank_PitchY_Pos',Pos_ShankPitchY_BK_R,'Shank_YawZ_Pos',Pos_ShankYawZ_BK_R, ...
            'Foot_X_Pos',Pos_FootX_BK_R,'Foot_Y_Pos',Pos_FootY_BK_R,'Foot_Z_Pos',Pos_FootZ_BK_R, ...
            'Foot_RollX_Pos',Pos_FootRollX_BK_R,'Foot_PitchY_Pos',Pos_FootPitchY_BK_R,'Foot_YawZ_Pos',Pos_FootYawZ_BK_R,...
            'Thigh_X_Vel',Vel_ThighX_BK_R,'Thigh_Y_Vel',Vel_ThighY_BK_R,'Thigh_Z_Vel',Vel_ThighZ_BK_R, ...
            'Thigh_RollX_Vel',Vel_ThighRollX_BK_R,'Thigh_PitchY_Vel',Vel_ThighPitchY_BK_R, ...
            'Thigh_YawZ_Vel',Vel_ThighYawZ_BK_R,'Shank_X_Vel',Vel_ShankX_BK_R, ...
            'Shank_Y_Vel',Vel_ShankY_BK_R,'Shank_Z_Vel',Vel_ShankZ_BK_R,'Shank_RollX_Vel',Vel_ShankRollX_BK_R, ...
            'Shank_PitchY_Vel',Vel_ShankPitchY_BK_R,'Shank_YawZ_Vel',Vel_ShankYawZ_BK_R, ...
            'Foot_X_Vel',Vel_FootX_BK_R,'Foot_Y_Vel',Vel_FootY_BK_R,'Foot_Z_Vel',Vel_FootZ_BK_R, ...
            'Foot_RollX_Vel',Vel_FootRollX_BK_R,'Foot_PitchY_Vel',Vel_FootPitchY_BK_R,'Foot_YawZ_Vel',Vel_FootYawZ_BK_R, ...
            'Thigh_X_Acc',Acc_ThighX_BK_R,'Thigh_Y_Acc',Acc_ThighY_BK_R,'Thigh_Z_Acc',Acc_ThighZ_BK_R, ...
            'Thigh_RollX_Acc',Acc_ThighRollX_BK_R,'Thigh_PitchY_Acc',Acc_ThighPitchY_BK_R, ...
            'Thigh_YawZ_Acc',Acc_ThighYawZ_BK_R,'Shank_X_Acc',Acc_ShankX_BK_R, ...
            'Shank_Y_Acc',Acc_ShankY_BK_R,'Shank_Z_Acc',Acc_ShankZ_BK_R,'Shank_RollX_Acc',Acc_ShankRollX_BK_R, ...
            'Shank_PitchY_Acc',Acc_ShankPitchY_BK_R,'Shank_YawZ_Acc',Acc_ShankYawZ_BK_R, ...
            'Foot_X_Acc',Acc_FootX_BK_R,'Foot_Y_Acc',Acc_FootY_BK_R,'Foot_Z_Acc',Acc_FootZ_BK_R, ...
            'Foot_RollX_Acc',Acc_FootRollX_BK_R,'Foot_PitchY_Acc',Acc_FootPitchY_BK_R,'Foot_YawZ_Acc',Acc_FootYawZ_BK_R, ...
            'HT_stiffness',HT_stiffness,...
            'HT_neutral_angle',HT_neutral_angle,'HT_ankle_angle',HT_ankle_angle,...
            'HT_force_plate',HT_force_plate,'HT_encoder_velocity',HT_encoder_velocity, ...
            'HT_exo_torque',HT_exo_torque);
    end

    % Save each trial's segmented data in a struct
    cut_data(SaveIdx).left = left;
    cut_data(SaveIdx).right = right;
    SaveIdx = SaveIdx + 1;

end


end