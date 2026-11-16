% Script to extract and segment data for the AFO Emulator Project
function AFO_Segmentation(trialparams)

%% Intialization

% File path
filepath = [trialparams.StudyPath 'OutFiles\'];

% Subject ID
subject = trialparams.Subject;

% Session
session = trialparams.Session;

% AFO number
AFOnum = trialparams.AFOnum;

% Mode
mode = trialparams.mode;
% modes = {'Inc'};

% Time Range
tstart = 75;
tend = 105;

% Timestep
timestep = 5000;

%% Segmentation

IK = Osim.readMOT([filepath subject '\' session '\IK\' subject '_' AFOnum '_' mode '_IK.mot']);
ID = Osim.readSTO([filepath subject '\' session '\ID\' subject '_' AFOnum '_' mode '_ID.sto']);
GRF = Osim.readMOT([filepath subject '\' session '\ExtLoads\' subject '_' AFOnum '_' mode '_GRF.mot']);
load([filepath subject '\' session '\Power\' subject '_' AFOnum '_' mode '_FootPower.mat']);
JP = FootPowerTable;

IK1_headers = IK.Properties.VariableNames;
ID1_headers = ID.Properties.VariableNames;

range = [find(IK.Header == tstart), find(IK.Header == tend)];
GRF = GRF(find(GRF.Header == tstart):find(GRF.Header == tend),:);
IK = interp1(IK{range(1):range(2),1}, IK{range(1):range(2),:}, GRF{:,1}, 'linear');
ID = interp1(ID{range(1):range(2),1}, ID{range(1):range(2),:}, GRF{:,1}, 'linear');
JP = interp1(JP{range(1):range(2),1}, JP{range(1):range(2),:}, GRF{:,1}, 'linear');

if(contains(session,'CoreEmulator'))
    load([filepath subject '\' session '\Humotech\' subject '_' AFOnum '_' mode '.mat'])
    HT = data;

    col_trigbutton = find(strcmp(signalNames, 'Trigger_Button'));
    col_time = find(strcmp(signalNames, 'Time'));
    col_angle = find(strcmp(signalNames, 'Angle (deg)+tare'));
    col_torque = find(strcmp(signalNames, 'Net Torque (Nm)'));
    col_power = find(strcmp(signalNames, 'Ankle Power (W)'));

    trigger_threshold = 1;
    below = HT(:,col_trigbutton) < trigger_threshold;   % logical array
    HT_range = find(diff([0; below]) == 1);
    if length(HT_range) == 1
        HT_range = [HT_range length(HT)];
    elseif length(HT_range) == 0
        HT_range = [1 length(HT)];
    end
    HT = HT(HT_range(1):HT_range(2),:);
    HT(:,col_time) = HT(:,col_time) - HT(1,col_time);
    range = [find(round(HT(:,col_time),2) == tstart), find(round(HT(:,col_time),2) == tend)];
    HT = interp1(HT(range(1):range(2),col_time), HT(range(1):range(2),:), GRF{:,1}, 'linear');

    HT_angle = HT(:,col_angle);
    HT_torque = HT(:,col_torque);
    HT_power = HT(:,col_power);
end

HC_upper_threshold = 75;
HC_lower_threshold = 20;
minDist = 1000; 
HC_L = find(GRF.FPLeft_vy(2:end) > HC_lower_threshold & GRF.FPLeft_vy(2:end) < HC_upper_threshold & diff(GRF.FPLeft_vy) > 0 ) + 1;
HC_L = HC_L([true; diff(HC_L) > minDist]);
TO_L = find(GRF.FPLeft_vy(2:end) > HC_lower_threshold & GRF.FPLeft_vy(2:end) < HC_upper_threshold & diff(GRF.FPLeft_vy) < 0 ) + 1;
TO_L = TO_L([true; diff(TO_L) > minDist]);
HC_R = find(GRF.FPRight_vy(2:end) > HC_lower_threshold & GRF.FPRight_vy(2:end) < HC_upper_threshold & diff(GRF.FPRight_vy) > 0 ) + 1;
HC_R = HC_R([true; diff(HC_R) > minDist]);
TO_R = find(GRF.FPRight_vy(2:end) > HC_lower_threshold & GRF.FPRight_vy(2:end) < HC_upper_threshold & diff(GRF.FPRight_vy) < 0 ) + 1;
TO_R = TO_R([true; diff(TO_R) > minDist]);

% HC_L = find(diff(GRF1.FPLeft_vy) > HC_threshold);
% TO_L = find(diff(GRF1.FPLeft_vy) < -HC_threshold);
% HC_R = find(diff(GRF1.FPRight_vy) > HC_threshold);
% TO_R = find(diff(GRF1.FPRight_vy) < -HC_threshold);

if HC_L(1) > TO_L(1)
    HC_L = HC_L(1:end-1);
    TO_L = TO_L(2:end);
end

if HC_R(1) > TO_R(1)
    HC_R = HC_R(1:end-1);
    TO_R = TO_R(2:end);
end

for i = 1:length(HC_L)-1
    time_L{i} = IK(HC_L(i):HC_L(i+1),1);
    Hip_IK_L{i} = IK(HC_L(i):HC_L(i+1),16);
    Knee_IK_L{i} = IK(HC_L(i):HC_L(i+1),19);
    Ankle_IK_L{i} = IK(HC_L(i):HC_L(i+1),21);
    Hip_ID_L{i} = ID(HC_L(i):HC_L(i+1),11);
    Knee_ID_L{i} = ID(HC_L(i):HC_L(i+1),19);
    Ankle_ID_L{i} = ID(HC_L(i):HC_L(i+1),28);
    GRF_L{i} = GRF.FPLeft_vy(HC_L(i):HC_L(i+1));
    Ankle_JP_L{i} = JP(HC_L(i):HC_L(i+1),2);

    HT_angle_L{i} = -HT_angle(HC_L(i):HC_L(i+1));
    HT_torque_L{i} = -HT_torque(HC_L(i):HC_L(i+1));
    HT_power_L{i} = HT_power(HC_L(i):HC_L(i+1));

    bio_torque_L{i} = Ankle_ID_L{i}-HT_torque_L{i};
    bio_power_L{i} = Ankle_JP_L{i}-HT_power_L{i};
end

for i = 1:length(HC_R)-1
    time_R{i} = IK(HC_R(i):HC_R(i+1),1);
    Hip_IK_R{i} = IK(HC_R(i):HC_R(i+1),8);
    Knee_IK_R{i} = IK(HC_R(i):HC_R(i+1),11);
    Ankle_IK_R{i} = IK(HC_R(i):HC_R(i+1),13);
    Hip_ID_R{i} = ID(HC_R(i):HC_R(i+1),8);
    Knee_ID_R{i} = ID(HC_R(i):HC_R(i+1),17);
    Ankle_ID_R{i} = ID(HC_R(i):HC_R(i+1),27);
    GRF_R{i} = GRF.FPRight_vy(HC_R(i):HC_R(i+1));
    Ankle_JP_R{i} = JP(HC_R(i):HC_R(i+1),3);

    HT_angle_R{i} = -HT_angle(HC_R(i):HC_R(i+1));
    HT_torque_R{i} = -HT_torque(HC_R(i):HC_R(i+1));
    HT_power_R{i} = HT_power(HC_R(i):HC_R(i+1));

    bio_torque_R{i} = Ankle_ID_R{i}-HT_torque_R{i};
    bio_power_R{i} = Ankle_JP_R{i}-HT_power_R{i};
end

left = struct('Time', time_L, ...
              'Hip_IK', Hip_IK_L, 'Knee_IK', Knee_IK_L, 'Ankle_IK', Ankle_IK_L, ...
              'Hip_ID', Hip_ID_L, 'Knee_ID', Knee_ID_L, 'Ankle_ID', Ankle_ID_L, ...
              'GRF', GRF_L, 'Ankle_JP', Ankle_JP_L, ...
              'HT_Angle', HT_angle_L, 'HT_Torque', HT_torque_L, 'HT_Power', HT_power_L, ...
              'Bio_Torque', bio_torque_L, 'Bio_Power', bio_power_L);
right = struct('Time', time_R, ...
               'Hip_IK', Hip_IK_R, 'Knee_IK', Knee_IK_R, 'Ankle_IK', Ankle_IK_R, ...
               'Hip_ID', Hip_ID_R, 'Knee_ID', Knee_ID_R, 'Ankle_ID', Ankle_ID_R, ...
               'GRF', GRF_R, 'Ankle_JP', Ankle_JP_R, ...
               'HT_Angle', HT_angle_R, 'HT_Torque', HT_torque_R, 'HT_Power', HT_power_R, ...
               'Bio_Torque', bio_torque_R, 'Bio_Power', bio_power_R);


save([filepath subject '\' session '\Segmentation\' subject '_' AFOnum '_' mode '_segmented_results.mat'],'left', 'right')
