% Script to extract and segment data for the AFO Emulator Project

clear
clc
close all

%% Intialization

import org.opensim.modeling.*
% File path
% filepath = 'E:\AFO Emulator\OutFiles\';
filepath = 'C:\Users\nazon\OneDrive\Documents\Walter Reed\Preference Project\Exploratory Aim Data\Exploratory Aim';

% Subject ID
subject = 'AFOWR02';

% Session
session = 'AFOWR02';

% Mode
trials = 5;
% modes = {'Inc'};

% Timestep
timestep = 5000;

%% Segmentation
for k = 1:trials
    
    IK1 = Osim.readMOT([filepath '\' session '\IK\' subject '_EA_' num2str(k) '_IK.mot']);
    ID1 = Osim.readSTO([filepath '\' session '\ID\' subject '_EA_' num2str(k) '_ID.sto']);
    GRF1 = Osim.readMOT([filepath '\' session '\ExtLoads\' subject '_EA_' num2str(k) '_GRF.mot']);
    load([filepath '\' session '\Power\' subject '_EA_' num2str(k) '_FootPower.mat']);
    JP1 = FootPowerTable;
    HT = load([filepath '\' session '\Humotech\' subject '_EA_' num2str(k) '.mat']);
    
    IK1_headers = IK1.Properties.VariableNames;
    ID1_headers = ID1.Properties.VariableNames;
    HT_headers = HT.signalNames;
    
    tstart = IK1.Header(1);
    tstart_idx = find(round(IK1.Header,3) == round(tstart,3));
    GRF1 = GRF1(find(round(GRF1.Header,3) == round(tstart,3),1):end,:);
    GRF_time_vector = GRF1{:,1};
    HT_time_vector = HT.data(tstart_idx:end,91)-HT.data(1,91);
    HT1 = interp1(HT_time_vector, HT.data(tstart_idx:end,[14,61,71,88,89,90]), GRF_time_vector, 'linear');
    IK1 = interp1(IK1{tstart_idx:end,1}, IK1{tstart_idx:end,:}, GRF_time_vector, 'linear');
    ID1 = interp1(ID1{tstart_idx:end,1}, ID1{tstart_idx:end,:}, GRF_time_vector, 'linear');
    JP1 = interp1(JP1{tstart_idx:end,1}, JP1{tstart_idx:end,:}, GRF_time_vector, 'linear');

    % range = [find(GRF1.Header == tstart), find(GRF1.Header == tend)];
    % IK1 = IK1(find(IK1.Header == tstart):find(IK1.Header == tend),:);
    % GRF1 = interp1(GRF1{tstart_idx:end,1}, GRF1{tstart_idx:end,:}, IK1{:,1}, 'linear');
    % ID1 = ID1{tstart_idx:end,:};
    % JP1 = JP1{tstart_idx:end,:};
    
    HC_upper_threshold = 75;
    HC_lower_threshold = 20;
    minDist = 1000; 
    HC_L = find(GRF1.FPLeft_vy(2:end) > HC_lower_threshold & GRF1.FPLeft_vy(2:end) < HC_upper_threshold & diff(GRF1.FPLeft_vy) > 0 ) + 1;
    HC_L = HC_L([true; diff(HC_L) > minDist]);
    TO_L = find(GRF1.FPLeft_vy(2:end) > HC_lower_threshold & GRF1.FPLeft_vy(2:end) < HC_upper_threshold & diff(GRF1.FPLeft_vy) < 0 ) + 1;
    TO_L = TO_L([true; diff(TO_L) > minDist]);
    HC_R = find(GRF1.FPRight_vy(2:end) > HC_lower_threshold & GRF1.FPRight_vy(2:end) < HC_upper_threshold & diff(GRF1.FPRight_vy) > 0 ) + 1;
    HC_R = HC_R([true; diff(HC_R) > minDist]);
    TO_R = find(GRF1.FPRight_vy(2:end) > HC_lower_threshold & GRF1.FPRight_vy(2:end) < HC_upper_threshold & diff(GRF1.FPRight_vy) < 0 ) + 1;
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
        time_L{i} = IK1(HC_L(i):HC_L(i+1),1);
        Hip_IK_L{i} = IK1(HC_L(i):HC_L(i+1),16);
        Knee_IK_L{i} = IK1(HC_L(i):HC_L(i+1),19);
        Ankle_IK_L{i} = IK1(HC_L(i):HC_L(i+1),21);
        Hip_ID_L{i} = ID1(HC_L(i):HC_L(i+1),11);
        Knee_ID_L{i} = ID1(HC_L(i):HC_L(i+1),19);
        Ankle_ID_L{i} = ID1(HC_L(i):HC_L(i+1),28);
        GRF_L{i} = GRF1.FPLeft_vy(HC_L(i):HC_L(i+1));
        Ankle_JP_L{i} = JP1(HC_L(i):HC_L(i+1),2);
        HT_stiffness_L{i} = HT1(HC_L(i):HC_L(i+1),1);
        HT_neutral_angle_L{i} = HT1(HC_L(i):HC_L(i+1),2);
        HT_ankle_angle_L{i} = HT1(HC_L(i):HC_L(i+1),3);
        HT_force_plate_L{i} = HT1(HC_L(i):HC_L(i+1),4);
        HT_trigger_L{i} = HT1(HC_L(i):HC_L(i+1),6);
    end
    
    for i = 1:length(HC_R)-1
        time_R{i} = IK1(HC_R(i):HC_R(i+1),1);
        Hip_IK_R{i} = IK1(HC_R(i):HC_R(i+1),8);
        Knee_IK_R{i} = IK1(HC_R(i):HC_R(i+1),11);
        Ankle_IK_R{i} = IK1(HC_R(i):HC_R(i+1),13);
        Hip_ID_R{i} = ID1(HC_R(i):HC_R(i+1),8);
        Knee_ID_R{i} = ID1(HC_R(i):HC_R(i+1),17);
        Ankle_ID_R{i} = ID1(HC_R(i):HC_R(i+1),27);
        GRF_R{i} = GRF1.FPRight_vy(HC_R(i):HC_R(i+1));
        Ankle_JP_R{i} = JP1(HC_R(i):HC_R(i+1),3);
        % HT_stiffness_R{i} = HT1(HC_R(i):HC_R(i+1),1);
        % HT_neutral_angle_R{i} = HT1(HC_R(i):HC_R(i+1),2);
        % HT_ankle_angle_R{i} = HT1(HC_R(i):HC_R(i+1),3);
        HT_force_plate_R{i} = HT1(HC_R(i):HC_R(i+1),5);
        HT_trigger_R{i} = HT1(HC_R(i):HC_R(i+1),6);
    end
    
    % left = struct('Time', time_L, ...
    %               'Hip_IK', Hip_IK_L, 'Knee_IK', Knee_IK_L, 'Ankle_IK', Ankle_IK_L, ...
    %               'Hip_ID', Hip_ID_L, 'Knee_ID', Knee_ID_L, 'Ankle_ID', Ankle_ID_L, ...
    %               'GRF', GRF_L, 'Ankle_JP', Ankle_JP_L);
    % right = struct('Time', time_R, ...
    %                'Hip_IK', Hip_IK_R, 'Knee_IK', Knee_IK_R, 'Ankle_IK', Ankle_IK_R, ...
    %                'Hip_ID', Hip_ID_R, 'Knee_ID', Knee_ID_R, 'Ankle_ID', Ankle_ID_R, ...
    %                'GRF', GRF_R, 'Ankle_JP', Ankle_JP_R);

    left = struct('Time', time_L, ...
        'Hip_IK', Hip_IK_L, 'Knee_IK', Knee_IK_L, 'Ankle_IK', Ankle_IK_L, ...
        'Hip_ID', Hip_ID_L, 'Knee_ID', Knee_ID_L, 'Ankle_ID', Ankle_ID_L, ...
        'GRF', GRF_L, 'Ankle_JP', Ankle_JP_L,'HT_stiffness_L',HT_stiffness_L,...
        'HT_neutral_angle_L',HT_neutral_angle_L,'HT_ankle_angle_L',HT_ankle_angle_L,...
        'HT_force_plate_L',HT_force_plate_L);
    % right = struct('Time', time_R, ...
    %     'Hip_IK', Hip_IK_R, 'Knee_IK', Knee_IK_R, 'Ankle_IK', Ankle_IK_R, ...
    %     'Hip_ID', Hip_ID_R, 'Knee_ID', Knee_ID_R, 'Ankle_ID', Ankle_ID_R, ...
    %     'GRF', GRF_R, 'Ankle_JP', Ankle_JP_R,'HT_stiffness_R',HT_stiffness_R,...
    %     'HT_neutral_angle_R',HT_neutral_angle_R,'HT_ankle_angle_R',HT_ankle_angle_R,...
    %     'HT_force_plate_R',HT_force_plate_R);
    right = struct('Time', time_R, ...
        'Hip_IK', Hip_IK_R, 'Knee_IK', Knee_IK_R, 'Ankle_IK', Ankle_IK_R, ...
        'Hip_ID', Hip_ID_R, 'Knee_ID', Knee_ID_R, 'Ankle_ID', Ankle_ID_R, ...
        'GRF', GRF_R, 'Ankle_JP', Ankle_JP_R,'HT_force_plate_R',HT_force_plate_R);
    
    
    save([filepath '\' session '\YN_Segmentation\' subject '_EA_' num2str(k) '_segmented_results.mat'],'left', 'right')
end