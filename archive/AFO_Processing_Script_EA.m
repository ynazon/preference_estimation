%Script to process through inverse dynamics for the AFO Emulator project.
clear
clc

cd 'E:\AFO Emulator\Scripts'

addpath('E:\AFO Emulator\Scripts\ProcessingScripts');
addpath('E:\AFO Emulator\Scripts\HelperScripts')

%% Script to process AFO Emulator participants through OpenSim

import org.opensim.modeling.*
% GeometryPath='C:\OpenSim 4.4\Geometry';
%Path to helper scripts

%Subject ID
Subject='AFOWR03';

%Session
Session=[Subject '_EA'];

%AFO Side
Side='L';
Int_Side='R';

%Participant stature (in no exo condition) (m)
% Height=1.7;
Height=1.88;

%Participant weight (in no exo condition) from FP (kg)
% Mass=98.3;
Mass=72.2;

StudyPath='E:\AFO Emulator\';
% StudyPath='\Volumes\STORE N GO\AFO Emulator\';

GenIKSetupFile=[StudyPath 'GenFiles\AFO_Gen_IK_Setup.xml'];
GenIDSetupFile=[StudyPath 'GenFiles\AFO_Gen_ID_Setup.xml'];

%Convert static file to .trc
NoExo_CalTrial= [Subject '_Static_NoExo'];
Exo_CalTrial= [Subject '_Static_Exo'];

for i = 1:5
    trialnames{i} = [Subject '_EA_' num2str(i)];
end
%% TRC

%Convert calibration and static trials to TRC
%No Exo
StaticTRC=Vicon.C3DtoTRC([StudyPath 'Outfiles\' Subject '\' Session '\Data\' NoExo_CalTrial '.c3d']);
Osim.writeTRC(StaticTRC,'FilePath',[StudyPath 'Outfiles\' Subject '\' Session '\Data\' NoExo_CalTrial '_NoVirtual.trc']);
StaticTRC=generateVirtualMarkers_AFO(StaticTRC,'Harrington',StaticTRC);
Osim.writeTRC(StaticTRC,'FilePath',[StudyPath 'Outfiles\' Subject '\' Session '\Data\' NoExo_CalTrial '_Virtual.trc']);

%Exo
StaticTRC=Vicon.C3DtoTRC([StudyPath 'Outfiles\' Subject '\' Session '\Data\' Exo_CalTrial '.c3d']);
Osim.writeTRC(StaticTRC,'FilePath',[StudyPath 'Outfiles\' Subject '\' Session '\Data\' Exo_CalTrial '_NoVirtual.trc']);

%Convert locomotion trials to TRC
for tr=1:size(trialnames,2)
    trial=trialnames{tr};

    TRC=Vicon.C3DtoTRC([StudyPath 'Outfiles\' Subject '\' Session '\Data\' trial '.c3d']);
    Osim.writeTRC(TRC,'FilePath',[StudyPath 'Outfiles\' Subject '\' Session '\Data\' trial '.trc']);
end

clear trial TRC


Static_TRC=Osim.readTRC([StudyPath 'Outfiles\' Subject '\' Session '\Data\' NoExo_CalTrial '_Virtual.trc']);

%% Generate file folders
mkdir([StudyPath 'OutFiles'],Subject)
mkdir([StudyPath 'OutFiles\' Subject '\' Session],'Scaling')
mkdir([StudyPath 'OutFiles\' Subject '\' Session],'IK')
mkdir([StudyPath 'OutFiles\' Subject '\' Session],'BK')
mkdir([StudyPath 'OutFiles\' Subject '\' Session],'WBAM')
mkdir([StudyPath 'OutFiles\' Subject '\' Session],'ID')
mkdir([StudyPath 'OutFiles\' Subject '\' Session],'ExtLoads')
mkdir([StudyPath 'OutFiles\' Subject '\' Session],'Trialparams')
mkdir([StudyPath 'OutFiles\' Subject '\' Session],'Power')
mkdir([StudyPath 'OutFiles\' Subject '\' Session],'Segmentation')

%% If does not exist, generate models used for marker adjustment
mkdir([StudyPath 'OutFiles\' Subject '\' Session '\Scaling'],'AdjustMe')

if ~exist([StudyPath 'OutFiles\' Subject '\' Session '\Scaling\AdjustMe\' Subject '_NoExo_AdjustMe.osim'])
    copyfile([StudyPath 'Models\' 'AFOModel_NoAFO_' Side '.osim'],[StudyPath 'OutFiles\' Subject '\' Session '\Scaling\AdjustMe\' Subject '_NoExo_AdjustMe.osim']);
end

%% Adjust virtual markers after making adjustme changes

AFO_NoExo_AdjustVirtualMarkersInModel([StudyPath 'OutFiles\' Subject '\' Session '\Scaling\AdjustMe\' Subject '_NoExo_AdjustMe.osim']);


%% Get scale factors of body segments
MasterScale=AFO_Exo_MasterScaleGenerator(StudyPath,Subject,Session,NoExo_CalTrial);

%% Scaling No Exo Model

%Manual adjustments to scale factors
MasterScale.Torso(1)=1.1;
MasterScale.Torso(2)=1.1;
MasterScale.Pelvis(3)=1.1;
% 
MasterScale.RFoot(2)=1.2;
MasterScale.LFoot(2)=1.2;

MasterScale.LHumerus(2)=0.9;
MasterScale.RHumerus(2)=0.9;

MasterScale.LWRRad(2)=0.85;
MasterScale.RWWRad(2)=0.85;

% Scale no exo model
NoExo_ScaleSet=AFO_NoExo_ScaleSet(MasterScale);

AFO_NoExo_Model=Model([StudyPath 'OutFiles\' Subject '\' Session '\Scaling\AdjustMe\' Subject '_NoExo_AdjustMe.osim']);
s=AFO_NoExo_Model.initSystem;

AFO_NoExo_Model.scale(s,NoExo_ScaleSet,true,Mass);

AFO_NoExo_Model.print([StudyPath 'OutFiles\' Subject '\' Session '\Scaling\AdjustMe\' Subject '_NoExo_PreMarkers.osim']);

%% Run marker placer on No Exo model
funcName = ['MarkerPlacer_' Subject '_EA'];
MP_function = str2func(funcName);
MP_function(Subject,Session,NoExo_CalTrial,StudyPath);

%% Convert No Exo model to Exo model

if ~exist([StudyPath 'OutFiles\' Subject '\' Session '\Scaling\AdjustMe\' Subject '_Exo_AdjustMe.osim'])
    copyfile([StudyPath 'Models\' 'AFOModel_EXO_' Side '.osim'],[StudyPath 'OutFiles\' Subject '\' Session '\Scaling\AdjustMe\' Subject '_Exo_AdjustMe.osim']);
end

% Scale exo model
AFO_Exo_Model=Model([StudyPath 'OutFiles\' Subject '\' Session '\Scaling\AdjustMe\' Subject '_Exo_AdjustMe.osim']);
s=AFO_Exo_Model.initSystem;
AFO_Exo_Model.scale(s,NoExo_ScaleSet,true,Mass);

AFO_NoExo_Model=Model([StudyPath 'OutFiles\' Subject '\' Session '\Scaling\' Subject '_NoExo.osim']);

%Pull NoExo markerset from scaled no exo model and place on exo model
NoExo_MarkerSet=AFO_NoExo_Model.getMarkerSet();
NoExo_MarkerSet.cloneAndAppend(MarkerSet([StudyPath 'Models\' 'Exo_Markers_' Side '.xml']).get(0));
NoExo_MarkerSet.cloneAndAppend(MarkerSet([StudyPath 'Models\' 'Exo_Markers_' Side '.xml']).get(1));
NoExo_MarkerSet.cloneAndAppend(MarkerSet([StudyPath 'Models\' 'Exo_Markers_' Side '.xml']).get(2));
NoExo_MarkerSet.cloneAndAppend(MarkerSet([StudyPath 'Models\' 'Exo_Markers_' Side '.xml']).get(3));
NoExo_MarkerSet.cloneAndAppend(MarkerSet([StudyPath 'Models\' 'Exo_Markers_' Side '.xml']).get(4));
NoExo_MarkerSet.cloneAndAppend(MarkerSet([StudyPath 'Models\' 'Exo_Markers_' Side '.xml']).get(5));
NoExo_MarkerSet.cloneAndAppend(MarkerSet([StudyPath 'Models\' 'Exo_Markers_' Side '.xml']).get(6));

AFO_Exo_Model.set_MarkerSet(NoExo_MarkerSet);

AFO_Exo_Model.print([StudyPath 'OutFiles\' Subject '\' Session '\Scaling\' Subject '_Exo.osim']);

%% Run Test IK on Model

% % Run IK on a sample trial and get kinematics
% TestTrial= [Subject '_EA_1_test'];
% 
% TestTRC=Vicon.C3DtoTRC([StudyPath 'Outfiles\' Subject '\' Session '\Data\' TestTrial '.c3d']);
% Osim.writeTRC(TestTRC,'FilePath',[StudyPath 'Outfiles\' Subject '\' Session '\Data\' TestTrial '.trc']);
% 
% TestIKTrialName=[StudyPath 'Outfiles\' Subject '\' Session '\Data\' TestTrial '.trc'];
% IKSetup=[StudyPath 'OutFiles\' Subject '\' Session '\Scaling\AdjustMe\' Subject '_TestIK_Setup.xml'];
% Osim.editSetupXML(GenIKSetupFile,'model_file',[StudyPath 'OutFiles\' Subject '\' Session '\Scaling\' Subject '_Exo.osim'],'FilePath',IKSetup);
% % markerWeightChange(IKSetup,'LTH1',0);
% TestIKMotOut=[StudyPath 'OutFiles\' Subject '\' Session '\Scaling\AdjustMe\' Subject '_Exo_TestIK.mot'];
% % Running IK
% [jointAngleTable, ~, ~] = Osim.IK(TestIKTrialName,IKSetup,TestIKMotOut);
% 
% % Figures
% figure;
% hold on
% subplot(3,1,1)
% hold on
% plot(jointAngleTable.Header,jointAngleTable.hip_flexion_r,'r')
% plot(jointAngleTable.Header,jointAngleTable.hip_flexion_l,'b')
% subplot(3,1,2)
% hold on
% plot(jointAngleTable.Header,jointAngleTable.knee_angle_r,'r')
% plot(jointAngleTable.Header,jointAngleTable.knee_angle_l,'b')
% subplot(3,1,3)
% hold on
% plot(jointAngleTable.Header,jointAngleTable.ankle_angle_r,'r')
% plot(jointAngleTable.Header,jointAngleTable.ankle_angle_l,'b')
% 
% clear scaling jointAngleTable IKSetup

%% Processing dynamic trials

KinematicFilterFreq=6;
Filterorder=4;
TimeFrameOffset=1; %Frames to expand the analysis period by to ensure accurate initial and ending values
close all

%% Process through inverse dynamics
TrialCount=struct;

for i=1:size(trialnames,2)

    C3D=btkReadAcquisition([StudyPath 'Outfiles\' Subject '\' Session '\Data\' trialnames{i} '.c3d']);

    trialparams=struct;
    trialparams.Trialname=trialnames{i};
    trialparams.Subject=Subject;
    trialparams.Session=Session;
    trialparams.StudyPath=StudyPath;
    trialparams.GenIKSetupFile=GenIKSetupFile;
    trialparams.GenIDSetupFile=GenIDSetupFile;
    trialparams.KinematicFilterFreq=KinematicFilterFreq;
    trialparams.Filterorder=Filterorder;
    trialparams.Mass=Mass;
    trialparams.Height=Height;
    trialparams.TimeFrameOffset=TimeFrameOffset;
    trialparams.ModelFile=[StudyPath 'OutFiles\' Subject '\' Session '\Scaling\' Subject '_Exo.osim'];
    trialparams.applied_to_body{1}='calcn_r';
    trialparams.applied_to_body{2}='calcn_l';
    trialparams.KineticFilterFreq=12;
    trialparams.GrfThresh=50;
    trialparams.DetectThresh=100;

    TRCTable=Osim.readTRC([StudyPath 'Outfiles\' Subject '\' Session '\Data\' trialnames{i} '.trc']);

    trialparams.tstart=TRCTable.Header(1);
    trialparams.tend=TRCTable.Header(end);

    [GaitEventStruct]=findGaitEvents(TRCTable,trialparams.KinematicFilterFreq,trialparams.Filterorder,'RHEE','LHEE','R2MT','L2MT','RASIS','LASIS','RPSIS','LPSIS');

    %Determine start and end time - using GCs
    trialparams.FrameSamplingRate=btkGetPointFrequency(C3D);
    trialparams.AnalogSamplingRate=btkGetAnalogSampleNumberPerFrame(C3D).*trialparams.FrameSamplingRate;

    %Inverse kinematics
    % Setup file that will be used to run inverse kinematics
    trialparams.IKSetup=[trialparams.StudyPath 'OutFiles\' trialparams.Subject '\' trialparams.Session '\IK\' trialparams.Trialname '_IKSetup.xml'];
    Osim.editSetupXML(trialparams.GenIKSetupFile,'model_file',trialparams.ModelFile,'FilePath',trialparams.IKSetup);
    % markerWeightChange(trialparams.IKSetup,'LTH1',0);

    % MOT file for output of inverse kinematics
    trialparams.IKOutputFile=[trialparams.StudyPath 'OutFiles\' trialparams.Subject '\' trialparams.Session '\IK\' trialparams.Trialname '_IK.mot'];

    %Running IK
    Osim.IK([trialparams.StudyPath 'Outfiles\' trialparams.Subject '\' trialparams.Session '\Data\' trialparams.Trialname '.trc'],trialparams.IKSetup, trialparams.IKOutputFile);


    %Body kinematics
    trialparams=AFO_EXO_BK(trialparams);

    State_Table=Osim.readSTO([trialparams.StudyPath 'OutFiles\' trialparams.Subject '\' trialparams.Session '\BK\' trialparams.Trialname '_SR_states.sto']);
    BK_P_Table=Osim.readSTO([trialparams.StudyPath 'OutFiles\' trialparams.Subject '\' trialparams.Session '\BK\' trialparams.Trialname '_BodyKinematics_pos_global.sto']);
    BK_V_Table=Osim.readSTO([trialparams.StudyPath 'OutFiles\' trialparams.Subject '\' trialparams.Session '\BK\' trialparams.Trialname '_BodyKinematics_vel_global.sto']);

    %WBAM
    WBAM(State_Table,BK_P_Table,BK_V_Table,trialparams.ModelFile,trialparams.Height,trialparams.Mass,[trialparams.StudyPath '\OutFiles\' trialparams.Subject '\' trialparams.Session '\WBAM\'  trialparams.Trialname '_AngMom.mat']);
%% 
    %External loads
    trialparams=AFO_ExtLoads_EA(trialparams, false);

    %ID
    trialparams=AFO_ID(trialparams);

    %% Foot Power
    SH_CTRL_FootPower(trialparams);

    save([trialparams.StudyPath 'OutFiles\' trialparams.Subject '\' trialparams.Session '\Trialparams\' trialparams.Trialname '_Trialparams.mat'],'trialparams');

    btkCloseAcquisition(C3D);
end


 