function results = processOpenSimGaitCycles(grfFile, ikFile, idFile, outputDir, cfg)
%PROCESSOPENSIMGAITCYCLES Segment OpenSim GRF, IK, and ID into gait cycles.
%
% results = processOpenSimGaitCycles(grfFile,ikFile,idFile,outputDir)
% results = processOpenSimGaitCycles(...,cfg)
%
% A gait cycle is defined separately for each limb as consecutive ipsilateral
% heel strikes. Heel strike and toe-off are detected from vertical GRF.
% The function saves raw cycles, 101-point normalized cycles, event tables,
% a MAT file, CSV files, and a diagnostic event plot.
%
% Required inputs
%   grfFile  - OpenSim external-loads .mot file
%   ikFile   - inverse-kinematics .mot file
%   idFile   - inverse-dynamics .sto or .mot file
%   outputDir- folder for results
%
% Optional cfg fields
%   thresholdN       = 20;       % contact threshold, N
%   hysteresisN      = 5;        % lower release threshold, N
%   minContactSec    = 0.20;     % reject shorter contacts
%   minSwingSec      = 0.15;     % merge shorter gaps
%   minCycleSec      = 0.50;
%   maxCycleSec      = 2.00;
%   nPoints          = 101;
%   rightVerticalGRF = "";       % exact column name, or auto-detect
%   leftVerticalGRF  = "";
%
% Example
%   cfg.thresholdN = 25;
%   R = processOpenSimGaitCycles('trial_GRF.mot','trial_IK.mot', ...
%       'trial_ID.sto','processed',cfg);

arguments
    grfFile (1,1) string
    ikFile (1,1) string
    idFile (1,1) string
    outputDir (1,1) string = "GaitCycleResults"
    cfg.thresholdN (1,1) double {mustBeNonnegative} = 20
    cfg.hysteresisN (1,1) double {mustBeNonnegative} = 5
    cfg.minContactSec (1,1) double {mustBePositive} = 0.20
    cfg.minSwingSec (1,1) double {mustBeNonnegative} = 0.15
    cfg.minCycleSec (1,1) double {mustBePositive} = 0.50
    cfg.maxCycleSec (1,1) double {mustBePositive} = 2.00
    cfg.nPoints (1,1) double {mustBeInteger,mustBeGreaterThan(cfg.nPoints,2)} = 101
    cfg.rightVerticalGRF (1,1) string = ""
    cfg.leftVerticalGRF (1,1) string = ""
end

if cfg.hysteresisN >= cfg.thresholdN
    error('cfg.hysteresisN must be smaller than cfg.thresholdN.');
end
if cfg.maxCycleSec <= cfg.minCycleSec
    error('cfg.maxCycleSec must exceed cfg.minCycleSec.');
end

grf = readOpenSimStorage(grfFile);
ik  = readOpenSimStorage(ikFile);
id  = readOpenSimStorage(idFile);
validateTime(grf, "GRF"); validateTime(ik, "IK"); validateTime(id, "ID");

[rName,lName] = resolveVerticalGRF(grf, cfg.rightVerticalGRF, ...
    cfg.leftVerticalGRF);
fprintf('Right vertical GRF: %s\nLeft vertical GRF:  %s\n',rName,lName);

rightEvents = detectEvents(grf.time, grf.(rName), cfg);
leftEvents  = detectEvents(grf.time, grf.(lName), cfg);

right = makeCycles("right", rightEvents, grf, ik, id, cfg);
left  = makeCycles("left",  leftEvents,  grf, ik, id, cfg);

if ~isfolder(outputDir), mkdir(outputDir); end
writeEventTable(rightEvents, outputDir, "right");
writeEventTable(leftEvents,  outputDir, "left");
writeNormalizedCSVs(right, outputDir);
writeNormalizedCSVs(left, outputDir);

results = struct('sourceFiles',struct('GRF',grfFile,'IK',ikFile,'ID',idFile), ...
    'verticalGRFColumns',struct('right',rName,'left',lName), ...
    'config',cfg,'right',right,'left',left);
save(fullfile(outputDir,'gait_cycles.mat'),'results','-v7.3');
plotEvents(grf, rName, lName, rightEvents, leftEvents, outputDir, cfg);

fprintf('Saved %d right and %d left valid gait cycles to %s\n', ...
    numel(right.raw),numel(left.raw),outputDir);
end

function T = readOpenSimStorage(file)
fid = fopen(file,'rt');
if fid < 0, error('Cannot open %s.',file); end
cleanup = onCleanup(@() fclose(fid));
found = false;
while ~feof(fid)
    line = fgetl(fid);
    if strcmpi(strtrim(line),'endheader'), found = true; break; end
end
if ~found, error('No endheader line found in %s.',file); end
names = split(string(strtrim(fgetl(fid)))); names(names=="") = [];
fmt = repmat('%f',1,numel(names));
C = textscan(fid,fmt,'Delimiter',{' ','\t'},'MultipleDelimsAsOne',true, ...
    'CollectOutput',true,'TreatAsEmpty',{'NaN','nan'});
if isempty(C{1}), error('No numeric rows found in %s.',file); end
validNames = matlab.lang.makeUniqueStrings(matlab.lang.makeValidName(cellstr(names)));
T = array2table(C{1},'VariableNames',validNames);
if ~ismember('time',T.Properties.VariableNames)
    error('%s has no time column.',file);
end
end

function validateTime(T,label)
t = T.time;
if any(~isfinite(t)) || any(diff(t)<=0)
    error('%s time must be finite and strictly increasing.',label);
end
end

function [rName,lName] = resolveVerticalGRF(T,rOverride,lOverride)
vars = string(T.Properties.VariableNames);
if strlength(rOverride)>0 && strlength(lOverride)>0
    rName = char(matlab.lang.makeValidName(rOverride));
    lName = char(matlab.lang.makeValidName(lOverride));
    if ~all(ismember({rName,lName},T.Properties.VariableNames))
        error('Specified vertical-GRF columns were not found. Available: %s',join(vars,', '));
    end
    return
end
% Common OpenSim conventions: ground_force_r_vy, ground_force_1_vy,
% R_ground_force_vy, forceplate1_vy. Edit overrides if auto-detection differs.
low = lower(vars);
isVertical = endsWith(low,"_vy") | endsWith(low,"_fz") | ...
    contains(low,"vertical");
isForce = contains(low,"force") | contains(low,"grf");
candidates = vars(isVertical & isForce);
if strlength(rOverride)>0, rName=char(matlab.lang.makeValidName(rOverride));
else, rName = chooseSide(candidates,"right"); end
if strlength(lOverride)>0, lName=char(matlab.lang.makeValidName(lOverride));
else, lName = chooseSide(candidates,"left"); end
if strcmp(rName,lName) || ~all(ismember({rName,lName},T.Properties.VariableNames))
    error(['Could not uniquely identify bilateral vertical GRF columns. ' ...
        'Set cfg.rightVerticalGRF and cfg.leftVerticalGRF. Candidates: %s'], ...
        join(candidates,', '));
end
end

function name = chooseSide(candidates,side)
low = lower(candidates);
if side=="right"
    score = contains(low,"right")*4 + contains(low,"_r_")*4 + ...
        startsWith(low,"r_")*3 + contains(low,"force_r")*4 + ...
        contains(low,"_1_") + contains(low,"plate1");
else
    score = contains(low,"left")*4 + contains(low,"_l_")*4 + ...
        startsWith(low,"l_")*3 + contains(low,"force_l")*4 + ...
        contains(low,"_2_") + contains(low,"plate2");
end
[best,idx] = max(score);
if isempty(idx) || best==0, name=''; else, name=char(candidates(idx)); end
end

function E = detectEvents(t,Fz,cfg)
Fz = double(Fz(:));
% Make upward GRF positive if the file uses the opposite convention.
if abs(prctile(Fz,1)) > abs(prctile(Fz,99)), Fz=-Fz; end
Fz(~isfinite(Fz)) = 0;
contact = hysteresisContact(Fz,cfg.thresholdN,cfg.hysteresisN);
contact = cleanContact(contact,t,cfg.minContactSec,cfg.minSwingSec);
hsIdx = find(diff([false;contact])==1);
toIdx = find(diff([contact;false])==-1);
% Pair every heel strike with the first subsequent toe-off.
hs=[]; toe=[];
for k=1:numel(hsIdx)
    j=find(toIdx>=hsIdx(k),1);
    if ~isempty(j), hs(end+1,1)=hsIdx(k); toe(end+1,1)=toIdx(j); end %#ok<AGROW>
end
E = table(t(hs),t(toe),'VariableNames',{'heelStrike','toeOff'});
end

function c = hysteresisContact(x,onThreshold,offThreshold)
c=false(size(x)); state=false;
for i=1:numel(x)
    if ~state && x(i)>=onThreshold, state=true;
    elseif state && x(i)<=offThreshold, state=false;
    end
    c(i)=state;
end
end

function c = cleanContact(c,t,minContact,minSwing)
% Repeatedly remove short contact islands and fill short swing gaps.
for pass=1:2
    c = replaceShortRuns(c,t,true,minContact,false);
    c = replaceShortRuns(c,t,false,minSwing,true);
end
end

function x = replaceShortRuns(x,t,value,minDuration,replacement)
d=diff([false;x==value;false]); starts=find(d==1); stops=find(d==-1)-1;
for k=1:numel(starts)
    duration=t(stops(k))-t(starts(k));
    if duration<minDuration, x(starts(k):stops(k))=replacement; end
end
end

function out = makeCycles(side,E,grf,ik,id,cfg)
raw=struct([]); normalized=struct([]); rows=[]; count=0;
for k=1:height(E)-1
    t0=E.heelStrike(k); t1=E.heelStrike(k+1); duration=t1-t0;
    if duration<cfg.minCycleSec || duration>cfg.maxCycleSec, continue; end
    toe=E.toeOff(k);
    if toe<=t0 || toe>=t1, continue; end
    count=count+1;
    raw(count).cycle=count; raw(count).startTime=t0; raw(count).toeOffTime=toe; %#ok<AGROW>
    raw(count).endTime=t1; raw(count).duration=duration;
    raw(count).stancePercent=100*(toe-t0)/duration;
    raw(count).GRF=sliceTable(grf,t0,t1);
    raw(count).IK=sliceTable(ik,t0,t1);
    raw(count).ID=sliceTable(id,t0,t1);
    normalized(count).cycle=count; %#ok<AGROW>
    normalized(count).gaitPercent=linspace(0,100,cfg.nPoints)';
    normalized(count).GRF=normalizeTable(grf,t0,t1,cfg.nPoints);
    normalized(count).IK=normalizeTable(ik,t0,t1,cfg.nPoints);
    normalized(count).ID=normalizeTable(id,t0,t1,cfg.nPoints);
    rows=[rows; count t0 toe t1 duration raw(count).stancePercent]; %#ok<AGROW>
end
out=struct('side',side,'events',E,'cycleSummary',array2table(rows, ...
    'VariableNames',{'cycle','heelStrike','toeOff','nextHeelStrike', ...
    'duration','stancePercent'}),'raw',raw,'normalized',normalized);
end

function S = sliceTable(T,t0,t1)
idx=T.time>=t0 & T.time<=t1; S=T(idx,:); S.gaitPercent=100*(S.time-t0)/(t1-t0);
S=movevars(S,'gaitPercent','After','time');
end

function N = normalizeTable(T,t0,t1,n)
q=linspace(t0,t1,n)'; X=T{:,2:end};
Y=interp1(T.time,X,q,'pchip',NaN);
N=array2table([linspace(0,100,n)' Y], ...
    'VariableNames',[{'gaitPercent'},T.Properties.VariableNames(2:end)]);
end

function writeEventTable(E,out,side)
writetable(E,fullfile(out,side+'_events.csv'));
end

function writeNormalizedCSVs(S,out)
for k=1:numel(S.normalized)
    stem=sprintf('%s_cycle_%03d_',S.side,k);
    writetable(S.normalized(k).GRF,fullfile(out,[stem 'GRF.csv']));
    writetable(S.normalized(k).IK, fullfile(out,[stem 'IK.csv']));
    writetable(S.normalized(k).ID, fullfile(out,[stem 'ID.csv']));
end
writetable(S.cycleSummary,fullfile(out,S.side+'_cycle_summary.csv'));
end

function plotEvents(grf,rName,lName,rE,lE,out,cfg)
f=figure('Color','w','Visible','off'); tiledlayout(2,1);
plotOne(nexttile,grf.time,grf.(rName),rE,'Right',cfg.thresholdN);
plotOne(nexttile,grf.time,grf.(lName),lE,'Left',cfg.thresholdN);
xlabel('Time (s)'); exportgraphics(f,fullfile(out,'GRF_event_check.png'),'Resolution',200); close(f);
end

function plotOne(ax,t,y,E,label,threshold)
plot(ax,t,y,'k'); hold(ax,'on'); yline(ax,threshold,'--','Threshold');
xline(ax,E.heelStrike,'g-'); xline(ax,E.toeOff,'r-'); grid(ax,'on');
ylabel(ax,[label ' vertical GRF (N)']); title(ax,[label ': green=HS, red=TO']);
end
