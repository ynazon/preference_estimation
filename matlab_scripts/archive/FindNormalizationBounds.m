function [bounds, normalization_scaling_vector, detail] = FindNormalizationBounds(projectDir, outputDir)
% Bounds of unnormalized gait segments used by NormalizeSubjectData.
% Pools AFOWR01/AFOWR02, five trials each, and both sides. Ignores
% NaN/Inf, reports missing signals, and retains extrema provenance.
if nargin < 1
    projectDir = 'C:\Users\nazon\OneDrive\Documents\Walter Reed\Preference Project\Exploratory Aim Data\Exploratory Aim';
end
if nargin < 2, outputDir = fileparts(mfilename('fullpath')); end
addpath(projectDir);
source = fileread(fullfile(projectDir,'NormalizeSubjectData.m'));
block = regexp(source,'normalization_scaling_vector\s*=\s*\{([\s\S]*?)\};','tokens','once');
assert(~isempty(block),'Normalization vector not found.');
tokens = regexp(block{1},'(?m)^\s*[-+\d.eE]+\s*,\s*[-+\d.eE]+\s*,\s*''([^'']+)''','tokens');
names = string(cellfun(@(x)x{1},tokens,'UniformOutput',false))';
assert(numel(unique(names)) == numel(names),'Duplicate signal names.');
subjects = ["AFOWR01","AFOWR02"];
rows = cell(0,9);
for subject = subjects
    segments = SegmentSubjectData(char(subject),'L');
    for trial = 1:numel(segments)
        for side = ["left","right"]
            cycles = segments(trial).(side);
            for n = 1:numel(names)
                signal = names(n); lo = Inf; hi = -Inf; count = 0;
                minCycle = NaN; maxCycle = NaN;
                for c = 1:numel(cycles)
                    if ~isfield(cycles(c),signal), continue; end
                    x = cycles(c).(signal);
                    x = x(isfinite(x));
                    if isempty(x), continue; end
                    count = count + numel(x);
                    if min(x) < lo, lo = min(x); minCycle = c; end
                    if max(x) > hi, hi = max(x); maxCycle = c; end
                end
                if count > 0
                    rows(end+1,:) = {subject,trial,side,signal,lo,hi,count,minCycle,maxCycle}; %#ok<AGROW>
                end
            end
        end
    end
end
detail = cell2table(rows,'VariableNames',{'Subject','Trial','Side','Signal','Minimum','Maximum','FiniteSampleCount','MinCycle','MaxCycle'});
bounds = table(names,NaN(size(names)),NaN(size(names)),zeros(size(names)),strings(size(names)),strings(size(names)),...
    'VariableNames',{'Signal','Minimum','Maximum','FiniteSampleCount','MinSource','MaxSource'});
for n = 1:numel(names)
    d = detail(string(detail.Signal) == names(n),:);
    if isempty(d), continue; end
    [bounds.Minimum(n),a] = min(d.Minimum);
    [bounds.Maximum(n),b] = max(d.Maximum);
    bounds.FiniteSampleCount(n) = sum(d.FiniteSampleCount);
    bounds.MinSource(n) = sprintf('%s trial %d %s cycle %d',string(d.Subject(a)),d.Trial(a),string(d.Side(a)),d.MinCycle(a));
    bounds.MaxSource(n) = sprintf('%s trial %d %s cycle %d',string(d.Subject(b)),d.Trial(b),string(d.Side(b)),d.MaxCycle(b));
end
normalization_scaling_vector = [num2cell(bounds.Maximum),num2cell(bounds.Minimum),cellstr(names)];
if ~exist(outputDir,'dir'), mkdir(outputDir); end
writetable(bounds,fullfile(outputDir,'normalization_bounds.csv'));
writetable(detail,fullfile(outputDir,'normalization_bounds_by_trial_side.csv'));
subjectRows = cell(0,4);
for subject = subjects
    for n = 1:numel(names)
        d = detail(string(detail.Subject)==subject & string(detail.Signal)==names(n),:);
        if isempty(d), lo=NaN; hi=NaN; else, lo=min(d.Minimum); hi=max(d.Maximum); end
        subjectRows(end+1,:) = {subject,names(n),lo,hi}; %#ok<AGROW>
    end
end
writetable(cell2table(subjectRows,'VariableNames',{'Subject','Signal','Minimum','Maximum'}),fullfile(outputDir,'normalization_bounds_by_subject.csv'));
save(fullfile(outputDir,'normalization_bounds.mat'),'bounds','normalization_scaling_vector','detail');
fid = fopen(fullfile(outputDir,'normalization_scaling_vector.txt'),'w');
cleanup = onCleanup(@()fclose(fid));
fprintf(fid,'normalization_scaling_vector = { ...\n');
for n=1:numel(names)
    fprintf(fid,'    %.17g, %.17g, ''%s''; ...\n',bounds.Maximum(n),bounds.Minimum(n),names(n));
end
fprintf(fid,'};\n');
disp(bounds);
if any(~isfinite(bounds.Minimum)), warning('Some signals have no finite samples.'); end
if any(bounds.Minimum==bounds.Maximum), warning('Constant signals cannot use min-max division without a guard.'); end
end
