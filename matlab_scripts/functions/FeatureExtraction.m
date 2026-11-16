function extracted_features = FeatureExtraction(data)
% extracted features:
%   - max value
%   - min value
%   - mean
%   - std dev
%   - initial value
%   - final value

extracted_features = table();
data_names = data.Properties.VariableNames;
[~,TimeIdx] = find(strcmp(data_names,'Time'));
for i = 1:TimeIdx
    extracted_features{:,i} = data{:,i};
end

% extract features from interpolated data
for i=1:height(data)
    for j=TimeIdx+1:width(data)
        MaxValue = max(data.(data_names{j}){i});
        MinValue = min(data.(data_names{j}){i});
        extracted_features{i,j} = ...
            [MaxValue,MinValue,mean(data.(data_names{j}){i}),...
            std(data.(data_names{j}){i}),data.(data_names{j}){i}(1),...
            data.(data_names{j}){i}(end)];
    end
end

extracted_features.Properties.VariableNames = data_names;

end