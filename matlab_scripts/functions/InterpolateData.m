function interpolated_data = InterpolateData(data)

interpolated_data = table();
data_names = data.Properties.VariableNames;
[~,TimeIdx] = find(strcmp(data_names,'Time'));
num_of_interpolated_points = 101;
for i = 1:TimeIdx-1
    interpolated_data{:,i} = data{:,i};
end

% interpolate data to be 101 points
for i=1:height(data)
    for j=TimeIdx:width(data)
        dt = (data.(data_names{TimeIdx}){i}(end) - data.(data_names{TimeIdx}){i}(1))...
            /(num_of_interpolated_points-1);
        interpolated_time_vector = data.(data_names{TimeIdx}){i}(1):dt:data.(data_names{TimeIdx}){i}(end);
        interpolated_data{i,j} = interp1(data.(data_names{TimeIdx}){i},...
            data.(data_names{j}){i},interpolated_time_vector);

    end
end

interpolated_data.Properties.VariableNames = data_names;

end