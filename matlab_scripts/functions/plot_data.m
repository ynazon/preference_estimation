plot_idx = 2;
figure
for i=1:length(segmented_data(plot_idx).left)
    plot(segmented_data(plot_idx).left(i).Time - segmented_data(plot_idx).left(i).Time(1),segmented_data(plot_idx).left(i).Ankle_IK,'b');
    hold on
    plot(segmented_data(plot_idx).left(i).Time - segmented_data(plot_idx).left(i).Time(1),segmented_data(plot_idx).left(i).HT_ankle_angle,'r');
    % plot(segmented_data(plot_idx).left(i).Time - segmented_data(plot_idx).left(i).Time(1),segmented_data(plot_idx).left(i).HT_neutral_angle,'g');
end
title('Pos')

figure
for i=1:length(segmented_data(plot_idx).left)
    plot(segmented_data(plot_idx).left(i).Time(1:end-1) - segmented_data(plot_idx).left(i).Time(1),diff(segmented_data(plot_idx).left(i).Ankle_IK)./diff(segmented_data(plot_idx).left(i).Time),'b');
    hold on
    plot(segmented_data(plot_idx).left(i).Time - segmented_data(plot_idx).left(i).Time(1),segmented_data(plot_idx).left(i).HT_encoder_velocity,'r');
    plot(segmented_data(plot_idx).left(i).Time(1:end-1) - segmented_data(plot_idx).left(i).Time(1),diff(segmented_data(plot_idx).left(i).HT_ankle_angle)./diff(segmented_data(plot_idx).left(i).Time),'g')
end
title('Vel')

figure
for i=1:length(segmented_data(plot_idx).left)
    plot(segmented_data(plot_idx).left(i).Time - segmented_data(plot_idx).left(i).Time(1),segmented_data(plot_idx).left(i).HT_force_plate,'b');
    hold on
    plot(segmented_data(plot_idx).left(i).Time - segmented_data(plot_idx).left(i).Time(1),segmented_data(plot_idx).left(i).HT_exo_torque,'r');
    plot(segmented_data(plot_idx).left(i).Time - segmented_data(plot_idx).left(i).Time(1),segmented_data(plot_idx).left(i).GRF,'g');
end
title('GRF')
