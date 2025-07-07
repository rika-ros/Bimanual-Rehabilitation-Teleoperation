data = load('xyz_final3_saved.mat');
plot(data.time, data.pos_box, 'b', data.time, data.zd_traj, 'r--');
xlabel('Time (s)');
ylabel('Z Position (m)');
legend('Actual Position', 'Desired Trajectory');
grid on;
