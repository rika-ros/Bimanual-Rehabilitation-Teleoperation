load('tim_.mat');       % time
load('pos_.mat');       % actual pos_box [N x 3]
load('pos_des_Z.mat');  % desired Z trajectory

plot(tim_, pos_(:,3), 'b', 'LineWidth', 1.5)
hold on
plot(tim_, pos_des_Z, 'r--', 'LineWidth', 1.5)
legend('Actual Z', 'Desired Z')
xlabel('Time [s]')
ylabel('Z Position [m]')
grid on
