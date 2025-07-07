clear;
clc;

% Load the data
data = load('xyz_final3_saved.mat');

time = data.time;
LzsHh = data.LzsHh;
LzdHh = data.LzdHh;
LzrR = data.LzrR;
LzlR = data.LzlR;

% Plot Human Gains
figure;
subplot(2,1,1);
plot(time, LzsHh(:,1), 'b', 'LineWidth', 1.5); hold on;
plot(time, LzsHh(:,2), 'b--', 'LineWidth', 1.5);
plot(time, LzdHh(:,1), 'r', 'LineWidth', 1.5);
plot(time, LzdHh(:,2), 'r--', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Gain Value');
title('Human Control Gains (Z-direction)');
legend('LzsHh(1)', 'LzsHh(2)', 'LzdHh(1)', 'LzdHh(2)');
grid on;

% Plot Robot Gains
subplot(2,1,2);
plot(time, LzrR(:,1), 'g', 'LineWidth', 1.5); hold on;
plot(time, LzrR(:,2), 'g--', 'LineWidth', 1.5);
plot(time, LzlR(:,1), 'm', 'LineWidth', 1.5);
plot(time, LzlR(:,2), 'm--', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Gain Value');
title('Robot Control Gains (Z-direction)');
legend('LzrR(1)', 'LzrR(2)', 'LzlR(1)', 'LzlR(2)');
grid on;
