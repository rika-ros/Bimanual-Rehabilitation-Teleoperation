% Load data
load('tim_.mat');          % time -> tim_
load('pos_.mat');          % actual position -> pos_
load('pos_des_X.mat');     % desired X -> pos_des_X
load('pos_des_Y.mat');     % desired Y -> pos_des_Y
load('pos_des_Z.mat');     % desired Z -> pos_des_Z

t = tim_;
x = pos_(1, :)';
y = pos_(2, :)';
z = pos_(3, :)';

xd = pos_des_X;
yd = pos_des_Y;
zd = pos_des_Z;

% Plot X
figure;
plot(t, x, 'b-', 'LineWidth', 2); hold on;
plot(t, xd, 'r--', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('X Position (m)');
title('X Axis: Actual vs Desired');
legend('Actual X', 'Desired X'); grid on;

% Plot Y
figure;
plot(t, y, 'b-', 'LineWidth', 2); hold on;
plot(t, yd, 'r--', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Y Position (m)');
title('Y Axis: Actual vs Desired');
legend('Actual Y', 'Desired Y'); grid on;

% Plot Z
figure;
plot(t, z, 'b-', 'LineWidth', 2); hold on;
plot(t, zd, 'r--', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Z Position (m)');
title('Z Axis: Actual vs Desired');
legend('Actual Z', 'Desired Z'); grid on;
