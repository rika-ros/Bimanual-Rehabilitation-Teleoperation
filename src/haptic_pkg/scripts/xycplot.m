% Load data
actual = load('pos_.mat');
desX = load('pos_des_X.mat');
desY = load('pos_des_Y.mat');

% Extract actual position data
pos = actual.pos_;
pos_x = pos(1, :);
pos_y = pos(2, :);

% Extract desired position data
ref_x = desX.pos_des_X(:)';
ref_y = desY.pos_des_Y(:)';

% Create time vector (assuming dt = 0.01s)
t = linspace(0, 0.01*(length(pos_x)-1), length(pos_x));

% ---- Plot X axis ----
figure;
plot(t, pos_x, 'b-', 'LineWidth', 2); hold on;
plot(t, ref_x, 'k--', 'LineWidth', 2);  % dashed black
xlabel('Time (s)');
ylabel('X Position (m)');
legend('Actual X (blue)', 'Desired X (black dashed)');
title('X Axis: Actual vs Desired');
grid on;

% ---- Plot Y axis ----
figure;
plot(t, pos_y, 'r-', 'LineWidth', 2); hold on;
plot(t, ref_y, 'k--', 'LineWidth', 2);  % dashed black
xlabel('Time (s)');
ylabel('Y Position (m)');
legend('Actual Y (red)', 'Desired Y (black dashed)');
title('Y Axis: Actual vs Desired');
grid on;

% ---- Plot Y axis ----
figure;
plot(t, pos_z, 'r-', 'LineWidth', 2); hold on;
plot(t, ref_z, 'k--', 'LineWidth', 2);  % dashed black
xlabel('Time (s)');
ylabel('Y Position (m)');
legend('Actual z (red)', 'Desired Y (black dashed)');
title('z Axis: Actual vs Desired');
grid on;