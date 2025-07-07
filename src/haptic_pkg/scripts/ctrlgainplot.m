load('LxHh_.mat');     % Human control gain estimate (X-dir)
load('LylHh_.mat');    % Robot controller gain (left hand, Y-dir)
load('LyrHh_.mat');    % Robot controller gain (right hand, Y-dir)

% Create time vector
dt = 0.01;
time = dt * (0:size(LxHh_,1)-1);

figure;

% --- (a) LxHh(1) and LxHh(2) ---
subplot(3,2,1);
plot(time, LxHh_(:,1), 'b-', 'LineWidth', 1.2); hold on;
plot(time, LxHh_(:,2), 'r--', 'LineWidth', 1.2);
xlabel('Time (s)'); ylabel('LxHh');
legend('LxHh(1)', 'LxHh(2)'); grid on;
title('(a) Human gain estimates in X');

% --- (b) LylHh(1) and LylHh(2) ---
subplot(3,2,3);
plot(time, LylHh_(:,1), 'g-', 'LineWidth', 1.2); hold on;
plot(time, LylHh_(:,2), 'm--', 'LineWidth', 1.2);
xlabel('Time (s)'); ylabel('LylHh');
legend('LylHh(1)', 'LylHh(2)'); grid on;
title('(b) Robot left Y-gain');

% --- (c) LyrHh(1) and LyrHh(2) ---
subplot(3,2,5);
plot(time, LyrHh_(:,1), 'k-', 'LineWidth', 1.2); hold on;
plot(time, LyrHh_(:,2), 'c--', 'LineWidth', 1.2);
xlabel('Time (s)'); ylabel('LyrHh');
legend('LyrHh(1)', 'LyrHh(2)'); grid on;
title('(c) Robot right Y-gain');

sgtitle('Fig. 9: Human & Robot Gain Estimates');
