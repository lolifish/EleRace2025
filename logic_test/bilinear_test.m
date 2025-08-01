% 清除工作区和命令行
clear;
clc;

% 1. 定义滤波器参数
Fs = 1e6;           % 采样频率 (Hz) - 设为1MHz，确保中心频率200kHz在Fs/2以内
F_center = 200e3;   % 模拟滤波器中心频率 (Hz)
BW = 50e3;          % 模拟滤波器带宽 (Hz)
Order = 2;          % 滤波器阶数 (二阶带通滤波器实际阶数为 2*Order)

% 计算模拟滤波器的截止频率
F_low = F_center - BW/2;
F_high = F_center + BW/2;

% 将截止频率转换为模拟角频率 (rad/s)
Wn_analog = [F_low, F_high] * 2 * pi;

% 2. 设计模拟带通滤波器
[b_analog, a_analog] = butter(Order, Wn_analog, 'bandpass', 's'); % [4]

% 3. 计算模拟滤波器频率响应
% 生成模拟频率点
w_analog = logspace(log10(Wn_analog(1)/10), log10(Wn_analog(2)*10), 1000);
[H_analog, w_analog_rad] = freqs(b_analog, a_analog, w_analog); % [15]

% 4. 执行双线性变换
[b_digital, a_digital] = bilinear(b_analog, a_analog, Fs); % [1, 5, 10]

% 5. 计算数字滤波器频率响应
w_digital_norm = linspace(0, 1, 1000) * pi;
[H_digital, w_digital_rad] = freqz(b_digital, a_digital, w_digital_norm); % [2, 6, 8]
w_digital_hz = w_digital_rad / (2 * pi) * Fs;

% 6. 绘制并比较频响
figure;

% 幅频响应
subplot(2,1,1);
plot(w_analog_rad / (2*pi), 20*log10(abs(H_analog)), 'b', 'LineWidth', 1.5);
hold on;
plot(w_digital_hz, 20*log10(abs(H_digital)), 'r--', 'LineWidth', 1.5);
grid on;
title('带通滤波器幅频响应比较');
xlabel('频率 (Hz)');
ylabel('幅度 (dB)');
legend('模拟滤波器', '数字滤波器 (双线性变换)');
xlim([0 max(w_digital_hz)]);

% 相频响应
subplot(2,1,2);
plot(w_analog_rad / (2*pi), unwrap(angle(H_analog)) * 180/pi, 'b', 'LineWidth', 1.5);
hold on;
plot(w_digital_hz, unwrap(angle(H_digital)) * 180/pi, 'r--', 'LineWidth', 1.5);
grid on;
title('带通滤波器相频响应比较');
xlabel('频率 (Hz)');
ylabel('相位 (度)');
legend('模拟滤波器', '数字滤波器 (双线性变换)');
xlim([0 max(w_digital_hz)]);

% 双线性变换在高频处会导致频率压缩。