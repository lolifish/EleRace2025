% 1. 定义真实系统的系数
b_true = [0.9, 5]; % 分子系数 [b_1, b_0]
a_true = [1, 2, 20]; % 分母系数 [a_2, a_1, a_0]

% 2. 生成频率点 (例如，从0到100 rad/s)
w = linspace(0, 100, 500); % 500个频率点，单位：rad/s

% 3. 使用 freqresp (或 freqs) 获取真实系统的频率响应
% freqresp 函数需要 control system toolbox. 如果没有，可以使用 freqs
% H_true = freqs(b_true, a_true, w); 
% 或者更直接地计算：
H_true = polyval(b_true, 1j*w) ./ polyval(a_true, 1j*w); 

% 4. 使用我们自定义的函数来估计系数
nb_estimate = 1; % 我们知道分子阶数是1
[b_estimated, a_estimated] = my_invfreqs(H_true, w, nb_estimate);

disp('=== 真实系统系数 ===');
disp(['b_true: ', mat2str(b_true)]);
disp(['a_true: ', mat2str(a_true)]);

disp('=== 估计系统系数 ===');
disp(['b_estimated: ', mat2str(b_estimated)]);
disp(['a_estimated: ', mat2str(a_estimated)]);

% 5. (可选) 验证估计结果
% 绘制频率响应以比较
H_estimated = polyval(b_estimated, 1j*w) ./ polyval(a_estimated, 1j*w);

figure;
subplot(2,1,1);
plot(w, 20*log10(abs(H_true)), 'b', w, 20*log10(abs(H_estimated)), 'r--');
title('幅频响应');
xlabel('频率 (rad/s)');
ylabel('增益 (dB)');
legend('真实系统', '估计系统');
grid on;

subplot(2,1,2);
plot(w, rad2deg(angle(H_true)), 'b', w, rad2deg(angle(H_estimated)), 'r--');
title('相频响应');
xlabel('频率 (rad/s)');
ylabel('相位 (度)');
legend('真实系统', '估计系统');
grid on;