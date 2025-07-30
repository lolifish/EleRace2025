import numpy as np
import scipy.signal as signal
import matplotlib.pyplot as plt

from invfreqz import invfreqz as invfreqz_rlc

# 采样频率和归一化频率
fs = 1  # 归一化采样率，方便演示
f0 = 0.3  # 中心频率（0~0.5）
Q = 10    # 品质因数，控制带宽

# 频率点，从0到pi弧度
w = np.linspace(0, np.pi, 1000)

# %% 1. 设计数字带通滤波器（BPF）
# 数字二阶带通滤波器设计公式（参考数字滤波经典公式）
# bw = f0 / Q  # 带宽 (Scipy的 iirpeak/iirnotch 直接接受 Q 值)

# 用 SciPy 函数设计带通 IIR滤波器 (biquad)
# SciPy 的 iirpeak 函数的 w0 参数是归一化频率，范围从0到1，其中1代表奈奎斯特频率（fs/2）。
# 由于 MATLAB 中的 f0 范围是 0 到 0.5 (即相对于 fs/2 归一化)，
# 所以在 SciPy 中需要将 f0 乘以 2，使其范围变为 0 到 1。
b_bpf, a_bpf = signal.iirpeak(2 * f0, Q)

# 计算频率响应
# freqz 返回 (w, h)，w 是角频率，h 是复数频率响应
w_bpf_rad, Hd_bpf = signal.freqz(b_bpf, a_bpf, worN=w)

# 用自定义invfreqz_rlc拟合
b_est_bpf, a_est_bpf = invfreqz_rlc(Hd_bpf, w)

# %% 2. 设计数字带阻滤波器（BSF）
# 用 SciPy 函数设计带阻 IIR滤波器 (biquad)
# 同样，w0 参数需要将 f0 乘以 2 进行归一化。
b_bsf, a_bsf = signal.iirnotch(2 * f0, Q)

# 计算频率响应
w_bsf_rad, Hd_bsf = signal.freqz(b_bsf, a_bsf, worN=w)

# 用自定义invfreqz_rlc拟合
b_est_bsf, a_est_bsf = invfreqz_rlc(Hd_bsf, w)

# %% 3. 画图对比带通滤波器
plt.figure(figsize=(12, 10)) # 设置图窗大小

plt.subplot(2, 2, 1)
# 绘制原始带通滤波器幅度响应
plt.plot(w_bpf_rad / np.pi, 20 * np.log10(np.abs(Hd_bpf)), 'b', label='原始')
# 绘制拟合带通滤波器幅度响应
# 注意：freqz 返回 (w, h)，我们只需要h (索引为1)
plt.plot(w_bpf_rad / np.pi, 20 * np.log10(np.abs(signal.freqz(b_est_bpf, a_est_bpf, worN=w)[1])), 'r--', label='拟合')
plt.title('带通滤波器幅度响应')
plt.xlabel('归一化频率 $\\times\\pi$') # LaTeX 风格的希腊字母
plt.ylabel('幅度 (dB)')
plt.legend()
plt.grid(True)

plt.subplot(2, 2, 3)
# 绘制原始带通滤波器相位响应
plt.plot(w_bpf_rad / np.pi, np.angle(Hd_bpf), 'b', label='原始')
# 绘制拟合带通滤波器相位响应
plt.plot(w_bpf_rad / np.pi, np.angle(signal.freqz(b_est_bpf, a_est_bpf, worN=w)[1]), 'r--', label='拟合')
plt.title('带通滤波器相位响应')
plt.xlabel('归一化频率 $\\times\\pi$')
plt.ylabel('相位 (rad)')
plt.legend()
plt.grid(True)

# %% 4. 画图对比带阻滤波器
plt.subplot(2, 2, 2)
# 绘制原始带阻滤波器幅度响应
plt.plot(w_bsf_rad / np.pi, 20 * np.log10(np.abs(Hd_bsf)), 'b', label='原始')
# 绘制拟合带阻滤波器幅度响应
plt.plot(w_bsf_rad / np.pi, 20 * np.log10(np.abs(signal.freqz(b_est_bsf, a_est_bsf, worN=w)[1])), 'r--', label='拟合')
plt.title('带阻滤波器幅度响应')
plt.xlabel('归一化频率 $\\times\\pi$')
plt.ylabel('幅度 (dB)')
plt.legend()
plt.grid(True)

plt.subplot(2, 2, 4)
# 绘制原始带阻滤波器相位响应
plt.plot(w_bsf_rad / np.pi, np.angle(Hd_bsf), 'b', label='原始')
# 绘制拟合带阻滤波器相位响应
plt.plot(w_bsf_rad / np.pi, np.angle(signal.freqz(b_est_bsf, a_est_bsf, worN=w)[1]), 'r--', label='拟合')
plt.title('带阻滤波器相位响应')
plt.xlabel('归一化频率 $\\times\\pi$')
plt.ylabel('相位 (rad)')
plt.legend()
plt.grid(True)

plt.tight_layout() # 自动调整子图参数，使之填充整个图像区域
plt.show() # 显示所有图窗