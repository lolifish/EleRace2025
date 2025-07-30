import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import freqs, TransferFunction

from invfreqs import invfreqs

# ====== 原始模拟滤波器：RLC带通滤波器 ======
# 例如 R=5kΩ, L=5mH, C=50nF
R = 5000
L = 5e-3
C = 50e-9

# 带通传递函数: H(s) = sRC / (s^2*LC + sRC + 1)
num = [R * C, 0]
den = [L * C, R * C, 1]

# 定义频率轴（rad/s）
w = np.logspace(2, 6, 512)  # 100 rad/s 到 1M rad/s
w_hz = w / (2 * np.pi)      # Hz单位（仅用于绘图）

# 原始频率响应
Hs = TransferFunction(num, den)
_, H_target = freqs(num, den, w)

# ====== 调用 invfreqs 进行拟合 ======
nb = 2  # 分子2阶
na = 2  # 分母2阶
b_fit, a_fit = invfreqs(H_target, w, nb, na)

# 拟合后响应
_, H_fit = freqs(b_fit, a_fit, w)

# ====== 绘图比较 ======
plt.figure(figsize=(10, 6))
plt.subplot(2, 1, 1)
plt.semilogx(w_hz, 20 * np.log10(abs(H_target)), label='原始 H(s)')
plt.semilogx(w_hz, 20 * np.log10(abs(H_fit)), '--', label='拟合 H(s)')
plt.ylabel('幅度 (dB)')
plt.grid(True)
plt.legend()

plt.subplot(2, 1, 2)
plt.semilogx(w_hz, np.angle(H_target, deg=True), label='原始 H(s)')
plt.semilogx(w_hz, np.angle(H_fit, deg=True), '--', label='拟合 H(s)')
plt.ylabel('相位 (deg)')
plt.xlabel('频率 (Hz)')
plt.grid(True)
plt.legend()

plt.suptitle("invfreqs 拟合测试（RLC 带通滤波器）")
plt.tight_layout()
plt.show()