import numpy as np
import matplotlib.pyplot as plt

# (my_invfreqs_2nd_order_fixed 函数的代码不变，此处省略，假定已定义)
# 为了代码的完整性，我在此处再次包含该函数。
def invfreqs(H, w, nb):
    na = 2
    n_freq = len(w)
    n_unknowns = (nb + 1) + na

    if n_freq < n_unknowns:
        raise ValueError(
            f'Insufficient frequency points. For the specified numerator order ({nb}) '
            f'and 2nd order denominator, at least {n_unknowns} frequency points are required, '
            f'but only {n_freq} were provided.'
        )

    A = np.zeros((n_freq, n_unknowns), dtype=complex)
    B = np.zeros(n_freq, dtype=complex)

    for k in range(n_freq):
        wk = w[k]
        Hk = H[k]
        jwk = 1j * wk
        
        for j_idx in range(nb + 1):
            A[k, j_idx] = jwk**j_idx

        for i_idx in range(na):
            A[k, (nb + 1) + i_idx] = -Hk * (jwk**i_idx)

        B[k] = Hk * (jwk**na)

    x = np.linalg.lstsq(A, B, rcond=None)[0]

    b_coeffs_temp = x[0 : (nb + 1)]
    a_coeffs_remaining = x[(nb + 1) : ]
    a_coeffs_ascending = np.concatenate((a_coeffs_remaining, [1]))
    
    b_coeffs = np.flipud(b_coeffs_temp)
    a_coeffs = np.flipud(a_coeffs_ascending)

    return b_coeffs, a_coeffs


if __name__ == '__main__':
    # 1. 定义一个真实的2阶分子和2阶分母系统
    # Transfer function: H(s) = (1s^2 + 0.5s + 10) / (1s^2 + 2s + 20)
    b_true = np.array([0,0,5])  # Numerator coefficients [b_2, b_1, b_0]
    a_true = np.array([10**-8, 3*10**-4, 1])    # Denominator coefficients [a_2, a_1, a_0]

    # 2. 生成频率点 (angular frequency in rad/s)
    # Start from a small non-zero value for log scale
    w = np.logspace(np.log10(100), np.log10(1000000), 1000) # Frequencies from 1 to 100 rad/s (log scale)

    # 3. Get the frequency response of the true system
    H_true = np.polyval(b_true, 1j * w) / np.polyval(a_true, 1j * w)

    # 4. Use our custom function to estimate coefficients, set numerator order to 2
    nb_estimate = 2  # Specify numerator order as 2
    b_estimated, a_estimated = my_invfreqs_2nd_order_fixed(H_true, w, nb_estimate)

    print('=== True System Coefficients ===')
    print(f'b_true: {b_true}')
    print(f'a_true: {a_true}')

    print('\n=== Estimated System Coefficients ===')
    # Print real parts, as float operations might introduce tiny imaginary parts
    print(f'b_estimated: {b_estimated.real}')
    print(f'a_estimated: {a_estimated.real}')

    # 5. Validate the estimated results and plot
    H_estimated = np.polyval(b_estimated, 1j * w) / np.polyval(a_estimated, 1j * w)

    # Convert angular frequency (rad/s) to frequency (Hz) for plotting
    f_hz = w / (2 * np.pi)

    plt.figure(figsize=(10, 8))

    # Magnitude Plot
    plt.subplot(2, 1, 1)
    plt.semilogx(f_hz, 20 * np.log10(np.abs(H_true)), 'b', label='True System')
    plt.semilogx(f_hz, 20 * np.log10(np.abs(H_estimated)), 'r--', label='Estimated System')
    plt.title('Magnitude Response')
    plt.xlabel('Frequency (Hz)')
    plt.ylabel('Gain (dB)')
    plt.legend()
    plt.grid(True, which="both", ls="-") # Add grid for both major and minor ticks

    # Phase Plot
    plt.subplot(2, 1, 2)
    plt.semilogx(f_hz, np.degrees(np.angle(H_true)), 'b', label='True System')
    plt.semilogx(f_hz, np.degrees(np.angle(H_estimated)), 'r--', label='Estimated System')
    plt.title('Phase Response')
    plt.xlabel('Frequency (Hz)')
    plt.ylabel('Phase (degrees)')
    plt.legend()
    plt.grid(True, which="both", ls="-") # Add grid for both major and minor ticks

    plt.tight_layout()
    plt.show()