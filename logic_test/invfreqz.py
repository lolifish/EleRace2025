import numpy as np
import matplotlib.pyplot as plt
import scipy.signal as signal # Still needed for freqz function to calculate the frequency response of the fitted digital filters

# =========================================================================
# Custom invfreqz function provided by the user
# =========================================================================
def invfreqz(Hd, w):
    """
    Hd: Target frequency response (complex array)
    w: Frequency points (in rad/sample)

    Returns:
    b: Numerator coefficients array [b0, b1, b2]
    a: Denominator coefficients array [1, a1, a2]
    """
    Hd = np.asarray(Hd).flatten()
    w = np.asarray(w).flatten()

    N = len(w)
    z = np.exp(1j * w)  # z = e^(j*w)

    # A matrix for solving b0, b1, b2, a1, a2
    # The equation is: H(z) = (b0 + b1*z^-1 + b2*z^-2) / (1 + a1*z^-1 + a2*z^-2)
    # Rearranging: H(z)*(1 + a1*z^-1 + a2*z^-2) = b0 + b1*z^-1 + b2*z^-2
    # H(z) = b0*z^0 + b1*z^-1 + b2*z^-2 - H(z)*a1*z^-1 - H(z)*a2*z^-2
    # This is effectively H = [1 z^-1 z^-2 -H*z^-1 -H*z^-2] * [b0 b1 b2 a1 a2]^T
    A = np.zeros((N, 5), dtype=complex)
    y = Hd # This is the target H(z)

    for k in range(N):
        zk = z[k]
        A[k, 0] = 1            # Coefficient for b0
        A[k, 1] = 1 / zk       # Coefficient for b1
        A[k, 2] = 1 / (zk**2)  # Coefficient for b2
        A[k, 3] = -y[k] / zk   # Coefficient for a1 (note the negative sign because a_terms are moved to LHS)
        A[k, 4] = -y[k] / (zk**2) # Coefficient for a2

    # Solve the linear system A * x = y using least squares
    # x will contain [b0, b1, b2, a1, a2]
    # IMPORTANT: Added rcond to handle potential ill-conditioning
    x, residuals, rank, s = np.linalg.lstsq(A, y, rcond=1e-10) # Added rcond parameter

    # Extract and return coefficients, ensuring they are real
    b = np.real(x[0:3]) # b0, b1, b2
    a = np.concatenate(([1], np.real(x[3:5]))) # 1, a1, a2

    return b, a

# --- RLC Component Values ---
# Choose specific R, L, C values within the given ranges for demonstration
R_val = 5e3   # 5 kΩ (Range: 1kΩ ~ 10kΩ)
L_val = 5e-3  # 5 mH (Range: 1mH ~ 10mH)
C_val = 50e-9 # 50 nF (Range: 10nF ~ 100nF)

# Print chosen component values and estimated resonant frequency
f_resonant = 1 / (2 * np.pi * np.sqrt(L_val * C_val))
print(f"Chosen RLC Component Values: R={R_val/1e3:.1f} kΩ, L={L_val*1e3:.1f} mH, C={C_val*1e9:.1f} nF")
print(f"Approximate Resonant Frequency for this RLC combination: {f_resonant/1e3:.2f} kHz")

# --- Analog Frequency Range Definition ---
# Generate frequency points (Hz) for analog circuit analysis.
# Use a logarithmic scale centered around the resonant frequency for better visualization.
f_analog_display = np.logspace(np.log10(f_resonant / 10), np.log10(f_resonant * 10), 512)
w_analog_display = 2 * np.pi * f_analog_display # Convert to angular frequency (rad/s)

# --- 1. Calculate Analog Bandpass Filter (BPF) Response ---
# Configuration: Series RLC circuit, output voltage measured across R.
# Vin ---- R ---- L ---- C ---- Ground
#        |      |
#        ---- Vout ----
# Transfer Function H(jw) = Vout / Vin = R / (R + jwL + 1/(jwC))

Z_R_analog = R_val
Z_L_analog = 1j * w_analog_display * L_val
Z_C_analog = 1 / (1j * w_analog_display * C_val)

Z_total_series_analog = Z_R_analog + Z_L_analog + Z_C_analog
Hd_bpf_analog = Z_R_analog / Z_total_series_analog

# --- 2. Calculate Analog Bandstop Filter (BSF) / Notch Filter Response ---
# Configuration: Series RLC circuit, output voltage measured across L and C in series.
# Vin ---- R ---- L ---- C ---- Ground
#              |      |
#              ---- Vout ----
# Transfer Function H_notch(jw) = Vout / Vin = (jwL + 1/(jwC)) / (R + jwL + 1/(jwC))

Z_LC_series_analog = Z_L_analog + Z_C_analog
Hd_bsf_analog = Z_LC_series_analog / Z_total_series_analog

# --- 3. Use custom invfreqz to find equivalent digital systems ---

# Conceptual sampling frequency for the target digital system.
# This choice is crucial as it defines the mapping from analog to digital frequencies.
# Let's choose fs_digital_system such that the resonant frequency (f_resonant)
# maps to a normalized digital frequency around 0.3 (similar to f0 in original MATLAB code).
# Normalized digital frequency = f_analog / (fs_digital_system / 2)
# So, f_resonant / (fs_digital_system / 2) = 0.3 => fs_digital_system = f_resonant / 0.15
fs_digital_system = f_resonant / 0.15
print(f"Conceptual Digital System Sampling Frequency for invfreqz: {fs_digital_system:.2f} Hz")

# Generate normalized digital angular frequencies (0 to pi) for invfreqz input.
# IMPORTANT: Adjusted range to avoid 0 and pi exactly, which can cause numerical issues with tan mapping.
w_digital_for_invfreqz = np.linspace(1e-6, np.pi - 1e-6, 512) # Adjusted range

# Map these digital angular frequencies back to analog frequencies (using inverse bilinear transform relation)
# to sample the analog response appropriately for invfreqz.
# The relationship for analog frequency 'f_a' and digital angular frequency 'w_d'
# via bilinear transform is f_a = (fs / pi) * tan(w_d / 2).
f_analog_mapped_for_invfreqz = (fs_digital_system / np.pi) * np.tan(w_digital_for_invfreqz / 2)

# Calculate the analog responses at these *mapped* analog frequencies.
# This gives us the discrete frequency response data points for invfreqz.
# We need to re-calculate Z_L, Z_C, Z_total_series, Z_LC_series using these new analog frequencies.
Z_R_mapped = R_val
Z_L_mapped = 1j * (2 * np.pi * f_analog_mapped_for_invfreqz) * L_val
Z_C_mapped = 1 / (1j * (2 * np.pi * f_analog_mapped_for_invfreqz) * C_val)

Z_total_series_mapped = Z_R_mapped + Z_L_mapped + Z_C_mapped
# Fix from previous error: Define Z_LC_series_mapped here
Z_LC_series_mapped = Z_L_mapped + Z_C_mapped

Hd_bpf_analog_sampled = Z_R_mapped / Z_total_series_mapped
Hd_bsf_analog_sampled = Z_LC_series_mapped / Z_total_series_mapped

# Use the custom invfreqz to estimate digital filter coefficients
b_est_bpf, a_est_bpf = invfreqz(Hd_bpf_analog_sampled, w_digital_for_invfreqz)
b_est_bsf, a_est_bsf = invfreqz(Hd_bsf_analog_sampled, w_digital_for_invfreqz)

# Calculate frequency response of the estimated digital filters
# Use the same digital angular frequencies as used for invfreqz.
w_digital_response, Hd_bpf_est = signal.freqz(b_est_bpf, a_est_bpf, worN=w_digital_for_invfreqz)
w_digital_response, Hd_bsf_est = signal.freqz(b_est_bsf, a_est_bsf, worN=w_digital_for_invfreqz)

# --- 4. Plotting (All English) ---
plt.figure(figsize=(12, 10)) # Set figure size

# --- Plot Bandpass Filter ---
plt.subplot(2, 2, 1)
# Plot Analog BPF Magnitude Response
plt.semilogx(f_analog_display, 20 * np.log10(np.abs(Hd_bpf_analog)), 'b', label='Analog RLC (Original)')
# Plot Estimated Digital BPF Magnitude Response.
# The x-axis for the digital filter response needs to be transformed back to analog Hz
# using the same mapping as used for invfreqz input, for proper comparison.
plt.semilogx(f_analog_mapped_for_invfreqz, 20 * np.log10(np.abs(Hd_bpf_est)), 'r--', label='Digital (Estimated by invfreqz)')
plt.title('Bandpass Filter Magnitude Response')
plt.xlabel('Frequency (Hz)')
plt.ylabel('Magnitude (dB)')
plt.legend()
plt.grid(True, which="both", ls="-")

plt.subplot(2, 2, 3)
# Plot Analog BPF Phase Response
plt.semilogx(f_analog_display, np.angle(Hd_bpf_analog, deg=True), 'b', label='Analog RLC (Original)')
# Plot Estimated Digital BPF Phase Response
plt.semilogx(f_analog_mapped_for_invfreqz, np.angle(Hd_bpf_est, deg=True), 'r--', label='Digital (Estimated by invfreqz)')
plt.title('Bandpass Filter Phase Response')
plt.xlabel('Frequency (Hz)')
plt.ylabel('Phase (degrees)')
plt.legend()
plt.grid(True, which="both", ls="-")

# --- Plot Bandstop Filter ---
plt.subplot(2, 2, 2)
# Plot Analog BSF Magnitude Response
plt.semilogx(f_analog_display, 20 * np.log10(np.abs(Hd_bsf_analog)), 'b', label='Analog RLC (Original)')
# Plot Estimated Digital BSF Magnitude Response
plt.semilogx(f_analog_mapped_for_invfreqz, 20 * np.log10(np.abs(Hd_bsf_est)), 'r--', label='Digital (Estimated by invfreqz)')
plt.title('Bandstop Filter Magnitude Response')
plt.xlabel('Frequency (Hz)')
plt.ylabel('Magnitude (dB)')
plt.legend()
plt.grid(True, which="both", ls="-")

plt.subplot(2, 2, 4)
# Plot Analog BSF Phase Response
plt.semilogx(f_analog_display, np.angle(Hd_bsf_analog, deg=True), 'b', label='Analog RLC (Original)')
# Plot Estimated Digital BSF Phase Response
plt.semilogx(f_analog_mapped_for_invfreqz, np.angle(Hd_bsf_est, deg=True), 'r--', label='Digital (Estimated by invfreqz)')
plt.title('Bandstop Filter Phase Response')
plt.xlabel('Frequency (Hz)')
plt.ylabel('Phase (degrees)')
plt.legend()
plt.grid(True, which="both", ls="-")

plt.tight_layout() # Adjust subplot parameters for a tight layout
plt.show() # Display all figures