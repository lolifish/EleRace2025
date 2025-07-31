#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

void bilinear_transform_2nd_prewarped(
    float a0, float a1, float a2,
    float b0, float b1, float b2,
    float T,
    float f_match_hz, // 新增参数：预失真频率 (Hz)
    float *A, float *B)
{
    float K; // Bilinear变换中的常数 K

    // 根据是否需要预失真来计算 K 值
    if (f_match_hz == 0.0f) {
        // 标准双线性变换：s = (2/T) * (z-1)/(z+1)
        K = 2.0f / T;
    } else {
        // 预失真：使用 K = Omega_p / tan(Omega_p * T / 2)
        // 其中 Omega_p 是需要匹配的模拟角频率 (rad/s)
        float omega_match_rad_per_s = 2.0f * M_PI * f_match_hz;
        K = omega_match_rad_per_s / tanf(omega_match_rad_per_s * T / 2.0f);
    }

    float K2 = K * K; // K的平方，用于简化计算

    // 计算数字滤波器的分子系数 (B[0]*z^2 + B[1]*z + B[2])
    B[0] = b0 * K2 + b1 * K + b2;
    B[1] = -2.0f * b0 * K2 + 2.0f * b2;
    B[2] = b0 * K2 - b1 * K + b2;

    // 计算数字滤波器的分母系数 (A[0]*z^2 + A[1]*z + A[2])
    A[0] = a0 * K2 + a1 * K + a2;
    A[1] = -2.0f * a0 * K2 + 2.0f * a2;
    A[2] = a0 * K2 - a1 * K + a2;

    // 归一化系数，使 A[0] = 1，这是数字IIR滤波器实现的标准形式
    // 避免除以零或非常小的数
    if (fabs(A[0]) > 1e-9) { 
        float normalize_factor = A[0];
        A[0] /= normalize_factor;
        A[1] /= normalize_factor;
        A[2] /= normalize_factor;
        B[0] /= normalize_factor;
        B[1] /= normalize_factor;
        B[2] /= normalize_factor;
    } else {
        // 处理 A[0] 为零的情况，这通常不应该发生对于一个稳定的二阶滤波器
        // 实际应用中可能需要错误处理或警告
    }
}