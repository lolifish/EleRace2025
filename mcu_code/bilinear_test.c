#include <stdio.h>
#include <math.h>   // For tanf and fabs

// 确保 M_PI 在需要时被定义
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// 将提供的 bilinear_transform_2nd_prewarped 函数复制到此处
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
        printf("警告: A[0] 太接近于零，可能导致不稳定或无效的归一化。\n");
    }
}

// 主函数，用于运行测试用例
int main() {
    float A[3], B[3]; // 输出数字滤波器系数

    printf("--- Bilinear Transform Test Script ---\n\n");

    // --- 测试用例 1: 无预失真的标准双线性变换 ---
    // 模拟滤波器: H(s) = 1 / (s + 1)  (一阶低通)
    // 对应系数: a0=0, a1=1, a2=1, b0=0, b1=0, b2=1
    // 采样周期 T = 1.0s
    // 预失真频率 f_match_hz = 0.0 (不使用预失真)
    printf("--- 测试用例 1: 无预失真 (Low-Pass, s+1) ---\n");
    float a0_1 = 0.0f, a1_1 = 1.0f, a2_1 = 1.0f;
    float b0_1 = 0.0f, b1_1 = 0.0f, b2_1 = 1.0f;
    float T_1 = 1.0f;
    float f_match_hz_1 = 0.0f;

    printf("输入模拟系数: a=[%.4f, %.4f, %.4f], b=[%.4f, %.4f, %.4f]\n", a0_1, a1_1, a2_1, b0_1, b1_1, b2_1);
    printf("采样周期 T = %.4f, 预失真频率 f_match_hz = %.4f Hz\n", T_1, f_match_hz_1);

    bilinear_transform_2nd_prewarped(a0_1, a1_1, a2_1, b0_1, b1_1, b2_1, T_1, f_match_hz_1, A, B);
    printf("输出数字系数 (归一化 A[0]=1):\n");
    printf("A: [%.6f, %.6f, %.6f]\n", A[0], A[1], A[2]);
    printf("B: [%.6f, %.6f, %.6f]\n\n", B[0], B[1], B[2]);

    // --- 测试用例 2: 带预失真的双线性变换 ---
    // 模拟滤波器: H(s) = 1 / (s^2 + s + 1) (二阶低通)
    // 对应系数: a0=1, a1=1, a2=1, b0=0, b1=0, b2=1
    // 采样周期 T = 0.1s (采样频率 10 Hz)
    // 预失真频率 f_match_hz = 1.0 Hz (匹配1Hz的响应)
    printf("--- 测试用例 2: 带预失真 (Low-Pass, s^2+s+1) ---\n");
    float a0_2 = 1.0f, a1_2 = 1.0f, a2_2 = 1.0f;
    float b0_2 = 0.0f, b1_2 = 0.0f, b2_2 = 1.0f;
    float T_2 = 0.1f;
    float f_match_hz_2 = 1.0f; // 1 Hz 预失真

    printf("输入模拟系数: a=[%.4f, %.4f, %.4f], b=[%.4f, %.4f, %.4f]\n", a0_2, a1_2, a2_2, b0_2, b1_2, b2_2);
    printf("采样周期 T = %.4f, 预失真频率 f_match_hz = %.4f Hz\n", T_2, f_match_hz_2);

    bilinear_transform_2nd_prewarped(a0_2, a1_2, a2_2, b0_2, b1_2, b2_2, T_2, f_match_hz_2, A, B);
    printf("输出数字系数 (归一化 A[0]=1):\n");
    printf("A: [%.6f, %.6f, %.6f]\n", A[0], A[1], A[2]);
    printf("B: [%.6f, %.6f, %.6f]\n\n", B[0], B[1], B[2]);

    // --- 测试用例 3: 不同的预失真频率和采样周期 ---
    // 模拟滤波器: H(s) = (s+1) / (s^2 + 2s + 1)
    // 对应系数: a0=1, a1=2, a2=1, b0=0, b1=1, b2=1
    // 采样周期 T = 0.01s (采样频率 100 Hz)
    // 预失真频率 f_match_hz = 5.0 Hz (匹配5Hz的响应)
    printf("--- 测试用例 3: 带预失真 (s+1)/(s^2+2s+1) ---\n");
    float a0_3 = 1.0f, a1_3 = 2.0f, a2_3 = 1.0f;
    float b0_3 = 0.0f, b1_3 = 1.0f, b2_3 = 1.0f;
    float T_3 = 0.01f;
    float f_match_hz_3 = 5.0f; // 5 Hz 预失真

    printf("输入模拟系数: a=[%.4f, %.4f, %.4f], b=[%.4f, %.4f, %.4f]\n", a0_3, a1_3, a2_3, b0_3, b1_3, b2_3);
    printf("采样周期 T = %.4f, 预失真频率 f_match_hz = %.4f Hz\n", T_3, f_match_hz_3);

    bilinear_transform_2nd_prewarped(a0_3, a1_3, a2_3, b0_3, b1_3, b2_3, T_3, f_match_hz_3, A, B);
    printf("输出数字系数 (归一化 A[0]=1):\n");
    printf("A: [%.6f, %.6f, %.6f]\n", A[0], A[1], A[2]);
    printf("B: [%.6f, %.6f, %.6f]\n\n", B[0], B[1], B[2]);

    printf("--- 测试结束 ---\n");

    return 0;
}