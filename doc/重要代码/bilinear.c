#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

void bilinear_transform_2nd_prewarped(
    float a0, float a1, float a2,
    float b0, float b1, float b2,
    float T,
    float f_match_hz,
    float *A, float *B)
{
    float K; 

    if (f_match_hz == 0.0f) {
        K = 2.0f / T;
    } else {
        // 预失真
        float omega_match_rad_per_s = 2.0f * M_PI * f_match_hz;
        K = omega_match_rad_per_s / tanf(omega_match_rad_per_s * T / 2.0f);
    }

    float K2 = K * K; // K的平方

    // 计算数字滤波器的分子系数 (B[0]*z^2 + B[1]*z + B[2])
    B[0] = b0 * K2 + b1 * K + b2;
    B[1] = -2.0f * b0 * K2 + 2.0f * b2;
    B[2] = b0 * K2 - b1 * K + b2;

    // 计算数字滤波器的分母系数 (A[0]*z^2 + A[1]*z + A[2])
    A[0] = a0 * K2 + a1 * K + a2;
    A[1] = -2.0f * a0 * K2 + 2.0f * a2;
    A[2] = a0 * K2 - a1 * K + a2;

    // 归一化系数，使A[0]=1
    float normalize_factor = A[0];
    A[0] /= normalize_factor;
    A[1] /= normalize_factor;
    A[2] /= normalize_factor;
    B[0] /= normalize_factor;
    B[1] /= normalize_factor;
    B[2] /= normalize_factor;
}