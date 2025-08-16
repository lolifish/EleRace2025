#include <math.h>
#include <stdint.h>
#include <stdio.h> // 仅用于在main函数中打印结果

// --- 用户可配置参数 ---
#define MAX_FREQ_POINTS 125 // 最大频率点数量

// --- 固定算法参数 ---
#define NB 2 // 分子阶数
#define NA 2 // 分母阶数
#define N_UNKNOWNS (NB + 1 + NA) // 未知系数总数 = 5


// --- 复数定义 ---
typedef struct {
    double real;
    double imag;
} complex_double;

// --- 全局常量 ---
static const complex_double ONE_COMPLEX = {1.0, 0.0};
static const complex_double ZERO_COMPLEX = {0.0, 0.0};

// --- 复数运算 (设为static，内部使用) ---
static complex_double complex_add(complex_double a, complex_double b) {
    return (complex_double){a.real + b.real, a.imag + b.imag};
}
static complex_double complex_sub(complex_double a, complex_double b) {
    return (complex_double){a.real - b.real, a.imag - b.imag};
}
static complex_double complex_mul(complex_double a, complex_double b) {
    return (complex_double){a.real * b.real - a.imag * b.imag, a.real * b.imag + a.imag * b.real};
}
static complex_double complex_div(complex_double a, complex_double b) {
    double den = b.real * b.real + b.imag * b.imag;
    if (den == 0.0) return ZERO_COMPLEX; // 简化处理
    return (complex_double){(a.real * b.real + a.imag * b.imag) / den, (a.imag * b.real - a.real * b.imag) / den};
}
static complex_double complex_conj(complex_double a) {
    return (complex_double){a.real, -a.imag};
}
static double complex_abs_sq(complex_double a) {
    return a.real * a.real + a.imag * a.imag;
}
static complex_double complex_power_int(complex_double base, int exp) {
    complex_double result = ONE_COMPLEX;
    for (int i = 0; i < exp; i++) {
        result = complex_mul(result, base);
    }
    return result;
}

// --- 简化的矩阵求逆 ---
static void complex_matrix_invert_gauss_jordan(complex_double* matrix, int N) {
    static complex_double augmented_matrix[N_UNKNOWNS * (2 * N_UNKNOWNS)];

    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
            augmented_matrix[i * (2 * N) + j] = matrix[i * N + j];
            augmented_matrix[i * (2 * N) + (j + N)] = ZERO_COMPLEX;
        }
        augmented_matrix[i * (2 * N) + (i + N)] = ONE_COMPLEX;
    }

    for (int i = 0; i < N; i++) {
        int pivot_row = i;
        double max_abs_sq = complex_abs_sq(augmented_matrix[i * (2 * N) + i]);
        for (int k = i + 1; k < N; k++) {
            double current_abs_sq = complex_abs_sq(augmented_matrix[k * (2 * N) + i]);
            if (current_abs_sq > max_abs_sq) {
                max_abs_sq = current_abs_sq;
                pivot_row = k;
            }
        }

        if (max_abs_sq < 1e-20) {
            // 矩阵奇异，直接返回，不进行任何操作
            return;
        }

        if (pivot_row != i) {
            for (int j = 0; j < 2 * N; j++) {
                complex_double temp = augmented_matrix[i * (2 * N) + j];
                augmented_matrix[i * (2 * N) + j] = augmented_matrix[pivot_row * (2 * N) + j];
                augmented_matrix[pivot_row * (2 * N) + j] = temp;
            }
        }

        complex_double pivot_val = augmented_matrix[i * (2 * N) + i];
        for (int j = i; j < 2 * N; j++) {
            augmented_matrix[i * (2 * N) + j] = complex_div(augmented_matrix[i * (2 * N) + j], pivot_val);
        }

        for (int k = 0; k < N; k++) {
            if (k != i) {
                complex_double factor = augmented_matrix[k * (2 * N) + i];
                for (int j = i; j < 2 * N; j++) {
                    augmented_matrix[k * (2 * N) + j] = complex_sub(augmented_matrix[k * (2 * N) + j], complex_mul(factor, augmented_matrix[i * (2 * N) + j]));
                }
            }
        }
    }

    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
            matrix[i * N + j] = augmented_matrix[i * (2 * N) + (j + N)];
        }
    }
}

/**
 * @brief 简化的 invfreqs 算法 (STM32版本)，无错误返回。
 */
void invfreqs_stm32(const complex_double *H, const double *w, int n_freq,
                               complex_double *b_coeffs, complex_double *a_coeffs) {
    
    static complex_double A[MAX_FREQ_POINTS * N_UNKNOWNS];
    static complex_double B_vec[MAX_FREQ_POINTS];
    static complex_double A_H[N_UNKNOWNS * MAX_FREQ_POINTS];
    static complex_double M[N_UNKNOWNS * N_UNKNOWNS];
    static complex_double V[N_UNKNOWNS];
    static complex_double M_inv[N_UNKNOWNS * N_UNKNOWNS];
    static complex_double x[N_UNKNOWNS];
    
    // 省略输入验证

    for (int k = 0; k < n_freq; k++) {
        complex_double jwk = {0.0, w[k]};
        
        A[k * N_UNKNOWNS + 0] = ONE_COMPLEX;
        A[k * N_UNKNOWNS + 1] = jwk;
        A[k * N_UNKNOWNS + 2] = complex_mul(jwk, jwk);

        A[k * N_UNKNOWNS + 3] = complex_mul( (complex_double){-H[k].real, -H[k].imag}, ONE_COMPLEX);
        A[k * N_UNKNOWNS + 4] = complex_mul( (complex_double){-H[k].real, -H[k].imag}, jwk);
        
        B_vec[k] = complex_mul(H[k], A[k * N_UNKNOWNS + 2]);
    }

    for (int i = 0; i < n_freq; i++) {
        for (int j = 0; j < N_UNKNOWNS; j++) {
            A_H[j * n_freq + i] = complex_conj(A[i * N_UNKNOWNS + j]);
        }
    }

    for (int i = 0; i < N_UNKNOWNS; i++) {
        for (int j = 0; j < N_UNKNOWNS; j++) {
            M[i*N_UNKNOWNS+j] = ZERO_COMPLEX;
            for (int k = 0; k < n_freq; k++) {
                M[i*N_UNKNOWNS+j] = complex_add(M[i*N_UNKNOWNS+j], complex_mul(A_H[i*n_freq+k], A[k*N_UNKNOWNS+j]));
            }
        }
    }

    for (int i = 0; i < N_UNKNOWNS; i++) {
        V[i] = ZERO_COMPLEX;
        for (int k = 0; k < n_freq; k++) {
            V[i] = complex_add(V[i], complex_mul(A_H[i*n_freq+k], B_vec[k]));
        }
    }

    for (int i = 0; i < N_UNKNOWNS*N_UNKNOWNS; i++) M_inv[i] = M[i];
    
    complex_matrix_invert_gauss_jordan(M_inv, N_UNKNOWNS);

    for (int i = 0; i < N_UNKNOWNS; i++) {
        x[i] = ZERO_COMPLEX;
        for (int k = 0; k < N_UNKNOWNS; k++) {
            x[i] = complex_add(x[i], complex_mul(M_inv[i*N_UNKNOWNS+k], V[k]));
        }
    }

    b_coeffs[0] = x[2]; b_coeffs[1] = x[1]; b_coeffs[2] = x[0];
    a_coeffs[0] = ONE_COMPLEX; a_coeffs[1] = x[4]; a_coeffs[2] = x[3];
}
