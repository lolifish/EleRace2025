#include "invfreqs.h" // 如果你将代码拆分成 .h 和 .c 文件
#include <stdio.h>    // For fprintf, stderr
#include <stdlib.h>   // For calloc, free

// --- 复数定义和操作 ---

// 预定义的复数常量 (必须在 .c 文件中定义)
const complex_double J_COMPLEX = {0.0, 1.0};    // 虚数单位 i 或 j
const complex_double ONE_COMPLEX = {1.0, 0.0};  // 复数 1
const complex_double ZERO_COMPLEX = {0.0, 0.0}; // 复数 0

// 从实数创建复数
complex_double complex_from_real(double r) {
    return (complex_double){r, 0.0};
}

// 从虚数创建复数
complex_double complex_from_imag(double i) {
    return (complex_double){0.0, i};
}

// 复数加法
complex_double complex_add(complex_double a, complex_double b) {
    return (complex_double){a.real + b.real, a.imag + b.imag};
}

// 复数减法
complex_double complex_sub(complex_double a, complex_double b) {
    return (complex_double){a.real - b.real, a.imag - b.imag};
}

// 复数乘法
complex_double complex_mul(complex_double a, complex_double b) {
    return (complex_double){a.real * b.real - a.imag * b.imag,
                           a.real * b.imag + a.imag * b.real};
}

// 复数除法
complex_double complex_div(complex_double a, complex_double b) {
    double den = b.real * b.real + b.imag * b.imag;
    if (den == 0.0) {
        fprintf(stderr, "Warning: Division by zero in complex_div.\n");
        return (complex_double){NAN, NAN}; // 返回非数字，指示错误
    }
    return (complex_double){(a.real * b.real + a.imag * b.imag) / den,
                           (a.imag * b.real - a.real * b.imag) / den};
}

// 复数共轭
complex_double complex_conj(complex_double a) {
    return (complex_double){a.real, -a.imag};
}

// 计算复数的整数次幂 (非负指数)
complex_double complex_power_int(complex_double base, int exp) {
    complex_double result = ONE_COMPLEX;
    if (exp < 0) {
        fprintf(stderr, "Error: Negative exponent in complex_power_int is not supported.\n");
        return (complex_double){NAN, NAN};
    }
    for (int i = 0; i < exp; i++) {
        result = complex_mul(result, base);
    }
    return result;
}

// 计算复数幅度的平方 (用于主元选择)
double complex_abs_sq(complex_double a) {
    return a.real * a.real + a.imag * a.imag;
}

// --- 矩阵操作 (为 N_UNKNOWNS x N_UNKNOWNS 的复数矩阵简化) ---

// 执行复数矩阵乘法 C = A * B
void complex_matrix_mul(const complex_double* A, int rows_A, int cols_A,
                        const complex_double* B, int rows_B, int cols_B,
                        complex_double* C) {
    if (cols_A != rows_B) {
        // 维度不匹配，此处应由调用方确保，或添加错误处理
        return;
    }
    for (int i = 0; i < rows_A; i++) {
        for (int j = 0; j < cols_B; j++) {
            C[i * cols_B + j] = ZERO_COMPLEX; // 初始化结果矩阵元素
            for (int k = 0; k < cols_A; k++) {
                C[i * cols_B + j] = complex_add(C[i * cols_B + j],
                                                complex_mul(A[i * cols_A + k], B[k * cols_B + j]));
            }
        }
    }
}

// 计算复数矩阵的共轭转置 B = A_H
void complex_matrix_conj_transpose(const complex_double* A, int rows_A, int cols_A,
                                   complex_double* B) {
    for (int i = 0; i < rows_A; i++) {
        for (int j = 0; j < cols_A; j++) {
            B[j * rows_A + i] = complex_conj(A[i * cols_A + j]);
        }
    }
}

// 使用高斯-约旦消元法和部分主元选择来求复数方阵的逆矩阵
int complex_matrix_invert_gauss_jordan(complex_double* matrix, int N) {
    complex_double *augmented_matrix = (complex_double*) calloc(N * (2 * N), sizeof(complex_double));
    if (!augmented_matrix) {
        fprintf(stderr, "Error: Memory allocation failed for augmented_matrix in matrix inversion.\n");
        return -1;
    }

    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
            augmented_matrix[i * (2 * N) + j] = matrix[i * N + j];
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

        if (max_abs_sq < 1e-15) { // 使用更小的 epsilon 来检查 double 类型的奇异性
            fprintf(stderr, "Error: Matrix is singular or nearly singular (pivot too small) during inversion.\n");
            free(augmented_matrix);
            return -1;
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
                    augmented_matrix[k * (2 * N) + j] = complex_sub(augmented_matrix[k * (2 * N) + j],
                                                                    complex_mul(factor, augmented_matrix[i * (2 * N) + j]));
                }
            }
        }
    }

    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
            matrix[i * N + j] = augmented_matrix[i * (2 * N) + (j + N)];
        }
    }

    free(augmented_matrix);
    return 0;
}

// --- 主 invfreqs 函数 ---

// 固定参数：根据用户要求 nb=2，以及 Python 源代码中的 na=2
#define NB 2 // 分子阶数 (Numerator order)
#define NA 2 // 分母阶数 (Denominator order)
// 未知系数的总数: (nb+1)个分子系数 + (na)个分母系数 = 3 + 2 = 5
#define N_UNKNOWNS (NB + 1 + NA) 

/**
 * @brief 实现 invfreqs 函数的 C 语言版本，适用于嵌入式系统 (如 STM32)。
 *        该函数找到一个数字滤波器的传递函数 B(s)/A(s) 的系数，使其最符合给定的频率响应。
 *        函数假定 NB=2 (分子阶数) 和 NA=2 (分母阶数)。
 *        最终得到的 B(s) 和 A(s) 多项式的系数按 s 的降幂排列。
 *        B(s) = b_coeffs[0]*s^NB + b_coeffs[1]*s^(NB-1) + ... + b_coeffs[NB]
 *        A(s) = a_coeffs[0]*s^NA + a_coeffs[1]*s^(NA-1) + ... + a_coeffs[NA]，其中 a_coeffs[0] 固定为 1。
 *
 * @param H 复数频率响应数组。
 * @param w 角频率数组 (弧度/秒)。
 * @param n_freq 频率点数量。必须至少为 N_UNKNOWNS (即 5)。
 * @param b_coeffs 用于输出分子系数的数组 (大小为 NB+1)。
 * @param a_coeffs 用于输出分母系数的数组 (大小为 NA+1)。
 * @return 0 表示成功，-1 表示错误 (数据不足，内存分配失败，奇异矩阵等)。
 */
int invfreqs(const complex_double *H, const double *w, int n_freq,
             complex_double *b_coeffs, complex_double *a_coeffs) {

    // --- 输入验证 ---
    if (n_freq < N_UNKNOWNS) {
        fprintf(stderr, "Error: Insufficient frequency points. For NB=%d and NA=%d, at least %d frequency points are required, but only %d were provided.\n", NB, NA, N_UNKNOWNS, n_freq);
        return -1;
    }
    if (!H || !w || !b_coeffs || !a_coeffs) {
        fprintf(stderr, "Error: Null input pointer provided to invfreqs.\n");
        return -1;
    }

    // --- 为系统矩阵 A 和 B 分配内存 ---
    // A 是 n_freq x N_UNKNOWNS 维度的矩阵
    complex_double *A = (complex_double*) calloc(n_freq * N_UNKNOWNS, sizeof(complex_double));
    // B 是 n_freq x 1 维度的向量
    complex_double *B_vec = (complex_double*) calloc(n_freq, sizeof(complex_double));

    if (!A || !B_vec) {
        fprintf(stderr, "Error: Memory allocation failed for A or B_vec in invfreqs.\n");
        free(A); free(B_vec); // 释放已成功分配的内存
        return -1;
    }

    // --- 填充 A 和 B 矩阵 (最小二乘系统: Ax = B) ---
    for (int k = 0; k < n_freq; k++) {
        double wk = w[k];
        complex_double Hk = H[k];
        complex_double jwk = complex_from_imag(wk); // 1j * wk

        // 填充 A 矩阵的分子系数部分 (对应 b_0, b_1, ..., b_NB)
        // A[k, j_idx] = (jwk)^j_idx
        for (int j_idx = 0; j_idx <= NB; j_idx++) {
            A[k * N_UNKNOWNS + j_idx] = complex_power_int(jwk, j_idx);
        }

        // 填充 A 矩阵的分母系数部分 (对应 a_1, ..., a_NA)
        // A[k, (NB + 1) + i_idx] = -Hk * (jwk)^i_idx
        for (int i_idx = 0; i_idx < NA; i_idx++) {
            A[k * N_UNKNOWNS + (NB + 1) + i_idx] = complex_mul(complex_from_real(-1.0),
                                                               complex_mul(Hk, complex_power_int(jwk, i_idx)));
        }

        // 填充右侧向量 B_vec[k] = Hk * (jwk)^NA
        B_vec[k] = complex_mul(Hk, complex_power_int(jwk, NA));
    }

    // --- 使用正规方程求解最小二乘问题: (A_H A) x = A_H B ---

    // 1. 分配并计算 A_H (N_UNKNOWNS x n_freq)
    complex_double *A_H = (complex_double*) calloc(N_UNKNOWNS * n_freq, sizeof(complex_double));
    if (!A_H) { free(A); free(B_vec); fprintf(stderr, "Error: Memory allocation failed for A_H.\n"); return -1; }
    complex_matrix_conj_transpose(A, n_freq, N_UNKNOWNS, A_H);

    // 2. 分配并计算 M = A_H A (N_UNKNOWNS x N_UNKNOWNS)
    complex_double *M = (complex_double*) calloc(N_UNKNOWNS * N_UNKNOWNS, sizeof(complex_double));
    if (!M) { free(A); free(B_vec); free(A_H); fprintf(stderr, "Error: Memory allocation failed for M.\n"); return -1; }
    complex_matrix_mul(A_H, N_UNKNOWNS, n_freq, A, n_freq, N_UNKNOWNS, M);

    // 3. 分配并计算 V = A_H B (N_UNKNOWNS x 1)
    complex_double *V = (complex_double*) calloc(N_UNKNOWNS, sizeof(complex_double));
    if (!V) { free(A); free(B_vec); free(A_H); free(M); fprintf(stderr, "Error: Memory allocation failed for V.\n"); return -1; }
    for (int i = 0; i < N_UNKNOWNS; i++) {
        V[i] = ZERO_COMPLEX;
        for (int k = 0; k < n_freq; k++) {
            V[i] = complex_add(V[i], complex_mul(A_H[i * n_freq + k], B_vec[k]));
        }
    }

    // 4. 求解 x: 为 M_inv 和 x 分配内存，然后求 M 的逆并乘以 V
    complex_double *M_inv = (complex_double*) calloc(N_UNKNOWNS * N_UNKNOWNS, sizeof(complex_double));
    complex_double *x = (complex_double*) calloc(N_UNKNOWNS, sizeof(complex_double));
    if (!M_inv || !x) {
        free(A); free(B_vec); free(A_H); free(M); free(V); free(M_inv); free(x);
        fprintf(stderr, "Error: Memory allocation failed for M_inv or x.\n");
        return -1;
    }

    // 复制 M 到 M_inv，因为 complex_matrix_invert_gauss_jordan 会原地修改矩阵
    for (int i = 0; i < N_UNKNOWNS * N_UNKNOWNS; i++) {
        M_inv[i] = M[i];
    }

    // 求 M 的逆
    if (complex_matrix_invert_gauss_jordan(M_inv, N_UNKNOWNS) != 0) {
        fprintf(stderr, "Error: Failed to invert matrix M (singular or memory error).\n");
        free(A); free(B_vec); free(A_H); free(M); free(V); free(M_inv); free(x);
        return -1;
    }

    // 计算 x = M_inv * V
    for (int i = 0; i < N_UNKNOWNS; i++) {
        x[i] = ZERO_COMPLEX;
        for (int k = 0; k < N_UNKNOWNS; k++) {
            x[i] = complex_add(x[i], complex_mul(M_inv[i * N_UNKNOWNS + k], V[k]));
        }
    }

    // --- 提取并将系数反转为标准降幂顺序 ---
    // x 中包含的顺序是 [b_0, b_1, b_2, a_1_unknown, a_2_unknown]
    // b_coeffs 降幂: b2, b1, b0 => x[2], x[1], x[0]
    b_coeffs[0] = x[2];
    b_coeffs[1] = x[1];
    b_coeffs[2] = x[0];

    // a_coeffs 降幂: a2, a1, a0 => 1, x[4], x[3]
    a_coeffs[0] = ONE_COMPLEX;
    a_coeffs[1] = x[4];
    a_coeffs[2] = x[3];

    // --- 释放分配的内存 ---
    free(A);
    free(B_vec);
    free(A_H);
    free(M);
    free(V);
    free(M_inv);
    free(x);

    return 0; // 成功
}