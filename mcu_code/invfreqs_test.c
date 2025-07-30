#include <stdio.h>
#include <stdlib.h>
#include <math.h> // For NAN, fabs (for double)

// 为了方便测试，这里直接将 invfreqs 的代码放在 test 文件之前。
// 在实际项目中，应 #include "invfreqs.h" 并链接 invfreqs.c。

// --- 复数定义和操作 (复制自 invfreqs.c，并改为 double) ---
typedef struct {
    double real;
    double imag;
} complex_double;

const complex_double J_COMPLEX = {0.0, 1.0};
const complex_double ONE_COMPLEX = {1.0, 0.0};
const complex_double ZERO_COMPLEX = {0.0, 0.0};

complex_double complex_from_real(double r) { return (complex_double){r, 0.0}; }
complex_double complex_from_imag(double i) { return (complex_double){0.0, i}; }
complex_double complex_add(complex_double a, complex_double b) { return (complex_double){a.real + b.real, a.imag + b.imag}; }
complex_double complex_sub(complex_double a, complex_double b) { return (complex_double){a.real - b.real, a.imag - b.imag}; }
complex_double complex_mul(complex_double a, complex_double b) { return (complex_double){a.real * b.real - a.imag * b.imag, a.real * b.imag + a.imag * b.real}; }
complex_double complex_div(complex_double a, complex_double b) {
    double den = b.real * b.real + b.imag * b.imag;
    if (den == 0.0) { fprintf(stderr, "Warning: Division by zero in complex_div.\n"); return (complex_double){NAN, NAN}; }
    return (complex_double){(a.real * b.real + a.imag * b.imag) / den, (a.imag * b.real - a.real * b.imag) / den};
}
complex_double complex_conj(complex_double a) { return (complex_double){a.real, -a.imag}; }
complex_double complex_power_int(complex_double base, int exp) {
    complex_double result = ONE_COMPLEX;
    if (exp < 0) { fprintf(stderr, "Error: Negative exponent in complex_power_int is not supported.\n"); return (complex_double){NAN, NAN}; }
    for (int i = 0; i < exp; i++) { result = complex_mul(result, base); }
    return result;
}
double complex_abs_sq(complex_double a) { return a.real * a.real + a.imag * a.imag; }






// --- 矩阵操作 (复制自 invfreqs.c，并改为 double) ---
void complex_matrix_mul(const complex_double* A, int rows_A, int cols_A, const complex_double* B, int rows_B, int cols_B, complex_double* C) {
    if (cols_A != rows_B) { return; }
    for (int i = 0; i < rows_A; i++) { for (int j = 0; j < cols_B; j++) { C[i * cols_B + j] = ZERO_COMPLEX; for (int k = 0; k < cols_A; k++) { C[i * cols_B + j] = complex_add(C[i * cols_B + j], complex_mul(A[i * cols_A + k], B[k * cols_B + j])); } } }
}
void complex_matrix_conj_transpose(const complex_double* A, int rows_A, int cols_A, complex_double* B) {
    for (int i = 0; i < rows_A; i++) { for (int j = 0; j < cols_A; j++) { B[j * rows_A + i] = complex_conj(A[i * cols_A + j]); } }
}
int complex_matrix_invert_gauss_jordan(complex_double* matrix, int N) {
    complex_double *augmented_matrix = (complex_double*) calloc(N * (2 * N), sizeof(complex_double));
    if (!augmented_matrix) { fprintf(stderr, "Error: Memory allocation failed for augmented_matrix in matrix inversion.\n"); return -1; }
    for (int i = 0; i < N; i++) { for (int j = 0; j < N; j++) { augmented_matrix[i * (2 * N) + j] = matrix[i * N + j]; } augmented_matrix[i * (2 * N) + (i + N)] = ONE_COMPLEX; }
    for (int i = 0; i < N; i++) {
        int pivot_row = i; double max_abs_sq = complex_abs_sq(augmented_matrix[i * (2 * N) + i]);
        for (int k = i + 1; k < N; k++) { double current_abs_sq = complex_abs_sq(augmented_matrix[k * (2 * N) + i]); if (current_abs_sq > max_abs_sq) { max_abs_sq = current_abs_sq; pivot_row = k; } }
        if (max_abs_sq < 1e-15) { fprintf(stderr, "Error: Matrix is singular or nearly singular (pivot too small) during inversion.\n"); free(augmented_matrix); return -1; }
        if (pivot_row != i) { for (int j = 0; j < 2 * N; j++) { complex_double temp = augmented_matrix[i * (2 * N) + j]; augmented_matrix[i * (2 * N) + j] = augmented_matrix[pivot_row * (2 * N) + j]; augmented_matrix[pivot_row * (2 * N) + j] = temp; } }
        complex_double pivot_val = augmented_matrix[i * (2 * N) + i];
        for (int j = i; j < 2 * N; j++) { augmented_matrix[i * (2 * N) + j] = complex_div(augmented_matrix[i * (2 * N) + j], pivot_val); }
        for (int k = 0; k < N; k++) { if (k != i) { complex_double factor = augmented_matrix[k * (2 * N) + i]; for (int j = i; j < 2 * N; j++) { augmented_matrix[k * (2 * N) + j] = complex_sub(augmented_matrix[k * (2 * N) + j], complex_mul(factor, augmented_matrix[i * (2 * N) + j])); } } }
    }
    for (int i = 0; i < N; i++) { for (int j = 0; j < N; j++) { matrix[i * N + j] = augmented_matrix[i * (2 * N) + (j + N)]; } }
    free(augmented_matrix); return 0;
}





// --- 主 invfreqs 函数 (复制自 invfreqs.c，并改为 double) ---
#define NB 2
#define NA 2
#define N_UNKNOWNS (NB + 1 + NA)
int invfreqs(const complex_double *H, const double *w, int n_freq,
             complex_double *b_coeffs, complex_double *a_coeffs) {
    if (n_freq < N_UNKNOWNS) { fprintf(stderr, "Error: Insufficient frequency points. For NB=%d and NA=%d, at least %d frequency points are required, but only %d were provided.\n", NB, NA, N_UNKNOWNS, n_freq); return -1; }
    if (!H || !w || !b_coeffs || !a_coeffs) { fprintf(stderr, "Error: Null input pointer provided to invfreqs.\n"); return -1; }
    complex_double *A = (complex_double*) calloc(n_freq * N_UNKNOWNS, sizeof(complex_double));
    complex_double *B_vec = (complex_double*) calloc(n_freq, sizeof(complex_double));
    if (!A || !B_vec) { fprintf(stderr, "Error: Memory allocation failed for A or B_vec in invfreqs.\n"); free(A); free(B_vec); return -1; }
    for (int k = 0; k < n_freq; k++) {
        double wk = w[k]; complex_double Hk = H[k]; complex_double jwk = complex_from_imag(wk);
        for (int j_idx = 0; j_idx <= NB; j_idx++) { A[k * N_UNKNOWNS + j_idx] = complex_power_int(jwk, j_idx); }
        for (int i_idx = 0; i_idx < NA; i_idx++) { A[k * N_UNKNOWNS + (NB + 1) + i_idx] = complex_mul(complex_from_real(-1.0), complex_mul(Hk, complex_power_int(jwk, i_idx))); }
        B_vec[k] = complex_mul(Hk, complex_power_int(jwk, NA));
    }
    complex_double *A_H = (complex_double*) calloc(N_UNKNOWNS * n_freq, sizeof(complex_double));
    if (!A_H) { free(A); free(B_vec); fprintf(stderr, "Error: Memory allocation failed for A_H.\n"); return -1; }
    complex_matrix_conj_transpose(A, n_freq, N_UNKNOWNS, A_H);
    complex_double *M = (complex_double*) calloc(N_UNKNOWNS * N_UNKNOWNS, sizeof(complex_double));
    if (!M) { free(A); free(B_vec); free(A_H); fprintf(stderr, "Error: Memory allocation failed for M.\n"); return -1; }
    complex_matrix_mul(A_H, N_UNKNOWNS, n_freq, A, n_freq, N_UNKNOWNS, M);
    complex_double *V = (complex_double*) calloc(N_UNKNOWNS, sizeof(complex_double));
    if (!V) { free(A); free(B_vec); free(A_H); free(M); fprintf(stderr, "Error: Memory allocation failed for V.\n"); return -1; }
    for (int i = 0; i < N_UNKNOWNS; i++) { V[i] = ZERO_COMPLEX; for (int k = 0; k < n_freq; k++) { V[i] = complex_add(V[i], complex_mul(A_H[i * n_freq + k], B_vec[k])); } }
    complex_double *M_inv = (complex_double*) calloc(N_UNKNOWNS * N_UNKNOWNS, sizeof(complex_double));
    complex_double *x = (complex_double*) calloc(N_UNKNOWNS, sizeof(complex_double));
    if (!M_inv || !x) { free(A); free(B_vec); free(A_H); free(M); free(V); free(M_inv); free(x); fprintf(stderr, "Error: Memory allocation failed for M_inv or x.\n"); return -1; }
    for (int i = 0; i < N_UNKNOWNS * N_UNKNOWNS; i++) { M_inv[i] = M[i]; }
    if (complex_matrix_invert_gauss_jordan(M_inv, N_UNKNOWNS) != 0) { fprintf(stderr, "Error: Failed to invert matrix M (singular or memory error).\n"); free(A); free(B_vec); free(A_H); free(M); free(V); free(M_inv); free(x); return -1; }
    for (int i = 0; i < N_UNKNOWNS; i++) { x[i] = ZERO_COMPLEX; for (int k = 0; k < N_UNKNOWNS; k++) { x[i] = complex_add(x[i], complex_mul(M_inv[i * N_UNKNOWNS + k], V[k])); } }
    b_coeffs[0] = x[2]; b_coeffs[1] = x[1]; b_coeffs[2] = x[0];
    a_coeffs[0] = ONE_COMPLEX; a_coeffs[1] = x[4]; a_coeffs[2] = x[3];
    free(A); free(B_vec); free(A_H); free(M); free(V); free(M_inv); free(x);
    return 0;
}








// --- 测试主函数 ---

// 打印复数
void print_complex(const char* label, complex_double c) {
    printf("%s: %.12f + %.12fi\n", label, c.real, c.imag); // 更高精度输出
}

// 打印系数数组
void print_coeffs(const char* label, const complex_double* coeffs, int count) {
    printf("%s [descending powers of s]:\n", label);
    for (int i = 0; i < count; i++) {
        printf("  coeff[%d]: %.12f + %.12fi\n", i, coeffs[i].real, coeffs[i].imag); // 更高精度输出
    }
}

// 定义一个小的浮点数比较容差 (针对 double)
#define EPSILON 1e-9 // 针对 double 精度，使用更小的容差

int main() {
    // 定义参考的传递函数系数 (降幂排列)
    // B(s) = 1*s^2 + 3*s + 2
    complex_double true_b_coeffs[NB + 1] = {
        complex_from_real(1.0),
        complex_from_real(3.0),
        complex_from_real(2.0)
    };

    // A(s) = 1*s^2 + 0.8*s + 0.1
    complex_double true_a_coeffs[NA + 1] = {
        complex_from_real(1.0),
        complex_from_real(0.8),
        complex_from_real(0.1)
    };

    // 设置频率点数量 (需要大于等于 N_UNKNOWNS=5)
    int n_freq = 20; // 更多的频率点通常能得到更好的拟合

    // 分配内存
    double *w = (double*) calloc(n_freq, sizeof(double));
    complex_double *H = (complex_double*) calloc(n_freq, sizeof(complex_double));
    complex_double *b_coeffs_out = (complex_double*) calloc(NB + 1, sizeof(complex_double));
    complex_double *a_coeffs_out = (complex_double*) calloc(NA + 1, sizeof(complex_double));

    if (!w || !H || !b_coeffs_out || !a_coeffs_out) {
        fprintf(stderr, "Error: Memory allocation failed in main.\n");
        return 1;
    }

    // 生成频率点和对应的频率响应
    // 频率范围从 0.1 rad/s 到 10 rad/s
    double w_start = 0.1;
    double w_end = 10.0;
    for (int k = 0; k < n_freq; k++) {
        w[k] = w_start + (double)k / (n_freq - 1) * (w_end - w_start);

        complex_double jwk = complex_from_imag(w[k]);

        // 计算 B(jwk)
        complex_double numerator_val = complex_add(
            complex_add(complex_mul(true_b_coeffs[0], complex_power_int(jwk, 2)),
                        complex_mul(true_b_coeffs[1], complex_power_int(jwk, 1))),
            complex_mul(true_b_coeffs[2], complex_power_int(jwk, 0)));

        // 计算 A(jwk)
        complex_double denominator_val = complex_add(
            complex_add(complex_mul(true_a_coeffs[0], complex_power_int(jwk, 2)),
                        complex_mul(true_a_coeffs[1], complex_power_int(jwk, 1))),
            complex_mul(true_a_coeffs[2], complex_power_int(jwk, 0)));

        H[k] = complex_div(numerator_val, denominator_val);
    }

    printf("--- Calling invfreqs ---\n");
    int result = invfreqs(H, w, n_freq, b_coeffs_out, a_coeffs_out);

    if (result == 0) {
        printf("\n--- invfreqs Succeeded ---\n");

        printf("\nOriginal B coefficients:\n");
        print_coeffs("  True B", true_b_coeffs, NB + 1);
        printf("\nCalculated B coefficients:\n");
        print_coeffs("  Found B", b_coeffs_out, NB + 1);

        printf("\nOriginal A coefficients:\n");
        print_coeffs("  True A", true_a_coeffs, NA + 1);
        printf("\nCalculated A coefficients:\n");
        print_coeffs("  Found A", a_coeffs_out, NA + 1);

        // --- 验证结果 ---
        printf("\n--- Verification ---\n");
        int success = 1;

        // 验证 B 系数
        for (int i = 0; i <= NB; i++) {
            if (fabs(true_b_coeffs[i].real - b_coeffs_out[i].real) > EPSILON ||
                fabs(true_b_coeffs[i].imag - b_coeffs_out[i].imag) > EPSILON) {
                printf("Mismatch in B[%d]! True: %.10f+%.10fi, Found: %.10f+%.10fi (Diff: %.10f+%.10fi)\n",
                       i, true_b_coeffs[i].real, true_b_coeffs[i].imag,
                       b_coeffs_out[i].real, b_coeffs_out[i].imag,
                       fabs(true_b_coeffs[i].real - b_coeffs_out[i].real),
                       fabs(true_b_coeffs[i].imag - b_coeffs_out[i].imag));
                success = 0;
            }
        }

        // 验证 A 系数
        for (int i = 0; i <= NA; i++) {
            if (fabs(true_a_coeffs[i].real - a_coeffs_out[i].real) > EPSILON ||
                fabs(true_a_coeffs[i].imag - a_coeffs_out[i].imag) > EPSILON) {
                printf("Mismatch in A[%d]! True: %.10f+%.10fi, Found: %.10f+%.10fi (Diff: %.10f+%.10fi)\n",
                       i, true_a_coeffs[i].real, true_a_coeffs[i].imag,
                       a_coeffs_out[i].real, a_coeffs_out[i].imag,
                       fabs(true_a_coeffs[i].real - a_coeffs_out[i].real),
                       fabs(true_a_coeffs[i].imag - a_coeffs_out[i].imag));
                success = 0;
            }
        }

        if (success) {
            printf("All coefficients are within tolerance. Test PASSED!\n");
        } else {
            printf("Some coefficients are outside tolerance. Test FAILED!\n");
        }

    } else {
        printf("\n--- invfreqs Failed with error code %d ---\n", result);
    }

    // 释放内存
    free(w);
    free(H);
    free(b_coeffs_out);
    free(a_coeffs_out);

    return 0;
}