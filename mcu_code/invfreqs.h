#ifndef INVFREQS_H
#define INVFREQS_H

#include <math.h> // For NAN, fabs

// 定义复数结构体
typedef struct {
    double real;
    double imag;
} complex_double;

// 预定义的复数常量
extern const complex_double J_COMPLEX;
extern const complex_double ONE_COMPLEX;
extern const complex_double ZERO_COMPLEX;

// 复数操作函数声明
complex_double complex_from_real(double r);
complex_double complex_from_imag(double i);
complex_double complex_add(complex_double a, complex_double b);
complex_double complex_sub(complex_double a, complex_double b);
complex_double complex_mul(complex_double a, complex_double b);
complex_double complex_div(complex_double a, complex_double b);
complex_double complex_conj(complex_double a);
complex_double complex_power_int(complex_double base, int exp);
double complex_abs_sq(complex_double a);

// 矩阵操作函数声明 (内部使用，也可以不暴露在头文件)
void complex_matrix_mul(const complex_double* A, int rows_A, int cols_A,
                        const complex_double* B, int rows_B, int cols_B,
                        complex_double* C);
void complex_matrix_conj_transpose(const complex_double* A, int rows_A, int cols_A,
                                   complex_double* B);
int complex_matrix_invert_gauss_jordan(complex_double* matrix, int N);

// invfreqs 主函数声明
int invfreqs(const complex_double *H, const double *w, int n_freq,
             complex_double *b_coeffs, complex_double *a_coeffs);

#endif // INVFREQS_H