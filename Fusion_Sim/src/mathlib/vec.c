#include <math.h>

/* Elementwise addition: out = a + b */
void vec_add(int n, const double *a, const double *b, double *out) {
    for (int i = 0; i < n; i++) {
        out[i] = a[i] + b[i];
    }
}

/* Elementwise subtraction: out = a - b */
void vec_sub(int n, const double *a, const double *b, double *out) {
    for (int i = 0; i < n; i++) {
        out[i] = a[i] - b[i];
    }
}

/* Dot product: return Σ (a[i] * b[i]) */
double vec_dot(int n, const double *a, const double *b) {
    double result = 0.0;
    for (int i = 0; i < n; i++) {
        result += a[i] * b[i];
    }
    return result;
}

/* Scale: out = s * a */
void vec_scale(int n, double s, const double *a, double *out) {
    for (int i = 0; i < n; i++) {
        out[i] = s * a[i];
    }
}

/* Euclidean norm: sqrt(Σ (a[i]^2)) */
double vec_norm(int n, const double *a) {
    double sum_sq = 0.0;
    for (int i = 0; i < n; i++) {
        sum_sq += a[i] * a[i];
    }
    return sqrt(sum_sq);
}

/* Copy vector: dst = src */
void vec_copy(int n, const double *src, double *dst) {
    for (int i = 0; i < n; i++) {
        dst[i] = src[i];
    }
}

