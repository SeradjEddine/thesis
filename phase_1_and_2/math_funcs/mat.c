#include <stdio.h>
#include <math.h>

/* ===== Matrix implementation ===== */

/* Set an n×n matrix A to identity */
void mat_set_identity(int n, double *A) {
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            A[i*n + j] = (i == j) ? 1.0 : 0.0;
        }
    }
}

/* Transpose an r×c matrix A into a c×r matrix AT */
void mat_transpose(int r, int c, const double *A, double *AT) {
    for (int i = 0; i < r; i++) {
        for (int j = 0; j < c; j++) {
            AT[j*r + i] = A[i*c + j];
        }
    }
}

/* Multiply rA×cA matrix A with cA×cB matrix B, result rA×cB matrix C */
void mat_mult(int rA, int cA, int cB, const double *A, const double *B, double *C) {
    for (int i = 0; i < rA; i++) {
        for (int j = 0; j < cB; j++) {
            double sum = 0.0;
            for (int k = 0; k < cA; k++) {
                sum += A[i*cA + k] * B[k*cB + j];
            }
            C[i*cB + j] = sum;
        }
    }
}

/* Multiply r×c matrix A with vector v of length c, result vector out of length r */
void mat_vec_mult(int r, int c, const double *A, const double *v, double *out) {
    for (int i = 0; i < r; i++) {
        double sum = 0.0;
        for (int j = 0; j < c; j++) {
            sum += A[i*c + j] * v[j];
        }
        out[i] = sum;
    }
}

/* Add two r×c matrices: C = A + B */
void mat_add(int r, int c, const double *A, const double *B, double *C) {
    for (int i = 0; i < r*c; i++) {
        C[i] = A[i] + B[i];
    }
}

/* Subtract two r×c matrices: C = A - B */
void mat_sub(int r, int c, const double *A, const double *B, double *C) {
    for (int i = 0; i < r*c; i++) {
        C[i] = A[i] - B[i];
    }
}




/* Inverse of a 2x2 matrix */
int mat_inverse_2x2(const double *A, double *Ainv) {
    double det = A[0]*A[3] - A[1]*A[2];
    if (fabs(det) < 1e-12) return -1; // Singular

    double invdet = 1.0 / det;
    Ainv[0] =  A[3] * invdet;
    Ainv[1] = -A[1] * invdet;
    Ainv[2] = -A[2] * invdet;
    Ainv[3] =  A[0] * invdet;
    return 0;
}

/* Inverse of a 3x3 matrix */
int mat_inverse_3x3(const double *A, double *Ainv) {
    double det =
        A[0]*(A[4]*A[8] - A[5]*A[7]) -
        A[1]*(A[3]*A[8] - A[5]*A[6]) +
        A[2]*(A[3]*A[7] - A[4]*A[6]);

    if (fabs(det) < 1e-12) return -1; // Singular

    double invdet = 1.0 / det;

    Ainv[0] =  (A[4]*A[8] - A[5]*A[7]) * invdet;
    Ainv[1] = -(A[1]*A[8] - A[2]*A[7]) * invdet;
    Ainv[2] =  (A[1]*A[5] - A[2]*A[4]) * invdet;

    Ainv[3] = -(A[3]*A[8] - A[5]*A[6]) * invdet;
    Ainv[4] =  (A[0]*A[8] - A[2]*A[6]) * invdet;
    Ainv[5] = -(A[0]*A[5] - A[2]*A[3]) * invdet;

    Ainv[6] =  (A[3]*A[7] - A[4]*A[6]) * invdet;
    Ainv[7] = -(A[0]*A[7] - A[1]*A[6]) * invdet;
    Ainv[8] =  (A[0]*A[4] - A[1]*A[3]) * invdet;

    return 0;
}

// Row-major layout: A[r*c]

void mat_scale(int r, int c, double s, const double *A, double *out)
{
    size_t n = (size_t)r * (size_t)c;
    if (out == A) {
        // in-place
        for (size_t i = 0; i < n; ++i) out[i] *= s;
    } else {
        for (size_t i = 0; i < n; ++i) out[i] = A[i] * s;
    }
}

/*
 * Solve Ax = b with Gaussian elimination with partial pivoting.
 * A is n x n, row-major; b is length n.
 * A and b are copied into local arrays, so inputs are preserved.
 */
int mat_solve_linear(int n, double *A_in, double *b_in, double *x_out)
{
    // Make local copies since we will modify
    double A[225]; // support up to 15x15 (225). Adjust if needed.
    double b[15];
    if (n > 15) return -1; // limit safety
    size_t nn = (size_t)n * (size_t)n;
    for (size_t i=0;i<nn;++i) A[i] = A_in[i];
    for (int i=0;i<n;++i) b[i] = b_in[i];

    // Gaussian elimination
    for (int k = 0; k < n; ++k) {
        // Partial pivot
        int piv = k;
        double maxv = fabs(A[k*n + k]);
        for (int i = k+1; i < n; ++i) {
            double v = fabs(A[i*n + k]);
            if (v > maxv) { maxv = v; piv = i; }
        }
        if (maxv < 1e-12) return -2; // singular / nearly singular

        // Swap rows k and piv
        if (piv != k) {
            for (int j = k; j < n; ++j) {
                double tmp = A[k*n + j];
                A[k*n + j] = A[piv*n + j];
                A[piv*n + j] = tmp;
            }
            double tmpb = b[k];
            b[k] = b[piv];
            b[piv] = tmpb;
        }

        // Eliminate
        double Akk = A[k*n + k];
        for (int i = k+1; i < n; ++i) {
            double factor = A[i*n + k] / Akk;
            A[i*n + k] = 0.0;
            for (int j = k+1; j < n; ++j) {
                A[i*n + j] -= factor * A[k*n + j];
            }
            b[i] -= factor * b[k];
        }
    }

    // Back substitution
    for (int i = n-1; i >= 0; --i) {
        double s = b[i];
        for (int j = i+1; j < n; ++j) s -= A[i*n + j] * x_out[j];
        double diag = A[i*n + i];
        if (fabs(diag) < 1e-12) return -3;
        x_out[i] = s / diag;
    }

    return 0;
}


/*



void print_matrix(int r, int c, const double *A) {
    for (int i = 0; i < r; i++) {
        printf("[");
        for (int j = 0; j < c; j++) {
            printf("%g", A[i*c + j]);
            if (j < c - 1) printf(", ");
        }
        printf("]\n");
    }
}



int main(void) {
    
    printf("Identity test:\n");
    double A[4] = {1, 2, 3, 4};   // 2×2 matrix
    double I[4];
    double C[4];
    mat_set_identity(2, I);
    mat_mult(2, 2, 2, A, I, C);
    print_matrix(2, 2, C);  // Expected [[1,2],[3,4]]


    printf("\nTranspose test:\n");
    double AT[4];
    double ATT[4];
    mat_transpose(2, 2, A, AT);
    mat_transpose(2, 2, AT, ATT);
    print_matrix(2, 2, ATT);  // Expected [[1,2],[3,4]]


    printf("\nMatrix-vector multiplication test:\n");
    double M[4] = {1, 2, 3, 4};   // 2×2
    double v[2] = {5, 6};
    double out[2];
    mat_vec_mult(2, 2, M, v, out);
    print_matrix(2, 1, out);  // Expected [[17],[39]]

   printf("\nInverse 2x2 test:\n");
double A2[4] = {1, 2, 3, 4};
double A2inv[4];
if (mat_inverse_2x2(A2, A2inv) == 0) {
    print_matrix(2, 2, A2inv); // Expected [[-2, 1], [1.5, -0.5]]
    double check[4];
    mat_mult(2, 2, 2, A2, A2inv, check);
    printf("A * Ainv ≈ I:\n");
    print_matrix(2, 2, check);
} else {
    printf("Matrix is singular!\n");
}


printf("\nInverse 3x3 test:\n");
double A3[9] = {1, 2, 3,
                0, 1, 4,
                5, 6, 0};
double A3inv[9];
if (mat_inverse_3x3(A3, A3inv) == 0) {
    print_matrix(3, 3, A3inv);
    double check3[9];
    mat_mult(3, 3, 3, A3, A3inv, check3);
    printf("A * Ainv ≈ I:\n");
    print_matrix(3, 3, check3);
} else {
    printf("Matrix is singular!\n");
} 
    return 0;
}

*/
