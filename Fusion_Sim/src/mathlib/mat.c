#include <stdio.h>
#include <math.h>

/* ===== Matrix implementation ===== */

/* Set an n×n matrix A to identity */
void mat_set_identity(int n, double *A)
{
    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < n; j++)
        {
            if ((A[i*n + j] = (i == j)))
                A[i*n + j] = 1.0;
            else
                A[i*n + j] = 0.0;
        }
    }
}

/* Transpose an r×c matrix A into a c×r matrix AT */
void mat_transpose(int r, int c, const double *A, double *AT)
{
    for (int i = 0; i < r; i++)
    {
        for (int j = 0; j < c; j++)
        {
            AT[j*r + i] = A[i*c + j];
        }
    }
}

/* Multiply rA×cA matrix A with cA×cB matrix B, result rA×cB matrix C */
void mat_mult(int rA, int cA, int cB, const double *A, const double *B, double *C)
{
    for (int i = 0; i < rA; i++)
    {
        for (int j = 0; j < cB; j++)
        {
            double sum = 0.0;
            for (int k = 0; k < cA; k++)
            {
                sum += A[i*cA + k] * B[k*cB + j];
            }
            C[i*cB + j] = sum;
        }
    }
}

/* Multiply r×c matrix A with vector v of length c, result vector out of length r */
void mat_vec_mult(int r, int c, const double *A, const double *v, double *out)
{
    for (int i = 0; i < r; i++)
    {
        double sum = 0.0;
        for (int j = 0; j < c; j++)
        {
            sum += A[i*c + j] * v[j];
        }
        out[i] = sum;
    }
}

/* Add two r×c matrices: C = A + B */
void mat_add(int r, int c, const double *A, const double *B, double *C)
{
    for (int i = 0; i < r*c; i++)
    {
        C[i] = A[i] + B[i];
    }
}

/* Subtract two r×c matrices: C = A - B */
void mat_sub(int r, int c, const double *A, const double *B, double *C)
{
    for (int i = 0; i < r*c; i++)
    {
        C[i] = A[i] - B[i];
    }
}




/* Inverse of a 2x2 matrix */
int mat_inverse_2x2(const double *A, double *Ainv)
{
    double det = A[0]*A[3] - A[1]*A[2];

    if (fabs(det) < 1e-12)
        return -1; // Singular

    double invdet = 1.0 / det;
    Ainv[0] =  A[3] * invdet;
    Ainv[1] = -A[1] * invdet;
    Ainv[2] = -A[2] * invdet;
    Ainv[3] =  A[0] * invdet;
    return 0;
}

/* Inverse of a 3x3 matrix */
int mat_inverse_3x3(const double *A, double *Ainv)
{
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

    if (out == A)
    {
        for (size_t i = 0; i < n; ++i) 
            out[i] *= s;
    } 
    else 
    {
        for (size_t i = 0; i < n; ++i)
            out[i] = A[i] * s;
    }
}

/*
 * Solve Ax = b with Gaussian elimination with partial pivoting.
 * A is n x n, row-major; b is length n.
 * A and b are copied into local arrays, so inputs are preserved.
 */

int mat_solve_linear(int n, double *A_in, double *b_in, double *x_out)
{
    double A[225];      // support up to 15x15 (225). (should be suffiecent)
    double b[15];       // Make local copies since we will modify
    size_t nn = (size_t)n * (size_t)n;

    if (n > 15)         // limit safety
        return -1; 
    
    for (size_t i=0;i<nn;++i)
        A[i] = A_in[i];
    
    for (int i=0;i<n;++i)
        b[i] = b_in[i];

    // Gaussian elimination
    for (int k = 0; k < n; ++k)
    {
        // Partial pivot
        int piv = k;
        double maxv = fabs(A[k*n + k]);
        for (int i = k+1; i < n; ++i)
        {
            double v = fabs(A[i*n + k]);
            if (v > maxv)
            {
                maxv = v;
                piv = i;
            }
        }
        if (maxv < 1e-12) 
            return -2;

        // Swap rows k and piv
        if (piv != k)
        {
            for (int j = k; j < n; ++j)
            {
                double tmp      = A[k*n + j];
                A[k*n + j]      = A[piv*n + j];
                A[piv*n + j]    = tmp;
            }
            double tmpb = b[k];
            b[k] = b[piv];
            b[piv] = tmpb;
        }

        // Eliminate
        double Akk = A[k*n + k];
        for (int i = k+1; i < n; ++i)
        {
            double factor = A[i*n + k] / Akk;
            A[i*n + k] = 0.0;
            for (int j = k+1; j < n; ++j)
            {
                A[i*n + j] -= factor * A[k*n + j];
            }
            b[i] -= factor * b[k];
        }
    }

    // Back substitution
    for (int i = n-1; i >= 0; --i)
    {
        double s = b[i];
        for (int j = i+1; j < n; ++j)
            s -= A[i*n + j] * x_out[j];
        double diag = A[i*n + i];
        if (fabs(diag) < 1e-12)
            return -3;
        x_out[i] = s / diag;
    }

    return 0;
}
