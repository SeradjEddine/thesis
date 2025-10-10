#ifndef MATHLIB_H
#define MATHLIB_H

/* ===== Vector operations ===== */

void vec_add(int n, const double *a, const double *b, double *out);
void vec_sub(int n, const double *a, const double *b, double *out);
void vec_scale(int n, double s, const double *a, double *out);
void vec_copy(int n, const double *src, double *dst);

double vec_dot(int n, const double *a, const double *b);
double vec_norm(int n, const double *a);

/* ===== Matrix operations ===== */


void mat_set_identity(int n, double *A);
void mat_transpose(int r, int c, const double *A, double *AT);

void mat_mult(int rA, int cA, int cB, const double *A, const double *B, double *C);
void mat_vec_mult(int r, int c, const double *A, const double *v, double *out);


// Scale matrix A (r x c) by s into out (can be same pointer as A)
void mat_scale(int r, int c, double s, const double *A, double *out);

// Solve A x = b for x. A is overwritten during elimination.
// Returns 0 on success, non-zero on singular matrix.
int mat_solve_linear(int n, double *A, double *b, double *x);

void mat_add(int r, int c, const double *A, const double *B, double *C);
void mat_sub(int r, int c, const double *A, const double *B, double *C);

int mat_inverse_2x2(const double *A, double *Ainv);
int mat_inverse_3x3(const double *A, double *Ainv);


/* ===== Quaternion operations ===== */

/* Normalization & multiplication */
void quat_normalize(double q[4]);
void quat_mul(const double q1[4], const double q2[4], double out[4]);

// q must be [w,x,y,z]
void quat_to_rotmat(const double q[4], double R[9]);

/* Rotations */
void quat_rotate_vec(const double q[4], const double v[3], double out[3]);

/* Construct from angular velocity */
void quat_from_omega(const double omega[3], double dt, double q_delta[4]);


void skew_symmetric(const double v[3], double S[9]);
void enforce_psd(int n, double *P);

#endif

