#include <stdio.h>
#include <math.h>
#include <stdlib.h>

/* ===== EKF Helpers ===== */



/* Compute Mahalanobis distance: sqrt(innov^T * Sinv * innov) */
/*
double mahalanobis_distance(const double *innov, const double *Sinv, int n) {
    double *temp = (double*)malloc(n * sizeof(double));
    double result = 0.0;

 
    for (int i = 0; i < n; i++) {
        double sum = 0.0;
        for (int j = 0; j < n; j++) {
            sum += Sinv[i*n + j] * innov[j];
        }
        temp[i] = sum;
    }

 
    for (int i = 0; i < n; i++) {
        result += innov[i] * temp[i];
    }

    free(temp);
    return sqrt(result);
}
*/
/* Build a 3×3 skew-symmetric matrix from vector v */
void skew_symmetric(const double v[3], double S[9]) {
    S[0] =  0.0;    S[1] = -v[2];  S[2] =  v[1];
    S[3] =  v[2];   S[4] =  0.0;   S[5] = -v[0];
    S[6] = -v[1];   S[7] =  v[0];  S[8] =  0.0;
}

// Very lightweight PSD enforcement:
// 1) Symmetrize
// 2) If diagonal element <= eps, add eps; where eps is small fraction of trace
void enforce_psd(int n, double *P)
{
    double trace = 0.0;
    for (int i=0;i<n;++i) trace += P[i*n + i];
    double eps = 1e-9;
    if (trace > 0.0) eps = 1e-9 * trace;

    // Symmetrize
    for (int r=0;r<n;++r) {
        for (int c=r+1;c<n;++c) {
            double avg = 0.5*(P[r*n + c] + P[c*n + r]);
            P[r*n + c] = avg;
            P[c*n + r] = avg;
        }
    }

    // Ensure diagonals are >= eps
    for (int i=0;i<n;++i) {
        if (P[i*n + i] < eps) P[i*n + i] = eps;
    }
}


/*

void print_matrix3(const double M[9]) {
    for (int i = 0; i < 3; i++) {
        printf("[%g, %g, %g]\n", M[i*3+0], M[i*3+1], M[i*3+2]);
    }
}



int main(void) {
    printf("Skew-symmetric test:\n");
    double v[3] = {1, 2, 3};
    double S[9];
    skew_symmetric(v, S);
    print_matrix3(S);
    // Expected [[0,-3,2],[3,0,-1],[-2,1,0]]

    printf("\nMahalanobis test:\n");
    double innov[2] = {1, 0};
    double Sinv[4] = {1, 0,
                      0, 1}; // Identity
    double d = mahalanobis_distance(innov, Sinv, 2);
    printf("%g\n", d); // Expected 1

    return 0;
}
*/
