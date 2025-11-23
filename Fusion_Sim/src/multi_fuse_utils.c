#include "../include/multi_fuse.h"

/* --- Robust helpers (copied / adapted from original) --- */

void component_median(double vals[][3], int n, double out[3])
{
    for (int j = 0; j < 3; ++j)
    {
        double tmp[32];
        for (int i = 0; i < n; ++i)
            tmp[i] = vals[i][j];

        /* insertion sort */
        for (int a = 1; a < n; ++a)
        {
            double v = tmp[a];
            int b = a - 1;
            while (b >= 0 && tmp[b] > v)
            {
                tmp[b + 1] = tmp[b];
                b--;
            }
            tmp[b + 1] = v;
        }

        if (n % 2)
            out[j] = tmp[n / 2];
        else
            out[j] = 0.5 * (tmp[n / 2 - 1] + tmp[n / 2]);
    }
}

void component_mad(double vals[][3], int n, const double med[3], double out[3])
{
    for (int j = 0; j < 3; ++j)
    {
        double dev[32];
        for (int i = 0; i < n; ++i)
            dev[i] = fabs(vals[i][j] - med[j]);

        /* sort deviations */
        for (int a = 1; a < n; ++a)
        {
            double v = dev[a];
            int b = a - 1;
            while (b >= 0 && dev[b] > v)
            {
                dev[b + 1] = dev[b];
                b--;
            }
            dev[b + 1] = v;
        }

        if (n % 2)
            out[j] = dev[n / 2];
        else
            out[j] = 0.5 * (dev[n / 2 - 1] + dev[n / 2]);

        out[j] *= 1.4826;
        if (out[j] < 1e-6)
            out[j] = 1e-6;
    }
}

double compute_alpha(const double z[3], const double med[3], const double mad[3], double k)
{
    double norm = 0.0;

    for (int j = 0; j < 3; ++j)
    {
        double  zscore = (z[j] - med[j]) / (k * mad[j]);
        norm += zscore * zscore;
    }
    norm = sqrt(norm);
    double alpha = exp(-0.5 * norm * norm);
    if (alpha < 0.05)
        alpha = 0.05;
    return alpha;
}

/* accumulate weighted sums for fusion */
void accumulate_weighted(const double *W, const double *v, double *sumW, double *sumWv)
{
    for (int i = 0; i < 9; ++i)
        sumW[i] += W[i];

    double tmp[3];
    mat3_vec(W, v, tmp);
    sumWv[0] += tmp[0];
    sumWv[1] += tmp[1];
    sumWv[2] += tmp[2];
}

/* build diagonal 3x3 matrix */
void build_diag3(const double diag[3], double out[9])
{
    for (int i = 0; i < 9; ++i)
        out[i] = 0.0;
    out[0] = diag[0];
    out[4] = diag[1];
    out[8] = diag[2];
}
