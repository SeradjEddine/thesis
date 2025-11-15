#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdio.h>

#include "../include/multi_fuse.h"
#include "mathlib/mathlib.h"

/* --- New helpers for robust weighting --- */

static void component_median(double vals[][3], int n, double out[3])
{
    for (int j = 0; j < 3; ++j)
    {
        double tmp[32];
        for (int i = 0; i < n; ++i)
            tmp[i] = vals[i][j];

        // Simple insertion sort
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

static void component_mad(double vals[][3], int n, const double med[3], double out[3])
{
    for (int j = 0; j < 3; ++j)
    {
        double dev[32];
        for (int i = 0; i < n; ++i)
            dev[i] = fabs(vals[i][j] - med[j]);

        // Sort deviations
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

        out[j] *= 1.4826; // normalize to Gaussian scale
        if (out[j] < 1e-6)
            out[j] = 1e-6;
    }
}

static double compute_robust_alpha(const double z[3], const double med[3], const double mad[3], double k)
{
    double norm = 0.0;
    for (int j = 0; j < 3; ++j)
    {
        double zscore = (z[j] - med[j]) / (k * mad[j]);
        norm += zscore * zscore;
    }
    norm = sqrt(norm);
    double alpha = exp(-0.5 * norm * norm);
    if (alpha < 0.05)
        alpha = 0.05;
    return alpha;
}



/* Multiply (3x3)*(3x1) -> out (3) */
static void mat3_vec(const double *A, const double *v, double *out)
{
    out[0] = A[0]*v[0] + A[1]*v[1] + A[2]*v[2];
    out[1] = A[3]*v[0] + A[4]*v[1] + A[5]*v[2];
    out[2] = A[6]*v[0] + A[7]*v[1] + A[8]*v[2];
}

/* Weighted sum: out = sum_i (W_i * v_i) where W_i is 3x3 and v_i is 3x1.
   We compute sumW = sum W_i, and sumWv = sum (W_i * v_i) */
static void accumulate_weighted(const double *W, const double *v, double *sumW, double *sumWv)
{
    /* sumW += W (3x3) */
    for (int i=0;i<9;++i) sumW[i] += W[i];
    /* tmp = W * v (3x1) */
    double tmp[3];
    mat3_vec(W, v, tmp);
    sumWv[0] += tmp[0];
    sumWv[1] += tmp[1];
    sumWv[2] += tmp[2];
}

/* build diagonal covariance matrix from diag array */
static void build_diag3(const double diag[3], double out[9])
{
    for (int i=0;i<9;++i) out[i]=0.0;
    out[0]=diag[0]; out[4]=diag[1]; out[8]=diag[2];
}

/* append fused imu node to list */
struct fused_imu *append_fused_imu(struct fused_imu **head, struct fused_imu **tail, struct fused_imu *node)
{
    node->next = NULL;
    if (*tail == NULL) {
        *head = *tail = node;
    } else {
        (*tail)->next = node;
        *tail = node;
    }
    return node;
}

/* append fused gps */
struct fused_gps *append_fused_gps(struct fused_gps **head, struct fused_gps **tail, struct fused_gps *node)
{
    if (!node)
        return NULL;

    node->next = NULL;
    if (*tail == NULL)
        *head = *tail = node;
    else {
        (*tail)->next = node;
        *tail = node;
    }
    return node;
}


/* Helper: find the next earliest timestamp among current heads (IMU). Returns 0 if none found. */
static int next_time_imu(struct imu_node **heads, int N, double *tmin)
{
    *tmin = 1e308;
    for (int i=0;i<N;++i)
    {
        if (heads[i] && heads[i]->s.t < *tmin)
            *tmin = heads[i]->s.t;
    }
    if (*tmin > 1e307)
        return 0;
    return 1;
}

/* same for gps */
static int next_time_gps(struct gps_node **heads, int N, double *tmin)
{
    *tmin = 1e308;
    for (int i=0;i<N;++i) {
        if (heads[i] && heads[i]->s.t < *tmin) {
            *tmin = heads[i]->s.t;
        }
    }
    if (*tmin > 1e307) return 0;
    return 1;
}


struct fused_imu *fuse_imus(struct imu_node **imu_heads, int N, double max_dt,
                            const double (*sensor_R_acc_diag)[3],
                            const double (*sensor_R_gyro_diag)[3])
{
    if (N <= 0 || imu_heads == NULL)
        return NULL;

    struct imu_node **heads = malloc(sizeof(struct imu_node *) * N);
    if (!heads)
        return NULL;
    for (int i = 0; i < N; ++i)
        heads[i] = imu_heads[i];

    struct fused_imu *head = NULL, *tail = NULL;

    while (1)
    {
        double tmin;
        if (!next_time_imu(heads, N, &tmin))
            break;

        /* Gather valid samples within window */
        double acc_vals[32][3];
        double gyro_vals[32][3];
        int used_idx[32];
        int count = 0;

        for (int i = 0; i < N; ++i)
        {
            if (!heads[i])
                continue;
            double dt = fabs(heads[i]->s.t - tmin);
            if (dt <= max_dt)
            {
                acc_vals[count][0] = heads[i]->s.ax;
                acc_vals[count][1] = heads[i]->s.ay;
                acc_vals[count][2] = heads[i]->s.az;
                gyro_vals[count][0] = heads[i]->s.wx;
                gyro_vals[count][1] = heads[i]->s.wy;
                gyro_vals[count][2] = heads[i]->s.wz;
                used_idx[count++] = i;
            }
        }

        if (count == 0)
            break;

        /* Robust statistics */
        double med_acc[3], mad_acc[3], med_gyro[3], mad_gyro[3];
        component_median(acc_vals, count, med_acc);
        component_median(gyro_vals, count, med_gyro);
        component_mad(acc_vals, count, med_acc, mad_acc);
        component_mad(gyro_vals, count, med_gyro, mad_gyro);

        double sumW_acc[9] = {0}, sumWv_acc[3] = {0};
        double sumW_gyro[9] = {0}, sumWv_gyro[3] = {0};

        /* Accumulate with robust weights */
        const double k = 3.0;

        for (int j = 0; j < count; ++j)
        {
            int i = used_idx[j];
            double acc_vec[3] = {acc_vals[j][0], acc_vals[j][1], acc_vals[j][2]};
            double gyro_vec[3] = {gyro_vals[j][0], gyro_vals[j][1], gyro_vals[j][2]};

            double alpha_acc = compute_robust_alpha(acc_vec, med_acc, mad_acc, k);
            double alpha_gyro = compute_robust_alpha(gyro_vec, med_gyro, mad_gyro, k);

            double Racc[9], Rgyro[9];
            if (sensor_R_acc_diag)
                build_diag3(sensor_R_acc_diag[i], Racc);
            else
            {
                double d[3] = {0.1, 0.1, 0.1};
                build_diag3(d, Racc);
            }
            if (sensor_R_gyro_diag)
                build_diag3(sensor_R_gyro_diag[i], Rgyro);
            else
            {
                double d[3] = {1e-3, 1e-3, 1e-3};
                build_diag3(d, Rgyro);
            }

            double Wacc[9], Wgyro[9];
            mat_inverse_3x3(Racc, Wacc);
            mat_inverse_3x3(Rgyro, Wgyro);

            for (int k = 0; k < 9; ++k)
            {
                Wacc[k] *= alpha_acc;
                Wgyro[k] *= alpha_gyro;
            }

            accumulate_weighted(Wacc, acc_vec, sumW_acc, sumWv_acc);
            accumulate_weighted(Wgyro, gyro_vec, sumW_gyro, sumWv_gyro);

            heads[i] = heads[i]->next;
            if (imu_heads[i] && imu_heads[i]->s.t == tmin)
                imu_heads[i] = imu_heads[i]->next;
        }

        /* Fuse */
        double Racc_fused[9], Rgyro_fused[9];
        if (mat_inverse_3x3(sumW_acc, Racc_fused) != 0)
            build_diag3((double[3]){1, 1, 1}, Racc_fused);
        if (mat_inverse_3x3(sumW_gyro, Rgyro_fused) != 0)
            build_diag3((double[3]){1, 1, 1}, Rgyro_fused);

        double zacc_fused[3], zgyro_fused[3];
        mat3_vec(Racc_fused, sumWv_acc, zacc_fused);
        mat3_vec(Rgyro_fused, sumWv_gyro, zgyro_fused);

        struct fused_imu *node = malloc(sizeof(*node));
        if (!node)
            break;
        memset(node, 0, sizeof(*node));
        node->t = tmin;
        node->ax = zacc_fused[0];
        node->ay = zacc_fused[1];
        node->az = zacc_fused[2];
        node->gx = zgyro_fused[0];
        node->gy = zgyro_fused[1];
        node->gz = zgyro_fused[2];
        memcpy(node->P_acc, Racc_fused, sizeof(node->P_acc));
        memcpy(node->P_gyro, Rgyro_fused, sizeof(node->P_gyro));
        node->n_used = count;

        append_fused_imu(&head, &tail, node);
    }

    free(heads);
    return head;
}

struct fused_gps *fuse_gps(struct gps_node **gps_heads, int N, double max_dt,
                           const double (*sensor_R_pos_diag)[3],
                           const double (*sensor_R_vel_diag)[3])
{
    if (N <= 0 || gps_heads == NULL)
        return NULL;

    struct gps_node **heads = malloc(sizeof(struct gps_node *) * N);
    if (!heads)
        return NULL;
    for (int i = 0; i < N; ++i)
        heads[i] = gps_heads[i];

    struct fused_gps *head = NULL, *tail = NULL;

    while (1)
    {
        double tmin;
        if (!next_time_gps(heads, N, &tmin))
            break;

        double pos_vals[32][3];
        double vel_vals[32][3];
        int used_idx[32];
        int count = 0;

        for (int i = 0; i < N; ++i)
        {
            if (!heads[i])
                continue;
            double dt = fabs(heads[i]->s.t - tmin);
            if (dt <= max_dt)
            {
                pos_vals[count][0] = heads[i]->s.lat;
                pos_vals[count][1] = heads[i]->s.lon;
                pos_vals[count][2] = heads[i]->s.alt;
                vel_vals[count][0] = heads[i]->s.vn;
                vel_vals[count][1] = heads[i]->s.ve;
                vel_vals[count][2] = heads[i]->s.vu;
                used_idx[count++] = i;
            }
        }

        if (count == 0)
            break;

        double med_pos[3], mad_pos[3], med_vel[3], mad_vel[3];
        component_median(pos_vals, count, med_pos);
        component_median(vel_vals, count, med_vel);
        component_mad(pos_vals, count, med_pos, mad_pos);
        component_mad(vel_vals, count, med_vel, mad_vel);

        double sumW_pos[9] = {0}, sumWv_pos[3] = {0};
        double sumW_vel[9] = {0}, sumWv_vel[3] = {0};

        const double k = 3.0;
        for (int j = 0; j < count; ++j)
        {
            int i = used_idx[j];
            double pos_vec[3] = {pos_vals[j][0], pos_vals[j][1], pos_vals[j][2]};
            double vel_vec[3] = {vel_vals[j][0], vel_vals[j][1], vel_vals[j][2]};

            double alpha_pos = compute_robust_alpha(pos_vec, med_pos, mad_pos, k);
            double alpha_vel = compute_robust_alpha(vel_vec, med_vel, mad_vel, k);

            double Rpos[9], Rvel[9];
            if (sensor_R_pos_diag)
                build_diag3(sensor_R_pos_diag[i], Rpos);
            else
            {
                double d[3] = {2.0, 2.0, 5.0};
                build_diag3(d, Rpos);
            }
            if (sensor_R_vel_diag)
                build_diag3(sensor_R_vel_diag[i], Rvel);
            else
            {
                double d[3] = {0.5, 0.5, 0.5};
                build_diag3(d, Rvel);
            }

            double Wpos[9], Wvel[9];
            mat_inverse_3x3(Rpos, Wpos);
            mat_inverse_3x3(Rvel, Wvel);

            for (int k = 0; k < 9; ++k)
            {
                Wpos[k] *= alpha_pos;
                Wvel[k] *= alpha_vel;
            }

            accumulate_weighted(Wpos, pos_vec, sumW_pos, sumWv_pos);
            accumulate_weighted(Wvel, vel_vec, sumW_vel, sumWv_vel);

            heads[i] = heads[i]->next;
            if (gps_heads[i] && gps_heads[i]->s.t == tmin)
                gps_heads[i] = gps_heads[i]->next;
        }

        double Rpos_fused[9], Rvel_fused[9];
        if (mat_inverse_3x3(sumW_pos, Rpos_fused) != 0)
            build_diag3((double[3]){1, 1, 1}, Rpos_fused);
        if (mat_inverse_3x3(sumW_vel, Rvel_fused) != 0)
            build_diag3((double[3]){1, 1, 1}, Rvel_fused);

        double pos_fused[3], vel_fused[3];
        mat3_vec(Rpos_fused, sumWv_pos, pos_fused);
        mat3_vec(Rvel_fused, sumWv_vel, vel_fused);

        struct fused_gps *node = malloc(sizeof(*node));
        if (!node)
            break;
        memset(node, 0, sizeof(*node));
        node->t = tmin;
        node->lat = pos_fused[0];
        node->lon = pos_fused[1];
        node->alt = pos_fused[2];
        node->vn = vel_fused[0];
        node->ve = vel_fused[1];
        node->vu = vel_fused[2];
        memcpy(node->P_pos, Rpos_fused, sizeof(node->P_pos));
        memcpy(node->P_vel, Rvel_fused, sizeof(node->P_vel));
        node->n_used = count;

        append_fused_gps(&head, &tail, node);
    }

    free(heads);
    return head;
}



/* free helpers */
void free_fused_imu_list(struct fused_imu *h)
{
    while (h) {
        struct fused_imu *n = h->next;
        free(h);
        h = n;
    }
}
void free_fused_gps_list(struct fused_gps *h)
{
    while (h) {
        struct fused_gps *n = h->next;
        free(h);
        h = n;
    }
}
