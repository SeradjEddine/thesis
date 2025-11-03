#include "multi_fuse.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdio.h>

#include "../include/multi_fuse.h"

/* minimal 3x3 inverse (row-major). returns 0 on success, -1 singular */
static int mat_inv_3x3(const double *A, double *Ainv)
{
    double a=A[0], b=A[1], c=A[2],
           d=A[3], e=A[4], f=A[5],
           g=A[6], h=A[7], i=A[8];
    double det = a*(e*i - f*h) - b*(d*i - f*g) + c*(d*h - e*g);
    if (fabs(det) < 1e-12) return -1;
    double invdet = 1.0 / det;
    Ainv[0] =  (e*i - f*h) * invdet;
    Ainv[1] = -(b*i - c*h) * invdet;
    Ainv[1] = -(b*i - c*h) * invdet;
    Ainv[2] =  (b*f - c*e) * invdet;
    Ainv[3] = -(d*i - f*g) * invdet;
    Ainv[4] =  (a*i - c*g) * invdet;
    Ainv[5] = -(a*f - c*d) * invdet;
    Ainv[6] =  (d*h - e*g) * invdet;
    Ainv[7] = -(a*h - b*g) * invdet;
    Ainv[8] =  (a*e - b*d) * invdet;
    return 0;
}

/* Multiply (3x3)*(3x1) -> out (3) */
static void mat3_vec(const double *A, const double *v, double *out)
{
    out[0] = A[0]*v[0] + A[1]*v[1] + A[2]*v[2];
    out[1] = A[3]*v[0] + A[4]*v[1] + A[5]*v[2];
    out[2] = A[6]*v[0] + A[7]*v[1] + A[8]*v[2];
}

/* Add two 3x1 vectors */

/* 
static void vec_add3(const double *a, const double *b, double *out)
{
    out[0]=a[0]+b[0]; out[1]=a[1]+b[1]; out[2]=a[2]+b[2];
}
*/

/* Weighted sum: out = sum_i (W_i * v_i) where W_i is 3x3 and v_i is 3x1. We compute sumW = sum W_i, and sumWv = sum (W_i * v_i) */
static void accumulate_weighted(const double *W, const double *v, double *sumW, double *sumWv)
{
    // sumW += W (3x3)
    for (int i=0;i<9;++i) sumW[i] += W[i];
    // tmp = W * v (3x1)
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
static struct fused_imu *append_fused_imu(struct fused_imu **head, struct fused_imu **tail, struct fused_imu *node)
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
static struct fused_gps *append_fused_gps(struct fused_gps **head, struct fused_gps **tail, struct fused_gps *node)
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
    // collect indices within tolerance later; caller handles tolerance
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

/* Mahalanobis distance for 3x1 residual with covariance S (3x3) and S_inv precomputed */

/*  unused
static double mahalanobis3(const double res[3], const double S_inv[9])
{
    double tmp[3];
    mat3_vec(S_inv, res, tmp);
    return res[0]*tmp[0] + res[1]*tmp[1] + res[2]*tmp[2];
}
*/ 

/* fuse IMU lists */
struct fused_imu *fuse_imus(struct imu_node **imu_heads, int N, double max_dt,
                            const double (*sensor_R_acc_diag)[3],
                            const double (*sensor_R_gyro_diag)[3])
{
    if (N <= 0 || imu_heads == NULL) return NULL;

    struct imu_node **heads = malloc(sizeof(struct imu_node*) * N);
    if (!heads) return NULL;
    for (int i=0;i<N;++i) heads[i] = imu_heads[i];

    struct fused_imu *head = NULL, *tail = NULL;

    /* iterate until all lists exhausted */
    while (1) {
        double tmin;
        if (!next_time_imu(heads, N, &tmin)) break;

        /* collect samples from sensors within [tmin, tmin + max_dt] */
        double sumW_acc[9] = {0}, sumWv_acc[3] = {0};
        double sumW_gyro[9] = {0}, sumWv_gyro[3] = {0};
        int count = 0;

        // collect raw samples and covariances
        for (int i=0;i<N;++i) {
            if (!heads[i]) continue;
            double dt = fabs(heads[i]->s.t - tmin);
            if (dt <= max_dt) {
                // build covariances
                double Racc[9], Rgyro[9];
                if (sensor_R_acc_diag) {
                    build_diag3(sensor_R_acc_diag[i], Racc);
                } else {
                    // default small covariance
                    double d[3] = {0.1, 0.1, 0.1};
                    build_diag3(d, Racc);
                }
                if (sensor_R_gyro_diag) {
                    build_diag3(sensor_R_gyro_diag[i], Rgyro);
                } else {
                    double d2[3] = {1e-3, 1e-3, 1e-3};
                    build_diag3(d2, Rgyro);
                }
                // compute W = R^-1 (weight) robustly
                double Wacc[9], Wgyro[9];
                if (mat_inv_3x3(Racc, Wacc) != 0) {
                    // singular fallback: use diag inverse
                    Wacc[0] = 1.0 / Racc[0]; Wacc[4] = 1.0 / Racc[4]; Wacc[8] = 1.0 / Racc[8];
                    Wacc[1]=Wacc[2]=Wacc[3]=Wacc[5]=Wacc[6]=Wacc[7]=0.0;
                }
                if (mat_inv_3x3(Rgyro, Wgyro) != 0) {
                    Wgyro[0] = 1.0 / Rgyro[0]; Wgyro[4] = 1.0 / Rgyro[4]; Wgyro[8] = 1.0 / Rgyro[8];
                }

                double acc_vec[3] = { heads[i]->s.ax, heads[i]->s.ay, heads[i]->s.az };
                double gyro_vec[3] = { heads[i]->s.wx, heads[i]->s.wy, heads[i]->s.wz };

                accumulate_weighted(Wacc, acc_vec, sumW_acc, sumWv_acc);
                accumulate_weighted(Wgyro, gyro_vec, sumW_gyro, sumWv_gyro);
                count++;

                // advance this head (consume this sample)
                heads[i] = heads[i]->next;
            }
        }

        if (count == 0) {
            // no sample matched; advance the earliest list to avoid infinite loop
            for (int i=0;i<N;++i) {
                if (imu_heads[i] && imu_heads[i]->s.t == tmin) {
                    // advance the original array head if matches
                    imu_heads[i] = imu_heads[i]->next;
                    break;
                }
            }
            continue;
        }

        // compute fused covariances and fused vectors: Rfused = inv(sumW), zfused = Rfused * sumWv
        double Racc_fused[9], Rgyro_fused[9];
        if (mat_inv_3x3(sumW_acc, Racc_fused) != 0) {
            // fallback: use diag of inverse sums
            double diag[3] = {1.0/(sumW_acc[0]+1e-12), 1.0/(sumW_acc[4]+1e-12), 1.0/(sumW_acc[8]+1e-12)};
            build_diag3(diag, Racc_fused);
        }
        if (mat_inv_3x3(sumW_gyro, Rgyro_fused) != 0) {
            double diag[3] = {1.0/(sumW_gyro[0]+1e-12), 1.0/(sumW_gyro[4]+1e-12), 1.0/(sumW_gyro[8]+1e-12)};
            build_diag3(diag, Rgyro_fused);
        }

        // compute zfused = Rfused * sumWv
        double zacc_fused[3], zgyro_fused[3];
        mat3_vec(Racc_fused, sumWv_acc, zacc_fused);
        mat3_vec(Rgyro_fused, sumWv_gyro, zgyro_fused);

        // create fused node
        struct fused_imu *node = malloc(sizeof(*node));
        if (!node) break;
        node->t = tmin;
        node->ax = zacc_fused[0]; node->ay = zacc_fused[1]; node->az = zacc_fused[2];
        node->gx = zgyro_fused[0]; node->gy = zgyro_fused[1]; node->gz = zgyro_fused[2];
        memcpy(node->P_acc, Racc_fused, sizeof(node->P_acc));
        memcpy(node->P_gyro, Rgyro_fused, sizeof(node->P_gyro));
        node->n_used = count;
        append_fused_imu(&head, &tail, node);
    }

    free(heads);
    return head;
}

/* fuse gps lists — similar logic but uses lat/lon/alt and velocities; we simply average with weights on (lat,lon,alt) components as 3x1 vectors (not converting to ENU here) */
struct fused_gps *fuse_gps(struct gps_node **gps_heads, int N, double max_dt,
                           const double (*sensor_R_pos_diag)[3],
                           const double (*sensor_R_vel_diag)[3])
{
    if (N <= 0 || gps_heads == NULL) return NULL;

    struct gps_node **heads = malloc(sizeof(struct gps_node*) * N);
    if (!heads) return NULL;
    for (int i=0;i<N;++i) heads[i] = gps_heads[i];

    struct fused_gps *head = NULL, *tail = NULL;

    while (1) {
        double tmin;
        if (!next_time_gps(heads, N, &tmin)) break;

        double sumW_pos[9] = {0}, sumWv_pos[3] = {0};
        double sumW_vel[9] = {0}, sumWv_vel[3] = {0};
        int count = 0;

        for (int i=0;i<N;++i) {
            if (!heads[i]) continue;
            double dt = fabs(heads[i]->s.t - tmin);
            if (dt <= max_dt) {
                double Rpos[9], Rvel[9];
                if (sensor_R_pos_diag) build_diag3(sensor_R_pos_diag[i], Rpos);
                else { double d[3]={2.0,2.0,5.0}; build_diag3(d,Rpos); }
                if (sensor_R_vel_diag) build_diag3(sensor_R_vel_diag[i], Rvel);
                else { double d[3]={0.5,0.5,0.5}; build_diag3(d,Rvel); }
                double Wpos[9], Wvel[9];
                if (mat_inv_3x3(Rpos, Wpos) != 0) {
                    Wpos[0] = 1.0/Rpos[0]; Wpos[4]=1.0/Rpos[4]; Wpos[8]=1.0/Rpos[8];
                }
                if (mat_inv_3x3(Rvel, Wvel) != 0) {
                    Wvel[0] = 1.0/Rvel[0]; Wvel[4]=1.0/Rvel[4]; Wvel[8]=1.0/Rvel[8];
                }
                // position vector as lat, lon, alt — user may prefer ENU; here we just treat components independently
                double pos_vec[3] = { heads[i]->s.lat, heads[i]->s.lon, heads[i]->s.alt };
                double vel_vec[3] = { heads[i]->s.vn, heads[i]->s.ve, heads[i]->s.vu };
                accumulate_weighted(Wpos, pos_vec, sumW_pos, sumWv_pos);
                accumulate_weighted(Wvel, vel_vec, sumW_vel, sumWv_vel);
                count++;
                heads[i] = heads[i]->next;
            }
        }

        if (count == 0) {
            // advance one head to avoid infinite loop
            for (int i=0;i<N;++i) {
                if (gps_heads[i] && gps_heads[i]->s.t == tmin) {
                    gps_heads[i] = gps_heads[i]->next;
                    break;
                }
            }
            continue;
        }

        double Rpos_fused[9], Rvel_fused[9];
        if (mat_inv_3x3(sumW_pos, Rpos_fused) != 0) {
            double diag[3] = {1.0/(sumW_pos[0]+1e-12), 1.0/(sumW_pos[4]+1e-12), 1.0/(sumW_pos[8]+1e-12)};
            build_diag3(diag, Rpos_fused);
        }
        if (mat_inv_3x3(sumW_vel, Rvel_fused) != 0) {
            double diag[3] = {1.0/(sumW_vel[0]+1e-12), 1.0/(sumW_vel[4]+1e-12), 1.0/(sumW_vel[8]+1e-12)};
            build_diag3(diag, Rvel_fused);
        }

        double pos_fused[3], vel_fused[3];
        mat3_vec(Rpos_fused, sumWv_pos, pos_fused);
        mat3_vec(Rvel_fused, sumWv_vel, vel_fused);

        struct fused_gps *node = malloc(sizeof(*node));
        if (!node) break;
        node->t = tmin;
        node->lat = pos_fused[0]; node->lon = pos_fused[1]; node->alt = pos_fused[2];
        node->vn = vel_fused[0]; node->ve = vel_fused[1]; node->vu = vel_fused[2];
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
