#include "ekf.h"
#include "sensors.h"     // struct imu_sample
#include "mathlib.h"     // your matrix/vec/quaternion functions + vec_dot, vec_norm, mat_inverse_2x2
#include <string.h>
#include <math.h>

static const double GRAVITY[3] = {0.0, 0.0, -9.81};

void ekf_init(struct ekf_state *x, double P[P_DIM], double t0)
{
    x->t = t0;
    x->p[0]=x->p[1]=x->p[2]=0.0;
    x->v[0]=x->v[1]=x->v[2]=0.0;
    x->q[0]=1.0; x->q[1]=x->q[2]=x->q[3]=0.0;
    x->bg[0]=x->bg[1]=x->bg[2]=0.0;
    x->ba[0]=x->ba[1]=x->ba[2]=0.0;

    /* Zero P */
    for (size_t i=0;i<P_DIM;++i) P[i]=0.0;

    /* Reasonable initial uncertainties */
    P[IDX(0,0)] = 1.0; P[IDX(1,1)] = 1.0; P[IDX(2,2)] = 1.0;      // position m^2
    P[IDX(3,3)] = 0.1; P[IDX(4,4)] = 0.1; P[IDX(5,5)] = 0.1;      // velocity (m/s)^2
    double ang_var = (1.0 * (M_PI/180.0))*(1.0 * (M_PI/180.0));  // 1 deg^2 in rad^2
    P[IDX(6,6)] = ang_var; P[IDX(7,7)] = ang_var; P[IDX(8,8)] = ang_var;
    double bg_var = (1e-3)*(1e-3);
    P[IDX(9,9)] = bg_var; P[IDX(10,10)] = bg_var; P[IDX(11,11)] = bg_var;
    double ba_var = (1e-2)*(1e-2);
    P[IDX(12,12)] = ba_var; P[IDX(13,13)] = ba_var; P[IDX(14,14)] = ba_var;
}

/* Build simple block-diagonal Qd (discrete) from noise densities */
void ekf_build_process_noise(double Qd[P_DIM], double dt,
                             double sigma_g, double sigma_a,
                             double sigma_bg_rw, double sigma_ba_rw)
{
    for (size_t i=0;i<P_DIM;++i) Qd[i]=0.0;

    double var_g = (sigma_g * sigma_g) * dt;
    double var_a = (sigma_a * sigma_a) * dt;
    double var_bg = (sigma_bg_rw * sigma_bg_rw) * dt;
    double var_ba = (sigma_ba_rw * sigma_ba_rw) * dt;

    /* orientation (theta) indices 6..8 */
    for (int i=0;i<3;++i) Qd[IDX(6+i,6+i)] = var_g;
    /* velocity 3..5 */
    for (int i=0;i<3;++i) Qd[IDX(3+i,3+i)] = var_a;
    /* biases */
    for (int i=0;i<3;++i) Qd[IDX(9+i,9+i)] = var_bg;
    for (int i=0;i<3;++i) Qd[IDX(12+i,12+i)] = var_ba;
}

/* Quaternion utility: create quaternion increment from angular rate omega (rad/s) */
static void quat_from_omega_local(const double omega[3], double dt, double q_delta[4])
{
    /* small-angle quaternion using axis-angle */
    double angle = sqrt(omega[0]*omega[0] + omega[1]*omega[1] + omega[2]*omega[2]) * dt;
    if (angle < 1e-8) {
        /* approximate: q = [1, 0.5*omega*dt] */
        q_delta[0] = 1.0;
        q_delta[1] = 0.5 * omega[0] * dt;
        q_delta[2] = 0.5 * omega[1] * dt;
        q_delta[3] = 0.5 * omega[2] * dt;
    } else {
        double ux = (omega[0]) / (angle/dt);
        double uy = (omega[1]) / (angle/dt);
        double uz = (omega[2]) / (angle/dt);
        double half = 0.5 * angle;
        q_delta[0] = cos(half);
        double s = sin(half);
        q_delta[1] = ux * s;
        q_delta[2] = uy * s;
        q_delta[3] = uz * s;
    }
    /* normalize */
    double n = sqrt(q_delta[0]*q_delta[0] + q_delta[1]*q_delta[1] + q_delta[2]*q_delta[2] + q_delta[3]*q_delta[3]);
    if (n > 0.0) {
        q_delta[0] /= n; q_delta[1] /= n; q_delta[2] /= n; q_delta[3] /= n;
    }
}

/* Multiply quaternion q1 * q2 -> out (w,x,y,z) */
static void quat_mul_local(const double a[4], const double b[4], double out[4])
{
    out[0] = a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3];
    out[1] = a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2];
    out[2] = a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1];
    out[3] = a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0];
}

/* Convert quaternion (w,x,y,z) to 3x3 rotation matrix (row-major) */
static void quat_to_rotmat_local(const double q[4], double R[9])
{
    double w=q[0], x=q[1], y=q[2], z=q[3];
    double ww = w*w, xx = x*x, yy = y*y, zz = z*z;
    R[0] = ww + xx - yy - zz;
    R[1] = 2*(x*y - w*z);
    R[2] = 2*(x*z + w*y);
    R[3] = 2*(x*y + w*z);
    R[4] = ww - xx + yy - zz;
    R[5] = 2*(y*z - w*x);
    R[6] = 2*(x*z - w*y);
    R[7] = 2*(y*z + w*x);
    R[8] = ww - xx - yy + zz;
}

/* skew symmetric for v -> S (3x3 row-major) */
static void skew_local(const double v[3], double S[9])
{
    S[0]=0.0;   S[1]=-v[2]; S[2]=v[1];
    S[3]=v[2];  S[4]=0.0;   S[5]=-v[0];
    S[6]=-v[1]; S[7]=v[0];  S[8]=0.0;
}

/* ekf_propagate: integrates nominal state and propagates covariance */
int ekf_propagate(struct ekf_state *x, double P[P_DIM],
                  const struct imu_sample *imu,
                  double dt)
{
    if (dt <= 0.0) return -1;

    /* 1) bias-corrected measurements */
    double omega_b[3] = { imu->ax - x->bg[0],
                          imu->ay - x->bg[1],
                          imu->az - x->bg[2] };

    double acc_b[3] = { imu->ax - x->ba[0],
                        imu->ay - x->ba[1],
                        imu->az - x->ba[2] };

    /* 2) quaternion increment */
    double dq[4];
    quat_from_omega_local(omega_b, dt, dq);

    double q_new[4];
    quat_mul_local(dq, x->q, q_new);
    /* normalize */
    double qn = sqrt(q_new[0]*q_new[0] + q_new[1]*q_new[1] + q_new[2]*q_new[2] + q_new[3]*q_new[3]);
    if (qn > 0.0) {
        q_new[0] /= qn; q_new[1] /= qn; q_new[2] /= qn; q_new[3] /= qn;
    } else {
        /* fallback: keep old */
        memcpy(q_new, x->q, 4*sizeof(double));
    }

    /* 3) rotation matrix */
    double Rwb[9];
    quat_to_rotmat_local(q_new, Rwb);

    /* 4) acceleration in world */
    double a_w[3];
    mat_vec_mult(3, 3, Rwb, acc_b, a_w); // mat_vec_mult(r,c,A,v,out)
    a_w[0] += GRAVITY[0]; a_w[1] += GRAVITY[1]; a_w[2] += GRAVITY[2];

    /* 5) semi-implicit Euler integration */
    x->v[0] += a_w[0] * dt;
    x->v[1] += a_w[1] * dt;
    x->v[2] += a_w[2] * dt;

    x->p[0] += x->v[0] * dt;
    x->p[1] += x->v[1] * dt;
    x->p[2] += x->v[2] * dt;

    /* update quaternion and time */
    x->q[0]=q_new[0]; x->q[1]=q_new[1]; x->q[2]=q_new[2]; x->q[3]=q_new[3];
    x->t += dt;

    /* ----- Covariance propagation ----- */
    /* Build continuous-time Jacobian Fc (15x15), then Fd = I + Fc*dt */
    double Fc[P_DIM];
    for (int i=0;i<P_DIM;++i) Fc[i]=0.0;

    /* dp/dv = I */
    for (int i=0;i<3;++i) Fc[IDX(0+i, 3+i)] = 1.0;

    /* dv/dtheta = -R * skew(acc_b)  */
    double Sacc[9];
    skew_local(acc_b, Sacc);
    double tmp[9];
    mat_mult(3,3,3, Rwb, Sacc, tmp); // tmp = Rwb * skew(acc_b)
    for (int r=0;r<3;++r)
        for (int c=0;c<3;++c)
            Fc[IDX(3+r, 6+c)] = - tmp[r*3 + c];

    /* dv/dba = -Rwb */
    for (int r=0;r<3;++r)
        for (int c=0;c<3;++c)
            Fc[IDX(3+r, 12+c)] = - Rwb[r*3 + c];

    /* dtheta/dbg = -I  (orientation error derivative w.r.t gyro bias) */
    for (int i=0;i<3;++i) Fc[IDX(6+i, 9+i)] = -1.0;

    /* Discretize */
    double Fd[P_DIM];
    for (int i=0;i<P_DIM;++i) Fd[i] = Fc[i] * dt;
    for (int i=0;i<STATE_DIM;++i) Fd[IDX(i,i)] += 1.0;

    /* Build Qd */
    double Qd[P_DIM];
    /* Default params; you can expose these to tune */
    const double sigma_g = 0.01;
    const double sigma_a = 0.1;
    const double sigma_bg_rw = 1e-5;
    const double sigma_ba_rw = 1e-3;
    ekf_build_process_noise(Qd, dt, sigma_g, sigma_a, sigma_bg_rw, sigma_ba_rw);

    /* P = Fd * P * Fd^T + Qd */
    double tmpA[P_DIM];
    double Ft[P_DIM];
    mat_mult(STATE_DIM, STATE_DIM, STATE_DIM, Fd, P, tmpA);    // tmpA = Fd * P
    mat_transpose(STATE_DIM, STATE_DIM, Fd, Ft);               // Ft = Fd^T
    double tmpB[P_DIM];
    mat_mult(STATE_DIM, STATE_DIM, STATE_DIM, tmpA, Ft, tmpB); // tmpB = Fd*P*Fd^T

    for (int i=0;i<P_DIM;++i) P[i] = tmpB[i] + Qd[i];

    /* enforce symmetry */
    for (int r=0;r<STATE_DIM;++r){
        for (int c=r+1;c<STATE_DIM;++c){
            double avg = 0.5*(P[IDX(r,c)] + P[IDX(c,r)]);
            P[IDX(r,c)] = avg;
            P[IDX(c,r)] = avg;
        }
    }

    return 0;
}

