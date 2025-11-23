#include "../include/ekf.h"

static const double GRAVITY[3] = { 0.0, 0.0, -9.81 };

void ekf_build_process_noise(double Qd[P_DIM], double dt, double sigma_g, double sigma_a, double sigma_bg_rw, double sigma_ba_rw)
{
    for (size_t i=0;i<P_DIM;++i)
        Qd[i]=0.0;

    double var_g  = (sigma_g * sigma_g) * dt;
    double var_a  = (sigma_a * sigma_a) * dt;
    double var_bg = (sigma_bg_rw * sigma_bg_rw) * dt;
    double var_ba = (sigma_ba_rw * sigma_ba_rw) * dt;

    for (int i=0;i<3;++i)
        Qd[IDX(6+i,6+i)] = var_g;
    for (int i=0;i<3;++i)
        Qd[IDX(3+i,3+i)] = var_a;
    for (int i=0;i<3;++i)
        Qd[IDX(9+i,9+i)] = var_bg;
    for (int i=0;i<3;++i)
        Qd[IDX(12+i,12+i)] = var_ba;
}

/* ekf_update_gps: uses ekf_update_block for position (index 0) and velocity (index 3) */
int ekf_update_gps(struct ekf_state *x, double P[P_DIM], const double gps_pos_enu[3], const double gps_vel_enu[3],
                   const double Rpos[9], const double Rvel[9], double *out_mahalanobis_pos, int *out_accepted_pos,
                   double *out_mahalanobis_vel, int *out_accepted_vel)
{
    /* Position update */
    double hpos[3] = { x->p[0], x->p[1], x->p[2] };
    int rc = ekf_update_block(x, P, gps_pos_enu, hpos, Rpos, 0, MAHALANOBIS_THRESH_POS,
                              out_mahalanobis_pos, out_accepted_pos);
    if (rc < 0)
        return rc;

    /* Velocity update */
    double hvel[3] = { x->v[0], x->v[1], x->v[2] };
    rc = ekf_update_block(x, P, gps_vel_enu, hvel, Rvel, 3, MAHALANOBIS_THRESH_VEL,
                          out_mahalanobis_vel, out_accepted_vel);
    if (rc < 0)
        return rc;

    return 0;
}

/* ekf_init: identical semantics as before but with the larger initial covariance you decided on */
void ekf_init(struct ekf_state *x, double P[P_DIM], double t0,
              const double *init_pos_enu, const double *init_vel_enu)
{
    x->t = t0;

    if (init_pos_enu)
    {
        x->p[0] = init_pos_enu[0];
        x->p[1] = init_pos_enu[1];
        x->p[2] = init_pos_enu[2];
    } 
    else
        x->p[0] = x->p[1] = x->p[2] = 0.0;

    if (init_vel_enu)
    {
        x->v[0] = init_vel_enu[0];
        x->v[1] = init_vel_enu[1];
        x->v[2] = init_vel_enu[2];
    }
    else
        x->v[0] = x->v[1] = x->v[2] = 0.0;

    x->q[0] = 1.0; x->q[1] = x->q[2] = x->q[3] = 0.0;

    memset(x->bg, 0, sizeof(x->bg));
    memset(x->ba, 0, sizeof(x->ba));

    for (size_t i=0;i<P_DIM;++i)
        P[i]=0.0;

    double ang_var = pow(1.0 * (M_PI / 180.0), 2);  /* 1 deg^2 */
    double bg_var = pow(1e-3, 2);
    double ba_var = pow(1e-2, 2);

    /* more permissive initial covariance to avoid initial rejections */
    P[IDX(0,0)] = 100.0; P[IDX(1,1)] = 100.0; P[IDX(2,2)] = 100.0;
    P[IDX(3,3)] = 10.0;  P[IDX(4,4)] = 10.0;  P[IDX(5,5)] = 10.0;

    P[IDX(6,6)] = ang_var; P[IDX(7,7)] = ang_var; P[IDX(8,8)] = ang_var;

    P[IDX(9,9)]   = bg_var;  P[IDX(10,10)] = bg_var;  P[IDX(11,11)] = bg_var;
    P[IDX(12,12)] = ba_var;  P[IDX(13,13)] = ba_var;  P[IDX(14,14)] = ba_var;
}

/* ekf_propagate unchanged except references to geodetic removed (no change in algorithm) */
int ekf_propagate(struct ekf_state *x, double P[P_DIM], const struct imu_sample *imu, double dt)
{
    if (dt <= 0.0)
        return -1;

    double omega_b[3] = {
        imu->wx - x->bg[0],
        imu->wy - x->bg[1],
        imu->wz - x->bg[2]
    };

    double acc_b[3] = {
        imu->ax - x->ba[0],
        imu->ay - x->ba[1],
        imu->az - x->ba[2]
    };

    /* quaternion increment and integrate */
    double dq[4];
    quat_from_omega(omega_b, dt, dq);

    double q_new[4];
    quat_mul(dq, x->q, q_new);

    double qn = sqrt(q_new[0]*q_new[0] + q_new[1]*q_new[1] + q_new[2]*q_new[2] + q_new[3]*q_new[3]);
    if (qn > 0.0)
    {
        for (int i=0;i<4;++i)
            q_new[i] /= qn;
    }
    else
    {
        memcpy(q_new, x->q, 4*sizeof(double));
    }

    double Rwb[9];
    quat_to_rotmat(q_new, Rwb);

    double a_w[3];
    mat_vec_mult(3, 3, Rwb, acc_b, a_w);
    a_w[0] += GRAVITY[0]; a_w[1] += GRAVITY[1]; a_w[2] += GRAVITY[2];

    x->v[0] += a_w[0] * dt;
    x->v[1] += a_w[1] * dt;
    x->v[2] += a_w[2] * dt;

    x->p[0] += x->v[0] * dt;
    x->p[1] += x->v[1] * dt;
    x->p[2] += x->v[2] * dt;

    x->q[0] = q_new[0]; x->q[1] = q_new[1]; x->q[2] = q_new[2]; x->q[3] = q_new[3];
    x->t += dt;

    /* Covariance propagation */
    double Fc[P_DIM];
    for (int i=0;i<P_DIM;++i)
        Fc[i] = 0.0;

    for (int i=0;i<3;++i)
        Fc[IDX(0+i,3+i)] = 1.0;

    double Sacc[9];
    skew(acc_b, Sacc);

    double tmp[9];
    mat_mult(3,3,3, Rwb, Sacc, tmp);
    for (int r=0;r<3;++r)
        for (int c=0;c<3;++c)
            Fc[IDX(3+r,6+c)] = - tmp[r*3 + c];

    for (int r=0;r<3;++r)
        for (int c=0;c<3;++c)
            Fc[IDX(3+r,12+c)] = - Rwb[r*3 + c];

    for (int i=0;i<3;++i)
        Fc[IDX(6+i,9+i)] = -1.0;

    double Fd[P_DIM];
    for (int i=0;i<P_DIM;++i)
        Fd[i] = Fc[i] * dt;
    for (int i=0;i<STATE_DIM;++i)
        Fd[IDX(i,i)] += 1.0;

    double Qd[P_DIM];
    const double sigma_g = 0.01;
    const double sigma_a = 0.1;
    const double sigma_bg_rw = 1e-5;
    const double sigma_ba_rw = 1e-3;

    ekf_build_process_noise(Qd, dt, sigma_g, sigma_a, sigma_bg_rw, sigma_ba_rw);

    double tmpA[P_DIM], Ft[P_DIM], tmpB[P_DIM];

    mat_mult(STATE_DIM, STATE_DIM, STATE_DIM, Fd, P, tmpA);
    mat_transpose(STATE_DIM, STATE_DIM, Fd, Ft);
    mat_mult(STATE_DIM, STATE_DIM, STATE_DIM, tmpA, Ft, tmpB);

    for (int i=0;i<P_DIM;++i)
        P[i] = tmpB[i] + Qd[i];

    for (int r=0;r<STATE_DIM;++r)
        for (int c=r+1;c<STATE_DIM;++c)
        {
            double avg = 0.5*(P[IDX(r,c)] + P[IDX(c,r)]);
            P[IDX(r,c)] = P[IDX(c,r)] = avg;
        }

    return 0;
}
