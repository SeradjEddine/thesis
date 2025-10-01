#include "ekf.h"
#include "vec.h"   // assumed functions: vec_copy, vec_scale, vec_add, vec_sub
#include "mat.h"   // mat_set_identity, mat_mult, mat_transpose, mat_add, mat_scale, mat_copy
#include "mat_inv.h"
#include "quat.h"  // quat_from_omega, quat_mul, quat_normalize, quat_to_rotmat
#include "ekf_utils.h" // skew_symmetric

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

    // initialize covariance P (15x15)
    for (int i=0;i<P_DIM;++i) P[i]=0.0;
    // small uncertainty on initial pose
    // position (idx 0..2)
    P[0*STATE_DIM + 0] = 1.0; // 1 m^2
    P[1*STATE_DIM + 1] = 1.0;
    P[2*STATE_DIM + 2] = 1.0;
    // velocity (3..5)
    P[3*STATE_DIM + 3] = 0.1; // (m/s)^2
    P[4*STATE_DIM + 4] = 0.1;
    P[5*STATE_DIM + 5] = 0.1;
    // orientation error (3..5 mapping) - represent small-angle variance
    P[6*STATE_DIM + 6] = (1.0*(M_PI/180.0))*(1.0*(M_PI/180.0)); // 1 deg^2 → rad^2
    P[7*STATE_DIM + 7] = P[6*STATE_DIM + 6];
    P[8*STATE_DIM + 8] = P[6*STATE_DIM + 6];

    // gyro bias (9..11)
    P[9*STATE_DIM + 9] = (1e-3)*(1e-3);
    P[10*STATE_DIM + 10] = (1e-3)*(1e-3);
    P[11*STATE_DIM + 11] = (1e-3)*(1e-3);

    // accel bias (12..14)
    P[12*STATE_DIM + 12] = (1e-2)*(1e-2);
    P[13*STATE_DIM + 13] = (1e-2)*(1e-2);
    P[14*STATE_DIM + 14] = (1e-2)*(1e-2);
}

/*
 * ekf_propagate:
 *  - Integrate the nominal state using IMU (body-frame accel & gyro)
 *  - Propagate covariance with linearized F, and discrete Qd
 *  - Uses first-order discretization Fd = I + F*dt
 *
 * Returns 0 on success.
 */
int ekf_propagate(struct ekf_state *x, double P[P_DIM], const struct imu_sample *imu, double dt)
{
    if (dt <= 0.0) return -1;

    // 1) Correct raw measurements with biases
    double omega_b[3] = { imu->gx - x->bg[0],
                          imu->gy - x->bg[1],
                          imu->gz - x->bg[2] };

    double acc_b[3] = { imu->ax - x->ba[0],
                        imu->ay - x->ba[1],
                        imu->az - x->ba[2] };

    // 2) Quaternion update: q <- q ⊗ exp(0.5 * omega * dt) style small-angle
    double dq[4];
    quat_from_omega(omega_b, dt, dq);      // dq represents rotation over dt
    double q_new[4];
    quat_mul(dq, x->q, q_new);
    quat_normalize(q_new);

    // 3) Compute rotation matrix R = R(q_new) (world <- body)
    double Rwb[9];
    quat_to_rotmat(q_new, Rwb);

    // 4) Acceleration in world frame: a_w = Rwb * acc_b + g
    double a_w[3];
    mat_vec_mult(3, 3, Rwb, acc_b, a_w); // Rwb * acc_b
    a_w[0] += GRAVITY[0];
    a_w[1] += GRAVITY[1];
    a_w[2] += GRAVITY[2];

    // 5) Update nominal state via simple Euler integration (could use mid-point for better accuracy)
    // v <- v + a_w * dt
    x->v[0] += a_w[0] * dt;
    x->v[1] += a_w[1] * dt;
    x->v[2] += a_w[2] * dt;

    // p <- p + v * dt  (uses updated v: semi-implicit Euler)
    x->p[0] += x->v[0] * dt;
    x->p[1] += x->v[1] * dt;
    x->p[2] += x->v[2] * dt;

    // assign updated quaternion and timestamp
    x->q[0]=q_new[0]; x->q[1]=q_new[1]; x->q[2]=q_new[2]; x->q[3]=q_new[3];
    x->t += dt;

    // ---------- Covariance propagation ----------
    // We build continuous-time F_c (15x15) linearized jacobian blocks and discretize with Fd = I + F_c*dt
    double Fc[STATE_DIM * STATE_DIM];
    for (int i=0;i<STATE_DIM*STATE_DIM;++i) Fc[i]=0.0;

    // Indices:
    // p: 0..2, v:3..5, theta_err:6..8, bg:9..11, ba:12..14

    // dp/dv = I (p_dot = v) -> partial p / partial v = I
    // so Fc (0:3,3:6) = I3
    for (int i=0;i<3;++i) Fc[(0+i)*STATE_DIM + (3+i)] = 1.0;

    // dv/dtheta = -R * skew(acc_b)  (because dv = R*(acc_b) + g; derivative wrt small rotation)
    double S_acc[9];
    skew_symmetric(acc_b, S_acc); // skew(acc_b)
    // compute -Rwb * S_acc  (3x3)
    double tmp3x3[9];
    mat_mult(3,3,3, Rwb, S_acc, tmp3x3); // Rwb * skew(acc_b)
    // put negative into Fc rows 3..5 cols 6..8
    for (int r=0;r<3;++r)
        for (int c=0;c<3;++c)
            Fc[(3+r)*STATE_DIM + (6+c)] = - tmp3x3[r*3 + c];

    // dv/dba = -Rwb (since acc_b = accel - ba)
    // dv/dba = -Rwb -> place at rows 3..5 cols 12..14
    for (int r=0;r<3;++r)
        for (int c=0;c<3;++c)
            Fc[(3+r)*STATE_DIM + (12+c)] = - Rwb[r*3 + c];

    // dtheta_dbg = -I  (orientation error derivative wrt gyro bias)
    // small-angle theta_dot ≈ - (I) * dbg  -> rows 6..8 cols 9..11 = -I
    for (int i=0;i<3;++i) Fc[(6+i)*STATE_DIM + (9+i)] = -1.0;

    // dbg/d(??) = 0 ; dba/d(??) = 0 ; biases random-walk handled in Q

    // Discretize Fd = I + Fc * dt (first order)
    double Fd[STATE_DIM * STATE_DIM];
    for (int i=0;i<STATE_DIM*STATE_DIM;++i) Fd[i] = Fc[i] * dt;
    for (int i=0;i<STATE_DIM;++i) Fd[i*STATE_DIM + i] += 1.0;

    // Build process noise Qd (15x15) using helper (user can tune sigmas)
    double Qd[P_DIM];
    // typical noise params (you should expose these for tuning)
    const double sigma_g = 0.01;        // rad/s / sqrt(sec) - gyro noise (example)
    const double sigma_a = 0.1;         // m/s^2 / sqrt(sec) - accel noise
    const double sigma_bg_rw = 1e-5;    // gyro bias random walk (rad/s / sqrt(sec))
    const double sigma_ba_rw = 1e-3;    // accel bias random walk (m/s^2 / sqrt(sec))
    ekf_build_process_noise(Qd, dt, sigma_g, sigma_a, sigma_bg_rw, sigma_ba_rw);

    // P <- Fd * P * Fd^T + Qd;
    double tmpA[STATE_DIM * STATE_DIM];
    double Ft[STATE_DIM * STATE_DIM];

    mat_mult(STATE_DIM, STATE_DIM, STATE_DIM, Fd, P, tmpA);       // tmpA = Fd * P
    mat_transpose(STATE_DIM, STATE_DIM, Fd, Ft);                 // Ft = Fd^T
    double tmpB[STATE_DIM * STATE_DIM];
    mat_mult(STATE_DIM, STATE_DIM, STATE_DIM, tmpA, Ft, tmpB);   // tmpB = Fd*P*Fd^T

    // P = tmpB + Qd
    for (int i=0;i<STATE_DIM*STATE_DIM;++i) P[i] = tmpB[i] + Qd[i];

    // small numerical fix: enforce symmetry
    for (int r=0;r<STATE_DIM;++r){
        for (int c=r+1;c<STATE_DIM;++c){
            double avg = 0.5*(P[r*STATE_DIM + c] + P[c*STATE_DIM + r]);
            P[r*STATE_DIM + c] = avg;
            P[c*STATE_DIM + r] = avg;
        }
    }

    return 0;
}

/*
 * ekf_build_process_noise:
 *   builds diagonal-ish Qd for discrete-time process.
 *   We place gyro & accel noise affecting orientation & velocity, and RW terms for biases.
 */
void ekf_build_process_noise(double Qd[P_DIM], double dt,
                             double sigma_g, double sigma_a, double sigma_bg_rw, double sigma_ba_rw)
{
    // Zero
    for (int i=0;i<P_DIM;++i) Qd[i]=0.0;

    // Orientation injected by gyro noise -> contributes to theta error covariance (indices 6..8)
    double var_g = (sigma_g*sigma_g) * dt; // variance over dt
    for (int i=0;i<3;++i) Qd[(6+i)*STATE_DIM + (6+i)] = var_g;

    // Velocity injected by accel noise -> indices 3..5
    double var_a = (sigma_a*sigma_a) * dt;
    for (int i=0;i<3;++i) Qd[(3+i)*STATE_DIM + (3+i)] = var_a;

    // biases random walks
    double var_bg = (sigma_bg_rw*sigma_bg_rw) * dt;
    double var_ba = (sigma_ba_rw*sigma_ba_rw) * dt;
    for (int i=0;i<3;++i){
        Qd[(9+i)*STATE_DIM + (9+i)] = var_bg;
        Qd[(12+i)*STATE_DIM + (12+i)] = var_ba;
    }
}

