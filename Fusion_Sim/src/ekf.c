#include "../include/ekf.h"
#include "../include/sensors.h"
#include "mathlib/mathlib.h"
#include <math.h>
#include <string.h>

static const double GRAVITY[3] = {0.0, 0.0, -9.81};

/* ---- WGS84 / ECEF helpers for lat/lon -> ENU ----
   We'll convert lat/lon/alt to ECEF then subtract the reference ECEF and rotate to ENU.
   Use first GPS sample as reference; call ekf_init_gps_ref(lat0,lon0,alt0).
*/

static double gps_ref_lat = 0.0;
static double gps_ref_lon = 0.0;
static double gps_ref_alt = 0.0;
static double gps_ref_ecef[3];
static double enu_rot[9];           // rotation matrix from ECEF->ENU at ref (row-major)

static const double WGS84_a = 6378137.0;          // semi-major axis

//static const double WGS84_f = 1.0 / 298.257223563;
//static const double WGS84_e2 = WGS84_f * (2.0 - WGS84_f);
#define WGS84_f  (0.0033528106647474805)
static const double WGS84_e2 = WGS84_f * (2.0 - WGS84_f);

static void geodetic_to_ecef(double lat_rad, double lon_rad, double alt, double ecef[3]) 
{
    double cos_lat = cos(lat_rad);
    double sin_lat = sin(lat_rad);
    double cos_lon = cos(lon_rad);
    double sin_lon = sin(lon_rad);
    double N = WGS84_a / sqrt(1.0 - WGS84_e2 * sin_lat * sin_lat);

    ecef[0] = (N + alt) * cos_lat * cos_lon;
    ecef[1] = (N + alt) * cos_lat * sin_lon;
    ecef[2] = ( (1.0 - WGS84_e2) * N + alt ) * sin_lat;
}

void ekf_init_gps_ref(double lat0_deg, double lon0_deg, double alt0_m)
{
    gps_ref_lat = lat0_deg; 
    gps_ref_lon = lon0_deg; 
    gps_ref_alt = alt0_m;

    double lat0 = gps_ref_lat * M_PI / 180.0;
    double lon0 = gps_ref_lon * M_PI / 180.0;

    geodetic_to_ecef(lat0, lon0, gps_ref_alt, gps_ref_ecef);

    double s_lat = sin(lat0), c_lat = cos(lat0);
    double s_lon = sin(lon0), c_lon = cos(lon0);

    /* Build rotation matrix from ECEF to ENU at reference:
       ENU = R * (ECEF - ecef_ref)
       Rows: e = [         -sin(lon),           cos(lon),   0     ]
             n = [-sin(lat)*cos(lon), -sin(lat)*sin(lon), cos(lat)]
             u = [ cos(lat)*cos(lon),  cos(lat)*sin(lon), sin(lat)]
       We'll store as row-major R (3x3).
    */

    enu_rot[0] = -s_lon;           enu_rot[1] =  c_lon;          enu_rot[2] = 0.0;
    enu_rot[3] = -s_lat * c_lon;   enu_rot[4] = -s_lat * s_lon;  enu_rot[5] = c_lat;
    enu_rot[6] =  c_lat * c_lon;   enu_rot[7] =  c_lat * s_lon;  enu_rot[8] = s_lat;
}

void latlon_to_enu(double lat_deg, double lon_deg, double alt_m, double enu_out[3])
{
    double lat = lat_deg * M_PI / 180.0;
    double lon = lon_deg * M_PI / 180.0;
    double ecef[3];
    
    geodetic_to_ecef(lat, lon, alt_m, ecef);
    
    double d[3] = { ecef[0] - gps_ref_ecef[0], ecef[1] - gps_ref_ecef[1], ecef[2] - gps_ref_ecef[2] };

    /* ENU = enu_rot * d */
    mat_vec_mult(3, 3, enu_rot, d, enu_out);
}

/* ekf_update_gps:
   We perform sequential updates:
     1) position measurement update (3D)
     2) velocity measurement update (3D)
   For each: compute innovation, S, Kalman gain, gating, update state and P.
*/


int ekf_update_gps(struct ekf_state *x, double P[P_DIM],
                   const double gps_pos_enu[3], const double gps_vel_enu[3],
                   const double Rpos[9], const double Rvel[9],
                   double *out_mahalanobis_pos, int *out_accepted_pos,
                   double *out_mahalanobis_vel, int *out_accepted_vel)
{

    // --- Safety: handle unused params in partial builds
    (void)gps_vel_enu;
    (void)out_mahalanobis_vel;
    (void)out_accepted_vel;

    /* ===================== 1) POSITION UPDATE ===================== */
    double zpos[3] = { gps_pos_enu[0], gps_pos_enu[1], gps_pos_enu[2] };
    double hpos[3] = { x->p[0], x->p[1], x->p[2] };
    double ypos[3];
    for (int i = 0; i < 3; ++i)
        ypos[i] = zpos[i] - hpos[i];

    double Ppp[9], Spos[9], Spos_inv[9];
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            Ppp[r*3 + c] = P[IDX(r, c)];

    for (int k = 0; k < 9; ++k)
        Spos[k] = Ppp[k] + Rpos[k];

    if (mat_inverse_3x3(Spos, Spos_inv) != 0) {
        if (out_mahalanobis_pos) *out_mahalanobis_pos = 1e9;
        if (out_accepted_pos) *out_accepted_pos = 0;
        return -1;
    }

    // --- Mahalanobis test for position ---
    double tmpv[3];
    mat_vec_mult(3, 3, Spos_inv, ypos, tmpv);
    double d2_pos = ypos[0]*tmpv[0] + ypos[1]*tmpv[1] + ypos[2]*tmpv[2];
    if (out_mahalanobis_pos) *out_mahalanobis_pos = d2_pos;

    if (d2_pos > MAHALANOBIS_THRESH_POS) {
        if (out_accepted_pos) *out_accepted_pos = 0;
    } else {
        if (out_accepted_pos) *out_accepted_pos = 1;

        /* Compute K = P(:,0:2) * Spos_inv  (15x3) */
        double P_col03[STATE_DIM * 3];
        for (int r = 0; r < STATE_DIM; ++r)
            for (int c = 0; c < 3; ++c)
                P_col03[r*3 + c] = P[IDX(r, c)];

        double K[STATE_DIM * 3];
        mat_mult(STATE_DIM, 3, 3, P_col03, Spos_inv, K);

        double dx[STATE_DIM];
        for (int r = 0; r < STATE_DIM; ++r) {
            double sum = 0.0;
            for (int c = 0; c < 3; ++c) sum += K[r*3 + c] * ypos[c];
            dx[r] = sum;
        }

        // --- Apply correction ---
        for (int i = 0; i < 3; ++i) x->p[i] += dx[i];
        for (int i = 0; i < 3; ++i) x->v[i] += dx[3+i];

        // --- Orientation small-angle update ---
        double dtheta[3] = { dx[6], dx[7], dx[8] };
        double qd[4] = { 1.0, 0.5*dtheta[0], 0.5*dtheta[1], 0.5*dtheta[2] };
        double qd_norm = sqrt(qd[0]*qd[0] + qd[1]*qd[1] + qd[2]*qd[2] + qd[3]*qd[3]);
        if (qd_norm > 0) for (int i=0; i<4; ++i) qd[i] /= qd_norm;

        double qnew[4];
        qnew[0] = qd[0]*x->q[0] - qd[1]*x->q[1] - qd[2]*x->q[2] - qd[3]*x->q[3];
        qnew[1] = qd[0]*x->q[1] + qd[1]*x->q[0] + qd[2]*x->q[3] - qd[3]*x->q[2];
        qnew[2] = qd[0]*x->q[2] - qd[1]*x->q[3] + qd[2]*x->q[0] + qd[3]*x->q[1];
        qnew[3] = qd[0]*x->q[3] + qd[1]*x->q[2] - qd[2]*x->q[1] + qd[3]*x->q[0];
        double nq = sqrt(qnew[0]*qnew[0]+qnew[1]*qnew[1]+qnew[2]*qnew[2]+qnew[3]*qnew[3]);
        if (nq > 0) for (int i=0;i<4;++i) x->q[i] = qnew[i]/nq;

        for (int i = 0; i < 3; ++i) x->bg[i] += dx[9+i];
        for (int i = 0; i < 3; ++i) x->ba[i] += dx[12+i];

        // --- Covariance update ---
        double P_first3rows[3 * STATE_DIM];
        for (int r=0; r<3; ++r)
            for (int c=0; c<STATE_DIM; ++c)
                P_first3rows[r*STATE_DIM + c] = P[IDX(r,c)];

        double KH[STATE_DIM * STATE_DIM];
        mat_mult(STATE_DIM, 3, STATE_DIM, K, P_first3rows, KH);
        for (int i=0;i<P_DIM;++i) P[i] -= KH[i];

        double KR[STATE_DIM * 3];
        mat_mult(STATE_DIM, 3, 3, K, Rpos, KR);
        double Kt[3 * STATE_DIM];
        for (int r=0; r<3; ++r)
            for (int c=0; c<STATE_DIM; ++c)
                Kt[r*STATE_DIM + c] = K[c*3 + r];

        double KRKT[STATE_DIM * STATE_DIM];
        mat_mult(STATE_DIM, 3, STATE_DIM, KR, Kt, KRKT);
        for (int i=0;i<P_DIM;++i) P[i] += KRKT[i];

        for (int r=0;r<STATE_DIM;++r)
            for (int c=r+1;c<STATE_DIM;++c) {
                double avg = 0.5*(P[IDX(r,c)] + P[IDX(c,r)]);
                P[IDX(r,c)] = P[IDX(c,r)] = avg;
            }
    }

    /* ===================== 2) VELOCITY UPDATE ===================== */
    double zvel[3] = { gps_vel_enu[0], gps_vel_enu[1], gps_vel_enu[2] };
    double hvel[3] = { x->v[0], x->v[1], x->v[2] };
    double yvel[3];
    for (int i=0;i<3;++i) yvel[i] = zvel[i] - hvel[i];

    double Pvv[9], Svel[9], Svel_inv[9];
    for (int r=0;r<3;++r)
        for (int c=0;c<3;++c)
            Pvv[r*3 + c] = P[IDX(3 + r, 3 + c)];

    for (int k=0;k<9;++k)
        Svel[k] = Pvv[k] + Rvel[k];

    if (mat_inverse_3x3(Svel, Svel_inv) != 0) {
        if (out_mahalanobis_vel) *out_mahalanobis_vel = 1e9;
        if (out_accepted_vel) *out_accepted_vel = 0;
        return -2;
    }

    double tmpv2[3];
    mat_vec_mult(3, 3, Svel_inv, yvel, tmpv2);
    double d2_vel = yvel[0]*tmpv2[0] + yvel[1]*tmpv2[1] + yvel[2]*tmpv2[2];
    if (out_mahalanobis_vel) *out_mahalanobis_vel = d2_vel;

    if (d2_vel > MAHALANOBIS_THRESH_VEL) {
        if (out_accepted_vel) *out_accepted_vel = 0;
    } else {
        if (out_accepted_vel) *out_accepted_vel = 1;

        double P_col3_5[STATE_DIM * 3];
        for (int r=0;r<STATE_DIM;++r)
            for (int c=0;c<3;++c)
                P_col3_5[r*3 + c] = P[IDX(r, 3 + c)];

        double Kvel[STATE_DIM * 3];
        mat_mult(STATE_DIM, 3, 3, P_col3_5, Svel_inv, Kvel);

        double dxv[STATE_DIM];
        for (int r=0;r<STATE_DIM;++r) {
            double sum=0.0;
            for (int c=0;c<3;++c) sum += Kvel[r*3 + c] * yvel[c];
            dxv[r]=sum;
        }

        for (int i=0;i<3;++i) x->p[i] += dxv[i];
        for (int i=0;i<3;++i) x->v[i] += dxv[3+i];

        double dtheta2[3] = { dxv[6], dxv[7], dxv[8] };
        double qd2[4] = { 1.0, 0.5*dtheta2[0], 0.5*dtheta2[1], 0.5*dtheta2[2] };
        double qd2n = sqrt(qd2[0]*qd2[0]+qd2[1]*qd2[1]+qd2[2]*qd2[2]+qd2[3]*qd2[3]);
        if (qd2n>0) for (int i=0;i<4;++i) qd2[i]/=qd2n;

        double qnew2[4];
        qnew2[0]=qd2[0]*x->q[0]-qd2[1]*x->q[1]-qd2[2]*x->q[2]-qd2[3]*x->q[3];
        qnew2[1]=qd2[0]*x->q[1]+qd2[1]*x->q[0]+qd2[2]*x->q[3]-qd2[3]*x->q[2];
        qnew2[2]=qd2[0]*x->q[2]-qd2[1]*x->q[3]+qd2[2]*x->q[0]+qd2[3]*x->q[1];
        qnew2[3]=qd2[0]*x->q[3]+qd2[1]*x->q[2]-qd2[2]*x->q[1]+qd2[3]*x->q[0];
        double n2=sqrt(qnew2[0]*qnew2[0]+qnew2[1]*qnew2[1]+qnew2[2]*qnew2[2]+qnew2[3]*qnew2[3]);
        if (n2>0) for (int i=0;i<4;++i) x->q[i]=qnew2[i]/n2;

        for (int i=0;i<3;++i) x->bg[i]+=dxv[9+i];
        for (int i=0;i<3;++i) x->ba[i]+=dxv[12+i];

        double P_rows_3_5[3 * STATE_DIM];
        for (int r=0;r<3;++r)
            for (int c=0;c<STATE_DIM;++c)
                P_rows_3_5[r*STATE_DIM + c] = P[IDX(3 + r,c)];

        double KH2[STATE_DIM * STATE_DIM];
        mat_mult(STATE_DIM, 3, STATE_DIM, Kvel, P_rows_3_5, KH2);
        for (int i=0;i<P_DIM;++i) P[i]-=KH2[i];

        double KRvel[STATE_DIM * 3];
        mat_mult(STATE_DIM, 3, 3, Kvel, Rvel, KRvel);
        double Ktvel[3 * STATE_DIM];
        for (int r=0;r<3;++r)
            for (int c=0;c<STATE_DIM;++c)
                Ktvel[r*STATE_DIM + c]=Kvel[c*3 + r];
        double KRKTvel[STATE_DIM * STATE_DIM];
        mat_mult(STATE_DIM,3,STATE_DIM,KRvel,Ktvel,KRKTvel);
        for (int i=0;i<P_DIM;++i) P[i]+=KRKTvel[i];

        for (int r=0;r<STATE_DIM;++r)
            for (int c=r+1;c<STATE_DIM;++c){
                double avg=0.5*(P[IDX(r,c)]+P[IDX(c,r)]);
                P[IDX(r,c)]=P[IDX(c,r)]=avg;
            }
    }

    return 0;
}



/* end of GPS part*/
void ekf_init(struct ekf_state *x, double P[P_DIM], double t0,
              const double *init_pos_enu, const double *init_vel_enu)
{
    x->t = t0;

    /* --- Initialize position and velocity --- */
    if (init_pos_enu) {
        x->p[0] = init_pos_enu[0];
        x->p[1] = init_pos_enu[1];
        x->p[2] = init_pos_enu[2];
    } else {
        x->p[0] = x->p[1] = x->p[2] = 0.0;
    }

    if (init_vel_enu) {
        x->v[0] = init_vel_enu[0];
        x->v[1] = init_vel_enu[1];
        x->v[2] = init_vel_enu[2];
    } else {
        x->v[0] = x->v[1] = x->v[2] = 0.0;
    }

    /* --- Orientation (identity quaternion) --- */
    x->q[0] = 1.0;
    x->q[1] = 0.0;
    x->q[2] = 0.0;
    x->q[3] = 0.0;

    /* --- Biases (zero initial) --- */
    memset(x->bg, 0, sizeof(x->bg));
    memset(x->ba, 0, sizeof(x->ba));

    /* --- Initialize covariance --- */
    for (size_t i = 0; i < P_DIM; ++i)
        P[i] = 0.0;

    /* Larger initial uncertainties to reduce early measurement rejection */
    double ang_var = pow(1.0 * (M_PI / 180.0), 2);  // 1 deg^2 in rad^2
    double bg_var = pow(1e-3, 2);
    double ba_var = pow(1e-2, 2);

    /* Position: high uncertainty (100 m²) */
    P[IDX(0,0)] = 100.0;
    P[IDX(1,1)] = 100.0;
    P[IDX(2,2)] = 100.0;

    /* Velocity: moderately high (10 (m/s)²) */
    P[IDX(3,3)] = 10.0;
    P[IDX(4,4)] = 10.0;
    P[IDX(5,5)] = 10.0;

    /* Orientation */
    P[IDX(6,6)] = ang_var;
    P[IDX(7,7)] = ang_var;
    P[IDX(8,8)] = ang_var;

    /* Biases */
    P[IDX(9,9)]   = bg_var;
    P[IDX(10,10)] = bg_var;
    P[IDX(11,11)] = bg_var;
    P[IDX(12,12)] = ba_var;
    P[IDX(13,13)] = ba_var;
    P[IDX(14,14)] = ba_var;
}

/* Build simple block-diagonal Qd (discrete) from noise densities */
void ekf_build_process_noise(double Qd[P_DIM], double dt, double sigma_g, double sigma_a, double sigma_bg_rw, double sigma_ba_rw)
{
    for (size_t i=0;i<P_DIM;++i)
        Qd[i]=0.0;

    double var_g  = (sigma_g * sigma_g) * dt;
    double var_a  = (sigma_a * sigma_a) * dt;
    double var_bg = (sigma_bg_rw * sigma_bg_rw) * dt;
    double var_ba = (sigma_ba_rw * sigma_ba_rw) * dt;

    /* orientation (theta) indices 6..8 */
    for (int i=0;i<3;++i) 
        Qd[IDX(6+i,6+i)] = var_g;
    /* velocity 3..5 */
    for (int i=0;i<3;++i)
        Qd[IDX(3+i,3+i)] = var_a;
    /* biases */
    for (int i=0;i<3;++i)
        Qd[IDX(9+i,9+i)] = var_bg;
    for (int i=0;i<3;++i)
        Qd[IDX(12+i,12+i)] = var_ba;
}

/* ekf_propagate: integrates nominal state and propagates covariance */
int ekf_propagate(struct ekf_state *x, double P[P_DIM], const struct imu_sample *imu, double dt)
{
    if (dt <= 0.0) 
        return (-1);

    /* 1) bias-corrected measurements */
    double omega_b[3] = 
    { 
        imu->wx - x->bg[0],
        imu->wy - x->bg[1],
        imu->wz - x->bg[2] 
    };

    double acc_b[3] = 
    { 
        imu->ax - x->ba[0],
        imu->ay - x->ba[1],
        imu->az - x->ba[2] 
    };

    /* 2) quaternion increment */
    double dq[4];
    quat_from_omega(omega_b, dt, dq);

    double q_new[4];
    quat_mul(dq, x->q, q_new);
    /* normalize */
    double qn = sqrt(q_new[0]*q_new[0] + q_new[1]*q_new[1] + q_new[2]*q_new[2] + q_new[3]*q_new[3]);
    if (qn > 0.0)
    {
        q_new[0] /= qn; 
        q_new[1] /= qn; 
        q_new[2] /= qn; 
        q_new[3] /= qn;
    }
    
    else 
        memcpy(q_new, x->q, 4*sizeof(double));         /* fallback: keep old */

    /* 3) rotation matrix */
    double Rwb[9];
    quat_to_rotmat(q_new, Rwb);

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
    x->q[0]=q_new[0]; 
    x->q[1]=q_new[1];
    x->q[2]=q_new[2]; 
    x->q[3]=q_new[3];
    x->t += dt;

    /* ----- Covariance propagation ----- */
    /* Build continuous-time Jacobian Fc (15x15), then Fd = I + Fc*dt */
    double Fc[P_DIM];
    for (int i=0;i<P_DIM;++i)
        Fc[i]=0.0;

    /* dp/dv = I */
    for (int i=0;i<3;++i)
        Fc[IDX(0+i, 3+i)] = 1.0;

    /* dv/dtheta = -R * skew(acc_b)  */
    double Sacc[9];
    skew(acc_b, Sacc);
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
    for (int i=0;i<3;++i)
        Fc[IDX(6+i, 9+i)] = -1.0;

    /* Discretize */
    double Fd[P_DIM];
    for (int i=0;i<P_DIM;++i)
        Fd[i] = Fc[i] * dt;
    for (int i=0;i<STATE_DIM;++i)
        Fd[IDX(i,i)] += 1.0;

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

    for (int i=0;i<P_DIM;++i)
        P[i] = tmpB[i] + Qd[i];

    /* enforce symmetry */
    for (int r=0;r<STATE_DIM;++r)
    {
        for (int c=r+1;c<STATE_DIM;++c)
        {
            double avg = 0.5*(P[IDX(r,c)] + P[IDX(c,r)]);
            P[IDX(r,c)] = avg;
            P[IDX(c,r)] = avg;
        }
    }

    return 0;
}

