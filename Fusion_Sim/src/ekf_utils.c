
#include "../include/ekf.h"

static const double WGS84_e2 = WGS84_f * (2.0 - WGS84_f);

/* Internal storage for ENU conversion */
static double gps_ref_lat = 0.0;
static double gps_ref_lon = 0.0;
static double gps_ref_alt = 0.0;
static double gps_ref_ecef[3];
static double enu_rot[9];

static void geodetic_to_ecef(double lat_rad, double lon_rad, double alt, double ecef[3])
{
    double sin_lat = sin(lat_rad);
    double cos_lat = cos(lat_rad);
    double sin_lon = sin(lon_rad);
    double cos_lon = cos(lon_rad);

    double N = WGS84_a / sqrt(1.0 - WGS84_e2 * sin_lat * sin_lat);
    ecef[0] = (N + alt) * cos_lat * cos_lon;
    ecef[1] = (N + alt) * cos_lat * sin_lon;
    ecef[2] = ((1.0 - WGS84_e2) * N + alt) * sin_lat;
}

/* Convert lat/lon/alt (deg,deg,m) to ENU relative to reference set by ekf_init_gps_ref */
void latlon_to_enu(double lat_deg, double lon_deg, double alt_m, double enu_out[3])
{
    double lat = lat_deg * M_PI / 180.0;
    double lon = lon_deg * M_PI / 180.0;
    double ecef[3];
    geodetic_to_ecef(lat, lon, alt_m, ecef);

    double d[3] = { ecef[0] - gps_ref_ecef[0], ecef[1] - gps_ref_ecef[1], ecef[2] - gps_ref_ecef[2] };
    mat_vec_mult(3, 3, enu_rot, d, enu_out);
}

/* initialize ENU reference from first fused_gps array pointer (same as previous usage) */
void ekf_init_gps_ref(struct fused_gps *gps_first)
{
    if (!gps_first)
        return;
    gps_ref_lat = gps_first[0].lat;
    gps_ref_lon = gps_first[0].lon;
    gps_ref_alt = gps_first[0].alt;

    double lat0 = gps_ref_lat * M_PI / 180.0;
    double lon0 = gps_ref_lon * M_PI / 180.0;

    geodetic_to_ecef(lat0, lon0, gps_ref_alt, gps_ref_ecef);

    double s_lat = sin(lat0), c_lat = cos(lat0);
    double s_lon = sin(lon0), c_lon = cos(lon0);

    /* Row-major rotation: ENU = enu_rot * (ECEF - ecef_ref) */
    enu_rot[0] = -s_lon;           enu_rot[1] =  c_lon;          enu_rot[2] = 0.0;
    enu_rot[3] = -s_lat * c_lon;   enu_rot[4] = -s_lat * s_lon;  enu_rot[5] = c_lat;
    enu_rot[6] =  c_lat * c_lon;   enu_rot[7] =  c_lat * s_lon;  enu_rot[8] = s_lat;
}

void apply_small_angle_quat(struct ekf_state *x, const double dtheta[3])
{
    double qd[4];
    qd[0] = 1.0;
    qd[1] = 0.5 * dtheta[0];
    qd[2] = 0.5 * dtheta[1];
    qd[3] = 0.5 * dtheta[2];

    double nqd = sqrt(qd[0]*qd[0] + qd[1]*qd[1] + qd[2]*qd[2] + qd[3]*qd[3]);
    if (nqd > 0.0)
    {
        for (int i = 0; i < 4; ++i)
            qd[i] /= nqd;
    }

    double qnew[4];
    qnew[0] = qd[0]*x->q[0] - qd[1]*x->q[1] - qd[2]*x->q[2] - qd[3]*x->q[3];
    qnew[1] = qd[0]*x->q[1] + qd[1]*x->q[0] + qd[2]*x->q[3] - qd[3]*x->q[2];
    qnew[2] = qd[0]*x->q[2] - qd[1]*x->q[3] + qd[2]*x->q[0] + qd[3]*x->q[1];
    qnew[3] = qd[0]*x->q[3] + qd[1]*x->q[2] - qd[2]*x->q[1] + qd[3]*x->q[0];

    double n2 = sqrt(qnew[0]*qnew[0] + qnew[1]*qnew[1] + qnew[2]*qnew[2] + qnew[3]*qnew[3]);
    if (n2 > 0.0)
    {
        for (int i = 0; i < 4; ++i)
            x->q[i] = qnew[i] / n2;
    }
}

int ekf_update_block(struct ekf_state *x, double P[P_DIM], const double z[3], const double h[3], const double R[9],
                     int state_index, double gate_thresh, double *out_mahalanobis, int *out_accepted)
{
   
    double y[3];
    for (int i = 0; i < 3; ++i)   /* Builds innovation y = z - h */
        y[i] = z[i] - h[i];

    double Pblock[9];
    for (int r = 0; r < 3; ++r) /* Extract P_block (3x3) corresponding to the measurement variables */
        for (int c = 0; c < 3; ++c)
            Pblock[r*3 + c] = P[IDX(state_index + r, state_index + c)];

    double S[9];
    for (int k = 0; k < 9; ++k)    /* S = Pblock + R */
        S[k] = Pblock[k] + R[k];

    double S_inv[9];
    if (safe_mat_inv_3x3(S, S_inv) != 0)    /* Invert S (safe) */
    {
        if (out_mahalanobis)
            *out_mahalanobis = 1e9;
        if (out_accepted)
            *out_accepted = 0;
        return -1;
    }

    /* Mahalanobis distance */
    double tmpv[3]; 
    mat_vec_mult(3, 3, S_inv, y, tmpv);
    double d2 = y[0]*tmpv[0] + y[1]*tmpv[1] + y[2]*tmpv[2];
    if (out_mahalanobis)
        *out_mahalanobis = d2;

    if (d2 > gate_thresh)
    {
        if (out_accepted)
            *out_accepted = 0;
        return 0; /* innovation rejected but not fatal */
    }

    if (out_accepted)
        *out_accepted = 1;

    /* Builds P_col = P (15x3) */
    double P_col[STATE_DIM * 3];
    for (int r = 0; r < STATE_DIM; ++r)
        for (int c = 0; c < 3; ++c)
            P_col[r*3 + c] = P[IDX(r, state_index + c)];

    double K[STATE_DIM * 3];
    mat_mult(STATE_DIM, 3, 3, P_col, S_inv, K);    /* Compute K = P_col * S_inv  (15x3) */

    double dx[STATE_DIM];
    for (int r = 0; r < STATE_DIM; ++r)
    {
        double s = 0.0;
        for (int c = 0; c < 3; ++c)     /* dx = K * y  (STATE_DIM vector) */
            s += K[r*3 + c] * y[c];
        dx[r] = s;
    }

    /* Apply corrections to nominal state */
    x->p[0] += dx[0];
    x->p[1] += dx[1];  /* position */
    x->p[2] += dx[2];

    x->v[0] += dx[3];
    x->v[1] += dx[4];  /* velocity (dx[3..5]) */
    x->v[2] += dx[5];

    double dtheta[3] = { dx[6], dx[7], dx[8] };
    apply_small_angle_quat(x, dtheta);     /* orientation small-angle correction dx[6..8] */

        /* biases */
    x->bg[0] += dx[9];
    x->bg[1] += dx[10];
    x->bg[2] += dx[11];

    x->ba[0] += dx[12];
    x->ba[1] += dx[13];
    x->ba[2] += dx[14];

    /* Covariance update (P = P - K * (H*P) + K * R * K^T)
       Where H*P is the measurement rows of P (3x15) i.e. rows state_index..state_index+2.
    */
    double P_rows[3 * STATE_DIM];
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < STATE_DIM; ++c)
            P_rows[r*STATE_DIM + c] = P[IDX(state_index + r, c)];

    double KH[STATE_DIM * STATE_DIM];
    mat_mult(STATE_DIM, 3, STATE_DIM, K, P_rows, KH);   /* KH = K * P_rows  => 15x15 */

    for (int i = 0; i < P_DIM; ++i)
        P[i] -= KH[i];

    double KR[STATE_DIM * 3];
    mat_mult(STATE_DIM, 3, 3, K, R, KR);     /* KR = K * R  (15x3) */

        /* KR * K^T  => KR * (K^T)  (15x15) */
    double Kt[3 * STATE_DIM];
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < STATE_DIM; ++c)
            Kt[r*STATE_DIM + c] = K[c*3 + r];

    double KRKT[STATE_DIM * STATE_DIM];
    mat_mult(STATE_DIM, 3, STATE_DIM, KR, Kt, KRKT);

    for (int i = 0; i < P_DIM; ++i)
        P[i] += KRKT[i];

    /* Ensure symmetry */
    for (int r = 0; r < STATE_DIM; ++r)
        for (int c = r+1; c < STATE_DIM; ++c)
        {
            double avg = 0.5 * (P[IDX(r,c)] + P[IDX(c,r)]);
            P[IDX(r,c)] = P[IDX(c,r)] = avg;
        }

    return 0;
}
