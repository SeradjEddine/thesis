#include "../include/consumer.h"

/* Default GPS measurement noise (standard deviation) used as fallback */
static const double GPS_POS_STD = 1.0; // meters
static const double GPS_VEL_STD = 0.2; // m/s

/* Helper: create directories recursively */
void create_directory_recursive(const char *path)
{
    char tmp[PATH_MAX];
    strncpy(tmp, path, sizeof(tmp));
    tmp[sizeof(tmp)-1] = '\0';
    for (char *p = tmp + 1; *p; ++p) {
        if (*p == '/') {
            *p = '\0';
            mkdir(tmp, 0777); /* ignore errors */
            *p = '/';
        }
    }
    mkdir(tmp, 0777);
}

/* Helper: copy fused GPS covariance (3x3) into R (row-major) */
void fused_pos_to_R(const double fusedP_pos[9], double R_out[9])
{
    /* fusedP_pos is already 3x3 row-major. Copy and guard tiny diagonal values. */
    for (int i = 0; i < 9; ++i) R_out[i] = fusedP_pos[i];
    /* ensure non-zero diagonal for numerical stability */
    const double eps = 1e-9;
    R_out[0] += eps; R_out[4] += eps; R_out[8] += eps;
}

/* Helper: copy fused velocity covariance to Rvel */
void fused_vel_to_R(const double fusedP_vel[9], double R_out[9])
{
    for (int i = 0; i < 9; ++i) R_out[i] = fusedP_vel[i];
    const double eps = 1e-9;
    R_out[0] += eps; R_out[4] += eps; R_out[8] += eps;
}

/* Fallback defaults used when fused covariances are not available */
void build_default_Rpos(double R_out[9])
{
    double var = GPS_POS_STD * GPS_POS_STD;
    for (int i=0;i<9;++i) R_out[i]=0.0;
    R_out[0] = var; R_out[4] = var; R_out[8] = var;
}
void build_default_Rvel(double R_out[9])
{
    double var = GPS_VEL_STD * GPS_VEL_STD;
    for (int i=0;i<9;++i) R_out[i]=0.0;
    R_out[0] = var; R_out[4] = var; R_out[8] = var;
}

/* Convert fused_imu -> imu_sample (struct layout must match sensors.h) */
void fused_imu_to_imu_sample(const struct fused_imu *f, struct imu_sample *out)
{
    out->t  = f->t;
    out->ax = f->ax;
    out->ay = f->ay;
    out->az = f->az;
    out->wx = f->wx;
    out->wy = f->wy;
    out->wz = f->wz;
    /* If imu_sample has other fields (padding, flags), zero them in caller if needed */
}
