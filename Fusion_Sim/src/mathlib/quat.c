#include <stdio.h>
#include <math.h>

/* ===== Quaternion implementation ===== */

/* Normalize quaternion to unit length */
void quat_normalize(double q[4])
{
    double norm = sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);

    if (norm > 1e-12)
    {
        q[0] /= norm;
        q[1] /= norm;
        q[2] /= norm;
        q[3] /= norm;
    }
}

/* Quaternion multiplication: out = q1 * q2 */
void quat_mul(const double q1[4], const double q2[4], double out[4])
{
    out[0] = q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3];
    out[1] = q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2];
    out[2] = q1[0]*q2[2] - q1[1]*q2[3] + q1[2]*q2[0] + q1[3]*q2[1];
    out[3] = q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1] + q1[3]*q2[0];
}

/* Rotate vector v by quaternion q: out = q * (0,v) * q^-1 */
void quat_rotate_vec(const double q[4], const double v[3], double out[3])
{
    /* Convert v to quaternion form */
    double vq[4] = {0, v[0], v[1], v[2]};
    double q_conj[4] = {q[0], -q[1], -q[2], -q[3]};
    double temp[4], result[4];

    quat_mul(q, vq, temp);
    quat_mul(temp, q_conj, result);

    out[0] = result[1];
    out[1] = result[2];
    out[2] = result[3];
}

void quat_from_omega(const double omega[3], double dt, double q_delta[4])
{
    /* small-angle quaternion using axis-angle */
    double angle = sqrt(omega[0]*omega[0] + omega[1]*omega[1] + omega[2]*omega[2]) * dt;

    if (angle < 1e-8)
    {
        /* approximate: q = [1, 0.5*omega*dt] */
        q_delta[0] = 1.0;
        q_delta[1] = 0.5 * omega[0] * dt;
        q_delta[2] = 0.5 * omega[1] * dt;
        q_delta[3] = 0.5 * omega[2] * dt;
    }

    else
    {
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

    if (n > 0.0)
    {
        q_delta[0] /= n;
        q_delta[1] /= n; 
        q_delta[2] /= n; 
        q_delta[3] /= n;
    }
}

/*
 * Convert quaternion (w,x,y,z) to row-major 3x3 rotation matrix.
 * R = R(q): world <- body (consistent with earlier code)
 */
void quat_to_rotmat(const double q[4], double R[9])
{
    double w = q[0], x = q[1], y = q[2], z = q[3];
    double ww = w*w, xx = x*x, yy = y*y, zz = z*z;
    double wx = w*x, wy = w*y, wz = w*z;
    double xy = x*y, xz = x*z, yz = y*z;

    // Row-major: R[r*3 + c]
    R[0] = ww + xx - yy - zz;
    R[1] = 2.0*(xy - wz);
    R[2] = 2.0*(xz + wy);

    R[3] = 2.0*(xy + wz);
    R[4] = ww - xx + yy - zz;
    R[5] = 2.0*(yz - wx);

    R[6] = 2.0*(xz - wy);
    R[7] = 2.0*(yz + wx);
    R[8] = ww - xx - yy + zz;
}
/* skew symmetric for v -> S (3x3 row-major) */
void skew(const double v[3], double S[9])
{
    S[0]= 0.0;   S[1]=-v[2];  S[2]= v[1];
    S[3]= v[2];  S[4]= 0.0;   S[5]=-v[0];
    S[6]=-v[1];  S[7]= v[0];  S[8]= 0.0;
}
