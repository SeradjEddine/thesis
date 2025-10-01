#include <stdio.h>
#include <math.h>

/* ===== Quaternion implementation ===== */

/* Normalize quaternion to unit length */
void quat_normalize(double q[4]) {
    double norm = sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if (norm > 1e-12) {
        q[0] /= norm;
        q[1] /= norm;
        q[2] /= norm;
        q[3] /= norm;
    }
}

/* Quaternion multiplication: out = q1 * q2 */
void quat_mul(const double q1[4], const double q2[4], double out[4]) {
    out[0] = q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3];
    out[1] = q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2];
    out[2] = q1[0]*q2[2] - q1[1]*q2[3] + q1[2]*q2[0] + q1[3]*q2[1];
    out[3] = q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1] + q1[3]*q2[0];
}

/* Rotate vector v by quaternion q: out = q * (0,v) * q^-1 */
void quat_rotate_vec(const double q[4], const double v[3], double out[3]) {
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

/* Build quaternion from angular velocity ω and timestep dt */
void quat_from_omega(const double omega[3], double dt, double q_delta[4]) {
    double theta = sqrt(omega[0]*omega[0] + omega[1]*omega[1] + omega[2]*omega[2]) * dt;
    if (theta < 1e-12) {
        q_delta[0] = 1.0;
        q_delta[1] = q_delta[2] = q_delta[3] = 0.0;
        return;
    }

    double half_theta = theta * 0.5;
//    double s = sin(half_theta) / (theta > 0 ? theta : 1.0);

    q_delta[0] = cos(half_theta);
    q_delta[1] = omega[0] * dt * 0.5 * (1.0 / (dt > 0 ? sqrt(omega[0]*omega[0] + omega[1]*omega[1] + omega[2]*omega[2]) : 1.0));
    q_delta[2] = omega[1] * dt * 0.5 * (1.0 / (dt > 0 ? sqrt(omega[0]*omega[0] + omega[1]*omega[1] + omega[2]*omega[2]) : 1.0));
    q_delta[3] = omega[2] * dt * 0.5 * (1.0 / (dt > 0 ? sqrt(omega[0]*omega[0] + omega[1]*omega[1] + omega[2]*omega[2]) : 1.0));
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

/*

void print_quat(const double q[4]) {
    printf("[%g, %g, %g, %g]\n", q[0], q[1], q[2], q[3]);
}

void print_vec3(const double v[3]) {
    printf("[%g, %g, %g]\n", v[0], v[1], v[2]);
}





int main(void) {
    printf("Normalize test:\n");
    double q1[4] = {2, 0, 0, 0};
    quat_normalize(q1);
    print_quat(q1);  // Expected [1,0,0,0]

    printf("\nRotation test (90 deg about Z):\n");
    double angle = M_PI / 2;
    double qz[4] = {cos(angle/2), 0, 0, sin(angle/2)}; // 90° around Z
    double v[3] = {1, 0, 0};
    double v_rot[3];
    quat_rotate_vec(qz, v, v_rot);
    print_vec3(v_rot); // Expected [0,1,0]

    printf("\nFrom omega test (w=[0,0,pi/2], dt=1):\n");
    double omega[3] = {0, 0, M_PI/2};
    double q_delta[4];
    quat_from_omega(omega, 1.0, q_delta);
    quat_normalize(q_delta);
    print_quat(q_delta);
    quat_rotate_vec(q_delta, v, v_rot);
    print_vec3(v_rot); // Expected [0,1,0]

    return 0;
}

*/
