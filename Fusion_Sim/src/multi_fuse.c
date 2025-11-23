#include "../include/multi_fuse.h"

int fuse_imus(struct imu_sample **imu_arrays, int N, int samples_per_sensor, struct fused_imu *out)
{
    if (!imu_arrays || !out || N <= 0 || samples_per_sensor <= 0)
        return -1;

    int written = 0;

    for (int s = 0; s < samples_per_sensor; ++s)
    {
        double acc_vals[MAX_SENSORS][3];
        double gyro_vals[MAX_SENSORS][3];

        int valid_sensors = 0;
        for (int i = 0; i < N; ++i)
        {
            acc_vals[i][0] = imu_arrays[i][s].ax;
            acc_vals[i][1] = imu_arrays[i][s].ay;
            acc_vals[i][2] = imu_arrays[i][s].az;

            gyro_vals[i][0] = imu_arrays[i][s].wx;
            gyro_vals[i][1] = imu_arrays[i][s].wy;
            gyro_vals[i][2] = imu_arrays[i][s].wz;

            valid_sensors++;
        }
        if (valid_sensors < 2)
            continue;

        double med_acc[3];
        double mad_acc[3];
        double med_gyro[3];
        double mad_gyro[3];

        component_median(acc_vals, valid_sensors, med_acc);
        component_mad(acc_vals, valid_sensors, med_acc, mad_acc);

        component_median(gyro_vals, valid_sensors, med_gyro);
        component_mad(gyro_vals, valid_sensors, med_gyro, mad_gyro);

        double sumW_acc[9]  = {0.0};
        double sumWv_acc[3] = {0.0};
        double sumW_gyro[9] = {0.0};
        double sumWv_gyro[3]= {0.0};

        int sensors_used = 0;
        for (int i = 0; i < valid_sensors; ++i)
        {
            double alpha_acc = compute_alpha(acc_vals[i], med_acc, mad_acc, K_ROBUST);
            double alpha_gyro = compute_alpha(gyro_vals[i], med_gyro, mad_gyro, K_ROBUST);

            if (alpha_acc < 0.05 || alpha_gyro < 0.05)
                continue;

            double d_acc[3] = {0.1, 0.1, 0.1};
            double d_gyro[3] = {1e-3, 1e-3, 1e-3};
            double Racc[9], Rgyro[9];
            build_diag3(d_acc, Racc);
            build_diag3(d_gyro, Rgyro);

            double Wacc[9];
            double Wgyro[9];
            mat_inverse_3x3(Racc, Wacc);
            mat_inverse_3x3(Rgyro, Wgyro);

            for (int k = 0; k < 9; ++k)
            {
                Wacc[k] *= alpha_acc;
                Wgyro[k] *= alpha_gyro;
            }

            accumulate_weighted(Wacc, acc_vals[i], sumW_acc, sumWv_acc);
            accumulate_weighted(Wgyro, gyro_vals[i], sumW_gyro, sumWv_gyro);
            sensors_used++;
        }

        if (sensors_used < 1)
            continue;

        double Racc_fused[9];
        double Rgyro_fused[9];

        if (mat_inverse_3x3(sumW_acc, Racc_fused) != 0)
            build_diag3((double[3]){1,1,1}, Racc_fused);
        if (mat_inverse_3x3(sumW_gyro, Rgyro_fused) != 0)
            build_diag3((double[3]){1,1,1}, Rgyro_fused);

        double zacc[3];
        double zgyro[3];
        mat3_vec(Racc_fused, sumWv_acc, zacc);
        mat3_vec(Rgyro_fused, sumWv_gyro, zgyro);

        struct fused_imu *fw = &out[written++];
        fw->t  = imu_arrays[0][s].t;
        fw->ax = zacc[0];
        fw->ay = zacc[1];
        fw->az = zacc[2];
        fw->wx = zgyro[0];
        fw->wy = zgyro[1];
        fw->wz = zgyro[2];
        memcpy(fw->P_acc,  Racc_fused,  sizeof(Racc_fused));
        memcpy(fw->P_gyro, Rgyro_fused, sizeof(Rgyro_fused));
        fw->n_used = sensors_used;
    }

    return written;
}

int fuse_gps(struct gps_sample **gps_arrays, int N, int samples_per_sensor, struct fused_gps *out)
{
    if (!gps_arrays || !out || N <= 0 || samples_per_sensor <= 0)
        return -1;

    int written = 0;

    for (int s = 0; s < samples_per_sensor; ++s)
    {
        double pos_vals[MAX_SENSORS][3];
        double vel_vals[MAX_SENSORS][3];

        int valid_sensors = 0;
        for (int i = 0; i < N; ++i)
        {
            pos_vals[i][0] = gps_arrays[i][s].lat;
            pos_vals[i][1] = gps_arrays[i][s].lon;
            pos_vals[i][2] = gps_arrays[i][s].alt;

            vel_vals[i][0] = gps_arrays[i][s].vn;
            vel_vals[i][1] = gps_arrays[i][s].ve;
            vel_vals[i][2] = gps_arrays[i][s].vu;

            valid_sensors++;
        }
        if (valid_sensors < 2)
            continue;

        double med_pos[3];
        double mad_pos[3];
        double med_vel[3];
        double mad_vel[3];

        component_median(pos_vals, valid_sensors, med_pos);
        component_mad(pos_vals, valid_sensors, med_pos, mad_pos);

        component_median(vel_vals, valid_sensors, med_vel);
        component_mad(vel_vals, valid_sensors, med_vel, mad_vel);

        double sumW_pos[9] = {0.0};
        double sumWv_pos[3] = {0.0};
        double sumW_vel[9] = {0.0};
        double sumWv_vel[3] = {0.0};

        int sensors_used = 0;
        for (int i = 0; i < valid_sensors; ++i)
        {
            double alpha_pos = compute_alpha(pos_vals[i], med_pos, mad_pos, K_ROBUST);
            double alpha_vel = compute_alpha(vel_vals[i], med_vel, mad_vel, K_ROBUST);

            if (alpha_pos < 0.05 || alpha_vel < 0.05)
                continue;

            double d_pos[3] = {2.0, 2.0, 5.0};
            double d_vel[3] = {0.5, 0.5, 0.5};
            double Rpos[9];
            double Rvel[9];
            build_diag3(d_pos, Rpos);
            build_diag3(d_vel, Rvel);

            double Wpos[9];
            double Wvel[9];
            mat_inverse_3x3(Rpos, Wpos);
            mat_inverse_3x3(Rvel, Wvel);

            for (int k = 0; k < 9; ++k)
            {
                Wpos[k] *= alpha_pos;
                Wvel[k] *= alpha_vel;
            }

            accumulate_weighted(Wpos, pos_vals[i], sumW_pos, sumWv_pos);
            accumulate_weighted(Wvel, vel_vals[i], sumW_vel, sumWv_vel);
            sensors_used++;
        }

        if (sensors_used < 1)
            continue;

        double Rpos_fused[9], Rvel_fused[9];
        if (mat_inverse_3x3(sumW_pos, Rpos_fused) != 0)
            build_diag3((double[3]){1,1,1}, Rpos_fused);
        if (mat_inverse_3x3(sumW_vel, Rvel_fused) != 0)
            build_diag3((double[3]){1,1,1}, Rvel_fused);

        double pos_fused[3];
        double vel_fused[3];

        mat3_vec(Rpos_fused, sumWv_pos, pos_fused);
        mat3_vec(Rvel_fused, sumWv_vel, vel_fused);

        struct fused_gps *fw = &out[written++];
        fw->t   = gps_arrays[0][s].t;
        fw->lat = pos_fused[0];
        fw->lon = pos_fused[1];
        fw->alt = pos_fused[2];
        fw->vn  = vel_fused[0];
        fw->ve  = vel_fused[1];
        fw->vu  = vel_fused[2];
        memcpy(fw->P_pos, Rpos_fused, sizeof(Rpos_fused));
        memcpy(fw->P_vel, Rvel_fused, sizeof(Rvel_fused));
        fw->n_used = sensors_used;
    }

    return written;
}
