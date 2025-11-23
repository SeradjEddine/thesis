#include "../include/consumer.h"

void *consumer_thread(void *arg)
{
    struct consumer_args *cargs = (struct consumer_args *)arg;

    struct ekf_state state;
    double P[P_DIM];

    double maha_pos = 0.0;
    double maha_vel = 0.0;
    double last_time = 0.0;
    size_t processed = 0;

    struct fused_imu first_fused_imu;
    struct fused_gps first_fused_gps;

    while (rb_is_empty(cargs->imu_rb) || rb_is_empty(cargs->gps_rb))
        usleep(10); // waits until both have at least one element 

    if (rb_peek(cargs->imu_rb, &first_fused_imu) != 0)
        return (fprintf(stderr, "[consumer] error: failed to peek first IMU fused entry\n"), NULL);
    
    if (rb_peek(cargs->gps_rb, &first_fused_gps) != 0)
        return (fprintf(stderr, "[consumer] error: failed to peek first GPS fused entry\n"), NULL);

    double pos0[3];  // Convert first GPS to ENU-based initial position and velocity for EKF init 
    latlon_to_enu(first_fused_gps.lat, first_fused_gps.lon, first_fused_gps.alt, pos0);

    double vel0[3];
    vel0[0] = first_fused_gps.ve;   // east
    vel0[1] = first_fused_gps.vn;  //north
    vel0[2] = first_fused_gps.vu; // up

    ekf_init(&state, P, first_fused_imu.t, pos0, vel0); //Init EKF with first samples (pos/vel)
    last_time = state.t;

    printf("[consumer] EKF initialized t=%.6f pos=[%.3f, %.3f, %.3f] vel=[%.3f, %.3f, %.3f]\n",
           state.t, pos0[0], pos0[1], pos0[2], vel0[0], vel0[1], vel0[2]);

    // defualt fuzzy supervisor Initialization
    fuzzy_params_t fparams = {
        .min_scale_R = 0.6,
        .max_scale_R = 3.0,
        .min_scale_Q = 0.7,
        .max_scale_Q = 2.0,
        .min_scale_gate = 0.85,
        .max_scale_gate = 1.5,
        .smoothing_alpha = 0.85 };

    fuzzy_init(&fparams);

    // Prepare logs 
    create_directory_recursive(cargs->output_dir);       
    char imu_path[PATH_MAX], gps_path[PATH_MAX], fuzzy_path[PATH_MAX];
    snprintf(imu_path, sizeof(imu_path), "%s/imu_prop.csv", cargs->output_dir);
    snprintf(gps_path, sizeof(gps_path), "%s/gps_updates.csv", cargs->output_dir);
    snprintf(fuzzy_path, sizeof(fuzzy_path), "%s/fuzzy.csv", cargs->output_dir);

    open_state_log(imu_path);
    open_gps_log(gps_path);
    open_fuzzy_log(fuzzy_path);


                // Main loop: processes fused_imu items as IMU propagation and fused_gps items as GPS updates.
    while (1)
    {
        struct fused_imu fused_imu_item;
        struct fused_gps fused_gps_item;

        if (rb_pop(cargs->imu_rb, &fused_imu_item) == 0)
        {
            struct imu_sample imu_in;
            fused_imu_to_imu_sample(&fused_imu_item, &imu_in);

            double dt = imu_in.t - last_time;
            if (dt <= 0.0)
                dt = 0.01; // defensive fallback

            ekf_propagate(&state, P, &imu_in, dt);
            last_time = state.t;

            log_state(&state, P);
            processed++;
            continue; // go back and pop more IMU if available
        }

        if (rb_pop(cargs->gps_rb, &fused_gps_item) == 0)
        {
            while (!rb_is_empty(cargs->imu_rb))
            {
                struct fused_imu lookahead;
                if (rb_peek(cargs->imu_rb, &lookahead) != 0)
                    break; /* peek fail -> exit loop */

                if (lookahead.t <= fused_gps_item.t)
                {
                    if (rb_pop(cargs->imu_rb, &lookahead) == 0)
                    {
                        struct imu_sample imu_in2;
                        fused_imu_to_imu_sample(&lookahead, &imu_in2);
                        double dt2 = imu_in2.t - last_time;
                        if (dt2 <= 0.0) dt2 = 0.01;
                        ekf_propagate(&state, P, &imu_in2, dt2);
                        last_time = state.t;
                        log_state(&state, P);
                        processed++;
                    }
                    else
                        break;
                }
                else
                    break;
            }

            double pos_enu[3];  // Convert fused_gps to measurement and build R matrices
            latlon_to_enu(fused_gps_item.lat, fused_gps_item.lon, fused_gps_item.alt, pos_enu);

            double vel_enu[3];
            vel_enu[0] = fused_gps_item.ve;   // east -> x
            vel_enu[1] = fused_gps_item.vn;  // north -> y
            vel_enu[2] = fused_gps_item.vu; // up -> z

            double Rpos[9];
            double Rvel[9];

            if ((fused_gps_item.P_pos[0] > 1e-12) || (fused_gps_item.P_pos[4] > 1e-12) || (fused_gps_item.P_pos[8] > 1e-12) )
                fused_pos_to_R(fused_gps_item.P_pos, Rpos);
            else
                build_default_Rpos(Rpos);

            if ((fused_gps_item.P_vel[0] > 1e-12) || (fused_gps_item.P_vel[4] > 1e-12) || (fused_gps_item.P_vel[8] > 1e-12) )
                fused_vel_to_R(fused_gps_item.P_vel, Rvel);
            else
                build_default_Rvel(Rvel);

            fuzzy_inputs_t fin;
            fuzzy_outputs_t fout;

            fin.mahalanobis_pos = maha_pos;
            fin.mahalanobis_vel = maha_vel;
            fin.cov_trace = P[IDX(0,0)] + P[IDX(1,1)] + P[IDX(2,2)];
            fin.accel_norm = 0.0; // accel_norm could be computed from last IMU. zero is placeholder for now

            fuzzy_update(&fin, &fout);

            /* Scale R matrices according to fuzzy outputs */

            double Rpos_scaled[9];
            double Rvel_scaled[9];
        
            mat_scale(3, 3, fout.scale_R_gps, Rpos, Rpos_scaled);
            mat_scale(3, 3, fout.scale_R_gps, Rvel, Rvel_scaled);

                                        /* EKF GPS update */
            int accepted_pos = 0;
            int accepted_vel = 0;
            ekf_update_gps(&state, P, pos_enu, vel_enu, Rpos_scaled, Rvel_scaled,
                           &maha_pos, &accepted_pos, &maha_vel, &accepted_vel);

            // Compute innovation values for logging (measurement - (state))
            double innov_pos[3] = { pos_enu[0] - state.p[0], pos_enu[1] - state.p[1], pos_enu[2] - state.p[2] };
            double innov_vel[3] = { vel_enu[0] - state.v[0], vel_enu[1] - state.v[1], vel_enu[2] - state.v[2] };

            
            log_gps_update(fused_gps_item.t, pos_enu, innov_pos, maha_pos, accepted_pos,
                                         vel_enu, innov_vel, maha_vel, accepted_vel);
            log_fuzzy(fused_gps_item.t, fout.scale_R_gps, fout.scale_Q, fout.scale_gate);

            continue;
        }

        // Termination condition: only when both producers finished and both buffers empty
        if (*(cargs->producers_done) >= 2 && rb_is_empty(cargs->imu_rb) && rb_is_empty(cargs->gps_rb))
            break;
        usleep(1000);  // Sleep briefly when nothing available to avoid busy loop
    }

    close_state_log();
    close_gps_log();
    close_fuzzy_log();

    printf("[consumer] finished, processed %zu IMU steps\n", processed);
    return NULL;
}
