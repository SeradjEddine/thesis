#ifndef LOGGER_H
#define LOGGER_H
#include <stdio.h>
#include "ekf.h"

int open_fuzzy_log(const char *filename);
void log_fuzzy(double t, double scale_R_gps, double scale_Q, double scale_gate);
void close_fuzzy_log(void);

int open_state_log(const char *filename);
void close_state_log(void);
void log_state(const struct ekf_state *x, const double P[P_DIM]);

int open_gps_log(const char *filename);
void close_gps_log(void);
void log_gps_update
    (
        double t, const double zpos[3], const double innov_pos[3],
        double mahalanobis_pos, int accepted_pos, const double zvel[3],
        const double innov_vel[3], double mahalanobis_vel, int accepted_vel
    );
    
#endif

