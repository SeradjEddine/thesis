#include "logger.h"
#include <stdio.h>
#include <math.h>


static FILE *g_state_f = NULL;
static FILE *g_gps_f = NULL;

int open_state_log(const char *filename)
{
    g_state_f = fopen(filename, "w");
    if (!g_state_f)
        return -1;
    fprintf(g_state_f, "t,px,py,pz,vx,vy,vz,qw,qx,qy,qz,traceP\n");
    return 0;
}

void close_state_log(void)
{
    if (g_state_f)
    {
        fclose(g_state_f);
        g_state_f = NULL;
    }
}

void log_state(const struct ekf_state *x, const double P[P_DIM])
{
    double trace;

    if (!g_state_f)
        return;

    trace = 0.0;
    for (int i=0; i<STATE_DIM; ++i)
        trace += P[IDX(i,i)];
    fprintf(g_state_f, "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.10f\n",
            x->t,
            x->p[0], x->p[1], x->p[2],
            x->v[0], x->v[1], x->v[2],
            x->q[0], x->q[1], x->q[2],
            x->q[3], trace);
    fflush(g_state_f);
}

/* GPS log */

int open_gps_log(const char *filename)
{
    g_gps_f = fopen(filename, "w");
    if (!g_gps_f)
        return -1;
    fprintf(g_gps_f, "t,zx,zy,zz,innov_x,innov_y,innov_z,mahalanobis_pos,accepted_pos,");
    fprintf(g_gps_f, "vx,vy,vz,innov_vx,innov_vy,innov_vz,mahalanobis_vel,accepted_vel\n");
    return 0;
}

void close_gps_log(void)
{
    if (g_gps_f)
    {
        fclose(g_gps_f);
        g_gps_f = NULL;
    }
}

void log_gps_update
    (
        double t, const double zpos[3], const double innov_pos[3],
        double mahalanobis_pos, int accepted_pos, const double zvel[3],
        const double innov_vel[3], double mahalanobis_vel, int accepted_vel
    )
{
    if (!g_gps_f)
        return;
    fprintf(g_gps_f, "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%d,",
            t,
            zpos[0], zpos[1], zpos[2],
            innov_pos[0], innov_pos[1], innov_pos[2],
            mahalanobis_pos, accepted_pos);
    fprintf(g_gps_f, "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%d\n",
            zvel[0], zvel[1], zvel[2],
            innov_vel[0], innov_vel[1], innov_vel[2],
            mahalanobis_vel, accepted_vel);
    fflush(g_gps_f);
}