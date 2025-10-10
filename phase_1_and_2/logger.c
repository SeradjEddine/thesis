#include "logger.h"
#include <stdio.h>
#include <math.h>

static FILE *g_state_f = NULL;

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

