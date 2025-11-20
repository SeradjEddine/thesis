#ifndef FUZZY_SUPERVISOR_H
#define FUZZY_SUPERVISOR_H

typedef struct {
    double mahalanobis_pos;
    double mahalanobis_vel;
    double cov_trace;
    double accel_norm;
} fuzzy_inputs_t;

typedef struct {
    double scale_R_gps;   /* scale applied to GPS measurement covariance R */
    double scale_Q;       /* scale applied to process noise Q */
    double scale_gate;    /* scale applied to Mahalanobis gating threshold */
} fuzzy_outputs_t;

typedef struct
{

    double min_scale_R;
    double max_scale_R;
    double min_scale_Q;
    double max_scale_Q;
    double min_scale_gate;
    double max_scale_gate;
    double smoothing_alpha; //(0..1). 0=no smoothing, 1=no change. Typical = 0.85 

} fuzzy_params_t;

void fuzzy_init(const fuzzy_params_t *params);

void fuzzy_update(const fuzzy_inputs_t *in, fuzzy_outputs_t *out);

void fuzzy_get_last_outputs(fuzzy_outputs_t *out);

#endif
