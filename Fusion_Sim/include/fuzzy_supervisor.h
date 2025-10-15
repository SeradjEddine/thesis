#ifndef FUZZY_SUPERVISOR_H
#define FUZZY_SUPERVISOR_H

/* Lightweight fuzzy supervisor API - firmware friendly */

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

typedef struct {
    /* allowed output bounds (clamped) */
    double min_scale_R;
    double max_scale_R;
    double min_scale_Q;
    double max_scale_Q;
    double min_scale_gate;
    double max_scale_gate;

    /* smoothing (0..1). 0=no smoothing, 1=no change. Typical = 0.85 */
    double smoothing_alpha;
} fuzzy_params_t;

/* Initialize fuzzy supervisor - pass NULL to use defaults */
void fuzzy_init(const fuzzy_params_t *params);

/* Compute outputs from inputs (stateless aside from smoothing). */
void fuzzy_update(const fuzzy_inputs_t *in, fuzzy_outputs_t *out);

/* Optional: retrieve last outputs (if needed elsewhere) */
void fuzzy_get_last_outputs(fuzzy_outputs_t *out);

#endif /* FUZZY_SUPERVISOR_H */
