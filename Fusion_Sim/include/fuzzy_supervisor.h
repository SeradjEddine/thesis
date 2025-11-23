#ifndef FUZZY_SUPERVISOR_H
#define FUZZY_SUPERVISOR_H

#include <math.h>
#include <string.h>
#include <stdbool.h>

            /* Mahalanobis 3-DoF high threshold ~ 16.27 (approx 3-sigma) */
#define MAHA_3DOF_THRESH  16.27

#define MAHA_LOW_B   3.0
#define MAHA_MED_B   8.0
#define MAHA_HIGH_A  6.0
#define MAHA_HIGH_B  16.27   /*  equal to MAHA_3DOF_THRESH */

#define TRACE_HIGH_A 15.0
#define TRACE_HIGH_B 60.0
#define ACC_HIGH_A   0.6 /* accel norm (m/s^2) */
#define ACC_HIGH_B   2.5

/* Numeric defuzzifier targets (tunable) */
#define R_TARGET_DEC  0.75
#define R_TARGET_NORM 1.0
#define R_TARGET_INC  1.5
#define Q_TARGET_DEC  0.75
#define Q_TARGET_NORM 1.0
#define Q_TARGET_INC  1.45
#define G_TARGET_DEC  0.90 /* tighten gate (multiply by <1) */
#define G_TARGET_NORM 1.0
#define G_TARGET_INC  1.2

typedef struct
{
    double mahalanobis_pos;
    double mahalanobis_vel;
    double cov_trace;
    double accel_norm;
} fuzzy_inputs_t;

typedef struct
{
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

typedef struct // Membership evaluation struct 
{
    double mpos_low;
    double mpos_high;
    double mvel_low;
    double mvel_high;
    double ct_high;
    double acc_high;
} memberships_t;

void fuzz_params_sanitize(fuzzy_params_t *p);
double mf_right(double x, double a, double b);
double mf_left(double x, double b, double c);
double clamp_d(double x, double lo, double hi);
double smooth_value(double last, double raw, double alpha);
double defuzzify_generic(double dec_s, double nor_s, double inc_s,
                                double dec_val, double nor_val, double inc_val);


void fuzzy_init(const fuzzy_params_t *params);
void fuzzy_update(const fuzzy_inputs_t *in, fuzzy_outputs_t *out);
void fuzzy_get_last_outputs(fuzzy_outputs_t *out);

#endif
