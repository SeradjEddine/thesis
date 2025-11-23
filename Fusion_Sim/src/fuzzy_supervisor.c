#include "../include/fuzzy_supervisor.h"

static const double Q_INC_THRESHOLD = 0.25;

static fuzzy_params_t Fparams; /*(user-supplied)*/
static fuzzy_outputs_t last_out = { 1.0, 1.0, 1.0 };

static const fuzzy_params_t default_params =
{
    /*Default parameter bounds (conservative)*/
    .min_scale_R = 0.6,
    .max_scale_R = 3.0,
    .min_scale_Q = 0.7,
    .max_scale_Q = 2.0,
    .min_scale_gate = 0.85,
    .max_scale_gate = 1.5,
    .smoothing_alpha = 0.85
};

static memberships_t evaluate_memberships(const fuzzy_inputs_t *in)
{
    memberships_t m = {0};

    m.mpos_low  = mf_left(in->mahalanobis_pos, MAHA_LOW_B, MAHA_MED_B);
    m.mpos_high = mf_right(in->mahalanobis_pos, MAHA_HIGH_A, MAHA_HIGH_B);

    m.mvel_low  = mf_left(in->mahalanobis_vel, MAHA_LOW_B, MAHA_MED_B);
    m.mvel_high = mf_right(in->mahalanobis_vel, MAHA_HIGH_A, MAHA_HIGH_B);

    m.ct_high = mf_right(in->cov_trace, TRACE_HIGH_A, TRACE_HIGH_B);

    m.acc_high = mf_right(in->accel_norm, ACC_HIGH_A, ACC_HIGH_B);

    return m;
}

/* ------------------------- Rule base application ------------------------ */
/* Based on fuzzy rules, Output strengths are returned in the arrays:
 * R_dec_norm_inc[3], Q_dec_norm_inc[3], G_dec_norm_inc[3]
 * Where index 0=dec,1=norm,2=inc
 */
static void apply_rulebase(const memberships_t *m,
                    double R_out[3], double Q_out[3], double G_out[3])
{
    /* init */
    R_out[0] = R_out[1] = R_out[2] = 0.0;
    Q_out[0] = Q_out[1] = Q_out[2] = 0.0;
    G_out[0] = G_out[1] = G_out[2] = 0.0;

    /* Rules (Mamdani-style) */

    /* R_inc: if M_pos is High OR M_vel is High -> increase R */
    R_out[2] = fmax(m->mpos_high, m->mvel_high);

    /* Q_inc: if Cov_trace High AND M_pos High -> increase Q */
    Q_out[2] = fmax(Q_out[2], fmin(m->ct_high, m->mpos_high));

    /* Q_dec: if Cov_trace High AND M_pos Low -> decrease Q */
    Q_out[0] = fmax(Q_out[0], fmin(m->ct_high, m->mpos_low));

    /* Q_inc: if velocity error and acceleration are high -> increase Q */
    Q_out[2] = fmax(Q_out[2], fmin(m->mvel_high, m->acc_high));

    /* G_dec: if both position and velocity mahalanobis low -> tighten gate */
    G_out[0] = fmax(G_out[0], fmin(m->mpos_low, m->mvel_low));

    /* R_inc & G_inc: if both pos and vel high -> increase R and loosen gate */
    R_out[2] = fmax(R_out[2], fmin(m->mpos_high, m->mvel_high));
    G_out[2] = fmax(G_out[2], fmin(m->mpos_high, m->mvel_high));

    // Baseline: Fill 'normal' as 1 - max(inc,dec), clipped to [0,1].
    R_out[1] = clamp_d(1.0 - fmax(R_out[2], R_out[0]), 0.0, 1.0);
    Q_out[1] = clamp_d(1.0 - fmax(Q_out[2], Q_out[0]), 0.0, 1.0);
    G_out[1] = clamp_d(1.0 - fmax(G_out[2], G_out[0]), 0.0, 1.0);

    /* Small thresholding: discard tiny Q_inc values to avoid noise */
    if (Q_out[2] < Q_INC_THRESHOLD)
        Q_out[2] = 0.0;
}

static void defuzzify_outputs(fuzzy_outputs_t *raw_out,
            const double R_out[3], const double Q_out[3], const double G_out[3])
{
    raw_out->scale_R_gps = defuzzify_generic(R_out[0], R_out[1], R_out[2],
                                             R_TARGET_DEC, R_TARGET_NORM, R_TARGET_INC);

    raw_out->scale_Q = defuzzify_generic(Q_out[0], Q_out[1], Q_out[2],
                                         Q_TARGET_DEC, Q_TARGET_NORM, Q_TARGET_INC);

    raw_out->scale_gate = defuzzify_generic(G_out[0], G_out[1], G_out[2],
                                            G_TARGET_DEC, G_TARGET_NORM, G_TARGET_INC);
}

static void smooth_and_clamp_outputs(const fuzzy_outputs_t *raw, fuzzy_outputs_t *out)
{
    /* Adaptive alpha for Q: we react slightly faster to Q increases.
     * Compute a simple heuristic alpha_Q (smaller alpha -> faster response)
     * Here we base it on how far raw->scale_Q is from last_out.scale_Q.
     */
    double base_alpha;
    double alpha_R;
    double alpha_G;
    double alpha_Q;

    alpha_Q = alpha_G = alpha_R = base_alpha = Fparams.smoothing_alpha;

    //If Q raw is larger than last, allow slightly faster adaptation
    if (raw->scale_Q > last_out.scale_Q)   // reduce alpha to respond faster
        alpha_Q = clamp_d(base_alpha - 0.25, 0.25, base_alpha); 

    out->scale_R_gps = smooth_value(last_out.scale_R_gps, raw->scale_R_gps, alpha_R);
    out->scale_Q     = smooth_value(last_out.scale_Q, raw->scale_Q, alpha_Q);
    out->scale_gate  = smooth_value(last_out.scale_gate, raw->scale_gate, alpha_G);

    /* clamp to user-specified ranges */
    out->scale_R_gps = clamp_d(out->scale_R_gps, Fparams.min_scale_R, Fparams.max_scale_R);
    out->scale_Q     = clamp_d(out->scale_Q, Fparams.min_scale_Q, Fparams.max_scale_Q);
    out->scale_gate  = clamp_d(out->scale_gate, Fparams.min_scale_gate, Fparams.max_scale_gate);

    last_out = *out;
}

/* ------------------------ Public API functions --------------------------- */

void fuzzy_init(const fuzzy_params_t *params)
{
    if (params)
    {
        Fparams = *params;
        fuzz_params_sanitize(&Fparams);
    }
    else
        Fparams = default_params;

    last_out.scale_R_gps = 1.0;
    last_out.scale_Q = 1.0;
    last_out.scale_gate = 1.0;
}

void fuzzy_update(const fuzzy_inputs_t *in, fuzzy_outputs_t *out)
{
    if (!in || !out)
        return;

    memberships_t m;
    fuzzy_outputs_t raw;

    double R_out[3];
    double Q_out[3];
    double G_out[3];

    m = evaluate_memberships(in);
    apply_rulebase(&m, R_out, Q_out, G_out);
    defuzzify_outputs(&raw, R_out, Q_out, G_out);
    smooth_and_clamp_outputs(&raw, out);
}

void fuzzy_get_last_outputs(fuzzy_outputs_t *out)
{
    if (!out)
        return;
    *out = last_out;
}
