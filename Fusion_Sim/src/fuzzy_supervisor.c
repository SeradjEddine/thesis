#include "../include/fuzzy_supervisor.h"
#include <math.h>

/* Default parameter bounds */
static const fuzzy_params_t default_params = {
    .min_scale_R = 0.5,
    .max_scale_R = 3.0,
    .min_scale_Q = 0.6,
    .max_scale_Q = 2.0,
    .min_scale_gate = 0.8,
    .max_scale_gate = 1.5,
    .smoothing_alpha = 0.55
};

/* Internal state */
static fuzzy_params_t Fparams;
static fuzzy_outputs_t last_out = {1.0, 1.0, 1.0};

/* --- Simple triangular/trapezoidal membership helpers --- */
/* returns value in [0,1] */
static double mf_tri(double x, double a, double b, double c)
{
    if (x <= a) return 0.0;
    if (x >= c) return 0.0;
    if (x == b) return 1.0;
    if (x < b) return (x - a) / (b - a);
    /* x > b */
    return (c - x) / (c - b);
}

/* left-shoulder: 1 up to b, then falls to 0 at c */
static double mf_left(double x, double b, double c)
{
    if (x <= b) return 1.0;
    if (x >= c) return 0.0;
    return (c - x) / (c - b);
}

/* right-shoulder: 0 up to a, then rises to 1 at b */
static double mf_right(double x, double a, double b)
{
    if (x <= a) return 0.0;
    if (x >= b) return 1.0;
    return (x - a) / (b - a);
}

/* clamp helper */
static double clamp(double x, double lo, double hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

/* Initialize fuzzy params (copy defaults if NULL) */
void fuzzy_init(const fuzzy_params_t *params)
{
    if (params) {
        Fparams = *params;
        /* sanitize */
        Fparams.min_scale_R = clamp(Fparams.min_scale_R, 0.1, 10.0);
        Fparams.max_scale_R = clamp(Fparams.max_scale_R, Fparams.min_scale_R, 10.0);
        Fparams.min_scale_Q = clamp(Fparams.min_scale_Q, 0.1, 10.0);
        Fparams.max_scale_Q = clamp(Fparams.max_scale_Q, Fparams.min_scale_Q, 10.0);
        Fparams.min_scale_gate = clamp(Fparams.min_scale_gate, 0.1, 5.0);
        Fparams.max_scale_gate = clamp(Fparams.max_scale_gate, Fparams.min_scale_gate, 5.0);
        Fparams.smoothing_alpha = clamp(Fparams.smoothing_alpha, 0.0, 0.99);
    } else {
        Fparams = default_params;
    }
    /* initialize last outputs to neutral */
    last_out.scale_R_gps = 1.0;
    last_out.scale_Q = 1.0;
    last_out.scale_gate = 1.0;
}

/* Map linguistic label -> numeric scale (per output). */
/* We use three levels: decrease / normal / increase. */
static void map_R_scale(double dec_strength, double nor_strength, double inc_strength,
                        double *out_scale)
{
    /* numeric targets for each label */
    const double dec_val = 0.75;
    const double nor_val = 1.0;
    const double inc_val = 1.5;
    double sum = dec_strength + nor_strength + inc_strength;
    if (sum <= 0.0) {
        *out_scale = 1.0;
        return;
    }
    *out_scale = (dec_strength*dec_val + nor_strength*nor_val + inc_strength*inc_val) / sum;
}

/* For Q scaling - slightly different numeric targets */
static void map_Q_scale(double dec_strength, double nor_strength, double inc_strength,
                        double *out_scale)
{
    const double dec_val = 0.85;
    const double nor_val = 1.0;
    const double inc_val = 1.3;
    double sum = dec_strength + nor_strength + inc_strength;
    if (sum <= 0.0) {
        *out_scale = 1.0;
        return;
    }
    *out_scale = (dec_strength*dec_val + nor_strength*nor_val + inc_strength*inc_val) / sum;
}

/* For gate scaling */
static void map_gate_scale(double dec_strength, double nor_strength, double inc_strength,
                           double *out_scale)
{
    const double dec_val = 0.9; /* tighten */
    const double nor_val = 1.0;
    const double inc_val = 1.2; /* loosen */
    double sum = dec_strength + nor_strength + inc_strength;
    if (sum <= 0.0) {
        *out_scale = 1.0;
        return;
    }
    *out_scale = (dec_strength*dec_val + nor_strength*nor_val + inc_strength*inc_val) / sum;
}

/* Main update function */
void fuzzy_update(const fuzzy_inputs_t *in, fuzzy_outputs_t *out)
{
    /* Input membership breakpoints - tuned conservative defaults */
    /* Mahalanobis (3 DOF threshold ~16.27) */
    const double m_low_b = 3.0;    /* below this: very good */
    const double m_med_a = 3.0, m_med_b = 8.0, m_med_c = 14.0;
    const double m_high_a = 8.0, m_high_b = 25.0;

    /* Covariance trace (depends on your state scaling) - conservative numbers */
    const double ct_low_b = 5.0;
    const double ct_med_a = 5.0, ct_med_b = 15.0, ct_med_c = 30.0;
    const double ct_high_a = 15.0, ct_high_b = 60.0;

    /* Accel norm (m/s^2) - motion energy */
    const double acc_low_b = 1.2;
    const double acc_high_a = 1.5, acc_high_b = 4.0;

    /* Evaluate memberships (0..1) */
    double mpos_low = mf_left(in->mahalanobis_pos, m_low_b, m_med_b);
    double mpos_med = mf_tri(in->mahalanobis_pos, m_med_a, m_med_b, m_med_c);
    double mpos_high = mf_right(in->mahalanobis_pos, m_high_a, m_high_b);

    double mvel_low = mf_left(in->mahalanobis_vel, m_low_b, m_med_b);
    double mvel_med = mf_tri(in->mahalanobis_vel, m_med_a, m_med_b, m_med_c);
    double mvel_high = mf_right(in->mahalanobis_vel, m_high_a, m_high_b);

    double ct_low = mf_left(in->cov_trace, ct_low_b, ct_med_b);
    double ct_med = mf_tri(in->cov_trace, ct_med_a, ct_med_b, ct_med_c);
    double ct_high = mf_right(in->cov_trace, ct_high_a, ct_high_b);

    double acc_low = mf_left(in->accel_norm, acc_low_b, acc_high_b);
    double acc_high = mf_right(in->accel_norm, acc_high_a, acc_high_b);

    /* RULES (examples). Each rule produces strengths for outputs. */
    /* We'll compute contributions for each linguistic label (dec/norm/inc) per output. */
    double R_dec = 0.0, R_norm = 0.0, R_inc = 0.0;
    double Q_dec = 0.0, Q_norm = 0.0, Q_inc = 0.0;
    double G_dec = 0.0, G_norm = 0.0, G_inc = 0.0;

    /* Rule set (Mamdani-style, combine antecedent via min()) */
    /* R1: if M_pos is High -> R increase */
    {
        double strength = mpos_high;
        R_inc = fmax(R_inc, strength);
    }

    /* R2: if M_vel is High -> R increase */
    {
        double strength = mvel_high;
        R_inc = fmax(R_inc, strength);
    }

    /* R3: if Cov_trace High AND M_pos High -> Q increase */
    {
        double strength = fmin(ct_high, mpos_high);
        Q_inc = fmax(Q_inc, strength);
    }

    /* R4: if Cov_trace High AND M_pos Low -> Q decrease (overconfident but trace high) */
    {
        double strength = fmin(ct_high, mpos_low);
        Q_dec = fmax(Q_dec, strength);
    }

    /* R5: if Accel high -> Q increase (dynamic motion) */
    {
        double strength = acc_high;
        Q_inc = fmax(Q_inc, strength);
    }

    /* R6: if M_pos Low AND M_vel Low -> tighten gate (decrease) */
    {
        double strength = fmin(mpos_low, mvel_low);
        G_dec = fmax(G_dec, strength);
    }

    /* R7: if both pos and vel high -> loosen gate and increase R */
    {
        double strength = fmin(mpos_high, mvel_high);
        G_inc = fmax(G_inc, strength);
        R_inc = fmax(R_inc, strength);
    }

    /* R8: baseline - if none triggered, prefer normal */
    {
        /* "normal" filler strength can be 1 - max(inc,dec) clipped */
        double maxR = fmax(R_inc, R_dec);
        R_norm = fmax(R_norm, clamp(1.0 - maxR, 0.0, 1.0));
        double maxQ = fmax(Q_inc, Q_dec);
        Q_norm = fmax(Q_norm, clamp(1.0 - maxQ, 0.0, 1.0));
        double maxG = fmax(G_inc, G_dec);
        G_norm = fmax(G_norm, clamp(1.0 - maxG, 0.0, 1.0));
    }

    /* Map linguistic strengths to numeric scales */
    double r_scale, q_scale, g_scale;
    map_R_scale(R_dec, R_norm, R_inc, &r_scale);
    map_Q_scale(Q_dec, Q_norm, Q_inc, &q_scale);
    map_gate_scale(G_dec, G_norm, G_inc, &g_scale);

    /* Clamp to allowed ranges */
    r_scale = clamp(r_scale, Fparams.min_scale_R, Fparams.max_scale_R);
    q_scale = clamp(q_scale, Fparams.min_scale_Q, Fparams.max_scale_Q);
    g_scale = clamp(g_scale, Fparams.min_scale_gate, Fparams.max_scale_gate);

    /* Smooth outputs to avoid flipping */
    double alpha = Fparams.smoothing_alpha;
    out->scale_R_gps = alpha * last_out.scale_R_gps + (1.0 - alpha) * r_scale;
    out->scale_Q     = alpha * last_out.scale_Q     + (1.0 - alpha) * q_scale;
    out->scale_gate  = alpha * last_out.scale_gate  + (1.0 - alpha) * g_scale;

    /* store last */
    last_out = *out;
}

/* optional accessor */
void fuzzy_get_last_outputs(fuzzy_outputs_t *out)
{
    *out = last_out;
}
