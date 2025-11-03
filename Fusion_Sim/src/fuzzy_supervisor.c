#include "../include/fuzzy_supervisor.h"
#include <math.h>

// Default parameter bounds
static const fuzzy_params_t default_params =
{
    .min_scale_R = 0.6,
    .max_scale_R = 3.0,
    .min_scale_Q = 0.7,
    .max_scale_Q = 2.0,
    .min_scale_gate = 0.85,
    .max_scale_gate = 1.5,
    .smoothing_alpha = 0.85
};

// Internal state
static fuzzy_params_t Fparams;
static fuzzy_outputs_t last_out = {1.0, 1.0, 1.0};

// left-shoulder: 1 up to b, then falls to 0 at c
static double mf_left(double x, double b, double c)
{
    if (x <= b) 
        return (1);
    if (x >= c) return (0);
        return ((c - x) / (c - b));
}

// right-shoulder: 0 up to a, then rises to 1 at b
static double mf_right(double x, double a, double b)
{
    if (x <= a)
        return (0);
    if (x >= b)
        return (0);
    return
        ((x - a) / (b - a));
}

// clamp helper
static double clamp(double x, double lo, double hi)
{
    if (x < lo)
        return (lo);
    if (x > hi) return (hi);
    return (x);
}

// Initialize fuzzy params (copy defaults if NULL)
void fuzzy_init(const fuzzy_params_t *params)
{
    if (params)
    {
        Fparams = *params;   // sanitize 
        Fparams.min_scale_R = clamp(Fparams.min_scale_R, 0.1, 10.0);
        Fparams.max_scale_R = clamp(Fparams.max_scale_R, Fparams.min_scale_R, 10.0);
        Fparams.min_scale_Q = clamp(Fparams.min_scale_Q, 0.1, 10.0);
        Fparams.max_scale_Q = clamp(Fparams.max_scale_Q, Fparams.min_scale_Q, 10.0);
        Fparams.min_scale_gate = clamp(Fparams.min_scale_gate, 0.1, 5.0);
        Fparams.max_scale_gate = clamp(Fparams.max_scale_gate, Fparams.min_scale_gate, 5.0);
        Fparams.smoothing_alpha = clamp(Fparams.smoothing_alpha, 0.0, 0.99);
    }
    else
        Fparams = default_params;

    // initialize last outputs to neutral
    last_out.scale_R_gps = 1.0;
    last_out.scale_Q = 1.0;
    last_out.scale_gate = 1.0;
}

// Map linguistic label -> numeric scale (per output).
// three levels: decrease / normal / increase.
static void map_R_scale(double dec_strength, double nor_strength, double inc_strength, double *out_scale)
{
    // numeric targets for each label
    const double dec_val = 0.75;
    const double nor_val = 1.0;
    const double inc_val = 1.5;
    double sum = dec_strength + nor_strength + inc_strength;
    if (sum <= 0.0)
    {
        *out_scale = 1.0;
        return;
    }
    *out_scale = (dec_strength*dec_val + nor_strength*nor_val + inc_strength*inc_val) / sum;
}

// For Q scaling - slightly different numeric targets

static void map_Q_scale(double Q_dec, double Q_norm, double Q_inc, double *q_scale)
{
    const double dec_val = 0.75;
    const double nor_val = 1.0;
    const double inc_val = 1.45;

    double sum = Q_dec + Q_norm + Q_inc;
    if (sum <= 0.0)
        *q_scale = 1.0;
    else
        *q_scale = (Q_dec*dec_val + Q_norm*nor_val + Q_inc*inc_val) / sum;
}

// For gate scaling
static void map_gate_scale(double dec_strength, double nor_strength, double inc_strength, double *out_scale)
{
    const double dec_val = 0.9; // tighten
    const double nor_val = 1.0;
    const double inc_val = 1.2; // loosen
    double sum = dec_strength + nor_strength + inc_strength;

    if (sum <= 0.0)
    {
        *out_scale = 1.0;
        return;
    }
    *out_scale = (dec_strength*dec_val + nor_strength*nor_val + inc_strength*inc_val) / sum;
}

// Main update function
void fuzzy_update(const fuzzy_inputs_t *in, fuzzy_outputs_t *out)
{
    // Input membership breakpoints - tuned conservative defaults
    // Mahalanobis (3 DOF threshold ~16.27)
    // Input membership breakpoints - tuned conservative defaults
    const double m_low_b  = 3.0;
    const double m_med_b  = 8.0;
    const double m_high_a = 6.0;
    const double m_high_b = 16.3;   // <- roughly 3σ

    // Covariance trace - conservative numbers
    const double ct_high_a = 15.0;
    const double ct_high_b = 60.0;

    const double acc_high_a = 0.6;
    const double acc_high_b = 2.5; // Accel norm (m/s^2)

    // Evaluate memberships (0..1)
    double mpos_low = mf_left(in->mahalanobis_pos, m_low_b, m_med_b);
    double mpos_high = mf_right(in->mahalanobis_pos, m_high_a, m_high_b);

    double mvel_low = mf_left(in->mahalanobis_vel, m_low_b, m_med_b);
    double mvel_high = mf_right(in->mahalanobis_vel, m_high_a, m_high_b);

    double ct_high = mf_right(in->cov_trace, ct_high_a, ct_high_b);
    double acc_high = mf_right(in->accel_norm, acc_high_a, acc_high_b);

    // RULES. Each rule produces strengths for outputs.
    // compute contributions for each linguistic label (dec/norm/inc) per output.
    double R_dec = 0.0, R_norm = 0.0, R_inc = 0.0;
    double Q_dec = 0.0, Q_norm = 0.0, Q_inc = 0.0;
    double G_dec = 0.0, G_norm = 0.0, G_inc = 0.0;

    // Rule set (Mamdani-style, combine antecedent via min())

    // R1: if M_pos is High -> R increase
    R_inc = fmax(R_inc, mpos_high);

    // R2: if M_vel is High -> R increase 
    R_inc = fmax(R_inc, mvel_high);

    // R3: if Cov_trace High AND M_pos High -> Q increase
    Q_inc = fmax(Q_inc, fmin(ct_high, mpos_high));

    // R4: if Cov_trace High AND M_pos Low -> Q decrease (overconfident but trace high)
    Q_dec = fmax(Q_dec, fmin(ct_high, mpos_low));

    // R5: if both velocity error and acceleration are high -> Q increase
    Q_inc = fmax(Q_inc, fmin(mvel_high, acc_high));

    // R6: if M_pos Low AND M_vel Low -> tighten gate (decrease)
    G_dec = fmax(G_dec, fmin(mpos_low, mvel_low));

    // R7: if both pos and vel high -> loosen gate and increase R
    {
        double strength = fmin(mpos_high, mvel_high);
        G_inc = fmax(G_inc, strength);
        R_inc = fmax(R_inc, strength);
    }

    // R8: baseline - if none triggered, prefer normal
    {
        // "normal" filler strength can be 1 - max(inc,dec) clipped
        double maxR = fmax(R_inc, R_dec);
        double maxQ = fmax(Q_inc, Q_dec);
        double maxG = fmax(G_inc, G_dec);

        R_norm = fmax(R_norm, clamp(1.0 - maxR, 0.0, 1.0));
        Q_norm = fmax(Q_norm, clamp(1.0 - maxQ, 0.0, 1.0));
        G_norm = fmax(G_norm, clamp(1.0 - maxG, 0.0, 1.0));
    }

    if (Q_inc < 0.25)
            Q_inc = 0.0;

    // Map linguistic strengths to numeric scales
    double r_scale;
    double q_scale;
    double g_scale;

    map_R_scale(R_dec, R_norm, R_inc, &r_scale);
    map_Q_scale(Q_dec, Q_norm, Q_inc, &q_scale);
    map_gate_scale(G_dec, G_norm, G_inc, &g_scale);

    // Clamp to allowed ranges
    r_scale = clamp(r_scale, Fparams.min_scale_R, Fparams.max_scale_R);
    q_scale = clamp(q_scale, Fparams.min_scale_Q, Fparams.max_scale_Q);
    g_scale = clamp(g_scale, Fparams.min_scale_gate, Fparams.max_scale_gate);

    // Adaptive alpha for Q: reacts faster when Q_inc is strong
    double alpha_Q = Fparams.smoothing_alpha - 0.4 * Q_inc;
    alpha_Q = clamp(alpha_Q, 0.6, Fparams.smoothing_alpha);


    // Smooth outputs to avoid flipping
    out->scale_R_gps    = Fparams.smoothing_alpha * last_out.scale_R_gps + (1.0 - Fparams.smoothing_alpha) * r_scale;
    out->scale_Q        = alpha_Q * last_out.scale_Q + (1.0 - alpha_Q) * q_scale;
    out->scale_gate     = Fparams.smoothing_alpha * last_out.scale_gate + (1.0 - Fparams.smoothing_alpha) * g_scale;
    last_out = *out;    // store last
}
