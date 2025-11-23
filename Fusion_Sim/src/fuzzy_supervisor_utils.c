#include "../include/fuzzy_supervisor.h"

double clamp_d(double x, double lo, double hi)
{
    if (x < lo)
        return lo;
    if (x > hi)
        return hi;
    return x;
}

double mf_left(double x, double b, double c)
{
    if (c <= b)
    {
        if ( x <= b)
            return (1.0);
        else 
            return (0.0);
    }
    if (x <= b)
        return (1.0);
    if (x >= c)
        return (0.0);
    return ((c - x) / (c - b));
}

double mf_right(double x, double a, double b)
{
    if (b <= a)
    {
        if ( x >= b)
            return (1.0);
        else 
            return (0.0);
    }
    if (x <= a)
       return (0.0);
    if (x >= b)
        return (1.0);
    return ((x - a) / (b - a));
}
// generic defuzzifier:
double defuzzify_generic(double dec_s, double nor_s, double inc_s,
                                double dec_val, double nor_val, double inc_val)
{
    double sum = dec_s + nor_s + inc_s;
    if (sum <= 0.0)
        return nor_val;
    return ((dec_s * dec_val + nor_s * nor_val + inc_s * inc_val) / sum);
}

/* smoothing helper: new_out = alpha * last + (1-alpha) * raw
 * alpha in [0,1], alpha=1 -> keep last (no change), alpha=0 -> immediate change
 */
double smooth_value(double last, double raw, double alpha)
{
    return (alpha * last + (1.0 - alpha) * raw);
}

// Validate and sanitize user fuzzy params 
void fuzz_params_sanitize(fuzzy_params_t *p)
{
    p->min_scale_R = clamp_d(p->min_scale_R, 0.01, 100.0);
    p->max_scale_R = clamp_d(p->max_scale_R, p->min_scale_R, 100.0);

    p->min_scale_Q = clamp_d(p->min_scale_Q, 0.01, 100.0);
    p->max_scale_Q = clamp_d(p->max_scale_Q, p->min_scale_Q, 100.0);

    p->min_scale_gate = clamp_d(p->min_scale_gate, 0.1, 10.0);
    p->max_scale_gate = clamp_d(p->max_scale_gate, p->min_scale_gate, 10.0);

    p->smoothing_alpha = clamp_d(p->smoothing_alpha, 0.0, 0.99);
}
