/*  butter3~.c  —  3rd order Butterworth lowpass filter (multichannel)
 *
 *  Cascaded 1st-order + 2nd-order sections. Bilinear transform with
 *  frequency prewarping. Suitable for smoothing sensor data at signal rate.
 *  Channel count derived from audio input inlet. Adapts to samplerate.
 *
 *  Creation:  [butter3~] or [butter3~ freq]   default 100 Hz
 *
 *  Inlets (multichannel — C channels each, wrap if C < N):
 *  0  signal input
 *  1  cutoff frequency Hz (cold, signal or control)
 *
 *  Outlet:
 *  0  filtered signal, N channels
 */

#include "m_pd.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

#ifndef M_PI
# define M_PI 3.14159265358979323846
#endif

#define BUTTER3_FC_DEF  100
#define INLET_CH(ch, nc) ((ch) % (nc))

static t_class *butter3_tilde_class;

/* Per-channel filter state and coefficients */
typedef struct {
    t_float y1, x1, s1, s2;
    t_float b0_1, a1_1;
    t_float b0_2, b1_2, b2_2, a1_2, a2_2;
    t_float fc_prev;
} Butter3Chan;

typedef struct _butter3_tilde {
    t_object      x_obj;
    int           x_n;
    Butter3Chan  *x_chan;
    t_float       x_sr;
    t_float       x_fc;        /* must be t_float for CLASS_MAINSIGNALIN */
    t_sample     *x_in;
    t_sample     *x_in_freq;
    t_sample     *x_out;
    int           x_bs;
    int           x_nc_a;
    int           x_nc_f;
} t_butter3_tilde;

static void butter3_coeffs(Butter3Chan *c, t_float fc, t_float sr)
{
    t_float f = fc;
    t_float s = sr;
    if (f < 0.1) f = 0.1;
    if (f > s * 0.49) f = s * 0.49;

    t_float K1 = tan(M_PI * f / s);
    c->b0_1 = K1 / (1 + K1);
    c->a1_1 = (K1 - 1) / (1 + K1);

    t_float w0 = 2.0 * M_PI * f / s;
    t_float cosw = cos(w0);
    t_float sinw = sin(w0);
    t_float alpha = sinw * 0.5;
    t_float norm = 1.0 / (1.0 + alpha);
    c->b0_2 = (1 - cosw) * 0.5 * norm;
    c->b1_2 = (1 - cosw) * norm;
    c->b2_2 = c->b0_2;
    c->a1_2 = -2.0 * cosw * norm;
    c->a2_2 = (1 - alpha) * norm;
    c->fc_prev = f;
}

static t_int *butter3_tilde_perform(t_int *w)
{
    t_butter3_tilde *x = (t_butter3_tilde *)(w[1]);
    if (x->x_n < 1 || !x->x_chan) return w + 2;

    int n = x->x_n;
    int bs = x->x_bs;
    t_float sr = x->x_sr;
    int nc_a = x->x_nc_a;
    int nc_f = x->x_nc_f;

    t_sample *in = x->x_in;
    t_sample *in_freq = x->x_in_freq;
    t_sample *out = x->x_out;

    for (int ch = 0; ch < n; ch++) {
        Butter3Chan *c = &x->x_chan[ch];
        t_sample *a = in + INLET_CH(ch, nc_a) * bs;
        t_sample *frq = in_freq + INLET_CH(ch, nc_f) * bs;
        t_sample *o = out + ch * bs;

        t_float y1 = c->y1, x1 = c->x1, s1 = c->s1, s2 = c->s2;
        t_float b0_1 = c->b0_1, a1_1 = c->a1_1;
        t_float b0_2 = c->b0_2, b1_2 = c->b1_2, b2_2 = c->b2_2;
        t_float a1_2 = c->a1_2, a2_2 = c->a2_2;
        t_float fc_prev = c->fc_prev;

        for (int i = 0; i < bs; i++) {
            t_float fc = frq[i];
            if (fc != fc_prev && fc == fc) {
                butter3_coeffs(c, fc, sr);
                b0_1 = c->b0_1; a1_1 = c->a1_1;
                b0_2 = c->b0_2; b1_2 = c->b1_2; b2_2 = c->b2_2;
                a1_2 = c->a1_2; a2_2 = c->a2_2;
                fc_prev = fc;
            }

            t_float x0 = a[i];
            t_float v1 = b0_1 * (x0 + x1) - a1_1 * y1;
            x1 = x0;
            y1 = v1;

            t_float s0 = v1 - a1_2 * s1 - a2_2 * s2;
            t_float v2 = b0_2 * s0 + b1_2 * s1 + b2_2 * s2;
            s2 = s1;
            s1 = s0;

            o[i] = v2;
        }

        c->y1 = y1;
        c->x1 = x1;
        c->s1 = s1;
        c->s2 = s2;
        c->fc_prev = fc_prev;
    }

    return w + 2;
}

static void butter3_tilde_dsp(t_butter3_tilde *x, t_signal **sp)
{
    if (!sp || !sp[0]) return;
    int n = sp[0]->s_nchans;
    if (n < 1) n = 1;
    if (n > 4096) n = 4096;

    x->x_sr = sp[0]->s_sr;

    if (n != x->x_n) {
        size_t oldsize = sizeof(Butter3Chan) * (size_t)x->x_n;
        size_t newsize = sizeof(Butter3Chan) * (size_t)n;
        x->x_chan = (Butter3Chan *)resizebytes(x->x_chan, oldsize, newsize);
        if (n > x->x_n) {
            for (int i = x->x_n; i < n; i++) {
                Butter3Chan *c = &x->x_chan[i];
                c->y1 = c->x1 = c->s1 = c->s2 = 0;
                c->fc_prev = -1;
                butter3_coeffs(c, x->x_fc, x->x_sr);
            }
        }
        x->x_n = n;
    }
    x->x_bs = sp[0]->s_n;
    x->x_in = sp[0]->s_vec;
    x->x_in_freq = sp[1]->s_vec;
    x->x_nc_a = sp[0]->s_nchans;
    x->x_nc_f = sp[1]->s_nchans;

    signal_setmultiout(&sp[2], x->x_n);
    x->x_out = sp[2]->s_vec;

    dsp_add(butter3_tilde_perform, 1, x);
}

static void butter3_tilde_float(t_butter3_tilde *x, t_float f)
{
    x->x_fc = f;
}

static void *butter3_tilde_new(t_floatarg f)
{
    t_butter3_tilde *x = (t_butter3_tilde *)pd_new(butter3_tilde_class);
    x->x_n = 0;
    x->x_chan = NULL;
    x->x_sr = 48000;
    x->x_fc = (f > 0) ? f : BUTTER3_FC_DEF;

    signalinlet_new(&x->x_obj, x->x_fc);
    outlet_new(&x->x_obj, gensym("signal"));

    return (void *)x;
}

static void butter3_tilde_free(t_butter3_tilde *x)
{
    if (x->x_chan && x->x_n > 0)
        freebytes(x->x_chan, sizeof(Butter3Chan) * x->x_n);
}

void butter3_tilde_setup(void)
{
    butter3_tilde_class = class_new(
        gensym("butter3~"),
        (t_newmethod)butter3_tilde_new,
        (t_method)butter3_tilde_free,
        sizeof(t_butter3_tilde),
        CLASS_MULTICHANNEL,
        A_DEFFLOAT, 0);

    CLASS_MAINSIGNALIN(butter3_tilde_class, t_butter3_tilde, x_fc);

    class_addmethod(butter3_tilde_class,
        (t_method)butter3_tilde_dsp, gensym("dsp"), A_CANT, 0);
    class_addmethod(butter3_tilde_class,
        (t_method)butter3_tilde_float, gensym("float"), A_FLOAT, 0);
}
