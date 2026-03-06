/*  vcfx~.c  —  vcf~ variant with signal-rate Q, damp, reset; multichannel
 *
 *  Based on Pd's vcf~ (one-pole complex filter). Adds:
 *  - Signal inlet for Q (was control-only)
 *  - Signal inlet for damp (extra decay, Hz)
 *  - Signal inlet for reset (rising edge zeros state)
 *  - Multichannel support (like svfbp~)
 *
 *  Creation:  [vcfx~] or [vcfx~ freq Q]   default freq 440, Q 1
 *             Channel count derived from audio input inlet.
 *
 *  Inlets (all multichannel — C channels each, where C may differ from N)
 *  -----------------------------------------------------------------------
 *  Channel mapping: if C < N, inlet channels wrap (ch % C).
 *                   if C = 1, that channel broadcasts to all resonators.
 *  0  audio input
 *  1  frequency Hz          [default: 440]
 *  2  Q (resonance)         [default: 1]   Q ≤ 0 = pass-through
 *  3  damp rate Hz          [default: 0]  0 = no extra damping
 *  4  reset trigger         signal > 0 zeros state (level-triggered)
 *
 *  Outlets (each N channels)
 *  -------------------------
 *  0  real (bandpass)
 *  1  imaginary (lowpass)
 *
 *  Messages
 *  --------
 *  reset        zero all resonator states
 *  clear        alias for reset
 */

#include "m_pd.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

#ifndef M_PI
# define M_PI 3.14159265358979323846
#endif

#define INLET_CH(ch, nc) ((ch) % (nc))

typedef struct {
    float re, im;
} VcfxResonator;

static t_class *vcfx_tilde_class;

typedef struct _vcfx_tilde {
    t_object       x_obj;
    int            x_n;
    VcfxResonator *x_res;
    float          x_sr;
    float          x_f;
    float         *x_in_audio;
    float         *x_in_freq;
    float         *x_in_q;
    float         *x_in_damp;
    float         *x_in_rst;
    float         *x_out_re;
    float         *x_out_im;
    int            x_bs;
    int            x_nc_a, x_nc_f, x_nc_q, x_nc_d, x_nc_r;
} t_vcfx_tilde;

static t_int *vcfx_tilde_perform(t_int *w)
{
    t_vcfx_tilde *x = (t_vcfx_tilde *)(w[1]);
    if (x->x_n < 1 || !x->x_res) return w + 2;
    int n = x->x_n;
    int bs = x->x_bs;
    float sr = x->x_sr;
    float isr = 1.f / sr;

    float *in_audio = x->x_in_audio;
    float *in_freq  = x->x_in_freq;
    float *in_q     = x->x_in_q;
    float *in_damp  = x->x_in_damp;
    float *in_rst   = x->x_in_rst;
    float *out_re   = x->x_out_re;
    float *out_im   = x->x_out_im;

    int nc_a = x->x_nc_a, nc_f = x->x_nc_f, nc_q = x->x_nc_q;
    int nc_d = x->x_nc_d, nc_r = x->x_nc_r;

    for (int ch = 0; ch < n; ch++) {
        VcfxResonator *r = &x->x_res[ch];
        float *a   = in_audio + INLET_CH(ch, nc_a) * bs;
        float *frq = in_freq  + INLET_CH(ch, nc_f) * bs;
        float *q   = in_q     + INLET_CH(ch, nc_q) * bs;
        float *dmp = in_damp  + INLET_CH(ch, nc_d) * bs;
        float *rst = in_rst   + INLET_CH(ch, nc_r) * bs;
        float *o_re = out_re  + ch * bs;
        float *o_im = out_im  + ch * bs;

        float re = r->re;
        float im = r->im;

        for (int i = 0; i < bs; i++) {
            if (rst[i] > 0.f) { re = 0.f; im = 0.f; }

            float freq = frq[i];
            float qval = q[i];
            float damp_hz = dmp[i];
            if (freq < 0.f) freq = 0.f;
            if (freq > sr * 0.49f) freq = sr * 0.49f;
            if (damp_hz < 0.f) damp_hz = 0.f;

            float cf = freq * isr;
            float qinv = (qval > 0.f) ? (1.f / qval) : 0.f;
            float rval = (qinv > 0.f) ? (1.f - cf * qinv) : 0.f;
            if (rval < 0.f) rval = 0.f;
            float oneminusr = 1.f - rval;

            float ampcorrect = (qval > 0.f) ? (2.f - 2.f / (qval + 2.f)) : 1.f;

            float coefr = rval * cosf((float)(2.0 * M_PI) * cf);
            float coefi = rval * sinf((float)(2.0 * M_PI) * cf);

            float f1 = a[i];
            float re2 = re;
            re = ampcorrect * oneminusr * f1 + coefr * re2 - coefi * im;
            im = coefi * re2 + coefr * im;

            float dc = (damp_hz > 0.f) ? expf(-damp_hz * isr) : 1.f;
            re *= dc;
            im *= dc;

            o_re[i] = re;
            o_im[i] = im;
        }

        if (re != re || im != im) { re = 0.f; im = 0.f; }
        r->re = re;
        r->im = im;
    }

    return w + 2;
}

static void vcfx_tilde_dsp(t_vcfx_tilde *x, t_signal **sp)
{
    if (!sp || !sp[0]) return;
    int n = sp[0]->s_nchans;
    if (n < 1) n = 1;
    if (n > 4096) n = 4096;

    if (n != x->x_n) {
        size_t oldsize = sizeof(VcfxResonator) * (size_t)x->x_n;
        size_t newsize = sizeof(VcfxResonator) * (size_t)n;
        x->x_res = (VcfxResonator *)resizebytes(x->x_res, oldsize, newsize);
        if (n > x->x_n)
            memset(x->x_res + x->x_n, 0, sizeof(VcfxResonator) * (size_t)(n - x->x_n));
        x->x_n = n;
    }

    x->x_sr       = sp[0]->s_sr;
    x->x_bs       = sp[0]->s_n;
    x->x_in_audio = sp[0]->s_vec;
    x->x_in_freq  = sp[1]->s_vec;
    x->x_in_q     = sp[2]->s_vec;
    x->x_in_damp  = sp[3]->s_vec;
    x->x_in_rst   = sp[4]->s_vec;
    x->x_nc_a     = sp[0]->s_nchans;
    x->x_nc_f     = sp[1]->s_nchans;
    x->x_nc_q     = sp[2]->s_nchans;
    x->x_nc_d     = sp[3]->s_nchans;
    x->x_nc_r     = sp[4]->s_nchans;
    signal_setmultiout(&sp[5], x->x_n);
    signal_setmultiout(&sp[6], x->x_n);
    x->x_out_re   = sp[5]->s_vec;
    x->x_out_im   = sp[6]->s_vec;
    dsp_add(vcfx_tilde_perform, 1, x);
}

static void vcfx_tilde_reset(t_vcfx_tilde *x)
{
    if (!x->x_res) return;
    for (int i = 0; i < x->x_n; i++) {
        x->x_res[i].re = 0.f;
        x->x_res[i].im = 0.f;
    }
}

static void *vcfx_tilde_new(t_floatarg f_arg, t_floatarg q_arg)
{
    float f_def = (f_arg > 0.f) ? f_arg : 440.f;
    float q_def = (q_arg > 0.f) ? q_arg : 1.f;

    t_vcfx_tilde *x = (t_vcfx_tilde *)pd_new(vcfx_tilde_class);
    x->x_n   = 0;
    x->x_sr  = 44100.f;
    x->x_f   = 0.f;
    x->x_res = NULL;

    signalinlet_new(&x->x_obj, f_def);
    signalinlet_new(&x->x_obj, q_def);
    signalinlet_new(&x->x_obj, 0.f);
    signalinlet_new(&x->x_obj, 0.f);

    outlet_new(&x->x_obj, gensym("signal"));
    outlet_new(&x->x_obj, gensym("signal"));

    return (void *)x;
}

static void vcfx_tilde_free(t_vcfx_tilde *x)
{
    if (x->x_res && x->x_n > 0)
        freebytes(x->x_res, sizeof(VcfxResonator) * x->x_n);
}

EXTERN void vcfx_tilde_setup(void)
{
    vcfx_tilde_class = class_new(
        gensym("vcfx~"),
        (t_newmethod)vcfx_tilde_new,
        (t_method)vcfx_tilde_free,
        sizeof(t_vcfx_tilde),
        CLASS_MULTICHANNEL,
        A_DEFFLOAT, A_DEFFLOAT,   /* freq, Q */
        0);

    CLASS_MAINSIGNALIN(vcfx_tilde_class, t_vcfx_tilde, x_f);

    class_addmethod(vcfx_tilde_class,
        (t_method)vcfx_tilde_dsp, gensym("dsp"), A_CANT, 0);
    class_addmethod(vcfx_tilde_class,
        (t_method)vcfx_tilde_reset, gensym("reset"), 0);
    class_addmethod(vcfx_tilde_class,
        (t_method)vcfx_tilde_reset, gensym("clear"), 0);
}
