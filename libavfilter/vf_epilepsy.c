/*
 * Copyright (c) 2019 Vladimir Panteleev
 *
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#include <float.h>

#include "libavutil/imgutils.h"
#include "libavutil/opt.h"
#include "libavutil/pixdesc.h"
#include "avfilter.h"

#include "formats.h"
#include "internal.h"
#include "video.h"

#define MAX_FRAMES 240
#define GRID_SIZE 8

typedef struct EpilepsyFrame {
    uint8_t grid[GRID_SIZE][GRID_SIZE][4];
} EpilepsyFrame;

typedef struct EpilepsyContext {
    const AVClass *class;

    int nb_frames;
    float threshold_multiplier;
    float blend_factor;

    int badness_threshold;

    /* Circular buffer */
    int history[MAX_FRAMES];
    int history_pos;
    int current_badness;

    EpilepsyFrame last_frame_e;
    AVFrame *last_frame_av;
} EpilepsyContext;

#define OFFSET(x) offsetof(EpilepsyContext, x)
#define FLAGS AV_OPT_FLAG_VIDEO_PARAM|AV_OPT_FLAG_FILTERING_PARAM

static const AVOption epilepsy_options[] = {
    { "frames",    "set how many frames to use"       ,  OFFSET(nb_frames           ), AV_OPT_TYPE_INT  , {.i64=5  }, 2, MAX_FRAMES, FLAGS },
    { "f",         "set how many frames to use"       ,  OFFSET(nb_frames           ), AV_OPT_TYPE_INT  , {.i64=5  }, 2, MAX_FRAMES, FLAGS },
    { "threshold", "detection threshold"              ,  OFFSET(threshold_multiplier), AV_OPT_TYPE_FLOAT, {.dbl=1  }, 0, FLT_MAX   , FLAGS },
    { "t"        , "detection threshold"              ,  OFFSET(threshold_multiplier), AV_OPT_TYPE_FLOAT, {.dbl=1  }, 0, FLT_MAX   , FLAGS },
    { "blend"    , "blend factor (0 = use last frame)",  OFFSET(blend_factor        ), AV_OPT_TYPE_FLOAT, {.dbl=0.1}, 0, 1         , FLAGS },
    { "b"        , "blend factor (0 = use last frame)",  OFFSET(blend_factor        ), AV_OPT_TYPE_FLOAT, {.dbl=0.1}, 0, 1         , FLAGS },
    { NULL }
};

AVFILTER_DEFINE_CLASS(epilepsy);

 static int query_formats(AVFilterContext *ctx)
{
    static const enum AVPixelFormat pixel_fmts[] = {
        AV_PIX_FMT_RGB24,
        AV_PIX_FMT_BGR24,
        AV_PIX_FMT_NONE
    };
    AVFilterFormats *formats = ff_make_format_list(pixel_fmts);
    if (!formats)
        return AVERROR(ENOMEM);
    return ff_set_common_formats(ctx, formats);
}

static void convert_frame(AVFrame *in, EpilepsyFrame* out)
{
    int gx, gy, x0, x1, y0, y1, x, y, c, sum;
    const uint8_t *row;

    for (c = 0; c < 3; c++) {
        for (gy = 0; gy < GRID_SIZE; gy++) {
            y0 = in->height *  gy    / GRID_SIZE;
            y1 = in->height * (gy+1) / GRID_SIZE;
            for (gx = 0; gx < GRID_SIZE; gx++) {
                x0 = in->width *  gx    / GRID_SIZE;
                x1 = in->width * (gx+1) / GRID_SIZE;
                sum = 0;
                for (y = y0; y < y1; y++) {
                    row = in->data[0] + y * in->linesize[0];
                    for (x = x0; x < x1; x++) {
                        //av_log(NULL, AV_LOG_INFO, "%d %d %d : (%d,%d) (%d,%d) -> %d,%d | *%d\n", c, gx, gy, x0, y0, x1, y1, x, y, (int)row);
                        sum += row[x * 3 + c]; // TODO: variable size
                    }
                }
                if (sum)
                    sum /= (y1 - y0) * (x1 - x0);
                out->grid[gy][gx][c] = sum;
            }
        }
    }
}

static void blend_frame(AVFrame *target, AVFrame *source, float factor)
{
    int x, y;
    uint8_t *t, *s;
    const uint16_t s_mul = (uint16_t)(factor * 0x100);
    const uint16_t t_mul = 0x100 - s_mul;

    for (y = 0; y < target->height; y++) {
        t = target->data[0] + y * target->linesize[0];
        s = source->data[0] + y * source->linesize[0];
        for (x = 0; x < target->linesize[0]; x++) {
            *t = (*t * t_mul + *s * s_mul) >> 8;
            t++; s++;
        }
    }
}

static int get_badness(EpilepsyFrame* a, EpilepsyFrame* b)
{
    int badness, x, y, c;
    badness = 0;
    for (c = 0; c < 3; c++) {
        for (y = 0; y < GRID_SIZE; y++) {
            for (x = 0; x < GRID_SIZE; x++) {
                badness += abs((int)a->grid[y][x][c] - (int)b->grid[y][x][c]);
                //av_log(NULL, AV_LOG_INFO, "%d - %d -> %d \n", a->grid[y][x], b->grid[y][x], badness);
                //av_log(NULL, AV_LOG_INFO, "%d -> %d \n", abs((int)a->grid[y][x] - (int)b->grid[y][x]), badness);
            }
        }
    }
    return badness;
}

/* static int epilepsy8(AVFilterContext *ctx, */
/*                       const uint8_t *src, ptrdiff_t src_linesize, */
/*                       uint8_t *dst, ptrdiff_t dst_linesize, */
/*                       int w, int h, float f) */
/* { */
/*     int x, y; */

/*     for (y = 0; y < h; y++) { */
/*         for (x = 0; x < w; x++) { */
/*             dst[x] = av_clip_uint8(src[x] * f); */
/*         } */

/*         dst += dst_linesize; */
/*         src += src_linesize; */
/*     } */

/*     return 0; */
/* } */

/* static int epilepsy16(AVFilterContext *ctx, */
/*                        const uint8_t *ssrc, ptrdiff_t src_linesize, */
/*                        uint8_t *ddst, ptrdiff_t dst_linesize, */
/*                        int w, int h, float f) */
/* { */
/*     EpilepsyContext *s = ctx->priv; */
/*     const uint16_t *src = (const uint16_t *)ssrc; */
/*     uint16_t *dst = (uint16_t *)ddst; */
/*     const int max = (1 << s->depth) - 1; */
/*     int x, y; */

/*     for (y = 0; y < h; y++) { */
/*         for (x = 0; x < w; x++) { */
/*             dst[x] = av_clip(src[x] * f, 0, max); */
/*         } */

/*         dst += dst_linesize / 2; */
/*         src += src_linesize / 2; */
/*     } */

/*     return 0; */
/* } */

/* static float calc_avgy8(AVFilterContext *ctx, AVFrame *in) */
/* { */
/*     EpilepsyContext *s = ctx->priv; */
/*     const uint8_t *src = in->data[0]; */
/*     int64_t sum = 0; */
/*     int y, x; */

/*     memset(s->histogram, 0, (1 << s->depth) * sizeof(*s->histogram)); */

/*     for (y = 0; y < s->planeheight[0]; y++) { */
/*         for (x = 0; x < s->planewidth[0]; x++) { */
/*             s->histogram[src[x]]++; */
/*         } */
/*         src += in->linesize[0]; */
/*     } */

/*     for (y = 0; y < 1 << s->depth; y++) { */
/*         sum += s->histogram[y] * y; */
/*     } */

/*     return 1.0f * sum / (s->planeheight[0] * s->planewidth[0]); */
/* } */

/* static float calc_avgy16(AVFilterContext *ctx, AVFrame *in) */
/* { */
/*     EpilepsyContext *s = ctx->priv; */
/*     const uint16_t *src = (const uint16_t *)in->data[0]; */
/*     int64_t sum = 0; */
/*     int y, x; */

/*     memset(s->histogram, 0, (1 << s->depth) * sizeof(*s->histogram)); */

/*     for (y = 0; y < s->planeheight[0]; y++) { */
/*         for (x = 0; x < s->planewidth[0]; x++) { */
/*             s->histogram[src[x]]++; */
/*         } */
/*         src += in->linesize[0] / 2; */
/*     } */

/*     for (y = 0; y < 1 << s->depth; y++) { */
/*         sum += s->histogram[y] * y; */
/*     } */

/*     return 1.0f * sum / (s->planeheight[0] * s->planewidth[0]); */
/* } */

/* static void get_am_factor(AVFilterContext *ctx, float *f) */
/* { */
/*     EpilepsyContext *s = ctx->priv; */
/*     int y; */

/*     *f = 0.0f; */

/*     for (y = 0; y < s->size; y++) { */
/*         *f += s->luminance[y]; */
/*     } */

/*     *f /= s->size; */
/*     *f /= s->luminance[0]; */
/* } */

/* static void get_gm_factor(AVFilterContext *ctx, float *f) */
/* { */
/*     EpilepsyContext *s = ctx->priv; */
/*     int y; */

/*     *f = 1; */

/*     for (y = 0; y < s->size; y++) { */
/*         *f *= s->luminance[y]; */
/*     } */

/*     *f = pow(*f, 1.0f / s->size); */
/*     *f /= s->luminance[0]; */
/* } */

/* static void get_hm_factor(AVFilterContext *ctx, float *f) */
/* { */
/*     EpilepsyContext *s = ctx->priv; */
/*     int y; */

/*     *f = 0.0f; */

/*     for (y = 0; y < s->size; y++) { */
/*         *f += 1.0f / s->luminance[y]; */
/*     } */

/*     *f = s->size / *f; */
/*     *f /= s->luminance[0]; */
/* } */

/* static void get_qm_factor(AVFilterContext *ctx, float *f) */
/* { */
/*     EpilepsyContext *s = ctx->priv; */
/*     int y; */

/*     *f = 0.0f; */

/*     for (y = 0; y < s->size; y++) { */
/*         *f += s->luminance[y] * s->luminance[y]; */
/*     } */

/*     *f /= s->size; */
/*     *f  = sqrtf(*f); */
/*     *f /= s->luminance[0]; */
/* } */

/* static void get_cm_factor(AVFilterContext *ctx, float *f) */
/* { */
/*     EpilepsyContext *s = ctx->priv; */
/*     int y; */

/*     *f = 0.0f; */

/*     for (y = 0; y < s->size; y++) { */
/*         *f += s->luminance[y] * s->luminance[y] * s->luminance[y]; */
/*     } */

/*     *f /= s->size; */
/*     *f  = cbrtf(*f); */
/*     *f /= s->luminance[0]; */
/* } */

/* static void get_pm_factor(AVFilterContext *ctx, float *f) */
/* { */
/*     EpilepsyContext *s = ctx->priv; */
/*     int y; */

/*     *f = 0.0f; */

/*     for (y = 0; y < s->size; y++) { */
/*         *f += powf(s->luminance[y], s->size); */
/*     } */

/*     *f /= s->size; */
/*     *f  = powf(*f, 1.0f / s->size); */
/*     *f /= s->luminance[0]; */
/* } */

/* static int comparef(const void *a, const void *b) */
/* { */
/*     const float *aa = a, *bb = b; */
/*     return round(aa - bb); */
/* } */

/* static void get_median_factor(AVFilterContext *ctx, float *f) */
/* { */
/*     EpilepsyContext *s = ctx->priv; */

/*     memcpy(s->sorted, s->luminance, sizeof(s->sorted)); */
/*     AV_QSORT(s->sorted, s->size, float, comparef); */

/*     *f = s->sorted[s->size >> 1] / s->luminance[0]; */
/* } */

static int config_input(AVFilterLink *inlink)
{
    /* const AVPixFmtDescriptor *desc = av_pix_fmt_desc_get(inlink->format); */
    AVFilterContext *ctx = inlink->dst;
    EpilepsyContext *s = ctx->priv;

    s->badness_threshold = (int)(GRID_SIZE * GRID_SIZE * 4 * 256 * s->nb_frames * s->threshold_multiplier / 16);

    return 0;
}

static int filter_frame(AVFilterLink *inlink, AVFrame *in)
{
    int this_badness, new_badness;
    EpilepsyFrame ef;
    AVFrame *src;

    AVFilterContext *ctx = inlink->dst;
    AVFilterLink *outlink = ctx->outputs[0];
    EpilepsyContext *s = ctx->priv;
    AVFrame *out = ff_get_video_buffer(outlink, in->width, in->height);
    if (!out) {
        av_frame_free(&in);
        return AVERROR(ENOMEM);
    }

    /* Pop last frame from history */
    s->current_badness -= s->history[s->history_pos];

    convert_frame(in, &ef);
    this_badness = get_badness(&ef, &s->last_frame_e);
    new_badness = s->current_badness + this_badness;
    av_log(s, AV_LOG_INFO, "badness: %6d/%6d (%s)                                                     \n",
        new_badness, s->badness_threshold, new_badness < s->badness_threshold ? "OK" : "EXCEEDED");

    if (new_badness < s->badness_threshold || !s->last_frame_av) {
        if (s->last_frame_av) {
            av_frame_free(&s->last_frame_av);
        }
        s->last_frame_av = src = in;
        s->last_frame_e = ef;
        s->history[s->history_pos] = this_badness;
        s->current_badness = new_badness;
    } else {
        if (s->blend_factor == 0) {
            s->history[s->history_pos] = 0; /* frame was duplicated, thus, delta is zero */
        } else {
            if (!av_frame_is_writable(s->last_frame_av)) {
                src = av_frame_clone(out);
                if (!src) {
                    av_frame_free(&in);
                    av_frame_free(&out);
                    return AVERROR(ENOMEM);
                }
                av_frame_copy_props(src, s->last_frame_av);
                av_frame_copy(src, s->last_frame_av);
                av_frame_free(&s->last_frame_av);
                s->last_frame_av = src;
            }
            blend_frame(s->last_frame_av, in, s->blend_factor);

            convert_frame(s->last_frame_av, &ef);
            this_badness = get_badness(&ef, &s->last_frame_e);
            new_badness = s->current_badness + this_badness;
            av_log(s, AV_LOG_INFO, "  fixed: %6d/%6d (%s)                                                     \n",
                new_badness, s->badness_threshold, new_badness < s->badness_threshold ? "FIXED" : "STILL EXCEEDED");
            s->last_frame_e = ef;
            s->history[s->history_pos] = this_badness;
            s->current_badness = new_badness;
        }
        src = s->last_frame_av;
    }
    s->history_pos = (s->history_pos + 1) % s->nb_frames;
    
    av_frame_copy_props(out, src);
    av_frame_copy(out, src);
    return ff_filter_frame(outlink, out);
}

/* static int request_frame(AVFilterLink *outlink) */
/* { */
/*     AVFilterContext *ctx = outlink->src; */
/*     EpilepsyContext *s = ctx->priv; */
/*     int ret; */

/*     ret = ff_request_frame(ctx->inputs[0]); */
/*     if (ret == AVERROR_EOF && s->available > 0) { */
/*         AVFrame *buf = av_frame_clone(ff_bufqueue_peek(&s->q, s->size - 1)); */
/*         if (!buf) */
/*             return AVERROR(ENOMEM); */

/*         s->eof = 1; */
/*         ret = filter_frame(ctx->inputs[0], buf); */
/*         s->available--; */
/*     } */

/*     return ret; */
/* } */

static av_cold void uninit(AVFilterContext *ctx)
{
    EpilepsyContext *s = ctx->priv;

    if (s->last_frame_av) {
        av_frame_free(&s->last_frame_av);
    }
}

static const AVFilterPad inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_VIDEO,
        .filter_frame = filter_frame,
        .config_props = config_input,
    },
    { NULL }
};

static const AVFilterPad outputs[] = {
    {
        .name          = "default",
        .type          = AVMEDIA_TYPE_VIDEO,
    },
    { NULL }
};

AVFilter ff_vf_epilepsy = {
    .name          = "epilepsy",
    .description   = NULL_IF_CONFIG_SMALL("Attempt to filter out epilepsy-inducing flashes."),
    .priv_size     = sizeof(EpilepsyContext),
    .priv_class    = &epilepsy_class,
    .uninit        = uninit,
    .query_formats = query_formats,
    .inputs        = inputs,
    .outputs       = outputs,
};
