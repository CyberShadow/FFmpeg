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
#define NUM_CHANNELS 3

typedef struct PhotosensitivityContext {
    const AVClass *class;

    int nb_frames;
    int skip;
    float threshold_multiplier;
    int bypass;

    int badness_threshold;

    /* Circular buffer */
    uint32_t *history[MAX_FRAMES];
    int history_pos;

    AVFrame *last_frame;
} PhotosensitivityContext;

#define OFFSET(x) offsetof(PhotosensitivityContext, x)
#define FLAGS AV_OPT_FLAG_VIDEO_PARAM|AV_OPT_FLAG_FILTERING_PARAM

static const AVOption photosensitivity_options[] = {
    { "frames",    "set how many frames to use"                        ,  OFFSET(nb_frames           ), AV_OPT_TYPE_INT  , {.i64=60}, 2, MAX_FRAMES, FLAGS },
    { "f",         "set how many frames to use"                        ,  OFFSET(nb_frames           ), AV_OPT_TYPE_INT  , {.i64=60}, 2, MAX_FRAMES, FLAGS },
    { "threshold", "set detection threshold factor (lower is stricter)",  OFFSET(threshold_multiplier), AV_OPT_TYPE_FLOAT, {.dbl= 1}, 0, FLT_MAX   , FLAGS },
    { "t"        , "set detection threshold factor (lower is stricter)",  OFFSET(threshold_multiplier), AV_OPT_TYPE_FLOAT, {.dbl= 1}, 0, FLT_MAX   , FLAGS },
    { "skip"     , "set pixels to skip when sampling frames"           ,  OFFSET(skip                ), AV_OPT_TYPE_INT  , {.i64= 1}, 1, 1024      , FLAGS },
    { "bypass"   , "leave frames unchanged"                            ,  OFFSET(bypass              ), AV_OPT_TYPE_BOOL , {.i64= 0}, 0, 1         , FLAGS },
    { NULL }
};

AVFILTER_DEFINE_CLASS(photosensitivity);

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

typedef struct ThreadData_process_frame
{
    PhotosensitivityContext *s;
    AVFrame *in, *out;
} ThreadData_process_frame;

static int process_frame_partial(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    int x, y, p, c, i, this_badness, current_badness, new_badness;
    float factor;
    uint32_t *history;
    uint8_t *p_in, *p_out, *p_last;

    PhotosensitivityContext *s = ctx->priv;
    ThreadData_process_frame *td = arg;
    const int slice_start = (td->out->height * jobnr) / nb_jobs;
    const int slice_end = (td->out->height * (jobnr+1)) / nb_jobs;
    const int width = td->out->width;

    p = slice_start * td->in->width;
    for (y = slice_start; y < slice_end; y++) {
        p_in   = &td->in       ->data[0][y * td->in       ->linesize[0]];
        p_out  = &td->out      ->data[0][y * td->out      ->linesize[0]];
        p_last = &s->last_frame->data[0][y * s->last_frame->linesize[0]];

        for (x = 0; x < width; x++) {
            this_badness = 0;
            for (c = 0; c < NUM_CHANNELS; c++)
                this_badness += abs((int)p_last[c] - (int)p_in[c]);

            current_badness = 0;
            for (i = 1; i < s->nb_frames; i++) {
                history = s->history[(s->history_pos + i) % s->nb_frames];
                if (history)
                    current_badness += i * history[p];
            }
            current_badness /= s->nb_frames;
            new_badness = current_badness + this_badness;

#ifndef VISUALIZE
            if (new_badness < s->badness_threshold || !s->last_frame || s->bypass) {
                for (c = 0; c < NUM_CHANNELS; c++)
                    p_out[c] = p_in[c];
            } else {
                factor = (float)(s->badness_threshold - current_badness) / (new_badness - current_badness);
                if (factor <= 0) {
                    /* just duplicate the frame */
                    for (c = 0; c < NUM_CHANNELS; c++)
                        p_out[c] = p_last[c];
                    this_badness = 0; /* frame was duplicated, thus, delta is zero */
                } else {
                    for (c = 0; c < NUM_CHANNELS; c++)
                        p_out[c] = (uint8_t)(p_in[c] * factor + p_last[c] * (1-factor));

                    this_badness = 0;
                    for (c = 0; c < NUM_CHANNELS; c++)
                        this_badness += abs((int)p_last[c] - (int)p_out[c]);
                    /* new_badness = current_badness + this_badness; */
                }
            }
#else
            p_out[0] = current_badness > s->badness_threshold ? 255 : 0;
            p_out[1] = p_out[2] = FFMIN(255, 255 * current_badness / (s->badness_threshold *  4));
#endif
            s->history[s->history_pos][p] = this_badness;

            p++;
            p_in   += NUM_CHANNELS;
            p_out  += NUM_CHANNELS;
            p_last += NUM_CHANNELS;
        }
    }
    return 0;
}

static int config_input(AVFilterLink *inlink)
{
    AVFilterContext *ctx = inlink->dst;
    PhotosensitivityContext *s = ctx->priv;

    s->badness_threshold = (int)(4 * 256 * s->nb_frames * s->threshold_multiplier / 32);

    return 0;
}

static int filter_frame(AVFilterLink *inlink, AVFrame *in)
{
    AVFrame *out;
    ThreadData_process_frame td;

    AVFilterContext *ctx = inlink->dst;
    AVFilterLink *outlink = ctx->outputs[0];
    PhotosensitivityContext *s = ctx->priv;

    if (!s->history[s->history_pos]) {
        s->history[s->history_pos] = malloc(in->width * in->height * sizeof(s->history[0][0]));
        if (!s->history[s->history_pos]) {
            av_frame_free(&in);
            return AVERROR(ENOMEM);
        }
    }

    out = ff_get_video_buffer(outlink, in->width, in->height);
    if (!out) {
        av_frame_free(&in);
        return AVERROR(ENOMEM);
    }
    av_frame_copy_props(out, in);

    if (!s->last_frame)
        s->last_frame = av_frame_clone(in);

    td.in = in;
    td.out = out;
    ctx->internal->execute(ctx, process_frame_partial, &td, NULL,
        FFMIN(ctx->outputs[0]->h, ff_filter_get_nb_threads(ctx)));

    av_frame_unref(s->last_frame);
#ifndef VISUALIZE
    av_frame_ref(s->last_frame, out);
#else
    av_frame_ref(s->last_frame, in);
#endif
    s->history_pos = (s->history_pos + 1) % s->nb_frames;

    return ff_filter_frame(outlink, out);
}

static av_cold void uninit(AVFilterContext *ctx)
{
    PhotosensitivityContext *s = ctx->priv;

    if (s->last_frame) {
        av_frame_free(&s->last_frame);
    }
    /* TODO free history */
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

AVFilter ff_vf_photosensitivity = {
    .name          = "photosensitivity",
    .description   = NULL_IF_CONFIG_SMALL("Attempt to filter out photosensitive epilepsy seizure-inducing flashes."),
    .priv_size     = sizeof(PhotosensitivityContext),
    .priv_class    = &photosensitivity_class,
    .uninit        = uninit,
    .query_formats = query_formats,
    .inputs        = inputs,
    .outputs       = outputs,
};
