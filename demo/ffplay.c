#if 1
/*
 * Copyright (c) 2003 Fabrice Bellard
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

/**
 * @file
 * simple media player based on the FFmpeg libraries
 */

#include "config.h"
#include "config_components.h"
#include <math.h>
#include <limits.h>
#include <signal.h>
#include <stdint.h>

#include "libavutil/avstring.h"
#include "libavutil/channel_layout.h"
#include "libavutil/mathematics.h"
#include "libavutil/mem.h"
#include "libavutil/pixdesc.h"
#include "libavutil/dict.h"
#include "libavutil/fifo.h"
#include "libavutil/samplefmt.h"
#include "libavutil/time.h"
#include "libavutil/bprint.h"
#include "libavformat/avformat.h"
#include "libavdevice/avdevice.h"
#include "libswscale/swscale.h"
#include "libavutil/opt.h"
#include "libavutil/tx.h"
#include "libswresample/swresample.h"

#include "libavfilter/avfilter.h"
#include "libavfilter/buffersink.h"
#include "libavfilter/buffersrc.h"
#include "libavutil/avassert.h"     // for av_assert0

#include <SDL.h>
#include <SDL_thread.h>

#include "cmdutils.h"
#include "ffplay_renderer.h"
#include "opt_common.h"

//#define SKC_VIDEO_ONLY

const char program_name[] = "ffplay";
const int program_birth_year = 2003;

#define MAX_QUEUE_SIZE (15 * 1024 * 1024)
#define MIN_FRAMES 25
#define EXTERNAL_CLOCK_MIN_FRAMES 2
#define EXTERNAL_CLOCK_MAX_FRAMES 10

/* Minimum SDL audio buffer size, in samples. */
#define SDL_AUDIO_MIN_BUFFER_SIZE 512
/* Calculate actual buffer size keeping in mind not cause too frequent audio callbacks */
#define SDL_AUDIO_MAX_CALLBACKS_PER_SEC 30

/* Step size for volume control in dB */
#define SDL_VOLUME_STEP (0.75)

/* no AV sync correction is done if below the minimum AV sync threshold */
#define AV_SYNC_THRESHOLD_MIN 0.04
/* AV sync correction is done if above the maximum AV sync threshold */
#define AV_SYNC_THRESHOLD_MAX 0.1
/* If a frame duration is longer than this, it will not be duplicated to compensate AV sync */
#define AV_SYNC_FRAMEDUP_THRESHOLD 0.1
/* no AV correction is done if too big error */
#define AV_NOSYNC_THRESHOLD 10.0

/* maximum audio speed change to get correct sync */
#define SAMPLE_CORRECTION_PERCENT_MAX 10

/* external clock speed adjustment constants for realtime sources based on buffer fullness */
#define EXTERNAL_CLOCK_SPEED_MIN  0.900
#define EXTERNAL_CLOCK_SPEED_MAX  1.010
#define EXTERNAL_CLOCK_SPEED_STEP 0.001

/* we use about AUDIO_DIFF_AVG_NB A-V differences to make the average */
#define AUDIO_DIFF_AVG_NB   20

/* polls for possible required screen refresh at least this often, should be less than 1/fps */
#define REFRESH_RATE 0.01

/* NOTE: the size must be big enough to compensate the hardware audio buffersize size */
/* TODO: We assume that a decoded and resampled frame fits into this buffer */
#define SAMPLE_ARRAY_SIZE (8 * 65536) // 524,288 = 512 *1024 = 512KB

#define CURSOR_HIDE_DELAY 1000000

#define USE_ONEPASS_SUBTITLE_RENDER 1

typedef struct MyAVPacketList {
    AVPacket *pkt;
    int serial;
} MyAVPacketList;

// serial: ť�� ��� �ִ� ��Ŷ���� ���� ������ ����(�Ǵ� ��� �帧)�� ���ϴ����� �����ϱ� ���� ��ȣ
// PacketQueue, MyAVPacketList, Clock, Frame
// packet_queue_start(), packet_queue_flush() ���� serial ����
typedef struct PacketQueue {
    AVFifo *pkt_list;
    int nb_packets;
    int size;
    int64_t duration;
    int abort_request;
    int serial;
    SDL_mutex *mutex;
    SDL_cond *cond;
} PacketQueue;

// VIDEO_PICTURE_QUEUE_SIZE �� SAMPLE_QUEUE_SIZE �� 1�� ���� �����ÿ��� �ش� ��Ʈ���� ���� ����
#define VIDEO_PICTURE_QUEUE_SIZE 3
#define VIDEO_PICTURE_QUEUE_SIZE 2
#define SUBPICTURE_QUEUE_SIZE 16
#define SAMPLE_QUEUE_SIZE 9
#define FRAME_QUEUE_SIZE FFMAX(SAMPLE_QUEUE_SIZE, FFMAX(VIDEO_PICTURE_QUEUE_SIZE, SUBPICTURE_QUEUE_SIZE))

typedef struct AudioParams {
    int freq;
    AVChannelLayout ch_layout;
    enum AVSampleFormat fmt;
    int frame_size;
    int bytes_per_sec;
} AudioParams;

typedef struct Clock {
    double pts;           /* clock base */
    double pts_drift;     /* clock base minus time at which we updated the clock */
    double last_updated;
    double speed;
    int serial;           /* clock is based on a packet with this serial */
    int paused;
    int *queue_serial;    /* pointer to the current packet queue serial, used for obsolete clock detection */
} Clock;

typedef struct FrameData {
    int64_t pkt_pos;
} FrameData;

/* Common struct for handling all types of decoded data and allocated render buffers. */
typedef struct Frame {
    AVFrame *frame;
    AVSubtitle sub;
    int serial;
    double pts;           /* presentation timestamp for the frame */
    double duration;      /* estimated duration of the frame */
    int64_t pos;          /* byte position of the frame in the input file */
    int width;
    int height;
    int format;
    AVRational sar;
    int uploaded;
    int flip_v;
} Frame;

typedef struct FrameQueue {
    Frame queue[FRAME_QUEUE_SIZE];
    int rindex;
    int windex;
    int size;
    int max_size;
    int keep_last;
    int rindex_shown;   // ���� rindex �������� ȭ�鿡 ��������� ����(0 or 1)
    SDL_mutex *mutex;
    SDL_cond *cond;
    PacketQueue *pktq;
} FrameQueue;

enum {
    AV_SYNC_AUDIO_MASTER, /* default choice */
    AV_SYNC_VIDEO_MASTER,
    AV_SYNC_EXTERNAL_CLOCK, /* synchronize to an external clock */
};

typedef struct Decoder {
    AVPacket *pkt;
    PacketQueue *queue;
    AVCodecContext *avctx;
    int pkt_serial;
    int finished;
    int packet_pending;
    SDL_cond *empty_queue_cond;
    int64_t start_pts;
    AVRational start_pts_tb;
    int64_t next_pts;
    AVRational next_pts_tb;
    SDL_Thread *decoder_tid;
} Decoder;

typedef struct VideoState {
    SDL_Thread *read_tid;
    const AVInputFormat *iformat;
    int abort_request;
    int force_refresh;
    int paused;
    int last_paused;
    int queue_attachments_req;
    int seek_req;
    int seek_flags;
    int64_t seek_pos;
    int64_t seek_rel;
    int read_pause_return;
    AVFormatContext *ic;
    int realtime;

    Clock audclk;
    Clock vidclk;
    Clock extclk;

    FrameQueue pictq;
    FrameQueue subpq;
    FrameQueue sampq;

    Decoder auddec;
    Decoder viddec;
    Decoder subdec;

    int audio_stream;

    int av_sync_type;

    double audio_clock;
    int audio_clock_serial;
    double audio_diff_cum; /* used for AV difference average computation */
    double audio_diff_avg_coef;
    double audio_diff_threshold;
    int audio_diff_avg_count;
    AVStream *audio_st;
    PacketQueue audioq;
    int audio_hw_buf_size;
    uint8_t *audio_buf;
    uint8_t *audio_buf1;
    unsigned int audio_buf_size; /* in bytes */
    unsigned int audio_buf1_size;
    int audio_buf_index; /* in bytes */
    int audio_write_buf_size;
    int audio_volume;
    int muted;
    struct AudioParams audio_src;
    struct AudioParams audio_filter_src;
    struct AudioParams audio_tgt;
    struct SwrContext *swr_ctx;
    int frame_drops_early;
    int frame_drops_late;

    enum ShowMode {
        SHOW_MODE_NONE = -1, SHOW_MODE_VIDEO = 0, SHOW_MODE_WAVES, SHOW_MODE_RDFT, SHOW_MODE_NB
    } show_mode;
    int16_t sample_array[SAMPLE_ARRAY_SIZE];
    int sample_array_index;
    int last_i_start;
    AVTXContext *rdft;
    av_tx_fn rdft_fn;
    int rdft_bits;
    float *real_data;
    AVComplexFloat *rdft_data;
    int xpos;
    double last_vis_time;
    SDL_Texture *vis_texture;
    SDL_Texture *sub_texture;
    SDL_Texture *vid_texture;

    int subtitle_stream;
    AVStream *subtitle_st;
    PacketQueue subtitleq;

    double frame_timer;
    double frame_last_returned_time;
    double frame_last_filter_delay;
    int video_stream;
    AVStream *video_st;
    PacketQueue videoq;
    double max_frame_duration;      // maximum duration of a frame - above this, we consider the jump a timestamp discontinuity
    struct SwsContext *sub_convert_ctx;
    int eof;

    char *filename;
    int width, height, xleft, ytop;
    int step;

    int vfilter_idx;
    AVFilterContext *in_video_filter;   // the first filter in the video chain
    AVFilterContext *out_video_filter;  // the last filter in the video chain
    AVFilterContext *in_audio_filter;   // the first filter in the audio chain
    AVFilterContext *out_audio_filter;  // the last filter in the audio chain
    AVFilterGraph *agraph;              // audio filter graph

    int last_video_stream, last_audio_stream, last_subtitle_stream;

    SDL_cond *continue_read_thread;
} VideoState;

/* options specified by the user */
static const AVInputFormat *file_iformat;
static const char *input_filename;
static const char *window_title;
static int default_width  = 640;
static int default_height = 480;
static int screen_width  = 0;
static int screen_height = 0;
static int screen_left = SDL_WINDOWPOS_CENTERED;
static int screen_top = SDL_WINDOWPOS_CENTERED;
static int audio_disable;
static int video_disable;
static int subtitle_disable;
static const char* wanted_stream_spec[AVMEDIA_TYPE_NB] = {0};
static int seek_by_bytes = -1;
static float seek_interval = 10;
static int display_disable;
static int borderless;
static int alwaysontop;
static int startup_volume = 100;
static int show_status = -1;
static int av_sync_type = AV_SYNC_AUDIO_MASTER;
static int64_t start_time = AV_NOPTS_VALUE;
static int64_t duration = AV_NOPTS_VALUE;
static int fast = 0;
static int genpts = 0;
static int lowres = 0;
static int decoder_reorder_pts = -1;
static int autoexit;
//static int autoexit = 1; // -autoexit ����� ���ھ��� ������ ������ ���
static int exit_on_keydown;
static int exit_on_mousedown;
static int loop = 1;
static int framedrop = -1;
static int infinite_buffer = -1;
static enum ShowMode show_mode = SHOW_MODE_NONE;
static const char *audio_codec_name;
static const char *subtitle_codec_name;
static const char *video_codec_name;
double rdftspeed = 0.02;
static int64_t cursor_last_shown;
static int cursor_hidden = 0;
static const char **vfilters_list = NULL;
static int nb_vfilters = 0;
static char *afilters = NULL;
static int autorotate = 1;
static int find_stream_info = 1;
static int filter_nbthreads = 0;
static int enable_vulkan = 0;
static char *vulkan_params = NULL;
static const char *hwaccel = NULL;

/* current context */
static int is_full_screen;
static int64_t audio_callback_time;

#define FF_QUIT_EVENT    (SDL_USEREVENT + 2)

static SDL_Window *window;
static SDL_Renderer *renderer;
static SDL_RendererInfo renderer_info = {0};
#ifndef SKC_VIDEO_ONLY
static SDL_AudioDeviceID audio_dev;
#endif

static VkRenderer *vk_renderer;

static const struct TextureFormatEntry {
    enum AVPixelFormat format;
    int texture_fmt;
} sdl_texture_format_map[] = {
    { AV_PIX_FMT_RGB8,           SDL_PIXELFORMAT_RGB332 },
    { AV_PIX_FMT_RGB444,         SDL_PIXELFORMAT_RGB444 },
    { AV_PIX_FMT_RGB555,         SDL_PIXELFORMAT_RGB555 },
    { AV_PIX_FMT_BGR555,         SDL_PIXELFORMAT_BGR555 },
    { AV_PIX_FMT_RGB565,         SDL_PIXELFORMAT_RGB565 },
    { AV_PIX_FMT_BGR565,         SDL_PIXELFORMAT_BGR565 },
    { AV_PIX_FMT_RGB24,          SDL_PIXELFORMAT_RGB24 },
    { AV_PIX_FMT_BGR24,          SDL_PIXELFORMAT_BGR24 },
    { AV_PIX_FMT_0RGB32,         SDL_PIXELFORMAT_RGB888 },
    { AV_PIX_FMT_0BGR32,         SDL_PIXELFORMAT_BGR888 },
    { AV_PIX_FMT_NE(RGB0, 0BGR), SDL_PIXELFORMAT_RGBX8888 },
    { AV_PIX_FMT_NE(BGR0, 0RGB), SDL_PIXELFORMAT_BGRX8888 },
    { AV_PIX_FMT_RGB32,          SDL_PIXELFORMAT_ARGB8888 },
    { AV_PIX_FMT_RGB32_1,        SDL_PIXELFORMAT_RGBA8888 },
    { AV_PIX_FMT_BGR32,          SDL_PIXELFORMAT_ABGR8888 },
    { AV_PIX_FMT_BGR32_1,        SDL_PIXELFORMAT_BGRA8888 },
    { AV_PIX_FMT_YUV420P,        SDL_PIXELFORMAT_IYUV },
    { AV_PIX_FMT_YUYV422,        SDL_PIXELFORMAT_YUY2 },
    { AV_PIX_FMT_UYVY422,        SDL_PIXELFORMAT_UYVY },
    { AV_PIX_FMT_NONE,           SDL_PIXELFORMAT_UNKNOWN },
};

#ifndef SKC_VIDEO_ONLY
static int opt_add_vfilter(void *optctx, const char *opt, const char *arg)
{
    int ret = GROW_ARRAY(vfilters_list, nb_vfilters);
    if (ret < 0)
        return ret;

    vfilters_list[nb_vfilters - 1] = av_strdup(arg);
    if (!vfilters_list[nb_vfilters - 1])
        return AVERROR(ENOMEM);

    return 0;
}

static inline
int cmp_audio_fmts(enum AVSampleFormat fmt1, int64_t channel_count1,
                   enum AVSampleFormat fmt2, int64_t channel_count2)
{
    /* If channel count == 1, planar and non-planar formats are the same */
    if (channel_count1 == 1 && channel_count2 == 1)
        return av_get_packed_sample_fmt(fmt1) != av_get_packed_sample_fmt(fmt2);
    else
        return channel_count1 != channel_count2 || fmt1 != fmt2;
}
#endif

static int packet_queue_put_private(PacketQueue *q, AVPacket *pkt)
{
    MyAVPacketList pkt1;
    int ret;

    if (q->abort_request)
       return -1;


    pkt1.pkt = pkt;
    pkt1.serial = q->serial;

    ret = av_fifo_write(q->pkt_list, &pkt1, 1);
    if (ret < 0)
        return ret;
    q->nb_packets++;
    q->size += pkt1.pkt->size + sizeof(pkt1);
    q->duration += pkt1.pkt->duration;
    /* XXX: should duplicate packet data in DV case */
    SDL_CondSignal(q->cond);
    return 0;
}

static int packet_queue_put(PacketQueue *q, AVPacket *pkt)
{
    AVPacket *pkt1;
    int ret;

    pkt1 = av_packet_alloc();
    if (!pkt1) {
        av_packet_unref(pkt);
        return -1;
    }
    av_packet_move_ref(pkt1, pkt); // �������� pkt �� �����

    SDL_LockMutex(q->mutex);
    ret = packet_queue_put_private(q, pkt1);
    SDL_UnlockMutex(q->mutex);

    if (ret < 0)
        av_packet_free(&pkt1);

    return ret;
}

static int packet_queue_put_nullpacket(PacketQueue *q, AVPacket *pkt, int stream_index)
{
    pkt->stream_index = stream_index;
    return packet_queue_put(q, pkt);
}

/* packet queue handling */
static int packet_queue_init(PacketQueue *q)
{
    memset(q, 0, sizeof(PacketQueue));
    q->pkt_list = av_fifo_alloc2(1, sizeof(MyAVPacketList), AV_FIFO_FLAG_AUTO_GROW);
    if (!q->pkt_list)
        return AVERROR(ENOMEM);
    q->mutex = SDL_CreateMutex();
    if (!q->mutex) {
        av_log(NULL, AV_LOG_FATAL, "SDL_CreateMutex(): %s\n", SDL_GetError());
        return AVERROR(ENOMEM);
    }
    q->cond = SDL_CreateCond();
    if (!q->cond) {
        av_log(NULL, AV_LOG_FATAL, "SDL_CreateCond(): %s\n", SDL_GetError());
        return AVERROR(ENOMEM);
    }
    q->abort_request = 1;
    return 0;
}

static void packet_queue_flush(PacketQueue *q)
{
    MyAVPacketList pkt1;

    SDL_LockMutex(q->mutex);
    while (av_fifo_read(q->pkt_list, &pkt1, 1) >= 0)
        av_packet_free(&pkt1.pkt);
    q->nb_packets = 0;
    q->size = 0;
    q->duration = 0;
    q->serial++;
    SDL_UnlockMutex(q->mutex);
}

static void packet_queue_destroy(PacketQueue *q)
{
    packet_queue_flush(q);
    av_fifo_freep2(&q->pkt_list);
    SDL_DestroyMutex(q->mutex);
    SDL_DestroyCond(q->cond);
}

static void packet_queue_abort(PacketQueue *q)
{
    SDL_LockMutex(q->mutex);

    q->abort_request = 1;

    SDL_CondSignal(q->cond);

    SDL_UnlockMutex(q->mutex);
}

static void packet_queue_start(PacketQueue *q)
{
    SDL_LockMutex(q->mutex);
    q->abort_request = 0;
    q->serial++;
    SDL_UnlockMutex(q->mutex);
}

/* return < 0 if aborted, 0 if no packet and > 0 if packet.  */
static int packet_queue_get(PacketQueue *q, AVPacket *pkt/*out*/, int block, int* serial/*out*/)
{
    MyAVPacketList pkt1;
    int ret;

    SDL_LockMutex(q->mutex);

    for (;;) {
        if (q->abort_request) {
            ret = -1;
            break;
        }

        if (av_fifo_read(q->pkt_list, &pkt1, 1) >= 0) {
            q->nb_packets--;
            q->size -= pkt1.pkt->size + sizeof(pkt1);
            q->duration -= pkt1.pkt->duration;
            av_packet_move_ref(pkt, pkt1.pkt);
            if (serial)
                *serial = pkt1.serial;
            av_packet_free(&pkt1.pkt);
            ret = 1;
            break;
        } else if (!block) {
            ret = 0;
            break;
        } else {
            SDL_CondWait(q->cond, q->mutex);
        }
    }
    SDL_UnlockMutex(q->mutex);
    return ret;
}

static int decoder_init(Decoder *d, AVCodecContext *avctx, PacketQueue *queue, SDL_cond *empty_queue_cond) {
    memset(d, 0, sizeof(Decoder));
    d->pkt = av_packet_alloc();
    if (!d->pkt)
        return AVERROR(ENOMEM);
    d->avctx = avctx;
    d->queue = queue;
    d->empty_queue_cond = empty_queue_cond;
    d->start_pts = AV_NOPTS_VALUE;
    d->pkt_serial = -1;
    return 0;
}

static int decoder_decode_frame(Decoder *d, AVFrame *frame, AVSubtitle *sub) {
    int ret = AVERROR(EAGAIN);

    for (;;) {
        if (d->queue->serial == d->pkt_serial) { // ���� 1, -1, seeking �� �޶���
            do {
                if (d->queue->abort_request)
                    return -1;

                switch (d->avctx->codec_type) {
                    case AVMEDIA_TYPE_VIDEO:
                        // avcodec_send_packet() ���� ���� avcodec_receive_frame �� ȣ��ž� ��
                        // (���ڴ� ���ο� ���� ��µ��� ���� �������� ���� ���� ���ɼ��� ����)
                        ret = avcodec_receive_frame(d->avctx, frame); // �ͺ�ŷ �Լ���. success: 0
                        if (ret >= 0) {
                            av_assert0(frame->sample_rate == 0 && frame->nb_samples == 0);
                            // 0 = off(���ڵ� ����(DTS ����)��� ���), 1 = on, -1 = auto
                            if (decoder_reorder_pts == -1) { // default
                                av_assert0(frame->pts == frame->best_effort_timestamp);

                                // ������ PTS�� ���� ����ϱ� ����� ��찡 ���� frame->best_effort_timestamp�� ���
                                // best_effort_timestamp �� B-�����Ӱ� ���� ������ ������ Ÿ���� ����Ͽ� ���� ����
                                // frame->pts=0, 3600, 7200, ...
                                frame->pts = frame->best_effort_timestamp;
                            } else if (!decoder_reorder_pts) {
                                // off(0) �̸� ������ �� �ϴϱ� pkt_dts(Decoding Time Stamp) ���� ���
                                // on(1) �̸� ������ ����ϴ� frame->pts �� �״�� ���(�̹� ���ڴ��� ��������Ų PTS ��)
                                av_assert0(0);
                                frame->pts = frame->pkt_dts;
                            }
                        }
                        break;
                    case AVMEDIA_TYPE_AUDIO:
                        ret = avcodec_receive_frame(d->avctx, frame);
                        if (ret >= 0) {
                            av_assert0(frame->sample_rate == 48000 && frame->nb_samples == 1024 && frame->width == 0);
                            AVRational tb = (AVRational){1, frame->sample_rate}; // {1, 48000}
                            av_assert0(frame->pts != AV_NOPTS_VALUE);

                            // frame->pts = 1024, 2048, ..., 480256(���� ����), frame->nb_samples �� 1024 ����
                            // printf("decoder_decode_frame() frame->pts=%d, frame->nb_samples=%d\n", frame->pts, frame->nb_samples);
                            static int called = 0;
                            av_assert0(frame->pts == 1024 * ++called);

                            if (frame->pts != AV_NOPTS_VALUE) {
                                // d->avctx->pkt_timebase = {1, 48000}
                                // frame->sample_rate ��(tb) d->avctx->pkt_timebase �� �ٸ� ���� ���� ����
                                av_assert0((tb.num == d->avctx->pkt_timebase.num) && (tb.den == d->avctx->pkt_timebase.den));
    
                                // ���ڵ��� frame->pts �� tb �� ���� �״�� �����(pkt_timebase �� �����ϹǷ� frame->pts �� �״�� ������)
                                frame->pts = av_rescale_q(frame->pts, d->avctx->pkt_timebase, tb);
                            }
                            else if (d->next_pts != AV_NOPTS_VALUE) {
                                av_assert0(0);
                                frame->pts = av_rescale_q(d->next_pts, d->next_pts_tb, tb);
                            }
                            if (frame->pts != AV_NOPTS_VALUE) {
                                d->next_pts = frame->pts + frame->nb_samples;
                                d->next_pts_tb = tb;
                            }
                        }
                        break;
                }
                if (ret == AVERROR_EOF) {
                    d->finished = d->pkt_serial;
                    avcodec_flush_buffers(d->avctx);
                    return 0;
                }
                if (ret >= 0)
                    return 1;
            // ���ϰ� ret �� AVERROR(EAGAIN)(-11) �� ��쿡��(�߰� �Է� ��Ŷ�� �ʿ���) ���� Ż��
            // avcodec_send_packet() �� ȣ������� ��
            } while (ret != AVERROR(EAGAIN));
        }

        do {
            if (d->queue->nb_packets == 0)
                SDL_CondSignal(d->empty_queue_cond);
            if (d->packet_pending) {
                // �Ʒ����� avcodec_send_packet() ���ϰ��� EAGAIN ����(�Է¹��� ������)�� ��� ����������
                // �ٽ� send �� �õ��ؾ� �ϹǷ� packet_queue_get() ���� ��Ŷ�� �о���� �ʾƾ� �Ѵ�.
                d->packet_pending = 0;
            } else {
                // ���ʿ� d->queue->serial(1) != d->pkt_serial(-1) �� ����� �ٷ� ����
                // d->pkt_serial = 1 �� ������
                int old_serial = d->pkt_serial;
                if (packet_queue_get(d->queue, d->pkt, 1/*block*/, &d->pkt_serial) < 0) // ��ŷ ���!!!
                    return -1;
                if (old_serial != d->pkt_serial) { // ���ʿ� �ѹ� ����(-1, 1), seeking �� �޶���
                    avcodec_flush_buffers(d->avctx);
                    d->finished = 0;
                    d->next_pts = d->start_pts; // AV_NOPTS_VALUE
                    d->next_pts_tb = d->start_pts_tb; // 0/0
                }
            }
            if (d->queue->serial == d->pkt_serial)
                break;
            av_packet_unref(d->pkt);
        } while (1);

        if (d->avctx->codec_type == AVMEDIA_TYPE_SUBTITLE) {
            int got_frame = 0;
            // ���ڵ��� �ڸ��� sub �� ���ϵ�. ���� ����Ʈ���� ����(ret)
            ret = avcodec_decode_subtitle2(d->avctx, sub, &got_frame, d->pkt);
            if (ret < 0) {
                ret = AVERROR(EAGAIN);
            } else {
                if (got_frame && !d->pkt->data) { // d->pkt->data == NULL �̸� flushing ��û�� ������
                    d->packet_pending = 1;
                }
                // �������� ���޾� �԰�, flushing ��û�̾����� AVERROR_EOF
                ret = got_frame ? 0 : (d->pkt->data ? AVERROR(EAGAIN) : AVERROR_EOF);
            }
            av_packet_unref(d->pkt);
        } else {
            if (d->pkt->buf && !d->pkt->opaque_ref) {
                FrameData *fd;

                d->pkt->opaque_ref = av_buffer_allocz(sizeof(*fd)); // AVBuffer �� ������ AVBufferRef �� ����
                if (!d->pkt->opaque_ref)
                    return AVERROR(ENOMEM);
                fd = (FrameData*)d->pkt->opaque_ref->data;
                // a: 168, 497, 801, ..., v: 5147, 19967, 22825, ... �� ���߾���
                fd->pkt_pos = d->pkt->pos;
            }

            // avcodec_send_packet() ���� EAGAIN �� �ԷºҰ�(�Է¹��� ������) �����̸�, avcodec_receive_frame() �� �о���� �Ѵ�.
            // avcodec_receive_frame() �� EAGAIN �� ���� ���ο� ���������� ���⼭ avcodec_send_packet() �� EAGAIN �����̸� API violation
            // avcodec_send_packet(), avcodec_receive_frame() �� �����ϰ� ���⼭�� ȣ������
            // send �� ������ �ؾ� ���� receive �� �����ϰ� ����
            if (avcodec_send_packet(d->avctx, d->pkt) == AVERROR(EAGAIN)) {
                av_log(d->avctx, AV_LOG_ERROR, "Receive_frame and send_packet both returned EAGAIN, which is an API violation.\n");
                d->packet_pending = 1;
            } else {
                av_packet_unref(d->pkt);
            }
        }
    }
}

static void decoder_destroy(Decoder *d) {
    av_packet_free(&d->pkt);
    avcodec_free_context(&d->avctx);
}

static void frame_queue_unref_item(Frame *vp)
{
    av_frame_unref(vp->frame);
    avsubtitle_free(&vp->sub);
}

static int frame_queue_init(FrameQueue *f, PacketQueue *pktq, int max_size, int keep_last)
{
    int i;
    memset(f, 0, sizeof(FrameQueue));
    if (!(f->mutex = SDL_CreateMutex())) {
        av_log(NULL, AV_LOG_FATAL, "SDL_CreateMutex(): %s\n", SDL_GetError());
        return AVERROR(ENOMEM);
    }
    if (!(f->cond = SDL_CreateCond())) {
        av_log(NULL, AV_LOG_FATAL, "SDL_CreateCond(): %s\n", SDL_GetError());
        return AVERROR(ENOMEM);
    }
    f->pktq = pktq;
    f->max_size = FFMIN(max_size, FRAME_QUEUE_SIZE);
    f->keep_last = !!keep_last;
    for (i = 0; i < f->max_size; i++)
        if (!(f->queue[i].frame = av_frame_alloc()))
            return AVERROR(ENOMEM);
    return 0;
}

static void frame_queue_destroy(FrameQueue *f)
{
    int i;
    for (i = 0; i < f->max_size; i++) {
        Frame *vp = &f->queue[i];
        frame_queue_unref_item(vp);
        av_frame_free(&vp->frame);
    }
    SDL_DestroyMutex(f->mutex);
    SDL_DestroyCond(f->cond);
}

static void frame_queue_signal(FrameQueue *f)
{
    SDL_LockMutex(f->mutex);
    SDL_CondSignal(f->cond);
    SDL_UnlockMutex(f->mutex);
}

static Frame *frame_queue_peek(FrameQueue *f)
{
    return &f->queue[(f->rindex + f->rindex_shown) % f->max_size];
}

static Frame *frame_queue_peek_next(FrameQueue *f)
{
    return &f->queue[(f->rindex + f->rindex_shown + 1) % f->max_size];
}

static Frame *frame_queue_peek_last(FrameQueue *f)
{
    return &f->queue[f->rindex];
}

static Frame *frame_queue_peek_writable(FrameQueue *f)
{
    /* wait until we have space to put a new frame */
    SDL_LockMutex(f->mutex);
    while (f->size >= f->max_size &&
           !f->pktq->abort_request) {
        SDL_CondWait(f->cond, f->mutex);
    }
    SDL_UnlockMutex(f->mutex);

    if (f->pktq->abort_request)
        return NULL;

    return &f->queue[f->windex];
}

static Frame *frame_queue_peek_readable(FrameQueue *f)
{
    /* wait until we have a readable a new frame */
    SDL_LockMutex(f->mutex);
    while (f->size - f->rindex_shown <= 0 &&
           !f->pktq->abort_request) {
        SDL_CondWait(f->cond, f->mutex);
    }
    SDL_UnlockMutex(f->mutex);

    if (f->pktq->abort_request)
        return NULL;

    return &f->queue[(f->rindex + f->rindex_shown) % f->max_size];
}

// frame_queue_peek_writable() �� ������������ ȣ���
static void frame_queue_push(FrameQueue *f)
{
    if (++f->windex == f->max_size)
        f->windex = 0;
    SDL_LockMutex(f->mutex);
    f->size++; // ���⼭ �����ϰ� ����
    SDL_CondSignal(f->cond);
    SDL_UnlockMutex(f->mutex);
}

static void frame_queue_next(FrameQueue *f)
{
    if (f->keep_last && !f->rindex_shown) {
        // �����ϰ� ���⼭�� �����ǰ� ����(video, audio ��½� ȣ���)
        // (pictq, sampq �� keep_last = 1 �� rindex_shown �� 1 �̵�)
        // rindex: 0, rindex_shown: 0 -> (0,0), (0, 1), (1, 1), (2, 1), ...
        f->rindex_shown = 1;
        return;
    }
    frame_queue_unref_item(&f->queue[f->rindex]);
    if (++f->rindex == f->max_size) // ���⼭ �����ϰ� ����
        f->rindex = 0;
    SDL_LockMutex(f->mutex);
    f->size--; // ���⼭ �����ϰ� ����
    SDL_CondSignal(f->cond);
    SDL_UnlockMutex(f->mutex);
}

/* return the number of undisplayed frames in the queue */
static int frame_queue_nb_remaining(FrameQueue *f)
{
    // ������ť���� ȭ�鿡 ������� �������� ����.
    return f->size - f->rindex_shown;
}

/* return last shown position */
static int64_t frame_queue_last_pos(FrameQueue *f)
{
    Frame *fp = &f->queue[f->rindex];
    if (f->rindex_shown && fp->serial == f->pktq->serial)
        return fp->pos;
    else
        return -1;
}

static void decoder_abort(Decoder *d, FrameQueue *fq)
{
    packet_queue_abort(d->queue);
    frame_queue_signal(fq);
    SDL_WaitThread(d->decoder_tid, NULL);
    d->decoder_tid = NULL;
    packet_queue_flush(d->queue);
}

static inline void fill_rectangle(int x, int y, int w, int h)
{
    SDL_Rect rect;
    rect.x = x;
    rect.y = y;
    rect.w = w;
    rect.h = h;
    if (w && h)
        SDL_RenderFillRect(renderer, &rect);
}

static int realloc_texture(SDL_Texture **texture, Uint32 new_format, int new_width, int new_height, SDL_BlendMode blendmode, int init_texture)
{
    Uint32 format;
    int access, w, h;
    if (!*texture || SDL_QueryTexture(*texture, &format, &access, &w, &h) < 0 || new_width != w || new_height != h || new_format != format) {
        void *pixels;
        int pitch;
        if (*texture)
            SDL_DestroyTexture(*texture);
        if (!(*texture = SDL_CreateTexture(renderer, new_format, SDL_TEXTUREACCESS_STREAMING, new_width, new_height)))
            return -1;
        if (SDL_SetTextureBlendMode(*texture, blendmode) < 0)
            return -1;
        if (init_texture) {
            if (SDL_LockTexture(*texture, NULL, &pixels, &pitch) < 0)
                return -1;
            memset(pixels, 0, pitch * new_height);
            SDL_UnlockTexture(*texture);
        }
        av_log(NULL, AV_LOG_VERBOSE, "Created %dx%d texture with %s.\n", new_width, new_height, SDL_GetPixelFormatName(new_format));
    }
    return 0;
}

static void calculate_display_rect(SDL_Rect *rect,
                                   int scr_xleft, int scr_ytop, int scr_width, int scr_height,
                                   int pic_width, int pic_height, AVRational pic_sar)
{
    AVRational aspect_ratio = pic_sar;
    int64_t width, height, x, y;

    if (av_cmp_q(aspect_ratio, av_make_q(0, 1)) <= 0)
        aspect_ratio = av_make_q(1, 1);

    aspect_ratio = av_mul_q(aspect_ratio, av_make_q(pic_width, pic_height));

    /* XXX: we suppose the screen has a 1.0 pixel ratio */
    height = scr_height;
    width = av_rescale(height, aspect_ratio.num, aspect_ratio.den) & ~1;
    if (width > scr_width) {
        width = scr_width;
        height = av_rescale(width, aspect_ratio.den, aspect_ratio.num) & ~1;
    }
    x = (scr_width - width) / 2;
    y = (scr_height - height) / 2;
    rect->x = scr_xleft + x;
    rect->y = scr_ytop  + y;
    rect->w = FFMAX((int)width,  1);
    rect->h = FFMAX((int)height, 1);
}

static void get_sdl_pix_fmt_and_blendmode(int format, Uint32 *sdl_pix_fmt, SDL_BlendMode *sdl_blendmode)
{
    int i;
    *sdl_blendmode = SDL_BLENDMODE_NONE;
    *sdl_pix_fmt = SDL_PIXELFORMAT_UNKNOWN;
    if (format == AV_PIX_FMT_RGB32   ||
        format == AV_PIX_FMT_RGB32_1 ||
        format == AV_PIX_FMT_BGR32   ||
        format == AV_PIX_FMT_BGR32_1)
        *sdl_blendmode = SDL_BLENDMODE_BLEND;
    for (i = 0; i < FF_ARRAY_ELEMS(sdl_texture_format_map) - 1; i++) {
        if (format == sdl_texture_format_map[i].format) {
            *sdl_pix_fmt = sdl_texture_format_map[i].texture_fmt;
            return;
        }
    }
}

static int upload_texture(SDL_Texture **tex, AVFrame *frame)
{
    int ret = 0;
    Uint32 sdl_pix_fmt;
    SDL_BlendMode sdl_blendmode;
    get_sdl_pix_fmt_and_blendmode(frame->format, &sdl_pix_fmt, &sdl_blendmode);
    if (realloc_texture(tex, sdl_pix_fmt == SDL_PIXELFORMAT_UNKNOWN ? SDL_PIXELFORMAT_ARGB8888 : sdl_pix_fmt, frame->width, frame->height, sdl_blendmode, 0) < 0)
        return -1;
    switch (sdl_pix_fmt) {
        case SDL_PIXELFORMAT_IYUV:
            if (frame->linesize[0] > 0 && frame->linesize[1] > 0 && frame->linesize[2] > 0) {
                ret = SDL_UpdateYUVTexture(*tex, NULL, frame->data[0], frame->linesize[0],
                                                       frame->data[1], frame->linesize[1],
                                                       frame->data[2], frame->linesize[2]);
            } else if (frame->linesize[0] < 0 && frame->linesize[1] < 0 && frame->linesize[2] < 0) {
                ret = SDL_UpdateYUVTexture(*tex, NULL, frame->data[0] + frame->linesize[0] * (frame->height                    - 1), -frame->linesize[0],
                                                       frame->data[1] + frame->linesize[1] * (AV_CEIL_RSHIFT(frame->height, 1) - 1), -frame->linesize[1],
                                                       frame->data[2] + frame->linesize[2] * (AV_CEIL_RSHIFT(frame->height, 1) - 1), -frame->linesize[2]);
            } else {
                av_log(NULL, AV_LOG_ERROR, "Mixed negative and positive linesizes are not supported.\n");
                return -1;
            }
            break;
        default:
            if (frame->linesize[0] < 0) {
                ret = SDL_UpdateTexture(*tex, NULL, frame->data[0] + frame->linesize[0] * (frame->height - 1), -frame->linesize[0]);
            } else {
                ret = SDL_UpdateTexture(*tex, NULL, frame->data[0], frame->linesize[0]);
            }
            break;
    }
    return ret;
}

static enum AVColorSpace sdl_supported_color_spaces[] = {
    AVCOL_SPC_BT709,
    AVCOL_SPC_BT470BG,
    AVCOL_SPC_SMPTE170M,
    AVCOL_SPC_UNSPECIFIED,
};

static void set_sdl_yuv_conversion_mode(AVFrame *frame)
{
#if SDL_VERSION_ATLEAST(2,0,8)
    SDL_YUV_CONVERSION_MODE mode = SDL_YUV_CONVERSION_AUTOMATIC;
    if (frame && (frame->format == AV_PIX_FMT_YUV420P || frame->format == AV_PIX_FMT_YUYV422 || frame->format == AV_PIX_FMT_UYVY422)) {
        if (frame->color_range == AVCOL_RANGE_JPEG)
            mode = SDL_YUV_CONVERSION_JPEG;
        else if (frame->colorspace == AVCOL_SPC_BT709)
            mode = SDL_YUV_CONVERSION_BT709;
        else if (frame->colorspace == AVCOL_SPC_BT470BG || frame->colorspace == AVCOL_SPC_SMPTE170M)
            mode = SDL_YUV_CONVERSION_BT601;
    }
    SDL_SetYUVConversionMode(mode); /* FIXME: no support for linear transfer */
#endif
}

static void video_image_display(VideoState *is)
{
    Frame *vp;
    Frame *sp = NULL;
    SDL_Rect rect;

    vp = frame_queue_peek_last(&is->pictq); // return &f->queue[f->rindex]; �� �׸��� ����
    if (vk_renderer) {
        vk_renderer_display(vk_renderer, vp->frame);
        return;
    }

    if (is->subtitle_st) {
        if (frame_queue_nb_remaining(&is->subpq) > 0) {
            sp = frame_queue_peek(&is->subpq);

            if (vp->pts >= sp->pts + ((float) sp->sub.start_display_time / 1000)) {
                if (!sp->uploaded) {
                    uint8_t* pixels[4];
                    int pitch[4];
                    int i;
                    if (!sp->width || !sp->height) {
                        sp->width = vp->width;
                        sp->height = vp->height;
                    }
                    if (realloc_texture(&is->sub_texture, SDL_PIXELFORMAT_ARGB8888, sp->width, sp->height, SDL_BLENDMODE_BLEND, 1) < 0)
                        return;

                    for (i = 0; i < sp->sub.num_rects; i++) {
                        AVSubtitleRect *sub_rect = sp->sub.rects[i];

                        sub_rect->x = av_clip(sub_rect->x, 0, sp->width );
                        sub_rect->y = av_clip(sub_rect->y, 0, sp->height);
                        sub_rect->w = av_clip(sub_rect->w, 0, sp->width  - sub_rect->x);
                        sub_rect->h = av_clip(sub_rect->h, 0, sp->height - sub_rect->y);

                        is->sub_convert_ctx = sws_getCachedContext(is->sub_convert_ctx,
                            sub_rect->w, sub_rect->h, AV_PIX_FMT_PAL8,
                            sub_rect->w, sub_rect->h, AV_PIX_FMT_BGRA,
                            0, NULL, NULL, NULL);
                        if (!is->sub_convert_ctx) {
                            av_log(NULL, AV_LOG_FATAL, "Cannot initialize the conversion context\n");
                            return;
                        }
                        if (!SDL_LockTexture(is->sub_texture, (SDL_Rect *)sub_rect, (void **)pixels, pitch)) {
                            sws_scale(is->sub_convert_ctx, (const uint8_t * const *)sub_rect->data, sub_rect->linesize,
                                      0, sub_rect->h, pixels, pitch);
                            SDL_UnlockTexture(is->sub_texture);
                        }
                    }
                    sp->uploaded = 1;
                }
            } else
                sp = NULL;
        }
    }

    calculate_display_rect(&rect, is->xleft, is->ytop, is->width, is->height, vp->width, vp->height, vp->sar);
    set_sdl_yuv_conversion_mode(vp->frame);

    if (!vp->uploaded) {
        if (upload_texture(&is->vid_texture, vp->frame) < 0) {
            set_sdl_yuv_conversion_mode(NULL);
            return;
        }
        vp->uploaded = 1;
        vp->flip_v = vp->frame->linesize[0] < 0;
    }

    SDL_RenderCopyEx(renderer, is->vid_texture, NULL, &rect, 0, NULL, vp->flip_v ? SDL_FLIP_VERTICAL : 0);
    set_sdl_yuv_conversion_mode(NULL);
    if (sp) {
#if USE_ONEPASS_SUBTITLE_RENDER
        SDL_RenderCopy(renderer, is->sub_texture, NULL, &rect);
#else
        int i;
        double xratio = (double)rect.w / (double)sp->width;
        double yratio = (double)rect.h / (double)sp->height;
        for (i = 0; i < sp->sub.num_rects; i++) {
            SDL_Rect *sub_rect = (SDL_Rect*)sp->sub.rects[i];
            SDL_Rect target = {.x = rect.x + sub_rect->x * xratio,
                               .y = rect.y + sub_rect->y * yratio,
                               .w = sub_rect->w * xratio,
                               .h = sub_rect->h * yratio};
            SDL_RenderCopy(renderer, is->sub_texture, sub_rect, &target);
        }
#endif
    }
}

#ifndef SKC_VIDEO_ONLY
static inline int compute_mod(int a, int b)
{
    return a < 0 ? a%b + b : a%b;
}

static void video_audio_display(VideoState *s)
{
    int i, i_start, x, y1, y, ys, delay, n, nb_display_channels;
    int ch, channels, h, h2;
    int64_t time_diff;
    int rdft_bits, nb_freq;

    for (rdft_bits = 1; (1 << rdft_bits) < 2 * s->height; rdft_bits++)
        ;
    nb_freq = 1 << (rdft_bits - 1); // rdft_bits = 9, nb_freq = 256

    /* compute display index : center on currently output samples */
    channels = s->audio_tgt.ch_layout.nb_channels;
    nb_display_channels = channels;
    if (!s->paused) {
        int data_used= s->show_mode == SHOW_MODE_WAVES ? s->width : (2*nb_freq); // s->width = 320
        n = 2 * channels;
        // ���� ��¾ȵǰ� ���� ����ũ���, ���� �׻� delay = 0(��, ��� ������� ����� ���� �ð�)
        delay = s->audio_write_buf_size;
        av_assert0(delay == 0);
        delay /= n;

        /* to be more precise, we take into account the time spent since
           the last buffer computation */
        if (audio_callback_time) {
            time_diff = av_gettime_relative() - audio_callback_time;
            delay -= (time_diff * s->audio_tgt.freq) / 1000000; // s->audio_tgt.freq = 48000
        }

        delay += 2 * data_used;
        if (delay < data_used)
            delay = data_used;

        // ����� ��� ������ ����Ͽ�, sample_array���� ����� ������ �׸��� ������ ��ġ�� ����
        // sample_array�� ����� ����Ǵ� ���õ��� �ð�ȭ�� ���� ���� ������ circular buffer
        // �׷��� audio_write_buf_size��ŭ ���� ������� �ʾ����Ƿ�(�̸�ŭ �� �׷���� �Ѵ�!),
        // �׸�ŭ ������ index�� �Ž��� �ö󰡼� �׷����� �׸��� �����ؾ� �մϴ�.
        i_start = x = compute_mod(s->sample_array_index - delay * channels, SAMPLE_ARRAY_SIZE/* 512*1024 */);
        if (s->show_mode == SHOW_MODE_WAVES) {
            h = INT_MIN;
            // ���� ����� ��� ������ ���� 1000 ���ÿ� ���� �ѹ��� ���� ������ ������ ã��!
            for (i = 0; i < 1000; i += channels) {
                // ����� ������ ȭ�鿡 �ð�ȭ�� ��,
                // "���� ������ �ް��� ��ȭ(��ũ)�� ����"�Ͽ� ������ ������(i_start)�� ã�Ƴ��� ��ƾ
                // ����� ���� ���ۿ��� "��ȭ�� ũ�� ���� ������ �ִ� ����"�� ã�� �ð�ȭ�� ������(i_start)���� ��´�.
                int idx = (SAMPLE_ARRAY_SIZE + x - i) % SAMPLE_ARRAY_SIZE;
                int a = s->sample_array[idx]; // ���� ���ð�
                int b = s->sample_array[(idx + 4 * channels) % SAMPLE_ARRAY_SIZE];
                int c = s->sample_array[(idx + 5 * channels) % SAMPLE_ARRAY_SIZE];
                int d = s->sample_array[(idx + 9 * channels) % SAMPLE_ARRAY_SIZE]; // 9 ä�� ���� ���ð�
                // �̷������� �б�, ���� �����忡�� lock �� �ʿ�������, ���� �����ų� ���� ������ 
                // ����� ü���� ���� ������ �����Ƿ� ���������� omit(����)
                int score = a - d;
                // (b ^ c) < 0 �� b�� c�� ��ȣ�� �ٸ���, XOR ����� �ֻ��� ��Ʈ�� 1 -> ����
                // e.g, b = 5 (0x00000005), c = -3 (0xFFFFFFFD), b ^ c = 0xFFFFFFF8 �� < 0
                // ��Ʈ�����̶� ������ �ǹ��� (b * c) < 0 ���� ������ �����÷ο� ���� X
                if (h < score && (b ^ c) < 0) {
                    h = score;
                    i_start = idx; // ���� x ��ġ���� �����ؼ� 1000 ���ó����� ���� ��ȭ�� ū ���� ��ġ�� ����
                }
            }
        }

        s->last_i_start = i_start;
    } else {
        i_start = s->last_i_start;
    }

    if (s->show_mode == SHOW_MODE_WAVES) {
        SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);

        /* total height for one channel */
        h = s->height / nb_display_channels; // â ������ ����(2 ä���� ���, 176 / 2 = 88)
        /* graph height / 2 */
        h2 = (h * 9) / 20;      // ������ ���� ����(45%), (88 * 9) / 20 = 39
        for (ch = 0; ch < nb_display_channels; ch++) {
            i = i_start + ch;
            // s->ytop == s->xleft == 0
            // y1 = h / 2 = 88 / 2 = 44(1ch), 88 + 44 = 132(2ch)
            y1 = s->ytop + ch * h + (h / 2); /* position of center line */
            for (x = 0; x < s->width; x++) {
                // s->sample_array �� 16��Ʈ ����� ���� ��. ������ �� [-32768, 32767]
                // y = (���ð� * ������ ����(h2)) / 32768 �� �����ϸ� [-h2, h2(39)] �ȼ� ����
                // y �� �߾Ӽ��� �������� ��/�Ʒ��� �׸� ���� ����(�ȼ�)��
                y = (s->sample_array[i] * h2) >> 15; // -3, 3 �� ���� �ʴµ���
                if (y < 0) { // y = 44(1ch) or 132(2ch) �� ������ �׸�?!
                    y = -y;         // �簢�� ���� = ���(0 ~ 39)
                    ys = y1 - y;    // �簢�� top ��ǥ = (44 ~ 5) or (132 ~ 93)
                } else {        // y = ���, 44(1ch) or 132(2ch) �Ʒ� ������ �׸�?!
                    ys = y1;    // y1 = �簢�� top ��ǥ 44(1ch) or 132(2ch)
                }
                fill_rectangle(s->xleft + x, ys, 1, y);
                i += channels;
                if (i >= SAMPLE_ARRAY_SIZE)
                    i -= SAMPLE_ARRAY_SIZE;
            }
        }

        SDL_SetRenderDrawColor(renderer, 0, 0, 255, 255); // 1 ��¥�� �Ķ��� vertical center line

        for (ch = 1; ch < nb_display_channels; ch++) {
            y = s->ytop + ch * h;
            fill_rectangle(s->xleft, y, s->width, 1);
        }
    } else {
        // ����Ʈ�� �м� (RDFT ��� �ð�ȭ)
        // ����� �����͸� ���ļ� ���������� ��ȯ�� ��, �� ����� ȭ�鿡 �÷� �׷����� ������
        int err = 0;
        if (realloc_texture(&s->vis_texture, SDL_PIXELFORMAT_ARGB8888, s->width, s->height, SDL_BLENDMODE_NONE, 1) < 0)
            return;

        if (s->xpos >= s->width) // ���� s->xpos = 0
            s->xpos = 0;
        nb_display_channels= FFMIN(nb_display_channels, 2);
        if (rdft_bits != s->rdft_bits) { // 9 != 0
            const float rdft_scale = 1.0;
            av_tx_uninit(&s->rdft);
            av_freep(&s->real_data);
            av_freep(&s->rdft_data);
            s->rdft_bits = rdft_bits;
            s->real_data = av_malloc_array(nb_freq, 4 *sizeof(*s->real_data));
            s->rdft_data = av_malloc_array(nb_freq + 1, 2 *sizeof(*s->rdft_data));
            // ��� ��ȯ(��ȯ��, transform) ������ �����ϴ� �Լ� �����͸� ȹ��(s->rdft_fn)
            // �ַ� FFT, DCT, MDCT, RDFT ��� ���� ���ļ� ��ȯ(Fourier Transform) �迭���� ����
            // Real Discrete Fourier Transform (�Ǽ� DFT)
            err = av_tx_init(&s->rdft, &s->rdft_fn, AV_TX_FLOAT_RDFT,
                             0, 1 << rdft_bits, &rdft_scale, 0);
        }
        if (err < 0 || !s->rdft_data) {
            av_log(NULL, AV_LOG_ERROR, "Failed to allocate buffers for RDFT, switching to waves display\n");
            s->show_mode = SHOW_MODE_WAVES;
        } else {
            float *data_in[2];
            AVComplexFloat *data[2];
            // s->xpos �� ���������� shift �Ǹ鼭, �Ʒ��� ���η� 1�� ����(rect)�� �׸���.
            SDL_Rect rect = {.x = s->xpos, .y = 0, .w = 1, .h = s->height};
            uint32_t *pixels;
            int pitch;
            for (ch = 0; ch < nb_display_channels; ch++) {
                data_in[ch] = s->real_data + 2 * nb_freq * ch;
                data[ch] = s->rdft_data + nb_freq * ch;
                i = i_start + ch;
                for (x = 0; x < 2 * nb_freq; x++) {
                    double w = (x-nb_freq) * (1.0 / nb_freq);
                    // �ع� ������ (1 - w��)�� ������ spectral leakage ��ȭ?!
                    data_in[ch][x] = s->sample_array[i] * (1.0 - w * w);
                    i += channels;
                    if (i >= SAMPLE_ARRAY_SIZE)
                        i -= SAMPLE_ARRAY_SIZE;
                }
                // RDFT ����� AVComplexFloat[] ������ data[ch]�� �����
                s->rdft_fn(s->rdft, data[ch], data_in[ch], sizeof(float));
                data[ch][0].im = data[ch][nb_freq].re;
                data[ch][nb_freq].re = 0;
            }
            /* Least efficient way to do this, we should of course
             * directly access it but it is more than fast enough. */
            if (!SDL_LockTexture(s->vis_texture, &rect, (void **)&pixels, &pitch)) {
                pitch >>= 2;
                pixels += pitch * s->height;
                for (y = 0; y < s->height; y++) {
                    double w = 1 / sqrt(nb_freq);
                    // a, b: ä�κ� RDFT ũ�⸦ ��� ������ ��ȯ (sqrt(magnitude))
                    int a = sqrt(w * sqrt(data[0][y].re * data[0][y].re + data[0][y].im * data[0][y].im));
                    int b = (nb_display_channels == 2 ) ? sqrt(w * hypot(data[1][y].re, data[1][y].im))
                                                        : a;
                    a = FFMIN(a, 255);
                    b = FFMIN(b, 255);
                    pixels -= pitch;
                    // ��� �ȼ��� RGB ����: (R=ä��1, G=ä��2, B=���) ��
                    *pixels = (a << 16) + (b << 8) + ((a+b) >> 1);
                }
                SDL_UnlockTexture(s->vis_texture);
            }
            SDL_RenderCopy(renderer, s->vis_texture, NULL, NULL);
        }
        if (!s->paused)
            s->xpos++; // �ð� �帧�� ���� �ð�ȭ�� �¡��� ��ũ�ѵǵ��� xpos ����
    }
}
#endif

static void stream_component_close(VideoState *is, int stream_index)
{
    AVFormatContext *ic = is->ic;
    AVCodecParameters *codecpar;

    if (stream_index < 0 || stream_index >= ic->nb_streams)
        return;
    codecpar = ic->streams[stream_index]->codecpar;

#ifdef SKC_VIDEO_ONLY
    switch (codecpar->codec_type) {
    case AVMEDIA_TYPE_VIDEO:
        decoder_abort(&is->viddec, &is->pictq);
        decoder_destroy(&is->viddec);
        break;
    default:
        break;
    }
    ic->streams[stream_index]->discard = AVDISCARD_ALL;
    switch (codecpar->codec_type) {
    case AVMEDIA_TYPE_VIDEO:
        is->video_st = NULL;
        is->video_stream = -1;
        break;
    default:
        break;
    }
#else
    switch (codecpar->codec_type) {
    case AVMEDIA_TYPE_AUDIO:
        decoder_abort(&is->auddec, &is->sampq);
        SDL_CloseAudioDevice(audio_dev);
        decoder_destroy(&is->auddec);
        swr_free(&is->swr_ctx);
        av_freep(&is->audio_buf1);
        is->audio_buf1_size = 0;
        is->audio_buf = NULL;

        if (is->rdft) {
            av_tx_uninit(&is->rdft);
            av_freep(&is->real_data);
            av_freep(&is->rdft_data);
            is->rdft = NULL;
            is->rdft_bits = 0;
        }
        break;
    case AVMEDIA_TYPE_VIDEO:
        decoder_abort(&is->viddec, &is->pictq);
        decoder_destroy(&is->viddec);
        break;
    case AVMEDIA_TYPE_SUBTITLE:
        decoder_abort(&is->subdec, &is->subpq);
        decoder_destroy(&is->subdec);
        break;
    default:
        break;
    }

    ic->streams[stream_index]->discard = AVDISCARD_ALL;
    switch (codecpar->codec_type) {
    case AVMEDIA_TYPE_AUDIO:
        is->audio_st = NULL;
        is->audio_stream = -1;
        break;
    case AVMEDIA_TYPE_VIDEO:
        is->video_st = NULL;
        is->video_stream = -1;
        break;
    case AVMEDIA_TYPE_SUBTITLE:
        is->subtitle_st = NULL;
        is->subtitle_stream = -1;
        break;
    default:
        break;
    }
#endif
}

static void stream_close(VideoState *is)
{
    /* XXX: use a special url_shutdown call to abort parse cleanly */
    is->abort_request = 1;
    SDL_WaitThread(is->read_tid, NULL);

    /* close each stream */
    if (is->audio_stream >= 0)
        stream_component_close(is, is->audio_stream);
    if (is->video_stream >= 0)
        stream_component_close(is, is->video_stream);
    if (is->subtitle_stream >= 0)
        stream_component_close(is, is->subtitle_stream);

    avformat_close_input(&is->ic);

    packet_queue_destroy(&is->videoq);
    packet_queue_destroy(&is->audioq);
    packet_queue_destroy(&is->subtitleq);

    /* free all pictures */
    frame_queue_destroy(&is->pictq);
    frame_queue_destroy(&is->sampq);
    frame_queue_destroy(&is->subpq);
    SDL_DestroyCond(is->continue_read_thread);
    sws_freeContext(is->sub_convert_ctx);
    av_free(is->filename);
    if (is->vis_texture)
        SDL_DestroyTexture(is->vis_texture);
    if (is->vid_texture)
        SDL_DestroyTexture(is->vid_texture);
    if (is->sub_texture)
        SDL_DestroyTexture(is->sub_texture);
    av_free(is);
}

static void do_exit(VideoState *is)
{
    if (is) {
        stream_close(is);
    }
    if (renderer)
        SDL_DestroyRenderer(renderer);
    if (vk_renderer)
        vk_renderer_destroy(vk_renderer);
    if (window)
        SDL_DestroyWindow(window);
    uninit_opts();
    for (int i = 0; i < nb_vfilters; i++)
        av_freep(&vfilters_list[i]);
    av_freep(&vfilters_list);
    av_freep(&video_codec_name);
#ifndef SKC_VIDEO_ONLY
    av_freep(&audio_codec_name);
    av_freep(&subtitle_codec_name);
#endif
    av_freep(&input_filename);
    avformat_network_deinit();
    if (show_status)
        printf("\n");
    SDL_Quit();
    av_log(NULL, AV_LOG_QUIET, "%s", "");
    exit(0);
}

static void sigterm_handler(int sig)
{
    exit(123);
}

static void set_default_window_size(int width, int height, AVRational sar)
{
    SDL_Rect rect;
    int max_width  = screen_width  ? screen_width  : INT_MAX;
    int max_height = screen_height ? screen_height : INT_MAX;
    if (max_width == INT_MAX && max_height == INT_MAX)
        max_height = height;
    calculate_display_rect(&rect, 0, 0, max_width, max_height, width, height, sar);
    default_width  = rect.w;
    default_height = rect.h;
}

static int video_open(VideoState *is)
{
    int w,h;

    w = screen_width ? screen_width : default_width;
    h = screen_height ? screen_height : default_height;

    if (!window_title)
        window_title = input_filename;
    SDL_SetWindowTitle(window, window_title);

    SDL_SetWindowSize(window, w, h);
    SDL_SetWindowPosition(window, screen_left, screen_top);
    if (is_full_screen)
        SDL_SetWindowFullscreen(window, SDL_WINDOW_FULLSCREEN_DESKTOP);
    SDL_ShowWindow(window);

    is->width  = w;
    is->height = h;

    return 0;
}

/* display the current picture, if any */
static void video_display(VideoState *is)
{
    if (!is->width)
        video_open(is);

    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
    SDL_RenderClear(renderer);
#ifdef SKC_VIDEO_ONLY
    if (is->video_st)
        video_image_display(is);
#else
    if (is->audio_st && is->show_mode != SHOW_MODE_VIDEO)
        video_audio_display(is);
    else if (is->video_st)
        video_image_display(is);
#endif
    SDL_RenderPresent(renderer);
}

static double get_clock(Clock *c)
{
    if (*c->queue_serial != c->serial)
        return NAN;
    if (c->paused) {
        return c->pts;
    } else {
        double time = av_gettime_relative() / 1000000.0; // second ����
        // c->pts_drift(pts - t1) + time(t2) �� (pts - t1) + t2 �̸�, �� pts �� �귯�� �ð�(t2 - t1)�� ������ ����
        // ���⿡ ���� �ӵ�(c->speed == 1) ���� ������(e.g, c->speed == 2) �ð��� �߰���(���� ����)
        //printf("get_clock() c=%x, pts=%f, time=%f\n", c, c->pts_drift + time, time);
        return c->pts_drift + time - (time - c->last_updated) * (1.0 - c->speed);
    }
}

static void set_clock_at(Clock *c, double pts, int serial, double time)
{
    //printf("set_clock_at() c=%x, pts=%f, time=%f\n", c, pts, time);
    c->pts = pts;
    c->last_updated = time;
    c->pts_drift = c->pts - time; // ���� Ƚ���� ����, float ���� ������ �ٰ� ��
    c->serial = serial;
}

static void set_clock(Clock *c, double pts, int serial)
{
    double time = av_gettime_relative() / 1000000.0;
    set_clock_at(c, pts, serial, time);
}

static void set_clock_speed(Clock *c, double speed)
{
    set_clock(c, get_clock(c), c->serial);
    c->speed = speed;
}

static void init_clock(Clock *c, int *queue_serial)
{
    c->speed = 1.0;
    c->paused = 0;
    c->queue_serial = queue_serial;
    set_clock(c, NAN, -1);
}

static void sync_clock_to_slave(Clock *c, Clock *slave)
{
    double clock = get_clock(c);
    double slave_clock = get_clock(slave);
    if (!isnan(slave_clock) && (isnan(clock) || fabs(clock - slave_clock) > AV_NOSYNC_THRESHOLD))
        set_clock(c, slave_clock, slave->serial);
}

// ���� -> ����� -> �ܺ�Ŭ��
static int get_master_sync_type(VideoState *is) {
    if (is->av_sync_type == AV_SYNC_VIDEO_MASTER) {
        if (is->video_st)
            return AV_SYNC_VIDEO_MASTER;
        else
            // ���� ��Ʈ���� ���µ� ���� ������ ���¸� ����� �����͸� ����
            // ����� ����� ��Ʈ���� ������ ��� �ȵǰ� ó���� ����(read_thread() ����)
            return AV_SYNC_AUDIO_MASTER;
    } else if (is->av_sync_type == AV_SYNC_AUDIO_MASTER) {
        if (is->audio_st)
            return AV_SYNC_AUDIO_MASTER;
        else
            // ����� ����Ʈ�ε� ������� ���� ���, �ܺ�Ŭ�� ����
            return AV_SYNC_EXTERNAL_CLOCK;
    } else {
        // ������� ������ �����Ͱ� �ƴ� ���, �ܺ�Ŭ�� ����
        return AV_SYNC_EXTERNAL_CLOCK;
    }
}

/* get the current master clock value */
static double get_master_clock(VideoState *is)
{
    double val;

    switch (get_master_sync_type(is)) {
        case AV_SYNC_VIDEO_MASTER:
            val = get_clock(&is->vidclk);
            break;
        case AV_SYNC_AUDIO_MASTER:
            val = get_clock(&is->audclk);
            break;
        default:
            val = get_clock(&is->extclk);
            break;
    }
    //printf("get_master_clock(%d) %.3f\n", get_master_sync_type(is), val);
    return val;
}

// ��Ŷť�� ���� ��Ŷ ������ ���� Ŭ�� �ӵ��� ����
static void check_external_clock_speed(VideoState *is) {
    // �Ʒ����� ������ ���� 0.900���� ����ϸ鼭, ������ ���� 1.100���� �ø��� �ʰ� 1.010������ �����ϴ� ������?
    // ���� ������� '����'�� Ƽ ���� ������, '����'�� �ٷ� Ƽ ����
    // ���� ������ 10% ���, ������ 1 % �� ��� �� ����ڰ� ��ġä�� ���� ���ؿ��� ����ȭ ����
   if (is->video_stream >= 0 && is->videoq.nb_packets <= EXTERNAL_CLOCK_MIN_FRAMES/*2*/ ||
       is->audio_stream >= 0 && is->audioq.nb_packets <= EXTERNAL_CLOCK_MIN_FRAMES/*2*/) {
       // ��Ŷ�� �����ϸ� �ӵ� ����(1, 0.999, 0.998, ..., 0.900)
       set_clock_speed(&is->extclk, FFMAX(EXTERNAL_CLOCK_SPEED_MIN/*0.900*/, is->extclk.speed - EXTERNAL_CLOCK_SPEED_STEP/*0.001*/));
   } else if ((is->video_stream < 0 || is->videoq.nb_packets > EXTERNAL_CLOCK_MAX_FRAMES/*10*/) &&
              (is->audio_stream < 0 || is->audioq.nb_packets > EXTERNAL_CLOCK_MAX_FRAMES/*10*/)) {
       // ��Ŷ�� ������ �ӵ� ����(1, 1.001, 1.002, ..., 1.010)
       set_clock_speed(&is->extclk, FFMIN(EXTERNAL_CLOCK_SPEED_MAX/*1.010*/, is->extclk.speed + EXTERNAL_CLOCK_SPEED_STEP/*0.001*/));
   } else {
       double speed = is->extclk.speed;
       if (speed != 1.0)
           // speed �� 1�� �ƴϰ�, �ּ�, �ִ� �������� �ִ� ���, speed �� ���������� 1.0 �� �ǰ� ������
           // (1.0 - speed) / fabs(1.0 - speed) �� +1, -1 �� ���⸸ ��Ÿ��
           set_clock_speed(&is->extclk, speed + EXTERNAL_CLOCK_SPEED_STEP/*0.001*/ * (1.0 - speed) / fabs(1.0 - speed));
   }
}

/* seek in the stream */
static void stream_seek(VideoState *is, int64_t pos, int64_t rel, int by_bytes)
{
    if (!is->seek_req) {
        is->seek_pos = pos;
        is->seek_rel = rel;
        is->seek_flags &= ~AVSEEK_FLAG_BYTE;
        if (by_bytes)
            is->seek_flags |= AVSEEK_FLAG_BYTE;
        is->seek_req = 1;
        SDL_CondSignal(is->continue_read_thread);
    }
}

/* pause or resume the video */
static void stream_toggle_pause(VideoState *is)
{
    if (is->paused) {
        is->frame_timer += av_gettime_relative() / 1000000.0 - is->vidclk.last_updated;
        // AVERROR(ENOSYS): �ش� ����� �������� ����(is->read_pause_return = av_read_pause(ic))
        if (is->read_pause_return != AVERROR(ENOSYS)) {
            is->vidclk.paused = 0;
        }
        // get_clock() �� set_clock() �ϴ� ������ Ÿ�̹� ���� ������ ���̱� ���� �纸��(re-sync)
        // ���� ������ ���� ������ �۰Ե� ���信 ������ �����ϰ� ��
        set_clock(&is->vidclk, get_clock(&is->vidclk), is->vidclk.serial);
    }
    set_clock(&is->extclk, get_clock(&is->extclk), is->extclk.serial);
    is->paused = is->audclk.paused = is->vidclk.paused = is->extclk.paused = !is->paused;
}

static void toggle_pause(VideoState *is)
{
    stream_toggle_pause(is);
    is->step = 0;
}

#ifndef SKC_VIDEO_ONLY
static void toggle_mute(VideoState *is)
{
    is->muted = !is->muted;
}

static void update_volume(VideoState *is, int sign, double step)
{
    double volume_level = is->audio_volume ? (20 * log(is->audio_volume / (double)SDL_MIX_MAXVOLUME) / log(10)) : -1000.0;
    int new_volume = lrint(SDL_MIX_MAXVOLUME * pow(10.0, (volume_level + sign * step) / 20.0)); // lrint() �ݿø�
    is->audio_volume = av_clip(is->audio_volume == new_volume ? (is->audio_volume + sign) : new_volume, 0, SDL_MIX_MAXVOLUME);
}
#endif

static void step_to_next_frame(VideoState *is)
{
    /* if the stream is paused unpause it, then step */
    if (is->paused)
        stream_toggle_pause(is);
    is->step = 1;
}

// is->vidclk �� �����͸� ���� ���� ��� �ð�(delay)�� ����� ����
static double compute_target_delay(double delay, VideoState *is)
{
    // delay �� ���� ������ ���� �ð�(duration) ���� �Ѿ���Ƿ� �׻� ���?!
    av_assert0(delay >= 0);
    double sync_threshold, diff = 0;
    double skc_org_delay = delay;

    /* update delay to follow master synchronisation source */
    if (get_master_sync_type(is) != AV_SYNC_VIDEO_MASTER) {
        /* if video is slave, we try to correct big delays by
           duplicating or deleting a frame */
        diff = get_clock(&is->vidclk) - get_master_clock(is);

        /* skip or repeat frame. We take into account the
           delay to compute the threshold. I still don't know
           if it is the best guess */
        sync_threshold = FFMAX(AV_SYNC_THRESHOLD_MIN/*0.04*/, FFMIN(AV_SYNC_THRESHOLD_MAX/*0.1*/, delay));
        if (!isnan(diff) && fabs(diff) < is->max_frame_duration/*3600.0*/) { // ������ �ʹ� ũ�� skip �ϸ�, delay �״�θ� ����
            // ������ (����� ����) ���� ���(diff(����) <= -sync_threshold)
            // ������ŭ(diff) delay �� �ٿ� ������ ���
            if (diff <= -sync_threshold)
                delay = FFMAX(0, delay + diff); // diff �� delay ��ŭ ������ delay = 0!!!
            // ������ ���� ���(diff >= sync_threshold)
            // 1) duration(=delay) �� ����ϸ� delay �� ������ŭ(diff) �÷� ������ ���
            else if (diff >= sync_threshold && delay > AV_SYNC_FRAMEDUP_THRESHOLD/*0.1*/) {
                //av_assert0(0);
                delay = delay + diff; // e.g, delay(0.3) = delay(0.2) + diff(0.1)
            }
            // 2) duration(=delay) �� ������� ������, 2�踸 ������ ���
            // ª�� �����ӵ��� ������ ���� �ð��� 2�� ����
            // ������ ���� �ð��� �ް��� ���ϰ� �� ������ �Ҷ� �����(stuttering) ������ ����?!
            else if (diff >= sync_threshold) {
                delay = 2 * delay; // e.g, delay(0.2) = 2 * delay(0.1)
            }
        }
    }

    av_log(NULL, AV_LOG_TRACE, "video: delay=%0.3f A-V=%f\n",
            delay, -diff);
    //printf("video: org=%.3f delay=%0.3f A-V=%f\n", skc_org_delay, delay, -diff);

    av_assert0(delay >= 0);
    return delay;
}

static double vp_duration(VideoState *is, Frame *vp, Frame *nextvp) {
    if (vp->serial == nextvp->serial) {
        double duration = nextvp->pts - vp->pts;
        if (isnan(duration) || duration <= 0 || duration > is->max_frame_duration)
            // �켱 pts �� ����� duration �� ����ϰ�, �̰� ��ȿ���� ������, vp->duration �� ���
            // Frame::pts, duration ���� ������
            // 1) ������� audio_thread() ���� ���ڵ� ��, 
            //    af->duration = { frame->nb_samples/*�����Ӵ� 1024 ����*/, frame->sample_rate/*�ʴ� 48000 ����*/ } �� ����(21.3ms)
            // 2) ������ video_thread() ���� ���Ϳ��� ���� frame_rate{25/1} �� queue_picture(duration) �Լ��� 
            //    duration/*40ms(����)*/ = {frame_rate.den/*1*/, frame_rate.num/*25*/} �� ���ڷ� ������ ����
            return vp->duration;
        else
            return duration;
    } else {
        return 0.0;
    }
}

static void update_video_pts(VideoState *is, double pts, int serial)
{
    /* update current video pts */
    static int skc_cnt = 0;
    //printf("VIDEO:: update_video_pts(%d) pts=%f\n", skc_cnt++, pts);
    set_clock(&is->vidclk, pts, serial);
    sync_clock_to_slave(&is->extclk, &is->vidclk);
}

/* called to display each frame */
static void video_refresh(void *opaque, double *remaining_time)
{
    VideoState *is = opaque;
    double time;

    Frame *sp, *sp2;

    if (!is->paused && get_master_sync_type(is) == AV_SYNC_EXTERNAL_CLOCK && is->realtime)
        check_external_clock_speed(is);

    if (!display_disable && is->show_mode != SHOW_MODE_VIDEO && is->audio_st) {
        // �����(����)�� ���⼭ ǥ���ϰ�, ������ SHOW_MODE_VIDEO �� ���, �Ʒ����� frame_drops_late ó�� ��, ���÷���
        time = av_gettime_relative() / 1000000.0;
        if (is->force_refresh || is->last_vis_time + rdftspeed/*0.02*/ < time) {
            video_display(is); // video_audio_display() ȣ��
            is->last_vis_time = time;
        }
        *remaining_time = FFMIN(*remaining_time, is->last_vis_time + rdftspeed - time);
    }

    if (is->video_st) {
retry:
        if (frame_queue_nb_remaining(&is->pictq) == 0) { // return f->size - f->rindex_shown;
            // nothing to do, no picture to display in the queue
        } else {
            double last_duration, duration, delay;
            Frame *vp, *lastvp;

            /* dequeue the picture */
            // lastvp, vp, nextvp(�Ʒ�) ������
            // vp �� lastvp �� rindex_shown �� ������ ������(lastvp �� ���� ������)
            // ���ʿ��� rindex_shown = 0 �̶� lastvp, vp �� ������
            lastvp = frame_queue_peek_last(&is->pictq); // f->rindex ����
            vp = frame_queue_peek(&is->pictq);          // f->rindex + f->rindex_shown ����

            if (vp->serial != is->videoq.serial) {
                av_assert0(0);
                frame_queue_next(&is->pictq);
                goto retry;
            }

            if (lastvp->serial != vp->serial) {
                av_assert0(0);
                is->frame_timer = av_gettime_relative() / 1000000.0;
            }

            if (is->paused)
                goto display;

            // lastvp �������� ����ð��� pts �� ����� ����, �ƴϸ� Frame::duration �� ����(lastvp, vp ������ ������)
            /* compute nominal last_duration */
            last_duration = vp_duration(is, lastvp, vp); // ���� 0.04

            // last_duration ����ð��� master clock �� ���� Ŭ�� ���̸�ŭ ������ ���� ������ ���� �ð�(delay)�� ȹ��
            delay = compute_target_delay(last_duration, is); // ���� ���� �� 0.04
            //printf("video_refresh() delay = compute_target_delay() --> %.3f\n", delay);

            time= av_gettime_relative()/1000000.0; // seconds ����
            // ���� �ð�(time)�� ���� ������ �� �ð�(is->frame_timer + delay) ������ ���
            if (time < is->frame_timer + delay) { // ���� ���� 3�� ���� �� 1�� �н�
                // ó���� is->frame_timer == 0 �̶� ���� ���� ���ϰ�, frame_timer �� ����ð����� ���� & �Ʒ��� ù �������� �׷���(rindex = 0)
                // ���� �����Խ� wall clock �� ������ frame_timer(���� ����ð�) + delay ���� ������, ���� ������ ���÷��̸� ����
                // ���� ���� �������� �����ִ� �ð��� ������ �ʾҴٴ� �ǹ�
                // ���� ��� �ð�(remaining_time) = ������ �� �ð�(is->frame_timer + delay) - ���� �ð�(time)
                *remaining_time = FFMIN(is->frame_timer + delay - time, *remaining_time);
                goto display;
            }
            // ���� ���� ������(rindex + 1)�� ǥ���� �ð���

            // ���� �ð��� ������ �� �ð�(is->frame_timer + delay)�� 0.1s �̻� �Ѿ����, frame_timer �缳��
            is->frame_timer += delay; // �� ����� �ð� ����
            //av_assert0(delay > 0); // delay == 0 ����
            // frame_timer �� ����ð�(time)�� ���󰡰� ������, ������ �ʹ� ũ��(������) �� frame_timer = time ���� ���� ��ũ
            if (delay > 0 && time - is->frame_timer > AV_SYNC_THRESHOLD_MAX/*0.1*/) {
                // ó���� is->frame_timer == 0 �̶� ���ʿ� �ѹ� ȣ���(�ߴ��� �ɸ� Ÿ�̹��� �ȸ¾� ������ ȣ���)
                is->frame_timer = time;
            }

            SDL_LockMutex(is->pictq.mutex);
            if (!isnan(vp->pts))
                // ������ �������� vp->pts(���۽ð�) �� vidclk �� ����
                update_video_pts(is, vp->pts, vp->serial);
            SDL_UnlockMutex(is->pictq.mutex);

            // ���� �ð��� ���� ������ �� �ð��� �Ѿ ��� ��� ó��
            if (frame_queue_nb_remaining(&is->pictq) > 1) { // e.g, return f->size(2) - f->rindex_shown(0) = 2
                Frame *nextvp = frame_queue_peek_next(&is->pictq); // e.g, return &f->queue[(f->rindex(0) + f->rindex_shown(0) + 1) % f->max_size];
                duration = vp_duration(is, vp, nextvp);
                if(!is->step &&     // step ��尡 �ƴϰ�
                    (framedrop>0 || (framedrop && get_master_sync_type(is) != AV_SYNC_VIDEO_MASTER)) && // ���� �����Ͱ� �ƴϰ�
                    // ���ڵ� ���� ���� �������� ǥ�õ� �ð��� �Ѿ ���, ������.
                    time > is->frame_timer + duration){ // ���� �ð��� ���� ������ �� �ð��� �Ѿ ��츦 �ǹ���
                    is->frame_drops_late++; // ������ ���
                    frame_queue_next(&is->pictq);
                    goto retry;
                }
            }

            if (is->subtitle_st) {
                while (frame_queue_nb_remaining(&is->subpq) > 0) {
                    sp = frame_queue_peek(&is->subpq);

                    if (frame_queue_nb_remaining(&is->subpq) > 1)
                        sp2 = frame_queue_peek_next(&is->subpq);
                    else
                        sp2 = NULL;

                    if (sp->serial != is->subtitleq.serial
                            || (is->vidclk.pts > (sp->pts + ((float) sp->sub.end_display_time / 1000)))
                            || (sp2 && is->vidclk.pts > (sp2->pts + ((float) sp2->sub.start_display_time / 1000))))
                    {
                        // ��ŷ�� �ưų� ������ ���� �ڸ� �� ��ġ�� �����ų�, ���� �ڸ� ���� ��ġ�� �����ų� �ϸ�
                        // ���� �ڸ� �̹���(is->sub_texture) ������ �����ϰ� ä�� ��, ���� ���������� �̵�
                        // (memset() ���� rgba �� ��� 0 ���� ä��)
                        if (sp->uploaded) {
                            int i;
                            for (i = 0; i < sp->sub.num_rects; i++) {
                                AVSubtitleRect *sub_rect = sp->sub.rects[i];
                                uint8_t *pixels;
                                int pitch, j;

                                if (!SDL_LockTexture(is->sub_texture, (SDL_Rect *)sub_rect, (void **)&pixels, &pitch)) {
                                    for (j = 0; j < sub_rect->h; j++, pixels += pitch)
                                        memset(pixels, 0, sub_rect->w << 2);
                                    SDL_UnlockTexture(is->sub_texture);
                                }
                            }
                        }
                        frame_queue_next(&is->subpq);
                    } else {
                        break;
                    }
                }
            }

            frame_queue_next(&is->pictq); // ���⼭ ���ʿ� is->pictq.rindex_shown 0 -> 1, ���� ++rindex
            is->force_refresh = 1;

            if (is->step && !is->paused)
                stream_toggle_pause(is);
        }
display:
        /* display picture */
        // ���ʿ� ��� frame_queue_nb_remaining() == 0 �̶� ���� �� ȣ��ǳ� is->pictq.rindex_shown == 0 �̹Ƿ� skip ��
        if (!display_disable && is->force_refresh && is->show_mode == SHOW_MODE_VIDEO && is->pictq.rindex_shown)
            // video_image_display() ���� frame_queue_peek_last() �� rindex �� �о �׸��� ����(rindex = 0, rindex_shown = 1)
            video_display(is);
    }
    is->force_refresh = 0;
    if (show_status) {
        AVBPrint buf;
        static int64_t last_time;
        int64_t cur_time;
        int aqsize, vqsize, sqsize;
        double av_diff;

        cur_time = av_gettime_relative();
        if (!last_time || (cur_time - last_time) >= 30000) {
            aqsize = 0;
            vqsize = 0;
            sqsize = 0;
            if (is->audio_st)
                aqsize = is->audioq.size;
            if (is->video_st)
                vqsize = is->videoq.size;
            if (is->subtitle_st)
                sqsize = is->subtitleq.size;
            av_diff = 0;
            if (is->audio_st && is->video_st) // "A-V" ����, ����� �Ѵ� �ִ� ���, ����� ������
                av_diff = get_clock(&is->audclk) - get_clock(&is->vidclk);
            else if (is->video_st) // "M-V" ������ �ִ� ���
                av_diff = get_master_clock(is) - get_clock(&is->vidclk);
            else if (is->audio_st) // "M-A" ������� �ִ� ���
                av_diff = get_master_clock(is) - get_clock(&is->audclk);

            av_bprint_init(&buf, 0, AV_BPRINT_SIZE_AUTOMATIC);
            // e.g,    7.39 A-V: -0.028 fd=  51 aq=   30KB vq=    1KB sq=    0B
            av_bprintf(&buf,
                      "%7.2f %s:%7.3f fd=%4d aq=%5dKB vq=%5dKB sq=%5dB \r",
                      get_master_clock(is),
                      (is->audio_st && is->video_st) ? "A-V" : (is->video_st ? "M-V" : (is->audio_st ? "M-A" : "   ")),
                      av_diff,
                      is->frame_drops_early + is->frame_drops_late,
                      aqsize / 1024,
                      vqsize / 1024,
                      sqsize); // \n �� ��� �׻� �� �������ٿ� �׷����� �ִ�.

            if (show_status == 1 && AV_LOG_INFO > av_log_get_level())
                fprintf(stderr, "%s", buf.str);
            else
                av_log(NULL, AV_LOG_INFO, "%s", buf.str);

            fflush(stderr);
            av_bprint_finalize(&buf, NULL);

            last_time = cur_time;
        }
    }
}

static int queue_picture(VideoState *is, AVFrame *src_frame, double pts, double duration, int64_t pos, int serial)
{
    Frame *vp;

#if defined(DEBUG_SYNC)
    printf("frame_type=%c pts=%0.3f\n",
           av_get_picture_type_char(src_frame->pict_type), pts);
#endif

    if (!(vp = frame_queue_peek_writable(&is->pictq)))
        return -1;

    vp->sar = src_frame->sample_aspect_ratio;
    vp->uploaded = 0;

    vp->width = src_frame->width;
    vp->height = src_frame->height;
    vp->format = src_frame->format;

    vp->pts = pts;
    vp->duration = duration;
    vp->pos = pos;
    vp->serial = serial;

    set_default_window_size(vp->width, vp->height, vp->sar); // ���ʿ��� ����(���� ������ ũ�� ���� X)

    av_frame_move_ref(vp->frame, src_frame);
    frame_queue_push(&is->pictq);
    //av_usleep(100 * 1000000); //skc test
    return 0;
}

static int get_video_frame(VideoState *is, AVFrame *frame)
{
    int got_picture;

    if ((got_picture = decoder_decode_frame(&is->viddec, frame, NULL)) < 0)
        return -1;

    if (got_picture) {
        double dpts = NAN;

        av_assert0(frame->pts != AV_NOPTS_VALUE);
        if (frame->pts != AV_NOPTS_VALUE) {
            // frame->pts �� best_effort_timestamp = 0, 3600, 7200, 10800, ..., 896400 ��
            // dpts = 0, 0.04, 0.08, 0.12, ..., 9.96 // �ʴ���
            dpts = av_q2d(is->video_st->time_base/*1/90000*/) * frame->pts;
        }

        frame->sample_aspect_ratio = av_guess_sample_aspect_ratio(is->ic, is->video_st, frame);

        if (framedrop>0 || (framedrop && get_master_sync_type(is) != AV_SYNC_VIDEO_MASTER)) {
            if (frame->pts != AV_NOPTS_VALUE) {
                double diff = dpts - get_master_clock(is); // e.g, V-A
                if (!isnan(diff) && fabs(diff) < AV_NOSYNC_THRESHOLD/*10.0*/ &&
                    // ���� �������� ���͸��� �ɸ� �ð�(= ���� �����ð�, is->frame_last_filter_delay, ���� 0)�� 
                    // diff(V-M) �̻� ���� �ɷ�����(������ ������) drop! (���ť�� �ֱ� ���� ���ڴ� �ʿ��� ����)
                    diff - is->frame_last_filter_delay < 0 &&
                    is->viddec.pkt_serial == is->vidclk.serial &&
                    is->videoq.nb_packets) {
                    is->frame_drops_early++;
                    av_frame_unref(frame);
                    got_picture = 0;
                }
            }
        }
    }

    return got_picture;
}

static int configure_filtergraph(AVFilterGraph *graph, const char *filtergraph,
                                 AVFilterContext *source_ctx, AVFilterContext *sink_ctx)
{
    int ret, i;
    int nb_filters = graph->nb_filters;
    AVFilterInOut *outputs = NULL, *inputs = NULL;

    if (filtergraph) {
        av_assert0(0);
        outputs = avfilter_inout_alloc();
        inputs  = avfilter_inout_alloc();
        if (!outputs || !inputs) {
            ret = AVERROR(ENOMEM);
            goto fail;
        }

        // [in] ���̺� ���͸� ã�� source_ctx ���Ϳ� ����
        // source_ctx �� [in] �� �����  // source_ctx �� ����� ����
        outputs->name       = av_strdup("in");
        outputs->filter_ctx = source_ctx;
        outputs->pad_idx    = 0;
        outputs->next       = NULL;

        // inputs�� �츮�� ���� �׷���(graph)�� "�����ϰ� ���� �Է���" �� �ǹ�
        // filtergraph ���� [out] ���̺��� ���͸� ã�Ƴ� �Է����� sink_ctx ���Ϳ� ����
        // [out] �� sink_ctx �� �����   // sink_ctx �� �Է¿� ����
        inputs->name        = av_strdup("out");
        inputs->filter_ctx  = sink_ctx;
        inputs->pad_idx     = 0;
        inputs->next        = NULL;

        if ((ret = avfilter_graph_parse_ptr(graph, filtergraph, &inputs, &outputs, NULL)) < 0)
            goto fail;
    } else {
        if ((ret = avfilter_link(source_ctx, 0, sink_ctx, 0)) < 0)
            goto fail;
    }

    /* Reorder the filters to ensure that inputs of the custom filters are merged first */
    for (i = 0; i < graph->nb_filters - nb_filters; i++)
        FFSWAP(AVFilterContext*, graph->filters[i], graph->filters[i + nb_filters]);

    ret = avfilter_graph_config(graph, NULL);
fail:
    avfilter_inout_free(&outputs);
    avfilter_inout_free(&inputs);
    return ret;
}

static int configure_video_filters(AVFilterGraph *graph, VideoState *is, const char *vfilters, AVFrame *frame)
{
    enum AVPixelFormat pix_fmts[FF_ARRAY_ELEMS(sdl_texture_format_map)];
    char sws_flags_str[512] = "";
    char buffersrc_args[256];
    int ret;
    AVFilterContext *filt_src = NULL, *filt_out = NULL, *last_filter = NULL;
    AVCodecParameters *codecpar = is->video_st->codecpar;
    AVRational fr = av_guess_frame_rate(is->ic, is->video_st, NULL);
    const AVDictionaryEntry *e = NULL;
    int nb_pix_fmts = 0;
    int i, j;
    AVBufferSrcParameters *par = av_buffersrc_parameters_alloc();

    if (!par)
        return AVERROR(ENOMEM);

    for (i = 0; i < renderer_info.num_texture_formats; i++) {
        for (j = 0; j < FF_ARRAY_ELEMS(sdl_texture_format_map) - 1; j++) {
            if (renderer_info.texture_formats[i] == sdl_texture_format_map[j].texture_fmt) {
                pix_fmts[nb_pix_fmts++] = sdl_texture_format_map[j].format;
                break;
            }
        }
    }
    pix_fmts[nb_pix_fmts] = AV_PIX_FMT_NONE;

    while ((e = av_dict_iterate(sws_dict, e))) {
        if (!strcmp(e->key, "sws_flags")) {
            av_strlcatf(sws_flags_str, sizeof(sws_flags_str), "%s=%s:", "flags", e->value);
        } else
            av_strlcatf(sws_flags_str, sizeof(sws_flags_str), "%s=%s:", e->key, e->value);
    }
    if (strlen(sws_flags_str))
        sws_flags_str[strlen(sws_flags_str)-1] = '\0';

    graph->scale_sws_opts = av_strdup(sws_flags_str);

    snprintf(buffersrc_args, sizeof(buffersrc_args),
             "video_size=%dx%d:pix_fmt=%d:time_base=%d/%d:pixel_aspect=%d/%d:"
             "colorspace=%d:range=%d",
             frame->width, frame->height, frame->format,
             is->video_st->time_base.num, is->video_st->time_base.den,
             codecpar->sample_aspect_ratio.num, FFMAX(codecpar->sample_aspect_ratio.den, 1),
             frame->colorspace, frame->color_range);
    if (fr.num && fr.den)
        av_strlcatf(buffersrc_args, sizeof(buffersrc_args), ":frame_rate=%d/%d", fr.num, fr.den);

    // avfilter_get_by_name("buffer") ���� "buffer" �� ����� ���� �ҽ� ����(source filter)�� ���� ��� �̸�
    // buffer ���ʹ� ���ڵ��� AVFrame�� ���͸� �׷����� �����ϴ� ������ ������ ��
    // �����Ǵ� sink ���ʹ� "buffersink"
    // ���� ��� �̸��� "abuffer", "abuffersink" �� INSERT_FILT() ���� "transpose" ��� �ټ� ������
    // avfilter_graph_create_filter() �Լ��� �׷����� "���(���� �ν��Ͻ�)"�� �߰��ϴ� �Լ�(��ũ�� X)
    if ((ret = avfilter_graph_create_filter(&filt_src,       // ������ ���� ���ؽ�Ʈ�� ��ȯ�� ������
                                            avfilter_get_by_name("buffer"), // ����� ����(�����)
                                            "ffplay_buffer", // �׷��� �������� ���� �̸�
                                            buffersrc_args,  // ���Ϳ� ������ �ɼ� ���ڿ�(��: video_size=640x480:pix_fmt=0:time_base=1/25)
                                            NULL,
                                            graph)) < 0)
        goto fail;
    par->hw_frames_ctx = frame->hw_frames_ctx;
    ret = av_buffersrc_parameters_set(filt_src, par);
    if (ret < 0)
        goto fail;

    ret = avfilter_graph_create_filter(&filt_out,
                                       avfilter_get_by_name("buffersink"),
                                       "ffplay_buffersink", NULL, NULL, graph);
    if (ret < 0)
        goto fail;

    if ((ret = av_opt_set_int_list(filt_out, "pix_fmts", pix_fmts,  AV_PIX_FMT_NONE, AV_OPT_SEARCH_CHILDREN)) < 0)
        goto fail;
    if (!vk_renderer &&
        (ret = av_opt_set_int_list(filt_out, "color_spaces", sdl_supported_color_spaces,  AVCOL_SPC_UNSPECIFIED, AV_OPT_SEARCH_CHILDREN)) < 0)
        goto fail;

    last_filter = filt_out;

/* Note: this macro adds a filter before the lastly added filter, so the
 * processing order of the filters is in reverse */
// last_filter == filt_ctx(n) -> ... -> filt_ctx(1) -> filt_ctx(0) -> filt_out ������ �����
#define INSERT_FILT(name, arg) do {                                          \
    AVFilterContext *filt_ctx;                                               \
                                                                             \
    ret = avfilter_graph_create_filter(&filt_ctx,                            \
                                       avfilter_get_by_name(name),           \
                                       "ffplay_" name, arg, NULL, graph);    \
    if (ret < 0)                                                             \
        goto fail;                                                           \
                                                                             \
    ret = avfilter_link(filt_ctx, 0, last_filter, 0);                        \
    if (ret < 0)                                                             \
        goto fail;                                                           \
                                                                             \
    last_filter = filt_ctx;                                                  \
} while (0)

    if (autorotate) {
        double theta = 0.0;
        int32_t *displaymatrix = NULL;
        AVFrameSideData *sd = av_frame_get_side_data(frame, AV_FRAME_DATA_DISPLAYMATRIX);
        if (sd)
            displaymatrix = (int32_t *)sd->data;
        if (!displaymatrix) {
            const AVPacketSideData *psd = av_packet_side_data_get(is->video_st->codecpar->coded_side_data,
                                                                  is->video_st->codecpar->nb_coded_side_data,
                                                                  AV_PKT_DATA_DISPLAYMATRIX);
            if (psd)
                displaymatrix = (int32_t *)psd->data;
        }
        theta = get_rotation(displaymatrix); // degree ����

        if (fabs(theta - 90) < 1.0) {
            INSERT_FILT("transpose", displaymatrix[3] > 0 ? "cclock_flip" : "clock");
        } else if (fabs(theta - 180) < 1.0) {
            if (displaymatrix[0] < 0)
                INSERT_FILT("hflip", NULL);
            if (displaymatrix[4] < 0)
                INSERT_FILT("vflip", NULL);
        } else if (fabs(theta - 270) < 1.0) {
            INSERT_FILT("transpose", displaymatrix[3] < 0 ? "clock_flip" : "cclock");
        } else if (fabs(theta) > 1.0) {
            char rotate_buf[64];
            snprintf(rotate_buf, sizeof(rotate_buf), "%f*PI/180", theta);
            INSERT_FILT("rotate", rotate_buf);
        } else {
            if (displaymatrix && displaymatrix[4] < 0)
                INSERT_FILT("vflip", NULL);
        }
    }

    // configure_filtergraph() ���� avfilter_link(source_ctx, 0, sink_ctx, 0) �� filt_src -> last_filter �� ����
    // filt_src -> last_filter == filt_ctx(n) -> ... -> filt_ctx(1) -> filt_ctx(0) -> filt_out �� ����(n ��������)
    if ((ret = configure_filtergraph(graph, vfilters, filt_src, last_filter)) < 0)
        goto fail;

    is->in_video_filter  = filt_src;
    is->out_video_filter = filt_out;

fail:
    av_freep(&par);
    return ret;
}

#ifndef SKC_VIDEO_ONLY
static int configure_audio_filters(VideoState *is, const char *afilters, int force_output_format)
{
    static const enum AVSampleFormat sample_fmts[] = { AV_SAMPLE_FMT_S16, AV_SAMPLE_FMT_NONE };
    int sample_rates[2] = { 0, -1 };
    AVFilterContext *filt_asrc = NULL, *filt_asink = NULL;
    char aresample_swr_opts[512] = "";
    const AVDictionaryEntry *e = NULL;
    AVBPrint bp;
    char asrc_args[256];
    int ret;

    avfilter_graph_free(&is->agraph);
    if (!(is->agraph = avfilter_graph_alloc()))
        return AVERROR(ENOMEM);
    is->agraph->nb_threads = filter_nbthreads;

    av_bprint_init(&bp, 0, AV_BPRINT_SIZE_AUTOMATIC);

    while ((e = av_dict_iterate(swr_opts, e)))
        av_strlcatf(aresample_swr_opts, sizeof(aresample_swr_opts), "%s=%s:", e->key, e->value);
    if (strlen(aresample_swr_opts))
        aresample_swr_opts[strlen(aresample_swr_opts)-1] = '\0';
    av_opt_set(is->agraph, "aresample_swr_opts", aresample_swr_opts, 0);

    av_channel_layout_describe_bprint(&is->audio_filter_src.ch_layout, &bp);

    ret = snprintf(asrc_args, sizeof(asrc_args),
                   "sample_rate=%d:sample_fmt=%s:time_base=%d/%d:channel_layout=%s",
                   is->audio_filter_src.freq, av_get_sample_fmt_name(is->audio_filter_src.fmt),
                   1, is->audio_filter_src.freq, bp.str);

    ret = avfilter_graph_create_filter(&filt_asrc,
                                       avfilter_get_by_name("abuffer"), "ffplay_abuffer",
                                       asrc_args, NULL, is->agraph);
    if (ret < 0)
        goto end;


    ret = avfilter_graph_create_filter(&filt_asink,
                                       avfilter_get_by_name("abuffersink"), "ffplay_abuffersink",
                                       NULL, NULL, is->agraph);
    if (ret < 0)
        goto end;

    if ((ret = av_opt_set_int_list(filt_asink, "sample_fmts", sample_fmts,  AV_SAMPLE_FMT_NONE, AV_OPT_SEARCH_CHILDREN)) < 0)
        goto end;
    if ((ret = av_opt_set_int(filt_asink, "all_channel_counts", 1, AV_OPT_SEARCH_CHILDREN)) < 0)
        goto end;

    if (force_output_format) {
        av_bprint_clear(&bp);
        av_channel_layout_describe_bprint(&is->audio_tgt.ch_layout, &bp);
        sample_rates   [0] = is->audio_tgt.freq;
        if ((ret = av_opt_set_int(filt_asink, "all_channel_counts", 0, AV_OPT_SEARCH_CHILDREN)) < 0)
            goto end;
        if ((ret = av_opt_set(filt_asink, "ch_layouts", bp.str, AV_OPT_SEARCH_CHILDREN)) < 0)
            goto end;
        if ((ret = av_opt_set_int_list(filt_asink, "sample_rates"   , sample_rates   ,  -1, AV_OPT_SEARCH_CHILDREN)) < 0)
            goto end;
    }


    if ((ret = configure_filtergraph(is->agraph, afilters, filt_asrc, filt_asink)) < 0)
        goto end;

    is->in_audio_filter  = filt_asrc;
    is->out_audio_filter = filt_asink;

end:
    if (ret < 0)
        avfilter_graph_free(&is->agraph);
    av_bprint_finalize(&bp, NULL);

    return ret;
}

static int audio_thread(void *arg)
{
    VideoState *is = arg;
    AVFrame *frame = av_frame_alloc();
    Frame *af;
    int last_serial = -1;
    int reconfigure;
    int got_frame = 0;
    AVRational tb;
    int ret = 0;

    if (!frame)
        return AVERROR(ENOMEM);

    do {
        if ((got_frame = decoder_decode_frame(&is->auddec, frame, NULL)) < 0) // packet_queue_get() ���� ���
            goto the_end;

        if (got_frame) {
                tb = (AVRational){1, frame->sample_rate};

                reconfigure =
                    cmp_audio_fmts(is->audio_filter_src.fmt, is->audio_filter_src.ch_layout.nb_channels,
                                   frame->format, frame->ch_layout.nb_channels)    ||
                    av_channel_layout_compare(&is->audio_filter_src.ch_layout, &frame->ch_layout) ||
                    is->audio_filter_src.freq           != frame->sample_rate ||
                    is->auddec.pkt_serial               != last_serial;

                if (reconfigure) { // ���ʿ� �ѹ��� 1 �̸�, ���Ŀ��� 0
                    char buf1[1024], buf2[1024];
                    // buf1 = "mono", "stereo", "5.1" ���
                    av_channel_layout_describe(&is->audio_filter_src.ch_layout, buf1, sizeof(buf1));
                    av_channel_layout_describe(&frame->ch_layout, buf2, sizeof(buf2));
                    av_log(NULL, AV_LOG_DEBUG,
                           "Audio frame changed from rate:%d ch:%d fmt:%s layout:%s serial:%d to rate:%d ch:%d fmt:%s layout:%s serial:%d\n",
                           is->audio_filter_src.freq, is->audio_filter_src.ch_layout.nb_channels, av_get_sample_fmt_name(is->audio_filter_src.fmt), buf1, last_serial,
                           frame->sample_rate, frame->ch_layout.nb_channels, av_get_sample_fmt_name(frame->format), buf2, is->auddec.pkt_serial);

                    is->audio_filter_src.fmt            = frame->format;
                    ret = av_channel_layout_copy(&is->audio_filter_src.ch_layout, &frame->ch_layout);
                    if (ret < 0)
                        goto the_end;
                    is->audio_filter_src.freq           = frame->sample_rate;
                    last_serial                         = is->auddec.pkt_serial;

                    if ((ret = configure_audio_filters(is, afilters, 1)) < 0)
                        goto the_end;
                }

            if ((ret = av_buffersrc_add_frame(is->in_audio_filter, frame)) < 0)
                goto the_end;

            // frame ���ڿ� �о��(Get a frame with filtered data from sink and put it in frame)
            // ó���� ����(0) ��, ���� �� ȣ��� AVERROR(EAGAIN)(-11) �� ���� Ż��
            while ((ret = av_buffersink_get_frame_flags(is->out_audio_filter, frame/*out*/, 0)) >= 0) {
                FrameData *fd = frame->opaque_ref ? (FrameData*)frame->opaque_ref->data : NULL;
                // ���� ü���� ����ϸ� pts ������ ����� �� �����Ƿ� �Ʒ����� frame->pts * av_q2d(tb)
                tb = av_buffersink_get_time_base(is->out_audio_filter); // tb = {1/48000}
                if (!(af = frame_queue_peek_writable(&is->sampq)))
                    goto the_end;

                // ����� AVFrame::pts �� (frame->pts) tb ������ ��ȯ�� af->pts ��(Frame::pts) ����
                // ���⼭ 1024, 2048, ..., 480256(���� ����) -> 0.0213, 0.0426, ... 10.0053 �� ����� �����(1024/48000, ...)
                af->pts = (frame->pts == AV_NOPTS_VALUE) ? NAN : frame->pts * av_q2d(tb);

                av_assert0(fd && fd->pkt_pos);
                af->pos = fd ? fd->pkt_pos : -1;
                af->serial = is->auddec.pkt_serial;
                // frame->nb_samples: �� �����ӿ� ���Ե� ����� ���� ��
                af->duration = av_q2d((AVRational){frame->nb_samples/*1024*/, frame->sample_rate/*48000*/}); // 0.0213

                // frame->pts=1024, af->pts=0.0213, af->pos=497
                // frame->pts=2048, af->pts=0.0427, af->pos=801
                // ...
                // frame->pts=480256, af->pts=10.0053, af->pos=775461
                // printf("audio_thread() frame->pts=%d, af->pts=%.4f, af->pos=%d\n", frame->pts, af->pts, af->pos);

                av_frame_move_ref(af->frame, frame);
                frame_queue_push(&is->sampq);

                if (is->audioq.serial != is->auddec.pkt_serial)
                    break;
            }
            if (ret == AVERROR_EOF)
                is->auddec.finished = is->auddec.pkt_serial;
        }
    } while (ret >= 0 || ret == AVERROR(EAGAIN) || ret == AVERROR_EOF);
 the_end:
    avfilter_graph_free(&is->agraph);
    av_frame_free(&frame);
    return ret;
}
#endif

static int decoder_start(Decoder *d, int (*fn)(void *), const char *thread_name, void* arg)
{
    packet_queue_start(d->queue);
    d->decoder_tid = SDL_CreateThread(fn, thread_name, arg);
    if (!d->decoder_tid) {
        av_log(NULL, AV_LOG_ERROR, "SDL_CreateThread(): %s\n", SDL_GetError());
        return AVERROR(ENOMEM);
    }
    return 0;
}

static int video_thread(void *arg)
{
    VideoState *is = arg;
    AVFrame *frame = av_frame_alloc();
    double pts;
    double duration;
    int ret;
    AVRational tb = is->video_st->time_base;
    av_assert0(tb.num == 1 && tb.den == 90000);
    // �����󿡴� ��Ȯ�ϰ� �����ӷ���Ʈ�� ��õ��� ���� ��찡 ����
    // ���� FFmpeg�� �ܼ��� AVStream::avg_frame_rate(��Ʈ�� ��ü�� ����� ��� �����ӷ���Ʈ)
    // �Ǵ� r_frame_rate (����� ��ϵ� frame rate)�� �ƴ�,
    // av_guess_frame_rate() �Լ��� ���� ���� ��Ȯ�ϰ� �ŷڵ� ���� �����ӷ���Ʈ �����Ѵ�.

    // ���� frame_rate �� is->video_st->avg_frame_rate, r_frame_rate �� ���� ����({25/1})
    AVRational frame_rate = av_guess_frame_rate(is->ic, is->video_st, NULL);

    AVFilterGraph *graph = NULL;
    AVFilterContext *filt_out = NULL, *filt_in = NULL;
    int last_w = 0;
    int last_h = 0;
    enum AVPixelFormat last_format = -2;
    int last_serial = -1;
    int last_vfilter_idx = 0;

    if (!frame)
        return AVERROR(ENOMEM);

    for (;;) {
        // frame->pts �� best_effort_timestamp �� ���ϵ�(0, 3600, 7200, ...)
        ret = get_video_frame(is, frame);
        if (ret < 0)
            goto the_end;
        if (!ret) // is->frame_drops_early++ �� ���(ret == 0), ���� ������ ó��
            continue;

        if (   last_w != frame->width
            || last_h != frame->height
            || last_format != frame->format
            || last_serial != is->viddec.pkt_serial
            || last_vfilter_idx != is->vfilter_idx) {
            av_log(NULL, AV_LOG_DEBUG,
                   "Video frame changed from size:%dx%d format:%s serial:%d to size:%dx%d format:%s serial:%d\n",
                   last_w, last_h,
                   (const char *)av_x_if_null(av_get_pix_fmt_name(last_format), "none"), last_serial,
                   frame->width, frame->height,
                   (const char *)av_x_if_null(av_get_pix_fmt_name(frame->format), "none"), is->viddec.pkt_serial);
            avfilter_graph_free(&graph);
            graph = avfilter_graph_alloc();
            if (!graph) {
                ret = AVERROR(ENOMEM);
                goto the_end;
            }
            graph->nb_threads = filter_nbthreads;
            // vfilters_list ����
            // 1) ���̺� ���� �ܼ� ü�� ����
            // "scale=640:360,transpose=1,drawbox=x=10:y=10:w=100:h=100:color=red"
            // �� �ڵ����� �տ��� ���� ���� ����� ���� ���� �Է����� �����
            // 
            // 2) ���̺��� ����� ü�� ����
            // �Ʒ����� [in] ���� ���� �� ������ �����ϱ� ���� ����� ����� ���� ���̺���
            // buffer ������ ����� [in] �̶�� �ǹ�
            // const char *vfilters_list =
            // "buffer=width=640:height=480:pix_fmt=0:time_base=1/25[in];"
            // "[in]scale=320:240[scaled];"
            // "[scaled]drawbox=x=50:y=50:w=100:h=100:color=red@0.5[out];"
            // "[out]buffersink";
            // ���� vfilters_list == NULL ��. �׷��� ������� ���ο��� "video_size=%dx%d:pix_fmt=%d:..." ���� ������
            av_assert0(!vfilters_list);
            if ((ret = configure_video_filters(graph, is, vfilters_list ? vfilters_list[is->vfilter_idx] : NULL, frame)) < 0) {
                SDL_Event event;
                event.type = FF_QUIT_EVENT;
                event.user.data1 = is;
                SDL_PushEvent(&event);
                goto the_end;
            }
            filt_in  = is->in_video_filter;
            filt_out = is->out_video_filter;
            last_w = frame->width;
            last_h = frame->height;
            last_format = frame->format;
            last_serial = is->viddec.pkt_serial;
            last_vfilter_idx = is->vfilter_idx;
            frame_rate = av_buffersink_get_frame_rate(filt_out);
        }

        ret = av_buffersrc_add_frame(filt_in, frame); // frame �� reference �� ����� reset �ȴ�.
        if (ret < 0)
            goto the_end;

        while (ret >= 0) {
            FrameData *fd;

            is->frame_last_returned_time = av_gettime_relative() / 1000000.0;

            // sink �κ��� filtered data �� ȹ��(frame �� ä����)
            // 0, -11(AVERROR(EAGAIN)) ������ ������.
            // EAGAIN �� ���� �������� ���� �� �ִ� �������� �����Ƿ�, "�ٽ� �õ�(EAGAIN)"�϶�� �ǹ�(av_buffersrc_add_frame ȣ�� �ʿ�)
            ret = av_buffersink_get_frame_flags(filt_out, frame, 0); // ���͸��� frame �� ä����
            if (ret < 0) {
                if (ret == AVERROR_EOF)
                    is->viddec.finished = is->viddec.pkt_serial;
                ret = 0;
                break;
            }

            fd = frame->opaque_ref ? (FrameData*)frame->opaque_ref->data : NULL;

            //av_usleep(0.5 * 1000000); //skc test
            is->frame_last_filter_delay = av_gettime_relative() / 1000000.0 - is->frame_last_returned_time;
            if (fabs(is->frame_last_filter_delay) > AV_NOSYNC_THRESHOLD/*10.0*/ / 10.0)
                is->frame_last_filter_delay = 0;
            tb = av_buffersink_get_time_base(filt_out); // tb = 1/90000
            av_assert0(tb.num == 1 && tb.den == 90000);

            // frame_rate = 25/1 --> duration = 1/25 = 0.04
            duration = (frame_rate.num && frame_rate.den ? av_q2d((AVRational){frame_rate.den, frame_rate.num}) : 0);
            av_assert0(frame_rate.num == 25 && frame_rate.den == 1);
            av_assert0(duration == 0.04);
            // frame->pts = 0, 3600, 7200, 10800, ..., 896400, av_q2d(tb) = (1 / 90000)
            // pts = 0, 0.04, 0.08, 0.12, ..., 9.96 // �ʴ���
            // fd->pkt_pos �� 5147, 22825, 19967, 27078, ..., 7731740 �� ���߾���
            // ������ ���, frame->pts �� ������ avcodec_receive_frame() �� frame->best_effort_timestamp ������ ��������
            pts = (frame->pts == AV_NOPTS_VALUE) ? NAN : frame->pts * av_q2d(tb);
            av_assert0(frame->pts != AV_NOPTS_VALUE);

            av_assert0(fd);
            // duration �� 40ms ����(25fps), pts �� 0 ���� duration ��ŭ�� �̵�
            ret = queue_picture(is, frame, pts, duration, fd ? fd->pkt_pos : -1, is->viddec.pkt_serial); // ���� 0
            av_frame_unref(frame);
            if (is->videoq.serial != is->viddec.pkt_serial)
                break;
        }

        if (ret < 0)
            goto the_end;
    }
 the_end:
    avfilter_graph_free(&graph);
    av_frame_free(&frame);
    return 0;
}

#ifndef SKC_VIDEO_ONLY
static int subtitle_thread(void *arg)
{
    VideoState *is = arg;
    Frame *sp;
    int got_subtitle;
    double pts;

    for (;;) {
        if (!(sp = frame_queue_peek_writable(&is->subpq)))
            return 0;

        if ((got_subtitle = decoder_decode_frame(&is->subdec, NULL, &sp->sub)) < 0)
            break;

        pts = 0;

        if (got_subtitle && sp->sub.format == 0) {
            if (sp->sub.pts != AV_NOPTS_VALUE)
                pts = sp->sub.pts / (double)AV_TIME_BASE;
            sp->pts = pts;
            sp->serial = is->subdec.pkt_serial;
            sp->width = is->subdec.avctx->width;
            sp->height = is->subdec.avctx->height;
            sp->uploaded = 0;

            /* now we can update the picture count */
            frame_queue_push(&is->subpq);
        } else if (got_subtitle) {
            avsubtitle_free(&sp->sub);
        }
    }
    return 0;
}

/* copy samples for viewing in editor window */
static void update_sample_display(VideoState *is, short *samples, int samples_size)
{
    int size, len;

    size = samples_size / sizeof(short);
    while (size > 0) {
        len = SAMPLE_ARRAY_SIZE - is->sample_array_index;
        if (len > size)
            len = size;
        memcpy(is->sample_array + is->sample_array_index, samples, len * sizeof(short));
        samples += len;
        is->sample_array_index += len;
        if (is->sample_array_index >= SAMPLE_ARRAY_SIZE)
            is->sample_array_index = 0;
        size -= len;
    }
}

/* return the wanted number of samples to get better sync if sync_type is video
 * or external master clock */
static int synchronize_audio(VideoState *is, int nb_samples)
{
    int wanted_nb_samples = nb_samples;

    /* if not master, then we try to remove or add samples to correct the clock */
    if (get_master_sync_type(is) != AV_SYNC_AUDIO_MASTER) {
        double diff, avg_diff;
        int min_nb_samples, max_nb_samples;

        diff = get_clock(&is->audclk) - get_master_clock(is); // A-V

        // audio_diff_cum = ���� ����
        if (!isnan(diff) && fabs(diff) < AV_NOSYNC_THRESHOLD/*10.0*/) {
            // ���� ���̸� �����Ͽ� ���������� ����(��, ���ڱ� �������� �ʰ� �ε巴�� ����)
            // audio_diff_avg_coef: ���� diff ���� �󸶳� �ݿ����� �����ϴ� ���� ���
            // e.g, 
            // ���ʿ� �ѹ��� diff = 1 �̰�, �� ���� ��� 0 �̸�, audio_diff_cum = 1(1), 0.8(1), 0.64(1), 0.512(1), ...
            // ��� diff = 1 �̸�, audio_diff_cum = 1(1), 1.8(2), 2.44(3), 2.952(4), ...
            // ���� ���� ��հ�(audio_diff_cum)�� ���� diff�� ����(�����̵���� ���)
            // --> audio_diff_cum = diff + �� * prev_cum
            // �Ʒ����� ���� ��հ�(�ֱٰ� �ݿ���)�� (1 - ��) * cum
            is->audio_diff_cum = diff + is->audio_diff_avg_coef/*0.79*/ * is->audio_diff_cum;
            if (is->audio_diff_avg_count < AUDIO_DIFF_AVG_NB/*20*/) {
                /* not enough measures to have a correct estimate */
                is->audio_diff_avg_count++;
            } else {
                /* estimate the A-V difference */
                // AUDIO_DIFF_AVG_NB(20)�� ��ŭ ���� ���� audio_diff_cum �� ���� ��,
                // ������� ���� ���� ��ũ ����(diff)�� ��������� ����Ͽ� ���������� ����
                avg_diff = is->audio_diff_cum * (1.0 - is->audio_diff_avg_coef/*0.79*/);

                if (fabs(avg_diff) >= is->audio_diff_threshold/*0.04266 = 42ms*/) {
                    // ������� ������ nb_samples �� �߰�(���� ������ �Ҹ�), ������ nb_samples ���� ����
                    // ���� ���δ� avg_diff �� üũ�ϰ�, �����ÿ��� ���� ���� ���� diff �� �ݿ���
                    //av_assert0(0);
                    wanted_nb_samples = nb_samples + (int)(diff * is->audio_src.freq);
                    min_nb_samples = ((nb_samples * (100 - SAMPLE_CORRECTION_PERCENT_MAX/*10*/) / 100));
                    max_nb_samples = ((nb_samples * (100 + SAMPLE_CORRECTION_PERCENT_MAX) / 100));
                    // wanted_nb_samples �� �ּ�, �ִ�� nb_samples �� 90 ~ 110% ���� ����
                    // af->frame->nb_samples = 1024 �� ���޵ǹǷ� �׻� 0 �� �ƴ� ������� ���� ����?!
                    wanted_nb_samples = av_clip(wanted_nb_samples, min_nb_samples, max_nb_samples);
                }
                av_log(NULL, AV_LOG_TRACE, "diff=%f adiff=%f sample_diff=%d apts=%0.3f %f\n",
                        diff, avg_diff, wanted_nb_samples - nb_samples,
                        is->audio_clock, is->audio_diff_threshold);
            }
        } else {
            /* too big difference : may be initial PTS errors, so
               reset A-V filter */
            is->audio_diff_avg_count = 0;
            is->audio_diff_cum       = 0;
        }
    }

    av_assert0(wanted_nb_samples >= nb_samples); // ���� ������� �������� ������.
    return wanted_nb_samples;
}

/**
 * Decode one audio frame and return its uncompressed size.
 *
 * The processed audio frame is decoded, converted if required, and
 * stored in is->audio_buf, with size in bytes given by the return
 * value.
 */
static int audio_decode_frame(VideoState *is)
{
    int data_size, resampled_data_size;
    av_unused double audio_clock0;
    int wanted_nb_samples;
    Frame *af;

    if (is->paused)
        return -1;

    do {
#if defined(_WIN32)
        while (frame_queue_nb_remaining(&is->sampq) == 0) {
            // ����� �ݹ� �ѹ� ó���� �ð�(is->audio_hw_buf_size(8192) / is->audio_tgt.bytes_per_sec(192000) = 42.6ms)�� ������
            // �ʰ��ϰ� �Ǹ� -1 ���� ����(caller ���� ������ �����)
            // (�ʴ� ó���ϴ� ũ��(bytes_per_sec)�� audio_hw_buf_size ũ���� �ҿ� �ð��� �����)
            if ((av_gettime_relative() - audio_callback_time) > 1000000LL * is->audio_hw_buf_size / is->audio_tgt.bytes_per_sec / 2)
                return -1;
            av_usleep (1000);
        }
#endif
        if (!(af = frame_queue_peek_readable(&is->sampq)))
            return -1;
        frame_queue_next(&is->sampq);
    } while (af->serial != is->audioq.serial);

    data_size = av_samples_get_buffer_size(NULL, af->frame->ch_layout.nb_channels/*2*/,
                                           af->frame->nb_samples/*1024*/,
                                           af->frame->format/*2*/, 1); // �ܼ� ���
    av_assert0(data_size == 4096);

    wanted_nb_samples = synchronize_audio(is, af->frame->nb_samples);

    if (af->frame->format        != is->audio_src.fmt            ||
        av_channel_layout_compare(&af->frame->ch_layout, &is->audio_src.ch_layout) ||
        af->frame->sample_rate   != is->audio_src.freq           ||
        (wanted_nb_samples       != af->frame->nb_samples && !is->swr_ctx)) {
        // wanted_nb_samples = 1126, af->frame->nb_samples = 1024 ó�� �ٸ� ��찡 ������
        //av_assert0(0);
        int ret;
        swr_free(&is->swr_ctx);
        ret = swr_alloc_set_opts2(&is->swr_ctx,
                            &is->audio_tgt.ch_layout, is->audio_tgt.fmt, is->audio_tgt.freq,
                            &af->frame->ch_layout, af->frame->format, af->frame->sample_rate,
                            0, NULL);
        if (ret < 0 || swr_init(is->swr_ctx) < 0) {
            av_log(NULL, AV_LOG_ERROR,
                   "Cannot create sample rate converter for conversion of %d Hz %s %d channels to %d Hz %s %d channels!\n",
                    af->frame->sample_rate, av_get_sample_fmt_name(af->frame->format), af->frame->ch_layout.nb_channels,
                    is->audio_tgt.freq, av_get_sample_fmt_name(is->audio_tgt.fmt), is->audio_tgt.ch_layout.nb_channels);
            swr_free(&is->swr_ctx);
            return -1;
        }
        if (av_channel_layout_copy(&is->audio_src.ch_layout, &af->frame->ch_layout) < 0)
            return -1;
        is->audio_src.freq = af->frame->sample_rate;
        is->audio_src.fmt = af->frame->format;
    }

    if (is->swr_ctx) {
        av_assert0(0);
        const uint8_t **in = (const uint8_t **)af->frame->extended_data;
        uint8_t **out = &is->audio_buf1;
        // �����ø� �� ���� ������ �����ϱ� ���� out_count �� 256 ������ ��
        int out_count = (int64_t)wanted_nb_samples * is->audio_tgt.freq / af->frame->sample_rate + 256;
        int out_size  = av_samples_get_buffer_size(NULL, is->audio_tgt.ch_layout.nb_channels, out_count, is->audio_tgt.fmt, 0);
        int len2;
        if (out_size < 0) {
            av_log(NULL, AV_LOG_ERROR, "av_samples_get_buffer_size() failed\n");
            return -1;
        }
        if (wanted_nb_samples != af->frame->nb_samples) {
            // ������� ������ ����ȭ���� ���� ���, sample_delta��ŭ ������ �����Ͽ� compensation_distance ���� ������ ��ũ�� ���ߴ� ����
            // swr_set_compensation(struct SwrContext *s, int sample_delta, int compensation_distance);
            // sample_delta: ������ ���� ����, compensation_distance: ������ �Ÿ�(������ ��)
            //av_assert0(0); // wanted_nb_samples == 1126, af->frame->nb_samples == 1024 �� ��찡 ������
            if (swr_set_compensation(is->swr_ctx, (wanted_nb_samples - af->frame->nb_samples) * is->audio_tgt.freq / af->frame->sample_rate,
                                        wanted_nb_samples * is->audio_tgt.freq / af->frame->sample_rate) < 0) {
                av_log(NULL, AV_LOG_ERROR, "swr_set_compensation() failed\n");
                return -1;
            }
        }
        av_fast_malloc(&is->audio_buf1, &is->audio_buf1_size/*out ���� �Ҵ�� ũ��*/, out_size);
        if (!is->audio_buf1)
            return AVERROR(ENOMEM);
        // �����ø� ����! in_count ��ŭ �Է� ������ �޾Ƽ�, out_count ��ŭ ��� ������ ����� ��
        // ������ swr_set_compensation() �� ȣ��������, compensation_distance ���� ������ �ε巴�� �����
        len2 = swr_convert(is->swr_ctx, out, out_count, in, af->frame->nb_samples); // is->audio_buf1(out) �� ��ȯ
        if (len2 < 0) {
            av_log(NULL, AV_LOG_ERROR, "swr_convert() failed\n");
            return -1;
        }
        if (len2 == out_count) { // out_count ���ñ��� �ִ��� ä��
            // len2 == out_count�� �� ���� �� �� �� swr ���ο� �� ���� ���� ���� ����. �� ���, swr_init() �� ������ �ʱ�ȭ
            av_log(NULL, AV_LOG_WARNING, "audio buffer is probably too small\n");
            if (swr_init(is->swr_ctx) < 0)
                swr_free(&is->swr_ctx);
        }
        is->audio_buf = is->audio_buf1;
        resampled_data_size = len2 * is->audio_tgt.ch_layout.nb_channels * av_get_bytes_per_sample(is->audio_tgt.fmt);
    } else {
        is->audio_buf = af->frame->data[0]; // af->frame->data �迭�� ũ�� = 8
        resampled_data_size = data_size;
    }

    audio_clock0 = is->audio_clock; // ���ʿ� 0
    /* update the audio clock with the pts */
    if (!isnan(af->pts))
        // af->pts(����� �������� ���� �ð�)�� ������ ����ð��� ���� �� �ð��� is->audio_clock �� ����
        // nb_samples(1024) / sample_rate(48000) = 0.021(����)
        // af->pts = 0.021, 0.043, 0.064, ..., 10.0053
        // is->audio_clock = af->pts + 0.021
        // = 0.042(0.021 + 0.021), 0.064(0.043 + 0.021), ..., 10.0266(10.0053 + 0.0213) ������
        // af->pts �� 0 �� �ƴ϶� *** 0.0213 ���� ���� ***�ϰ� ����(������ 0 ���� ����), 0.0213 �� ����� ����
    
        // ���� ��Ʈ���� ��ü ���̴� v: 10.0, a: 10.0266 (read_thread() ���� st->duration �� Ȯ�� ����)
        // ���۽� ���ڵ� ������(pre-roll) ������(�ʱ� "warm-up" ����)�� ��ŵ�� ���� �ִٰ� ��
        // (�׷��� 0 �� �ƴ϶� 0.0213 ���� ����?!)
        is->audio_clock = af->pts + (double) af->frame->nb_samples / af->frame->sample_rate;
    else
        is->audio_clock = NAN;
    is->audio_clock_serial = af->serial;

    // delay �� clock(is->audio_clock) - clock0(last_clock )
    // --> 0.043(0.021 + 0.021 - 0), 0.021(0.043 + 0.021 - 0.043), 0.021(0.064 + 0.021 - 0.043), ...

    //audio: delay = 0.043 clock = 0.043 clock0 = 0.000 // ó������ delay = 0.043 �� ����, ���� 0.021
    //audio: delay = 0.021 clock = 0.064 clock0 = 0.043
    //audio: delay = 0.021 clock = 0.085 clock0 = 0.064
    // ...
    //audio: delay = 0.021 clock = 10.005 clock0 = 9.984 // ������ pts �� 10.0053 ����
    //audio: delay = 0.021 clock = 10.027 clock0 = 10.005 // ��� �� �ð��� 0.021 ���ؼ� 10.027 �� ��Ʈ���� duration ���� ��ġ��
#if 0 //def DEBUG
    {
        static double last_clock;
        av_assert0(last_clock == audio_clock0);
        printf("audio: delay=%0.3f clock=%0.3f clock0=%0.3f\n",
               is->audio_clock - last_clock,
               is->audio_clock, audio_clock0);
        last_clock = is->audio_clock;
    }
#endif
    return resampled_data_size;
}

/* prepare a new audio buffer */
static void sdl_audio_callback(void *opaque, Uint8 *stream, int len)
{
    VideoState *is = opaque;
    int audio_size, len1;

    audio_callback_time = av_gettime_relative();

    while (len > 0) {
        if (is->audio_buf_index >= is->audio_buf_size) { // ÷�� �Ѵ� 0
            // audio_decode_frame() ���� is->audio_clock = af->pts + duration; �� ��� �Ϸ� ������ �����ϰ�
            // �����ø��� resampled_data_size �� ����
            // ������ sdl_audio_callback() ������ set_clock_at(&audclk, audio_clock - ��������, serial, time);
           audio_size = audio_decode_frame(is); // ����� �����ø� ��, is->audio_buf �� �����ȴ�.
           //av_assert0(audio_size == 4096); // ����� -1 �� ����
           if (audio_size < 0) {
                /* if error, just output silence */
               is->audio_buf = NULL;
               is->audio_buf_size = SDL_AUDIO_MIN_BUFFER_SIZE/*512*/ / is->audio_tgt.frame_size * is->audio_tgt.frame_size; // 512
           } else {
               if (is->show_mode != SHOW_MODE_VIDEO)
                   update_sample_display(is, (int16_t *)is->audio_buf, audio_size); // is->audio_buf �� is->sample_array �� ����
               is->audio_buf_size = audio_size;
           }
           is->audio_buf_index = 0;
        }
        len1 = is->audio_buf_size - is->audio_buf_index;
        if (len1 > len)
            len1 = len;
        if (!is->muted && is->audio_buf && is->audio_volume == SDL_MIX_MAXVOLUME)
            memcpy(stream, (uint8_t *)is->audio_buf + is->audio_buf_index, len1);
        else {
            memset(stream, 0, len1);
            if (!is->muted && is->audio_buf)
                SDL_MixAudioFormat(stream, (uint8_t *)is->audio_buf + is->audio_buf_index, AUDIO_S16SYS, len1, is->audio_volume);
        }
        len -= len1; // ���� len = 8192, len1 = 4096 �� 2�� ���� ��
        stream += len1;
        is->audio_buf_index += len1; // ���(write)�� ��ŭ ����
    }
    is->audio_write_buf_size = is->audio_buf_size - is->audio_buf_index; // ���� ���� ũ��
    /* Let's assume the audio driver that is used by SDL has two periods. */
    if (!isnan(is->audio_clock)) {
        // ��µ��� ���� ���� �����͸� ����� ����� Ŭ���� ���� ���� ��� ���� ��ġ�� ����
        // ����� �����Ͱ� ���� ���۸��ɼ���(is->audio_write_buf_size �� Ŭ����)
        // ����� Ŭ��(pts ����)�� ������ ���(���ŷ� �̵�) ������ ����ȭ�ǵ��� ����
        // -> ����� ������ ����(buffering delay)�� ����Ͽ� ����� Ŭ��(is->audclk)�� ����

        // audio_clock �� ���ݱ��� ���� ��� ����� �����Ͱ� ��� �Ϸ�Ǿ��� �� ������ �ð�
        // *** ���� ���������� ���ۿ� ���� �縸ŭ ����� ��Ȯ�� ���� ��� �ð�(audclk)�� ���� �� �ִ�!!!

        // audio_clock�� '������ �� ����'�� �������� ��ƾ�, ���� ���۷��� �� �� ���� ��� ���� ��ġ�� ��Ȯ�� ����� �� �ִ�.

        /*
        | <------������ A------> | <------������ B------> | <------������ C------> |
        0s                     0.02s                    0.04s                  0.06s

            ������ A : 0s ~0.02s ���
            ������ B : 0.02s ~0.04s ���
            ������ C : 0.04s ~0.06s ���

            is->audio_clock = 0.06��(������ C���� ���� �Ϸ��� ����)

            �� ���¿���: ���ۿ� 0.01�ʾ�ġ ���Ҵٸ�
            ���� ���� ������� 0.06�� - 0.01�� = 0.05�ʸ� ��� ��
        */
        
        av_assert0(is->audio_write_buf_size == 0);
        static int skc_cnt = 0;
        double skc_delay = (double)(2 * is->audio_hw_buf_size/*8192*/ + is->audio_write_buf_size/*0*/) / is->audio_tgt.bytes_per_sec/*192000*/; // 0.0853
        //printf("AUDIO:: sdl_audio_callback() set_clock_at(%d) audio_clock=%.3f, pts=%.3f, time=%.3f\n",
        //    skc_cnt++, is->audio_clock, is->audio_clock - skc_delay, audio_callback_time / 1000000.0);

        // ��µ��� ���� ���� �����͸� ����� ����� Ŭ���� ���� ���� ��� ���� ��ġ�� ����
        // audio_callback_time ������ ����� Ŭ��(audclk)�� ������ �� �ð� - ���� ��¾ȵ� ���� ���� �ð� ��ġ�� ������̾�� ��?!
        set_clock_at(&is->audclk,
            is->audio_clock - (double)(2 * is->audio_hw_buf_size/*8192*/ + is->audio_write_buf_size/*0*/) / is->audio_tgt.bytes_per_sec/*192000*/, // - �޺κ��� �׻� 0.0853
            is->audio_clock_serial, audio_callback_time / 1000000.0);

        // �Ʒ� �α׿��� pts �� audclk �� ����Ǵ� ��
        //AUDIO::sdl_audio_callback() set_clock_at(0) audio_clock = 0.064, pts = -0.021, time = 1746734660.813
        //VIDEO::update_video_pts(0) pts = 0.000000
        //AUDIO::sdl_audio_callback() set_clock_at(1) audio_clock = 0.107, pts =  0.021, time = 1746734660.853
        //VIDEO::update_video_pts(1) pts = 0.040000
        //AUDIO::sdl_audio_callback() set_clock_at(2) audio_clock = 0.149, pts =  0.064, time = 1746734660.893
        //VIDEO::update_video_pts(2) pts = 0.080000
        // ...
        //VIDEO:: update_video_pts(248) pts=9.920000
        //AUDIO::sdl_audio_callback() set_clock_at(233) audio_clock = 10.005, pts = 9.920, time = 1746734670.754
        //VIDEO::update_video_pts(249) pts = 9.960000
        //AUDIO::sdl_audio_callback() set_clock_at(234) audio_clock = 10.027, pts = 9.941, time = 1746734670.794
        sync_clock_to_slave(&is->extclk, &is->audclk);
    }
}

static int audio_open(void *opaque, AVChannelLayout *wanted_channel_layout, int wanted_sample_rate, struct AudioParams *audio_hw_params)
{
    SDL_AudioSpec wanted_spec, spec;
    const char *env;
    static const int next_nb_channels[] = {0, 0, 1, 6, 2, 6, 4, 6};
    static const int next_sample_rates[] = {0, 44100, 48000, 96000, 192000};
    int next_sample_rate_idx = FF_ARRAY_ELEMS(next_sample_rates) - 1;
    int wanted_nb_channels = wanted_channel_layout->nb_channels;

    env = SDL_getenv("SDL_AUDIO_CHANNELS");
    if (env) {
        wanted_nb_channels = atoi(env);
        av_channel_layout_uninit(wanted_channel_layout);
        av_channel_layout_default(wanted_channel_layout, wanted_nb_channels);
    }
    if (wanted_channel_layout->order != AV_CHANNEL_ORDER_NATIVE) {
        av_channel_layout_uninit(wanted_channel_layout);
        av_channel_layout_default(wanted_channel_layout, wanted_nb_channels);
    }
    wanted_nb_channels = wanted_channel_layout->nb_channels;
    wanted_spec.channels = wanted_nb_channels;
    wanted_spec.freq = wanted_sample_rate;
    if (wanted_spec.freq <= 0 || wanted_spec.channels <= 0) {
        av_log(NULL, AV_LOG_ERROR, "Invalid sample rate or channel count!\n");
        return -1;
    }
    while (next_sample_rate_idx && next_sample_rates[next_sample_rate_idx] >= wanted_spec.freq)
        next_sample_rate_idx--;
    wanted_spec.format = AUDIO_S16SYS;
    wanted_spec.silence = 0;
    // auto skc_freq = wanted_spec.freq / SDL_AUDIO_MAX_CALLBACKS_PER_SEC;
    // auto skc_log = av_log2(skc_freq);
    // wanted_spec.freq/*48000*/ / SDL_AUDIO_MAX_CALLBACKS_PER_SEC/*30*/) = 1600
    // av_log2(1600) = 10.64385 = 10
    wanted_spec.samples = FFMAX(SDL_AUDIO_MIN_BUFFER_SIZE, 2 << av_log2(wanted_spec.freq / SDL_AUDIO_MAX_CALLBACKS_PER_SEC));
    wanted_spec.callback = sdl_audio_callback;
    wanted_spec.userdata = opaque;
    while (!(audio_dev = SDL_OpenAudioDevice(NULL, 0, &wanted_spec, &spec, SDL_AUDIO_ALLOW_FREQUENCY_CHANGE | SDL_AUDIO_ALLOW_CHANNELS_CHANGE))) {
        av_log(NULL, AV_LOG_WARNING, "SDL_OpenAudio (%d channels, %d Hz): %s\n",
               wanted_spec.channels, wanted_spec.freq, SDL_GetError());
        wanted_spec.channels = next_nb_channels[FFMIN(7, wanted_spec.channels)];
        if (!wanted_spec.channels) {
            wanted_spec.freq = next_sample_rates[next_sample_rate_idx--];
            wanted_spec.channels = wanted_nb_channels;
            if (!wanted_spec.freq) {
                av_log(NULL, AV_LOG_ERROR,
                       "No more combinations to try, audio open failed\n");
                return -1;
            }
        }
        av_channel_layout_default(wanted_channel_layout, wanted_spec.channels);
    }
    if (spec.format != AUDIO_S16SYS) {
        av_log(NULL, AV_LOG_ERROR,
               "SDL advised audio format %d is not supported!\n", spec.format);
        return -1;
    }
    if (spec.channels != wanted_spec.channels) {
        av_channel_layout_uninit(wanted_channel_layout);
        av_channel_layout_default(wanted_channel_layout, spec.channels);
        if (wanted_channel_layout->order != AV_CHANNEL_ORDER_NATIVE) {
            av_log(NULL, AV_LOG_ERROR,
                   "SDL advised channel count %d is not supported!\n", spec.channels);
            return -1;
        }
    }

    audio_hw_params->fmt = AV_SAMPLE_FMT_S16;
    audio_hw_params->freq = spec.freq;
    if (av_channel_layout_copy(&audio_hw_params->ch_layout, wanted_channel_layout) < 0)
        return -1;
    audio_hw_params->frame_size = av_samples_get_buffer_size(NULL, audio_hw_params->ch_layout.nb_channels, 1, audio_hw_params->fmt, 1);
    audio_hw_params->bytes_per_sec = av_samples_get_buffer_size(NULL, audio_hw_params->ch_layout.nb_channels, audio_hw_params->freq, audio_hw_params->fmt, 1);
    if (audio_hw_params->bytes_per_sec <= 0 || audio_hw_params->frame_size <= 0) {
        av_log(NULL, AV_LOG_ERROR, "av_samples_get_buffer_size failed\n");
        return -1;
    }
    return spec.size;
}
#endif

static int create_hwaccel(AVBufferRef **device_ctx)
{
    enum AVHWDeviceType type;
    int ret;
    AVBufferRef *vk_dev;

    *device_ctx = NULL;

    if (!hwaccel) // NULL
        return 0;
    
    av_assert0(0);
    type = av_hwdevice_find_type_by_name(hwaccel);
    if (type == AV_HWDEVICE_TYPE_NONE)
        return AVERROR(ENOTSUP);

    if (!vk_renderer) {
        av_log(NULL, AV_LOG_ERROR, "Vulkan renderer is not available\n");
        return AVERROR(ENOTSUP);
    }

    ret = vk_renderer_get_hw_dev(vk_renderer, &vk_dev);
    if (ret < 0)
        return ret;

    ret = av_hwdevice_ctx_create_derived(device_ctx, type, vk_dev, 0);
    if (!ret)
        return 0;

    if (ret != AVERROR(ENOSYS))
        return ret;

    av_log(NULL, AV_LOG_WARNING, "Derive %s from vulkan not supported.\n", hwaccel);
    ret = av_hwdevice_ctx_create(device_ctx, type, NULL, NULL, 0);
    return ret;
}

/* open a given stream. Return 0 if OK */
static int stream_component_open(VideoState *is, int stream_index)
{
    AVFormatContext *ic = is->ic;
    AVCodecContext *avctx;
    const AVCodec *codec;
    const char *forced_codec_name = NULL;
    AVDictionary *opts = NULL;
    int sample_rate;
    AVChannelLayout ch_layout = { 0 };
    int ret = 0;
    int stream_lowres = lowres;

    if (stream_index < 0 || stream_index >= ic->nb_streams)
        return -1;

    avctx = avcodec_alloc_context3(NULL);
    if (!avctx)
        return AVERROR(ENOMEM);

    // avcodec_parameters_to_context() ȣ��� avctx->sample_rate, sample_fmt ���� ä����
    // sample_rate: a = 48000, v = 0
    // sample_fmt: a=AV_SAMPLE_FMT_FLTP (8), v=AV_SAMPLE_FMT_NONE (-1)
    ret = avcodec_parameters_to_context(avctx, ic->streams[stream_index]->codecpar);
    if (ret < 0)
        goto fail;
    avctx->pkt_timebase = ic->streams[stream_index]->time_base;

    codec = avcodec_find_decoder(avctx->codec_id);

    switch(avctx->codec_type){
#ifndef SKC_VIDEO_ONLY
        case AVMEDIA_TYPE_AUDIO   : is->last_audio_stream    = stream_index; forced_codec_name =    audio_codec_name; break;
        case AVMEDIA_TYPE_SUBTITLE: is->last_subtitle_stream = stream_index; forced_codec_name = subtitle_codec_name; break;
#endif
        case AVMEDIA_TYPE_VIDEO   : is->last_video_stream    = stream_index; forced_codec_name =    video_codec_name; break;
    }
    if (forced_codec_name)
        codec = avcodec_find_decoder_by_name(forced_codec_name);
    if (!codec) {
        if (forced_codec_name) av_log(NULL, AV_LOG_WARNING,
                                      "No codec could be found with name '%s'\n", forced_codec_name);
        else                   av_log(NULL, AV_LOG_WARNING,
                                      "No decoder could be found for codec %s\n", avcodec_get_name(avctx->codec_id));
        ret = AVERROR(EINVAL);
        goto fail;
    }

    avctx->codec_id = codec->id;
    if (stream_lowres > codec->max_lowres) {
        av_log(avctx, AV_LOG_WARNING, "The maximum value for lowres supported by the decoder is %d\n",
                codec->max_lowres);
        stream_lowres = codec->max_lowres;
    }
    avctx->lowres = stream_lowres;

    if (fast)
        avctx->flags2 |= AV_CODEC_FLAG2_FAST;

    ret = filter_codec_opts(codec_opts, avctx->codec_id, ic,
                            ic->streams[stream_index], codec, &opts, NULL);
    if (ret < 0)
        goto fail;

    if (!av_dict_get(opts, "threads", NULL, 0))
        av_dict_set(&opts, "threads", "auto", 0);
    if (stream_lowres)
        av_dict_set_int(&opts, "lowres", stream_lowres, 0);

    av_dict_set(&opts, "flags", "+copy_opaque", AV_DICT_MULTIKEY);

    if (avctx->codec_type == AVMEDIA_TYPE_VIDEO) {
        ret = create_hwaccel(&avctx->hw_device_ctx);
        if (ret < 0)
            goto fail;
    }

    if ((ret = avcodec_open2(avctx, codec, &opts)) < 0) {
        goto fail;
    }
    ret = check_avoptions(opts);
    if (ret < 0)
        goto fail;

    is->eof = 0;
    ic->streams[stream_index]->discard = AVDISCARD_DEFAULT;
    switch (avctx->codec_type) {
#ifndef SKC_VIDEO_ONLY
    case AVMEDIA_TYPE_AUDIO:
        {
            AVFilterContext *sink;

            is->audio_filter_src.freq           = avctx->sample_rate;
            ret = av_channel_layout_copy(&is->audio_filter_src.ch_layout, &avctx->ch_layout);
            if (ret < 0)
                goto fail;
            is->audio_filter_src.fmt            = avctx->sample_fmt;
            av_assert0(!afilters);
            if ((ret = configure_audio_filters(is, afilters, 0)) < 0)
                goto fail;
            sink = is->out_audio_filter; // ���� configure_audio_filters() �Լ����� ä����
            sample_rate    = av_buffersink_get_sample_rate(sink);
            ret = av_buffersink_get_ch_layout(sink, &ch_layout);
            if (ret < 0)
                goto fail;
        }

        /* prepare audio output */
        // ���ϴ� ���(ch, sample rate)�� is->audio_tgt �� ȹ��
        // audio_open() �Լ����� ������ is->audio_tgt->fmt = AV_SAMPLE_FMT_S16 (1) �� ��������
        if ((ret = audio_open(is, &ch_layout, sample_rate, &is->audio_tgt)) < 0)
            goto fail;
        is->audio_hw_buf_size = ret;
        is->audio_src = is->audio_tgt;
        is->audio_buf_size  = 0;
        is->audio_buf_index = 0;

        /* init averaging filter */
        // log(0.01) / 20 = -0.23, exp(-0.23) = 0.79432
        is->audio_diff_avg_coef  = exp(log(0.01) / AUDIO_DIFF_AVG_NB); // log �� �ڿ��α� ln ��
        is->audio_diff_avg_count = 0;
        /* since we do not have a precise anough audio FIFO fullness,
           we correct audio sync only if larger than this threshold */
        // 8192 / 192000 = 0.04266
        is->audio_diff_threshold = (double)(is->audio_hw_buf_size) / is->audio_tgt.bytes_per_sec;

        is->audio_stream = stream_index;
        is->audio_st = ic->streams[stream_index];

        if ((ret = decoder_init(&is->auddec, avctx, &is->audioq, is->continue_read_thread)) < 0)
            goto fail;
        if (is->ic->iformat->flags & AVFMT_NOTIMESTAMPS) {
            is->auddec.start_pts = is->audio_st->start_time;
            is->auddec.start_pts_tb = is->audio_st->time_base;
        }
        if ((ret = decoder_start(&is->auddec, audio_thread, "audio_decoder", is)) < 0)
            goto out;
        SDL_PauseAudioDevice(audio_dev, 0);
        break;
#endif
    case AVMEDIA_TYPE_VIDEO:
        is->video_stream = stream_index;
        is->video_st = ic->streams[stream_index];

        if ((ret = decoder_init(&is->viddec, avctx, &is->videoq, is->continue_read_thread)) < 0)
            goto fail;
        if ((ret = decoder_start(&is->viddec, video_thread, "video_decoder", is)) < 0)
            goto out;
        is->queue_attachments_req = 1;
        break;
#ifndef SKC_VIDEO_ONLY
    case AVMEDIA_TYPE_SUBTITLE:
        is->subtitle_stream = stream_index;
        is->subtitle_st = ic->streams[stream_index];

        if ((ret = decoder_init(&is->subdec, avctx, &is->subtitleq, is->continue_read_thread)) < 0)
            goto fail;
        if ((ret = decoder_start(&is->subdec, subtitle_thread, "subtitle_decoder", is)) < 0)
            goto out;
        break;
#endif
    default:
        break;
    }
    goto out;

fail:
    avcodec_free_context(&avctx);
out:
    av_channel_layout_uninit(&ch_layout);
    av_dict_free(&opts);

    return ret;
}

static int decode_interrupt_cb(void *ctx)
{
    VideoState *is = ctx;
    return is->abort_request;
}

// ť�� ������ ���� ��� ���� �ְ�, ��� �ð��� 1�� �̻� Ȯ���ߴ��� ���θ� ����
static int stream_has_enough_packets(AVStream *st, int stream_id, PacketQueue *queue) {
    return stream_id < 0 ||
           queue->abort_request ||
           (st->disposition & AV_DISPOSITION_ATTACHED_PIC/*1024*/) || // ÷�ε� ���� ��Ʈ���̸�
           // �ּ� ������ �̻� �����ư�, ������ ���� �ð��� 0 �̰ų�
           // 1.0�� �̻��̸� ����ϴ� �� �Լ��� ȣ���ϴ� read_thread() ���� �������!
           queue->nb_packets > MIN_FRAMES/*25*/ && (!queue->duration || av_q2d(st->time_base) * queue->duration > 1.0);
}

static int is_realtime(AVFormatContext *s)
{
    if(   !strcmp(s->iformat->name, "rtp")
       || !strcmp(s->iformat->name, "rtsp")
       || !strcmp(s->iformat->name, "sdp")
    )
        return 1;

    if(s->pb && (   !strncmp(s->url, "rtp:", 4)
                 || !strncmp(s->url, "udp:", 4)
                )
    )
        return 1;
    return 0;
}

/* this thread gets the stream from the disk or the network */
static int read_thread(void *arg)
{
    VideoState *is = arg;
    AVFormatContext *ic = NULL;
    int err, i, ret;
    int st_index[AVMEDIA_TYPE_NB];
    AVPacket *pkt = NULL;
    int64_t stream_start_time;
    int pkt_in_play_range = 0;
    const AVDictionaryEntry *t;
    SDL_mutex *wait_mutex = SDL_CreateMutex();
    int scan_all_pmts_set = 0;
    int64_t pkt_ts;

    if (!wait_mutex) {
        av_log(NULL, AV_LOG_FATAL, "SDL_CreateMutex(): %s\n", SDL_GetError());
        ret = AVERROR(ENOMEM);
        goto fail;
    }

    memset(st_index, -1, sizeof(st_index));
    is->eof = 0;

    pkt = av_packet_alloc();
    if (!pkt) {
        av_log(NULL, AV_LOG_FATAL, "Could not allocate packet.\n");
        ret = AVERROR(ENOMEM);
        goto fail;
    }
    ic = avformat_alloc_context();
    if (!ic) {
        av_log(NULL, AV_LOG_FATAL, "Could not allocate context.\n");
        ret = AVERROR(ENOMEM);
        goto fail;
    }
    // blocking functions �� ����Ǵ� ���� ������� ���θ� ����(���� ȣ���)
    ic->interrupt_callback.callback = decode_interrupt_cb;
    ic->interrupt_callback.opaque = is;
    if (!av_dict_get(format_opts, "scan_all_pmts", NULL, AV_DICT_MATCH_CASE)) {
        // ���ʿ� format_opts == NULL �̶� ������
        av_dict_set(&format_opts, "scan_all_pmts", "1", AV_DICT_DONT_OVERWRITE);
        scan_all_pmts_set = 1;
    }
    // avformat_open_input() ���� ���� ic->streams[0]->time_base �� ä����(1/90000)
    // ������� ic->streams[1]->time_base �� ä����(1/48000)
    // ic->duration �� AV_NOPTS_VALUE ��
    err = avformat_open_input(&ic, is->filename, is->iformat/*NULL*/, &format_opts/*scan_all_pmts �Ѱ��� ���Ե�*/);
    if (err < 0) {
        print_error(is->filename, err);
        ret = -1;
        goto fail;
    }
    if (scan_all_pmts_set)
        av_dict_set(&format_opts, "scan_all_pmts", NULL, AV_DICT_MATCH_CASE);
    remove_avoptions(&format_opts, codec_opts); // �Ѵ� NULL

    ret = check_avoptions(format_opts); // ret = NULL
    if (ret < 0)
        goto fail;
    is->ic = ic;

    if (genpts)
        ic->flags |= AVFMT_FLAG_GENPTS;

    if (find_stream_info) {
        AVDictionary **opts;
        int orig_nb_streams = ic->nb_streams;

        // �Ʒ� �Լ����� filter_codec_opts() �� ȣ���ؼ� �ʿ��� �ڵ� �ɼǸ� ���޵ǵ��� ���͸���(�߸��� �ɼ� ����)
        // codec_opts = NULL �� output �� opts �� NULL
        err = setup_find_stream_info_opts(ic, codec_opts, &opts);
        if (err < 0) {
            av_log(NULL, AV_LOG_ERROR,
                   "Error setting up avformat_find_stream_info() options\n");
            ret = err;
            goto fail;
        }

        // ic �� start_time(0), duration(10026667), bit_rate(629116) ���� ä����
        err = avformat_find_stream_info(ic, opts);

        for (i = 0; i < orig_nb_streams; i++)
            av_dict_free(&opts[i]);
        av_freep(&opts);

        if (err < 0) {
            av_log(NULL, AV_LOG_WARNING,
                   "%s: could not find codec parameters\n", is->filename);
            ret = -1;
            goto fail;
        }
    }

    // ic->pb �� �� avformat_open_input() ���� �ڵ� ������(NULL �� �ƴ�)
    // ic->pb �� ���� �� ��Ʈ��ũ I/O�� �����ϴ� AVIOContext* Ÿ����
    if (ic->pb)
        ic->pb->eof_reached = 0; // FIXME hack, ffplay maybe should not use avio_feof() to test for the end

    // ���� seek_by_bytes = -1 (auto) �̹Ƿ� ����
    // AVFMT_NO_BYTE_SEEK = true, AVFMT_TS_DISCONT = false ���� seek_by_bytes = 0 (off) ���� ������
    if (seek_by_bytes < 0)
        seek_by_bytes = !(ic->iformat->flags & AVFMT_NO_BYTE_SEEK) &&
                        !!(ic->iformat->flags & AVFMT_TS_DISCONT) &&
                        strcmp("ogg", ic->iformat->name);

    is->max_frame_duration = (ic->iformat->flags & AVFMT_TS_DISCONT) ? 10.0 : 3600.0; // 3600.0 ������

    if (!window_title && (t = av_dict_get(ic->metadata, "title", NULL, 0)))
        window_title = av_asprintf("%s - %s", t->value, input_filename);

    // ���۽� �������� start_time �� ic->start_time ���� ���ؼ� seek
    /* if seeking requested, we execute it */
    if (start_time != AV_NOPTS_VALUE) {
        av_assert0(0);
        int64_t timestamp;

        timestamp = start_time;
        /* add the stream start time */
        if (ic->start_time != AV_NOPTS_VALUE)
            timestamp += ic->start_time;
        ret = avformat_seek_file(ic, -1, INT64_MIN, timestamp, INT64_MAX, 0);
        if (ret < 0) {
            av_log(NULL, AV_LOG_WARNING, "%s: could not seek to position %0.3f\n",
                    is->filename, (double)timestamp / AV_TIME_BASE);
        }
    }

    is->realtime = is_realtime(ic);

    if (show_status)
        av_dump_format(ic, 0, is->filename, 0);

    for (i = 0; i < ic->nb_streams; i++) {
        AVStream *st = ic->streams[i];
        enum AVMediaType type = st->codecpar->codec_type;
        st->discard = AVDISCARD_ALL;
        double skc_stream_duration_sec = st->duration * av_q2d(st->time_base); // i = 0 -> v: 10.0, i = 1 -> a: 10.0266
        auto skc_st = st->start_time; // v: 0, a: 0
        AVRational skc_tb = st->time_base; // v: 1/90000, a: 1/48000
        if (type >= 0 && wanted_stream_spec[type] && st_index[type] == -1) {
            av_assert0(0);
            // ��Ʈ�� ���� ���ڿ�(��: "v:0" �Ǵ� "a")�� �־����� ��, AVStream�� �� ���ǿ� �����ϴ��� �˻�
            if (avformat_match_stream_specifier(ic, st, wanted_stream_spec[type]) > 0) {
                av_assert0(0);
                st_index[type] = i;
            }
        }
    }
    for (i = 0; i < AVMEDIA_TYPE_NB; i++) {
        if (wanted_stream_spec[i] && st_index[i] == -1) {
            av_log(NULL, AV_LOG_ERROR, "Stream specifier %s does not match any %s stream\n", wanted_stream_spec[i], av_get_media_type_string(i));
            av_assert0(0);
            st_index[i] = INT_MAX;
        }
    }

    if (!video_disable)
        // �־��� �̵�� Ÿ�Կ� ���� ���� ������(�켱������ ����) ��Ʈ���� �ڵ����� ã���ִ� �Լ�
        st_index[AVMEDIA_TYPE_VIDEO] =
            av_find_best_stream(ic, AVMEDIA_TYPE_VIDEO,
                                st_index[AVMEDIA_TYPE_VIDEO], -1, NULL, 0);
#ifndef SKC_VIDEO_ONLY
    if (!audio_disable)
        st_index[AVMEDIA_TYPE_AUDIO] =
            av_find_best_stream(ic, AVMEDIA_TYPE_AUDIO,
                                st_index[AVMEDIA_TYPE_AUDIO],
                                st_index[AVMEDIA_TYPE_VIDEO],
                                NULL, 0);
    if (!video_disable && !subtitle_disable)
        // AVERROR_STREAM_NOT_FOUND/*-1381258232*/ �� ��������
        st_index[AVMEDIA_TYPE_SUBTITLE] =
            av_find_best_stream(ic, AVMEDIA_TYPE_SUBTITLE,
                                st_index[AVMEDIA_TYPE_SUBTITLE],
                                (st_index[AVMEDIA_TYPE_AUDIO] >= 0 ?
                                 st_index[AVMEDIA_TYPE_AUDIO] :
                                 st_index[AVMEDIA_TYPE_VIDEO]),
                                NULL, 0);
#endif

    is->show_mode = show_mode;
    if (st_index[AVMEDIA_TYPE_VIDEO] >= 0) {
        AVStream *st = ic->streams[st_index[AVMEDIA_TYPE_VIDEO]];
        AVCodecParameters *codecpar = st->codecpar;
        // frame �� sample aspect ratio �� stream(set by the demuxer) �� frame(set by the codec) �� ����ؼ� ����
        AVRational sar = av_guess_sample_aspect_ratio(ic, st, NULL/*frame*/); // 0/1 (no idea)
        if (codecpar->width)
            set_default_window_size(codecpar->width, codecpar->height, sar);
    }

    /* open the streams */
    if (st_index[AVMEDIA_TYPE_AUDIO] >= 0) {
        stream_component_open(is, st_index[AVMEDIA_TYPE_AUDIO]);
    }

    ret = -1;
    if (st_index[AVMEDIA_TYPE_VIDEO] >= 0) {
        ret = stream_component_open(is, st_index[AVMEDIA_TYPE_VIDEO]);
    }
    if (is->show_mode == SHOW_MODE_NONE)
        is->show_mode = ret >= 0 ? SHOW_MODE_VIDEO : SHOW_MODE_RDFT;

    if (st_index[AVMEDIA_TYPE_SUBTITLE] >= 0) {
        stream_component_open(is, st_index[AVMEDIA_TYPE_SUBTITLE]);
    }

    if (is->video_stream < 0 && is->audio_stream < 0) {
        av_log(NULL, AV_LOG_FATAL, "Failed to open file '%s' or configure filtergraph\n",
               is->filename);
        ret = -1;
        goto fail;
    }

    if (infinite_buffer < 0 && is->realtime)
        infinite_buffer = 1;

    for (;;) {
        if (is->abort_request)
            break;
        if (is->paused != is->last_paused) {
            av_assert0(0);
            is->last_paused = is->paused;
            if (is->paused)
                is->read_pause_return = av_read_pause(ic);
            else
                av_read_play(ic); // Start playing a network-based stream (e.g. RTSP stream) at the current position.
        }
#if CONFIG_RTSP_DEMUXER || CONFIG_MMSH_PROTOCOL
        if (is->paused &&
                (!strcmp(ic->iformat->name, "rtsp") ||
                 (ic->pb && !strncmp(input_filename, "mmsh:", 5)))) {
            /* wait 10 ms to avoid trying to get another packet */
            /* XXX: horrible */
            SDL_Delay(10);
            continue;
        }
#endif
        if (is->seek_req) {
            int64_t seek_target = is->seek_pos;
            int64_t seek_min    = is->seek_rel > 0 ? seek_target - is->seek_rel + 2: INT64_MIN;
            int64_t seek_max    = is->seek_rel < 0 ? seek_target - is->seek_rel - 2: INT64_MAX;
// FIXME the +-2 is due to rounding being not done in the correct direction in generation
//      of the seek_pos/seek_rel variables

            ret = avformat_seek_file(is->ic, -1, seek_min, seek_target, seek_max, is->seek_flags);
            if (ret < 0) {
                av_log(NULL, AV_LOG_ERROR,
                       "%s: error while seeking\n", is->ic->url);
            } else {
                if (is->audio_stream >= 0)
                    packet_queue_flush(&is->audioq);
                if (is->subtitle_stream >= 0)
                    packet_queue_flush(&is->subtitleq);
                if (is->video_stream >= 0)
                    packet_queue_flush(&is->videoq);
                if (is->seek_flags & AVSEEK_FLAG_BYTE) {
                   set_clock(&is->extclk, NAN, 0);
                } else {
                   // avformat_seek_file() �Ŀ��� set_clock() ���� seek_target �ð����� ���� Ŭ���� ����
                   set_clock(&is->extclk, seek_target / (double)AV_TIME_BASE, 0); // serial �� �ٽ� 0 ���� ����
                }
            }
            is->seek_req = 0;
            is->queue_attachments_req = 1; // ��ŷ�� ť�� �ʱ�ȭ�����Ƿ� 1 �� �ٽ� ������ �ʿ���
            is->eof = 0;
            if (is->paused)
                step_to_next_frame(is);
        }
        if (is->queue_attachments_req) {
            if (is->video_st && is->video_st->disposition & AV_DISPOSITION_ATTACHED_PIC/*1024*/) {
                av_assert0(0);
                // ��Ʈ�� ����� �پ��ִ� ���� �̹���(cover art)�� ��Ŷ(pkt)���� ���� ��, �ش� ť�� ����
                if ((ret = av_packet_ref(pkt, &is->video_st->attached_pic)) < 0)
                    goto fail;
                packet_queue_put(&is->videoq, pkt);
                packet_queue_put_nullpacket(&is->videoq, pkt, is->video_stream);
            }
            is->queue_attachments_req = 0;
        }

        /* if the queue are full, no need to read more */
        if (infinite_buffer<1 &&
              (is->audioq.size + is->videoq.size + is->subtitleq.size > MAX_QUEUE_SIZE
#ifndef SKC_VIDEO_ONLY
            || (stream_has_enough_packets(is->audio_st, is->audio_stream, &is->audioq) &&
                stream_has_enough_packets(is->video_st, is->video_stream, &is->videoq) &&
                stream_has_enough_packets(is->subtitle_st, is->subtitle_stream, &is->subtitleq)))) {
#else
                || (stream_has_enough_packets(is->video_st, is->video_stream, &is->videoq)))) {
#endif
            /* wait 10 ms */
            //av_assert0(0);
            // ���ڴ� ť�� ������� empty_queue_cond �ñ׳�(== continue_read_thread)�� �۽ŵǾ� wait ���� ���!
            SDL_LockMutex(wait_mutex);
            SDL_CondWaitTimeout(is->continue_read_thread, wait_mutex, 10);
            SDL_UnlockMutex(wait_mutex);
            continue;
        }
        // frame_queue_nb_remaining(&is->pictq) �� is->sampq �� (0, 0) ���� �����ؼ� ���� (2, 8) �����ϴ� ��� �Ϸ�� (0, 0)
        // printf("read_thread() remaining pictq=%d, sampq=%d\n", frame_queue_nb_remaining(&is->pictq),frame_queue_nb_remaining(&is->sampq));
        // is->auddec.finished = 0, is->audioq.serial = 1 �� ������
        if (!is->paused &&
            (!is->audio_st || (is->auddec.finished == is->audioq.serial && frame_queue_nb_remaining(&is->sampq) == 0)) &&
            (!is->video_st || (is->viddec.finished == is->videoq.serial && frame_queue_nb_remaining(&is->pictq) == 0))) {
            //av_assert0(0); // ����� ȣ������
            if (loop != 1 && (!loop || --loop)) {
                // �⺻ 1 �̸�, 0 �� ���ѷ���, 2 �̻��� �ش� Ƚ����ŭ ����
                stream_seek(is, start_time != AV_NOPTS_VALUE ? start_time : 0, 0, 0);
            } else if (autoexit) {
                ret = AVERROR_EOF;
                goto fail;
            }
        }
        // ������ frame ���� �ɰ� ��, �ϳ��� ����(������ 1��, ������� �ټ� ����)
        ret = av_read_frame(ic, pkt);
        if (ret < 0) {
            if ((ret == AVERROR_EOF || avio_feof(ic->pb)) && !is->eof) {
                // ���н� pkt �� ����� ���ϵȴ�.
                if (is->video_stream >= 0)
                    packet_queue_put_nullpacket(&is->videoq, pkt, is->video_stream);
                if (is->audio_stream >= 0)
                    packet_queue_put_nullpacket(&is->audioq, pkt, is->audio_stream);
                if (is->subtitle_stream >= 0)
                    packet_queue_put_nullpacket(&is->subtitleq, pkt, is->subtitle_stream);
                is->eof = 1;
            }
            if (ic->pb && ic->pb->error) {
                av_assert0(0);
                if (autoexit)
                    goto fail;
                else
                    break;
            }
            SDL_LockMutex(wait_mutex);
            SDL_CondWaitTimeout(is->continue_read_thread, wait_mutex, 10);
            SDL_UnlockMutex(wait_mutex);
            continue;
        } else {
            is->eof = 0;
        }
        /* check if packet is in play range specified by user, then queue, otherwise discard */
        stream_start_time = ic->streams[pkt->stream_index]->start_time;

        // ���� ������ B-frames �� ���� ���, pts �� AV_NOPTS_VALUE �� �ɼ� ������, �� ��� dts �� ���
        // pkt->pts = 0, 1024, 2048, 3072, ..., 889200 �����̳� 0 ���� �ٽ� �����ϰų� �پ��⵵ �Ѵ�.
        // pkt->dts �� ���� pts �� �����ϳ� �߰��� ������ �ö���⵵ �Ѵ�.
        pkt_ts = pkt->pts == AV_NOPTS_VALUE ? pkt->dts : pkt->pts;
        av_assert0(pkt->pts >= pkt->dts);

#if 1 // skc
        // ���� ��Ŷ�� ������� ���θ� ����(duration �� ���ų�, start_time ~ duration �̳��� ��� OK)
        // pkt->pts �� �������� start_time �� duration �̳��� ��쿡�� pkt_in_play_range = true �� ��
        // �������� duration == AV_NOPTS_VALUE �̱⸸ �ϸ� �ٸ����� �ʿ���� pkt_in_play_range = 1
        int dur_ok = duration == AV_NOPTS_VALUE; // �������� duration �� ����� ����(t) �� �������� �ʴ� �� AV_NOPTS_VALUE �����̹Ƿ� True ��
        int64_t pkt_ts_ = (pkt_ts - (stream_start_time != AV_NOPTS_VALUE ? stream_start_time : 0));
        double time_base_ = av_q2d(ic->streams[pkt->stream_index]->time_base); // (1/48000) -> 0.00002083
        double start_time_ = (double)(start_time != AV_NOPTS_VALUE ? start_time : 0) / 1000000;
        // ������ duration == AV_NOPTS_VALUE �� ��� �ٷ� ���ϵ����Ƿ�, ���⼭�� duration != AV_NOPTS_VALUE �� ������
        pkt_in_play_range = dur_ok || (pkt_ts_ * time_base_ - start_time_ <= ((double)duration / 1000000));
        av_assert0(pkt_in_play_range);
#else // org
        pkt_in_play_range = duration == AV_NOPTS_VALUE ||
            (pkt_ts - (stream_start_time != AV_NOPTS_VALUE ? stream_start_time : 0)) *
            av_q2d(ic->streams[pkt->stream_index]->time_base) -
            (double)(start_time != AV_NOPTS_VALUE ? start_time : 0) / 1000000
            <= ((double)duration / 1000000);
#endif

        // is->video_st->disposition = 1, & AV_DISPOSITION_ATTACHED_PIC -> false
        if (pkt->stream_index == is->audio_stream && pkt_in_play_range) {
            av_assert0(pkt->duration == 1024);
            packet_queue_put(&is->audioq, pkt);
        } else if (pkt->stream_index == is->video_stream/*0*/ && pkt_in_play_range
                   && !(is->video_st->disposition & AV_DISPOSITION_ATTACHED_PIC/*1024*/)) {
            // AV_DISPOSITION_ATTACHED_PIC �� ���� ������ ó�����̹Ƿ� ť�� �߰� ���Ѵ�.
            av_assert0(pkt->duration == 0);
            packet_queue_put(&is->videoq, pkt);
        } else if (pkt->stream_index == is->subtitle_stream && pkt_in_play_range) {
            packet_queue_put(&is->subtitleq, pkt);
        } else {
            // ���� pkt->stream_index == 2 �� ������. data �ε���.
            av_packet_unref(pkt);
        }
    }

    ret = 0;
 fail:
    if (ic && !is->ic)
        avformat_close_input(&ic);

    av_packet_free(&pkt);
    if (ret != 0) {
        SDL_Event event;

        event.type = FF_QUIT_EVENT;
        event.user.data1 = is;
        SDL_PushEvent(&event);
    }
    SDL_DestroyMutex(wait_mutex);
    return 0;
}

static VideoState *stream_open(const char *filename,
                               const AVInputFormat *iformat)
{
    VideoState *is;

    is = av_mallocz(sizeof(VideoState));
    if (!is)
        return NULL;
    is->last_video_stream = is->video_stream = -1;
    is->last_audio_stream = is->audio_stream = -1;
    is->last_subtitle_stream = is->subtitle_stream = -1;
    is->filename = av_strdup(filename);
    if (!is->filename)
        goto fail;
    is->iformat = iformat;
    is->ytop    = 0;
    is->xleft   = 0;

    /* start video display */
    if (frame_queue_init(&is->pictq, &is->videoq, VIDEO_PICTURE_QUEUE_SIZE, 1) < 0)
        goto fail;
    if (frame_queue_init(&is->subpq, &is->subtitleq, SUBPICTURE_QUEUE_SIZE, 0) < 0)
        goto fail;
    if (frame_queue_init(&is->sampq, &is->audioq, SAMPLE_QUEUE_SIZE, 1) < 0)
        goto fail;

    if (packet_queue_init(&is->videoq) < 0 ||
        packet_queue_init(&is->audioq) < 0 ||
        packet_queue_init(&is->subtitleq) < 0)
        goto fail;

    if (!(is->continue_read_thread = SDL_CreateCond())) {
        av_log(NULL, AV_LOG_FATAL, "SDL_CreateCond(): %s\n", SDL_GetError());
        goto fail;
    }

    init_clock(&is->vidclk, &is->videoq.serial);
    init_clock(&is->audclk, &is->audioq.serial);
    init_clock(&is->extclk, &is->extclk.serial);
#ifndef SKC_VIDEO_ONLY
    is->audio_clock_serial = -1;
    if (startup_volume < 0)
        av_log(NULL, AV_LOG_WARNING, "-volume=%d < 0, setting to 0\n", startup_volume);
    if (startup_volume > 100)
        av_log(NULL, AV_LOG_WARNING, "-volume=%d > 100, setting to 100\n", startup_volume);
    startup_volume = av_clip(startup_volume, 0, 100);
    startup_volume = av_clip(SDL_MIX_MAXVOLUME * startup_volume / 100, 0, SDL_MIX_MAXVOLUME);
    is->audio_volume = startup_volume;
    is->muted = 0;
#endif
    is->av_sync_type = av_sync_type;
    is->read_tid     = SDL_CreateThread(read_thread, "read_thread", is);
    if (!is->read_tid) {
        av_log(NULL, AV_LOG_FATAL, "SDL_CreateThread(): %s\n", SDL_GetError());
fail:
        stream_close(is);
        return NULL;
    }
    return is;
}

#ifndef SKC_VIDEO_ONLY
// �����, �ڸ�, Ȥ�� ���� ��Ʈ���� ���� �� ���� ���,
// ����ڰ� Ű �Է� ���� ���� ���� ��Ʈ���� ��ȯ(cycle)
static void stream_cycle_channel(VideoState *is, int codec_type)
{
    AVFormatContext *ic = is->ic;
    int start_index, stream_index;
    int old_index;
    AVStream *st;
    AVProgram *p = NULL;
    int nb_streams = is->ic->nb_streams;

    if (codec_type == AVMEDIA_TYPE_VIDEO) {
        start_index = is->last_video_stream;    // 0
        old_index = is->video_stream;           // 0
    } else if (codec_type == AVMEDIA_TYPE_AUDIO) {
        start_index = is->last_audio_stream;    // 1
        old_index = is->audio_stream;           // 1
    } else {
        start_index = is->last_subtitle_stream;
        old_index = is->subtitle_stream;
    }
    stream_index = start_index;

    if (codec_type != AVMEDIA_TYPE_VIDEO && is->video_stream != -1) {
        // ���� ��Ʈ���� ���α׷� ���ð� �����ϰ� �׻� ����� ���� ��Ʈ���̹Ƿ�(1���� ����),
        // �����, �ڸ� ��Ʈ���� "� ���� ���α׷��� ���� �ִ���" Ȯ���� �ʿ�(������ ����)

        // MPEG-TS ó�� �ϳ��� ���Ͽ� ���� ���(���α׷�)�� ���� �� Ư�� ��Ʈ��(index)�� 
        // � ���α׷�(program)�� ���ϴ����� ã�� ���� �Լ�
        // ������ ��Ʈ�� ��ȣ�� ���� ���α׷�(AVProgram)�� ã�� ��ȯ
        p = av_find_program_from_stream(ic, NULL, is->video_stream);
        if (p) {
            av_assert0(0);
            nb_streams = p->nb_stream_indexes;
            // p->stream_index �� ó������ ��ȸ�ϸ� ������ stream_index �� ã�´�.
            for (start_index = 0; start_index < nb_streams; start_index++)
                if (p->stream_index[start_index] == stream_index)
                    break;
            if (start_index == nb_streams)
                start_index = -1; // for �������� �� ã�� ���, start_index = -1, stream_index �� -1
            stream_index = start_index; // ���α׷� ����Ʈ���� stream_index �� ���� start_index ȹ��
        }
    }

    for (;;) {
        if (++stream_index >= nb_streams)
        {
            if (codec_type == AVMEDIA_TYPE_SUBTITLE)
            {
                // ������ ��ȯ�� ���, �ڸ��� ��Ȱ��ȭ(OFF) ó��
                stream_index = -1;
                // ������ �ڸ��� �ٽ� �� �� ó������ ���� ������ �����ϵ���(++stream_index �ϹǷ�)
                is->last_subtitle_stream = -1;
                goto the_end;
            }
            if (start_index == -1) {
                // ���� ���õ� ��Ʈ���� ���� ����(e.g, is->last_audio_stream == -1)
                av_assert0(0);
                return;
            }
            stream_index = 0;
        }
        if (stream_index == start_index) {
            // ��ü ��Ʈ���� �� ���� ��ȸ������ ���ǿ� �´� �ٸ� ��Ʈ���� �� ã�� -> �ƹ��͵� ���ϰ� ����
            av_assert0(0);
            return;
        }
        // ��ü ������ ���鼭 codec_type �� ��ġ�ϴ� ��Ʈ���� ���� ����!
        st = is->ic->streams[p ? p->stream_index[stream_index] : stream_index];
        if (st->codecpar->codec_type == codec_type) {
            /* check that parameters are OK */
            switch (codec_type) {
            case AVMEDIA_TYPE_AUDIO:
                if (st->codecpar->sample_rate != 0 &&
                    st->codecpar->ch_layout.nb_channels != 0)
                    goto the_end;
                break;
            case AVMEDIA_TYPE_VIDEO:
            case AVMEDIA_TYPE_SUBTITLE:
                goto the_end;
            default:
                break;
            }
        }
    }
 the_end:
    if (p && stream_index != -1)
        stream_index = p->stream_index[stream_index];
    av_log(NULL, AV_LOG_INFO, "Switch %s stream from #%d to #%d\n",
           av_get_media_type_string(codec_type),
           old_index,
           stream_index);

    // old_index, stream_index �� -1 �̸� �ƹ��͵� ����!
    stream_component_close(is, old_index);
    stream_component_open(is, stream_index);
}


static void toggle_full_screen(VideoState *is)
{
    is_full_screen = !is_full_screen;
    SDL_SetWindowFullscreen(window, is_full_screen ? SDL_WINDOW_FULLSCREEN_DESKTOP : 0);
}

static void toggle_audio_display(VideoState *is)
{
    int next = is->show_mode;
    do {
        next = (next + 1) % SHOW_MODE_NB;
    } while (next != is->show_mode && (next == SHOW_MODE_VIDEO && !is->video_st || next != SHOW_MODE_VIDEO && !is->audio_st));
    if (is->show_mode != next) {
        is->force_refresh = 1;
        is->show_mode = next;
    }
}
#endif

static void refresh_loop_wait_event(VideoState *is, SDL_Event *event) {
    double remaining_time = 0.0;
    SDL_PumpEvents();
    while (!SDL_PeepEvents(event, 1, SDL_GETEVENT, SDL_FIRSTEVENT, SDL_LASTEVENT)) {
        if (!cursor_hidden && av_gettime_relative() - cursor_last_shown > CURSOR_HIDE_DELAY) {
            SDL_ShowCursor(0);
            cursor_hidden = 1;
        }
        if (remaining_time > 0.0)
            av_usleep((int64_t)(remaining_time * 1000000.0)); // �ּ�ó���ص� ��� �ߵ�?!
        remaining_time = REFRESH_RATE;
        if (is->show_mode != SHOW_MODE_NONE && (!is->paused || is->force_refresh))
            video_refresh(is, &remaining_time);
        SDL_PumpEvents();
    }
}

#ifndef SKC_VIDEO_ONLY
static void seek_chapter(VideoState *is, int incr)
{
    int64_t pos = get_master_clock(is) * AV_TIME_BASE;
    int i;

    if (!is->ic->nb_chapters)
        return;

    /* find the current chapter */
    for (i = 0; i < is->ic->nb_chapters; i++) {
        // AVChapter �� Ÿ�Ӷ��λ��� ���� ������ ��Ÿ��(�̵�� ������ é�� ����)
        // ������ ����(����) ������ ���
        AVChapter *ch = is->ic->chapters[i];
        if (av_compare_ts(pos, AV_TIME_BASE_Q, ch->start, ch->time_base) < 0) {
            i--;
            break;
        }
    }

    i += incr;
    i = FFMAX(i, 0);
    if (i >= is->ic->nb_chapters)
        return;

    av_log(NULL, AV_LOG_VERBOSE, "Seeking to chapter %d.\n", i);
    stream_seek(is, av_rescale_q(is->ic->chapters[i]->start, is->ic->chapters[i]->time_base,
                                 AV_TIME_BASE_Q), 0, 0);
}
#endif

/* handle an event sent by the GUI */
static void event_loop(VideoState *cur_stream)
{
    SDL_Event event;
    double incr, pos, frac;

    for (;;) {
        double x;
        refresh_loop_wait_event(cur_stream, &event);
        switch (event.type) {
#ifndef SKC_VIDEO_ONLY
        case SDL_KEYDOWN:
            if (exit_on_keydown || event.key.keysym.sym == SDLK_ESCAPE || event.key.keysym.sym == SDLK_q) {
                do_exit(cur_stream);
                break;
            }
            // If we don't yet have a window, skip all key events, because read_thread might still be initializing...
            if (!cur_stream->width)
                continue;
            switch (event.key.keysym.sym) {
            case SDLK_f:
                toggle_full_screen(cur_stream);
                cur_stream->force_refresh = 1;
                break;
            case SDLK_p:
            case SDLK_SPACE:
                toggle_pause(cur_stream);
                break;
            case SDLK_m:
                toggle_mute(cur_stream);
                break;
            case SDLK_KP_MULTIPLY:
            case SDLK_0:
                update_volume(cur_stream, 1, SDL_VOLUME_STEP);
                break;
            case SDLK_KP_DIVIDE:
            case SDLK_9:
                update_volume(cur_stream, -1, SDL_VOLUME_STEP);
                break;
            case SDLK_s: // S: Step to next frame
                step_to_next_frame(cur_stream);
                break;
            case SDLK_a:
                stream_cycle_channel(cur_stream, AVMEDIA_TYPE_AUDIO);
                break;
            case SDLK_v:
                stream_cycle_channel(cur_stream, AVMEDIA_TYPE_VIDEO);
                break;
            case SDLK_c:
                stream_cycle_channel(cur_stream, AVMEDIA_TYPE_VIDEO);
                stream_cycle_channel(cur_stream, AVMEDIA_TYPE_AUDIO);
                stream_cycle_channel(cur_stream, AVMEDIA_TYPE_SUBTITLE);
                break;
            case SDLK_t:
                stream_cycle_channel(cur_stream, AVMEDIA_TYPE_SUBTITLE);
                break;
            case SDLK_w:
                if (cur_stream->show_mode == SHOW_MODE_VIDEO && cur_stream->vfilter_idx < nb_vfilters - 1) {
                    if (++cur_stream->vfilter_idx >= nb_vfilters)
                        cur_stream->vfilter_idx = 0;
                } else {
                    cur_stream->vfilter_idx = 0;
                    toggle_audio_display(cur_stream);
                }
                break;
            case SDLK_PAGEUP:
                if (cur_stream->ic->nb_chapters <= 1) {
                    incr = 600.0;
                    goto do_seek;
                }
                seek_chapter(cur_stream, 1);
                break;
            case SDLK_PAGEDOWN:
                if (cur_stream->ic->nb_chapters <= 1) {
                    incr = -600.0;
                    goto do_seek;
                }
                seek_chapter(cur_stream, -1);
                break;
            case SDLK_LEFT:
                incr = seek_interval ? -seek_interval : -10.0;
                goto do_seek;
            case SDLK_RIGHT:
                incr = seek_interval ? seek_interval : 10.0;
                goto do_seek;
            case SDLK_UP:
                incr = 60.0;
                goto do_seek;
            case SDLK_DOWN:
                incr = -60.0;
            do_seek:
                    if (seek_by_bytes) {
                        pos = -1;
                        if (pos < 0 && cur_stream->video_stream >= 0)
                            pos = frame_queue_last_pos(&cur_stream->pictq);
                        if (pos < 0 && cur_stream->audio_stream >= 0)
                            pos = frame_queue_last_pos(&cur_stream->sampq);
                        if (pos < 0)
                            pos = avio_tell(cur_stream->ic->pb);
                        if (cur_stream->ic->bit_rate)
                            incr *= cur_stream->ic->bit_rate / 8.0;
                        else
                            incr *= 180000.0;
                        pos += incr;
                        stream_seek(cur_stream, pos, incr, 1);
                    } else {
                        pos = get_master_clock(cur_stream);
                        if (isnan(pos))
                            pos = (double)cur_stream->seek_pos / AV_TIME_BASE;
                        pos += incr;
                        if (cur_stream->ic->start_time != AV_NOPTS_VALUE && pos < cur_stream->ic->start_time / (double)AV_TIME_BASE)
                            pos = cur_stream->ic->start_time / (double)AV_TIME_BASE;
                        stream_seek(cur_stream, (int64_t)(pos * AV_TIME_BASE), (int64_t)(incr * AV_TIME_BASE), 0);
                    }
                break;
            default:
                break;
            }
            break;
        case SDL_MOUSEBUTTONDOWN:
            if (exit_on_mousedown) {
                do_exit(cur_stream);
                break;
            }
            if (event.button.button == SDL_BUTTON_LEFT) {
                static int64_t last_mouse_left_click = 0;
                if (av_gettime_relative() - last_mouse_left_click <= 500000) {
                    toggle_full_screen(cur_stream);
                    cur_stream->force_refresh = 1;
                    last_mouse_left_click = 0;
                } else {
                    last_mouse_left_click = av_gettime_relative();
                }
            }
        case SDL_MOUSEMOTION:
            if (cursor_hidden) {
                SDL_ShowCursor(1);
                cursor_hidden = 0;
            }
            cursor_last_shown = av_gettime_relative();
            if (event.type == SDL_MOUSEBUTTONDOWN) {
                if (event.button.button != SDL_BUTTON_RIGHT)
                    break;
                x = event.button.x;
            } else {
                if (!(event.motion.state & SDL_BUTTON_RMASK))
                    break;
                x = event.motion.x;
            }
                if (seek_by_bytes || cur_stream->ic->duration <= 0) {
                    uint64_t size =  avio_size(cur_stream->ic->pb);
                    stream_seek(cur_stream, size*x/cur_stream->width, 0, 1);
                } else {
                    int64_t ts;
                    int ns, hh, mm, ss;
                    int tns, thh, tmm, tss;
                    tns  = cur_stream->ic->duration / 1000000LL;
                    thh  = tns / 3600;
                    tmm  = (tns % 3600) / 60;
                    tss  = (tns % 60);
                    frac = x / cur_stream->width;
                    ns   = frac * tns;
                    hh   = ns / 3600;
                    mm   = (ns % 3600) / 60;
                    ss   = (ns % 60);
                    av_log(NULL, AV_LOG_INFO,
                           "Seek to %2.0f%% (%2d:%02d:%02d) of total duration (%2d:%02d:%02d)       \n", frac*100,
                            hh, mm, ss, thh, tmm, tss);
                    ts = frac * cur_stream->ic->duration;
                    if (cur_stream->ic->start_time != AV_NOPTS_VALUE)
                        ts += cur_stream->ic->start_time;
                    stream_seek(cur_stream, ts, 0, 0);
                }
            break;
#endif
        case SDL_WINDOWEVENT:
            switch (event.window.event) {
                case SDL_WINDOWEVENT_SIZE_CHANGED:
                    screen_width  = cur_stream->width  = event.window.data1;
                    screen_height = cur_stream->height = event.window.data2;
                    if (cur_stream->vis_texture) {
                        SDL_DestroyTexture(cur_stream->vis_texture);
                        cur_stream->vis_texture = NULL;
                    }
                    if (vk_renderer)
                        vk_renderer_resize(vk_renderer, screen_width, screen_height);
                case SDL_WINDOWEVENT_EXPOSED:
                    cur_stream->force_refresh = 1;
            }
            break;
        case SDL_QUIT:
        case FF_QUIT_EVENT:
            do_exit(cur_stream);
            break;
        default:
            break;
        }
    }
}

#ifndef SKC_VIDEO_ONLY
static int opt_width(void *optctx, const char *opt, const char *arg)
{
    double num;
    int ret = parse_number(opt, arg, OPT_TYPE_INT64, 1, INT_MAX, &num);
    if (ret < 0)
        return ret;

    screen_width = num;
    return 0;
}

static int opt_height(void *optctx, const char *opt, const char *arg)
{
    double num;
    int ret = parse_number(opt, arg, OPT_TYPE_INT64, 1, INT_MAX, &num);
    if (ret < 0)
        return ret;

    screen_height = num;
    return 0;
}

static int opt_format(void *optctx, const char *opt, const char *arg)
{
    file_iformat = av_find_input_format(arg);
    if (!file_iformat) {
        av_log(NULL, AV_LOG_FATAL, "Unknown input format: %s\n", arg);
        return AVERROR(EINVAL);
    }
    return 0;
}

static int opt_sync(void *optctx, const char *opt, const char *arg)
{
    if (!strcmp(arg, "audio"))
        av_sync_type = AV_SYNC_AUDIO_MASTER;
    else if (!strcmp(arg, "video"))
        av_sync_type = AV_SYNC_VIDEO_MASTER;
    else if (!strcmp(arg, "ext"))
        av_sync_type = AV_SYNC_EXTERNAL_CLOCK;
    else {
        av_log(NULL, AV_LOG_ERROR, "Unknown value for %s: %s\n", opt, arg);
        exit(1);
    }
    return 0;
}

static int opt_show_mode(void *optctx, const char *opt, const char *arg)
{
    show_mode = !strcmp(arg, "video") ? SHOW_MODE_VIDEO :
                !strcmp(arg, "waves") ? SHOW_MODE_WAVES :
                !strcmp(arg, "rdft" ) ? SHOW_MODE_RDFT  : SHOW_MODE_NONE;

    if (show_mode == SHOW_MODE_NONE) {
        double num;
        int ret = parse_number(opt, arg, OPT_TYPE_INT, 0, SHOW_MODE_NB-1, &num);
        if (ret < 0)
            return ret;
        show_mode = num;
    }
    return 0;
}
#endif

static int opt_input_file(void *optctx, const char *filename)
{
    if (input_filename) {
        av_log(NULL, AV_LOG_FATAL,
               "Argument '%s' provided as input filename, but '%s' was already specified.\n",
                filename, input_filename);
        return AVERROR(EINVAL);
    }
    if (!strcmp(filename, "-"))
        filename = "fd:";
    input_filename = av_strdup(filename);
    if (!input_filename)
        return AVERROR(ENOMEM);

    return 0;
}

#ifndef SKC_VIDEO_ONLY
static int opt_codec(void *optctx, const char *opt, const char *arg)
{
   const char *spec = strchr(opt, ':');
   const char **name;
   if (!spec) {
       av_log(NULL, AV_LOG_ERROR,
              "No media specifier was specified in '%s' in option '%s'\n",
               arg, opt);
       return AVERROR(EINVAL);
   }
   spec++;

   switch (spec[0]) {
   case 'a' : name = &audio_codec_name;    break;
   case 's' : name = &subtitle_codec_name; break;
   case 'v' : name = &video_codec_name;    break;
   default:
       av_log(NULL, AV_LOG_ERROR,
              "Invalid media specifier '%s' in option '%s'\n", spec, opt);
       return AVERROR(EINVAL);
   }

   av_freep(name);
   *name = av_strdup(arg);
   return *name ? 0 : AVERROR(ENOMEM);
}

static int dummy;
#endif

static const OptionDef options[] = {
#ifndef SKC_VIDEO_ONLY
    CMDUTILS_COMMON_OPTIONS
    { "x",                  OPT_TYPE_FUNC, OPT_FUNC_ARG, { .func_arg = opt_width }, "force displayed width", "width" },
    { "y",                  OPT_TYPE_FUNC, OPT_FUNC_ARG, { .func_arg = opt_height }, "force displayed height", "height" },
    { "fs",                 OPT_TYPE_BOOL,            0, { &is_full_screen }, "force full screen" },
    { "an",                 OPT_TYPE_BOOL,            0, { &audio_disable }, "disable audio" },
    { "vn",                 OPT_TYPE_BOOL,            0, { &video_disable }, "disable video" },
    { "sn",                 OPT_TYPE_BOOL,            0, { &subtitle_disable }, "disable subtitling" },
    { "ast",                OPT_TYPE_STRING, OPT_EXPERT, { &wanted_stream_spec[AVMEDIA_TYPE_AUDIO] }, "select desired audio stream", "stream_specifier" },
    { "vst",                OPT_TYPE_STRING, OPT_EXPERT, { &wanted_stream_spec[AVMEDIA_TYPE_VIDEO] }, "select desired video stream", "stream_specifier" },
    { "sst",                OPT_TYPE_STRING, OPT_EXPERT, { &wanted_stream_spec[AVMEDIA_TYPE_SUBTITLE] }, "select desired subtitle stream", "stream_specifier" },
    { "ss",                 OPT_TYPE_TIME,            0, { &start_time }, "seek to a given position in seconds", "pos" },
    { "t",                  OPT_TYPE_TIME,            0, { &duration }, "play  \"duration\" seconds of audio/video", "duration" },
    { "bytes",              OPT_TYPE_INT,             0, { &seek_by_bytes }, "seek by bytes 0=off 1=on -1=auto", "val" },
    { "seek_interval",      OPT_TYPE_FLOAT,           0, { &seek_interval }, "set seek interval for left/right keys, in seconds", "seconds" },
    { "nodisp",             OPT_TYPE_BOOL,            0, { &display_disable }, "disable graphical display" },
    { "noborder",           OPT_TYPE_BOOL,            0, { &borderless }, "borderless window" },
    { "alwaysontop",        OPT_TYPE_BOOL,            0, { &alwaysontop }, "window always on top" },
    { "volume",             OPT_TYPE_INT,             0, { &startup_volume}, "set startup volume 0=min 100=max", "volume" },
    { "f",                  OPT_TYPE_FUNC, OPT_FUNC_ARG, { .func_arg = opt_format }, "force format", "fmt" },
    { "stats",              OPT_TYPE_BOOL,   OPT_EXPERT, { &show_status }, "show status", "" },
    { "fast",               OPT_TYPE_BOOL,   OPT_EXPERT, { &fast }, "non spec compliant optimizations", "" },
    { "genpts",             OPT_TYPE_BOOL,   OPT_EXPERT, { &genpts }, "generate pts", "" },
    { "drp",                OPT_TYPE_INT,    OPT_EXPERT, { &decoder_reorder_pts }, "let decoder reorder pts 0=off 1=on -1=auto", ""},
    { "lowres",             OPT_TYPE_INT,    OPT_EXPERT, { &lowres }, "", "" },
    { "sync",               OPT_TYPE_FUNC, OPT_FUNC_ARG | OPT_EXPERT, { .func_arg = opt_sync }, "set audio-video sync. type (type=audio/video/ext)", "type" },
    { "autoexit",           OPT_TYPE_BOOL,   OPT_EXPERT, { &autoexit }, "exit at the end", "" },
    { "exitonkeydown",      OPT_TYPE_BOOL,   OPT_EXPERT, { &exit_on_keydown }, "exit on key down", "" },
    { "exitonmousedown",    OPT_TYPE_BOOL,   OPT_EXPERT, { &exit_on_mousedown }, "exit on mouse down", "" },
    { "loop",               OPT_TYPE_INT,    OPT_EXPERT, { &loop }, "set number of times the playback shall be looped", "loop count" },
    { "framedrop",          OPT_TYPE_BOOL,   OPT_EXPERT, { &framedrop }, "drop frames when cpu is too slow", "" },
    { "infbuf",             OPT_TYPE_BOOL,   OPT_EXPERT, { &infinite_buffer }, "don't limit the input buffer size (useful with realtime streams)", "" },
    { "window_title",       OPT_TYPE_STRING,          0, { &window_title }, "set window title", "window title" },
    { "left",               OPT_TYPE_INT,    OPT_EXPERT, { &screen_left }, "set the x position for the left of the window", "x pos" },
    { "top",                OPT_TYPE_INT,    OPT_EXPERT, { &screen_top }, "set the y position for the top of the window", "y pos" },
    { "vf",                 OPT_TYPE_FUNC, OPT_FUNC_ARG | OPT_EXPERT, { .func_arg = opt_add_vfilter }, "set video filters", "filter_graph" },
    { "af",                 OPT_TYPE_STRING,          0, { &afilters }, "set audio filters", "filter_graph" },
    { "rdftspeed",          OPT_TYPE_INT, OPT_AUDIO | OPT_EXPERT, { &rdftspeed }, "rdft speed", "msecs" },
    { "showmode",           OPT_TYPE_FUNC, OPT_FUNC_ARG, { .func_arg = opt_show_mode}, "select show mode (0 = video, 1 = waves, 2 = RDFT)", "mode" },
    { "i",                  OPT_TYPE_BOOL,            0, { &dummy}, "read specified file", "input_file"},
    { "codec",              OPT_TYPE_FUNC, OPT_FUNC_ARG, { .func_arg = opt_codec}, "force decoder", "decoder_name" },
    { "acodec",             OPT_TYPE_STRING, OPT_EXPERT, {    &audio_codec_name }, "force audio decoder",    "decoder_name" },
    { "scodec",             OPT_TYPE_STRING, OPT_EXPERT, { &subtitle_codec_name }, "force subtitle decoder", "decoder_name" },
    { "vcodec",             OPT_TYPE_STRING, OPT_EXPERT, {    &video_codec_name }, "force video decoder",    "decoder_name" },
    { "autorotate",         OPT_TYPE_BOOL,            0, { &autorotate }, "automatically rotate video", "" },
    { "find_stream_info",   OPT_TYPE_BOOL, OPT_INPUT | OPT_EXPERT, { &find_stream_info },
        "read and decode the streams to fill missing information with heuristics" },
    { "filter_threads",     OPT_TYPE_INT,    OPT_EXPERT, { &filter_nbthreads }, "number of filter threads per graph" },
    { "enable_vulkan",      OPT_TYPE_BOOL,            0, { &enable_vulkan }, "enable vulkan renderer" },
    { "vulkan_params",      OPT_TYPE_STRING, OPT_EXPERT, { &vulkan_params }, "vulkan configuration using a list of key=value pairs separated by ':'" },
    { "hwaccel",            OPT_TYPE_STRING, OPT_EXPERT, { &hwaccel }, "use HW accelerated decoding" },
#endif
    { NULL, },
};

static void show_usage(void)
{
    av_log(NULL, AV_LOG_INFO, "Simple media player\n");
    av_log(NULL, AV_LOG_INFO, "usage: %s [options] input_file\n", program_name);
    av_log(NULL, AV_LOG_INFO, "\n");
}

// -?, -h, -help, --help ���� ����� opt_common.c �� show_help() ���� ȣ���
void show_help_default(const char *opt, const char *arg)
{
    av_log_set_callback(log_callback_help);
    show_usage();
    show_help_options(options, "Main options:", 0, OPT_EXPERT);
    show_help_options(options, "Advanced options:", OPT_EXPERT, 0);
    printf("\n");
    show_help_children(avcodec_get_class(), AV_OPT_FLAG_DECODING_PARAM);
    show_help_children(avformat_get_class(), AV_OPT_FLAG_DECODING_PARAM);
    show_help_children(avfilter_get_class(), AV_OPT_FLAG_FILTERING_PARAM);
    printf("\nWhile playing:\n"
           "q, ESC              quit\n"
           "f                   toggle full screen\n"
           "p, SPC              pause\n"
           "m                   toggle mute\n"
           "9, 0                decrease and increase volume respectively\n"
           "/, *                decrease and increase volume respectively\n"
           "a                   cycle audio channel in the current program\n"
           "v                   cycle video channel\n"
           "t                   cycle subtitle channel in the current program\n"
           "c                   cycle program\n"
           "w                   cycle video filters or show modes\n"
           "s                   activate frame-step mode\n"
           "left/right          seek backward/forward 10 seconds or to custom interval if -seek_interval is set\n"
           "down/up             seek backward/forward 1 minute\n"
           "page down/page up   seek backward/forward 10 minutes\n"
           "right mouse click   seek to percentage in file corresponding to fraction of width\n"
           "left double-click   toggle full screen\n"
           );
}

/* Called from the main */
int main(int argc, char **argv)
{
    int flags, ret;
    VideoState *is;

    init_dynload();

    av_log_set_flags(AV_LOG_SKIP_REPEATED);
    parse_loglevel(argc, argv, options);

    /* register all codecs, demux and protocols */
#if CONFIG_AVDEVICE
    avdevice_register_all();
#endif
    avformat_network_init();

#ifndef SKC_VIDEO_ONLY
    signal(SIGINT , sigterm_handler); /* Interrupt (ANSI).    */
    signal(SIGTERM, sigterm_handler); /* Termination (ANSI).  */
#endif

    show_banner(argc, argv, options);

    // ������ opt_input_file ���ڴ� - �� ���� ���ϴ� ���ڰ� ������ ��� ȣ���(non-option ó����)
    ret = parse_options(NULL, argc, argv, options, opt_input_file); // options: �Ľ��� �ɼ� ���̺�
    //input_filename = av_strdup("D:\\VideoPlayer\\VideoPlayer\\\\data\\mov_bbb.mp4"); ret = 0;
    if (ret < 0)
        exit(ret == AVERROR_EXIT ? 0 : 1);

    if (!input_filename) {
        show_usage();
        av_log(NULL, AV_LOG_FATAL, "An input file must be specified\n");
        av_log(NULL, AV_LOG_FATAL,
               "Use -h to get full help or, even better, run 'man %s'\n", program_name);
        exit(1);
    }

    if (display_disable) {
        video_disable = 1;
    }
#ifndef SKC_VIDEO_ONLY
    flags = SDL_INIT_VIDEO | SDL_INIT_AUDIO | SDL_INIT_TIMER;
    if (audio_disable)
        flags &= ~SDL_INIT_AUDIO;
    else {
        /* Try to work around an occasional ALSA buffer underflow issue when the
         * period size is NPOT due to ALSA resampling by forcing the buffer size. */
        if (!SDL_getenv("SDL_AUDIO_ALSA_SET_BUFFER_SIZE"))
            SDL_setenv("SDL_AUDIO_ALSA_SET_BUFFER_SIZE","1", 1);
    }
#else
    flags = SDL_INIT_VIDEO | SDL_INIT_TIMER;
#endif
    if (display_disable)
        flags &= ~SDL_INIT_VIDEO;
    if (SDL_Init (flags)) {
        av_log(NULL, AV_LOG_FATAL, "Could not initialize SDL - %s\n", SDL_GetError());
        av_log(NULL, AV_LOG_FATAL, "(Did you set the DISPLAY variable?)\n");
        exit(1);
    }

    SDL_EventState(SDL_SYSWMEVENT, SDL_IGNORE);
    SDL_EventState(SDL_USEREVENT, SDL_IGNORE);

    if (!display_disable) {
        int flags = SDL_WINDOW_HIDDEN;
        if (alwaysontop)
#if SDL_VERSION_ATLEAST(2,0,5)
            flags |= SDL_WINDOW_ALWAYS_ON_TOP;
#else
            av_log(NULL, AV_LOG_WARNING, "Your SDL version doesn't support SDL_WINDOW_ALWAYS_ON_TOP. Feature will be inactive.\n");
#endif
        if (borderless)
            flags |= SDL_WINDOW_BORDERLESS;
        else
            flags |= SDL_WINDOW_RESIZABLE;

#ifdef SDL_HINT_VIDEO_X11_NET_WM_BYPASS_COMPOSITOR
//#error skc SDL_HINT_VIDEO_X11_NET_WM_BYPASS_COMPOSITOR
        // SDL�� X11���� �������� ��ȸ�� ���� �ʵ��� �Ͽ�, ���� �������̳� ������ �����ϰ� �� �������� ���÷��̸� �����մϴ�.
        SDL_SetHint(SDL_HINT_VIDEO_X11_NET_WM_BYPASS_COMPOSITOR, "0");
#endif
        if (hwaccel && !enable_vulkan) {
            av_assert0(0);
            av_log(NULL, AV_LOG_INFO, "Enable vulkan renderer to support hwaccel %s\n", hwaccel);
            enable_vulkan = 1;
        }
        if (enable_vulkan) {
            av_assert0(0);
            vk_renderer = vk_get_renderer();
            if (vk_renderer) {
#if SDL_VERSION_ATLEAST(2, 0, 6)
                flags |= SDL_WINDOW_VULKAN;
#endif
            } else {
                av_log(NULL, AV_LOG_WARNING, "Doesn't support vulkan renderer, fallback to SDL renderer\n");
                enable_vulkan = 0;
            }
        }
        window = SDL_CreateWindow(program_name, SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, default_width, default_height, flags);
        SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "linear");
        if (!window) {
            av_log(NULL, AV_LOG_FATAL, "Failed to create window: %s", SDL_GetError());
            do_exit(NULL);
        }

        if (vk_renderer) { // NULL
            av_assert0(0);
            AVDictionary *dict = NULL;

            if (vulkan_params) {
                int ret = av_dict_parse_string(&dict, vulkan_params, "=", ":", 0);
                if (ret < 0) {
                    av_log(NULL, AV_LOG_FATAL, "Failed to parse, %s\n", vulkan_params);
                    do_exit(NULL);
                }
            }
            ret = vk_renderer_create(vk_renderer, window, dict);
            av_dict_free(&dict);
            if (ret < 0) {
                av_log(NULL, AV_LOG_FATAL, "Failed to create vulkan renderer, %s\n", av_err2str(ret));
                do_exit(NULL);
            }
        } else {
            //renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
            renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_SOFTWARE); // skc Fix crash error!
            if (!renderer) {
                av_log(NULL, AV_LOG_WARNING, "Failed to initialize a hardware accelerated renderer: %s\n", SDL_GetError());
                renderer = SDL_CreateRenderer(window, -1, 0);
            }
            if (renderer) {
                if (!SDL_GetRendererInfo(renderer, &renderer_info))
                    av_log(NULL, AV_LOG_VERBOSE, "Initialized %s renderer.\n", renderer_info.name);
            }
            if (!renderer || !renderer_info.num_texture_formats) {
                av_log(NULL, AV_LOG_FATAL, "Failed to create window or renderer: %s", SDL_GetError());
                do_exit(NULL);
            }
        }
    }

    is = stream_open(input_filename, file_iformat);
    if (!is) {
        av_log(NULL, AV_LOG_FATAL, "Failed to initialize VideoState!\n");
        do_exit(NULL);
    }

    event_loop(is);

    /* never returns */

    return 0;
}
#endif
