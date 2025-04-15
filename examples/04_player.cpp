// FFmpeg release/7.1 + SDL2 기반 C++ 오디오/비디오 동기화 재생 샘플
extern "C" {
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include <libswscale/swscale.h>
#include <libswresample/swresample.h>
#include <libavutil/opt.h>
#include <libavutil/time.h>
#include <SDL.h>
}

#include <iostream>
#include <thread>
#include <chrono>

#include "../include/common.h"

#ifdef _WIN32
#undef main // SDL.h 에서 #define main SDL_main 부분 복구
#endif

// TODO: 
// 1) 간혹 후반부에 윈도우 타이틀바에 응답없음 표시되며 화면이 멈추는 증상
//    창 전환(Alt + Tap)시에 거의 항상 발생하고 있음
//    (오디오 재생은 끝까지 정상 완료되고 있음)
// 2) 마지막 사과 떨어질때 오디오 1초 이상 느림

#define SDL_AUDIO_BUFFER_SIZE 1024
#define MAX_AUDIO_FRAME_SIZE 192000

typedef struct PacketQueue {
    AVPacketList* first_pkt, *last_pkt;
    int nb_packets;
    int size;
    SDL_mutex* mutex;
    SDL_cond* cond;
} PacketQueue;

static PacketQueue audioq;
static int quit = 0;

static void packet_queue_init(PacketQueue* q) {
    memset(q, 0, sizeof(PacketQueue));
    q->mutex = SDL_CreateMutex();
    q->cond = SDL_CreateCond();
}

int packet_queue_put(PacketQueue* q, AVPacket* pkt) {
    AVPacketList* pkt1;

    pkt1 = (AVPacketList*)av_malloc(sizeof(AVPacketList));
    if (!pkt1) return -1;

    pkt1->pkt = *pkt;
    pkt1->next = NULL;

    SDL_LockMutex(q->mutex);

    if (!q->last_pkt)
        q->first_pkt = pkt1;
    else
        q->last_pkt->next = pkt1;

    q->last_pkt = pkt1;
    q->nb_packets++;
    q->size += pkt1->pkt.size;

    SDL_CondSignal(q->cond);
    SDL_UnlockMutex(q->mutex);

    return 0;
}

int packet_queue_get(PacketQueue* q, AVPacket* pkt, int block) {
    AVPacketList* pkt1;
    int ret;

    SDL_LockMutex(q->mutex);

    for (;;) {
        if (quit) {
            ret = -1;
            break;
        }

        pkt1 = q->first_pkt;
        if (pkt1) {
            q->first_pkt = pkt1->next;
            if (!q->first_pkt)
                q->last_pkt = NULL;

            q->nb_packets--;
            q->size -= pkt1->pkt.size;

            *pkt = pkt1->pkt;
            av_free(pkt1);
            ret = 1;
            break;
        }
        else if (!block) {
            ret = 0;
            break;
        }
        else {
            SDL_CondWait(q->cond, q->mutex);
        }
    }

    SDL_UnlockMutex(q->mutex);
    return ret;
}

int audio_decode_frame(AVCodecContext* audio_codec_ctx, uint8_t* audio_buf, int buf_size) {
    static AVPacket pkt;
    static AVFrame* frame = NULL;
    static SwrContext* swr_ctx = NULL;

    int data_size = 0;

    if (!frame) {
        frame = av_frame_alloc();
    }

    if (!swr_ctx) {
#if 1 //skc
        swr_ctx = swr_alloc();
        int err = swr_alloc_set_opts2(&swr_ctx,
            &audio_codec_ctx->ch_layout, AV_SAMPLE_FMT_S16, 
            audio_codec_ctx->sample_rate/*skc 44100 도 재생됨(실제값 48000)*/,
            &audio_codec_ctx->ch_layout, audio_codec_ctx->sample_fmt/*AV_SAMPLE_FMT_FLTP(8)*/,
            audio_codec_ctx->sample_rate,
            0, NULL);
        swr_init(swr_ctx);
#else
        swr_ctx = swr_alloc();
        swr_ctx = swr_alloc_set_opts(NULL,
            AV_CH_LAYOUT_STEREO, AV_SAMPLE_FMT_S16, 44100,
            audio_codec_ctx->channel_layout, audio_codec_ctx->sample_fmt, audio_codec_ctx->sample_rate,
            0, NULL);
        swr_init(swr_ctx);
#endif
    }

    for (;;) {
        if (packet_queue_get(&audioq, &pkt, 1) < 0)
            return -1;

        int ret = avcodec_send_packet(audio_codec_ctx, &pkt);
        if (ret < 0)
            continue;

        ret = avcodec_receive_frame(audio_codec_ctx, frame);
        if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF)
            continue;

        if (ret < 0)
            return -1;

        // 리샘플링이 일어나는 경우(예: sample rate 또는 channel layout이 다를 때) 필수임
        // (현재는 audio_codec_ctx->sample_rate 가 동일하므로 그냥 frame->nb_samples를 사용해도 됨)
        // 실수 연산이 필요하고, **반올림 전략(UP, NEAR 등)**도 결정해야 하므로 av_rescale_rnd()가 안전함
        int dst_nb_samples = av_rescale_rnd(frame->nb_samples/*1024*/,
            audio_codec_ctx->sample_rate/*skc 44100 도 재생됨(실제값 48000)*/,
            audio_codec_ctx->sample_rate,
            AV_ROUND_UP);

        uint8_t* out_buffer = audio_buf;

        int out_samples = swr_convert(swr_ctx,
            &out_buffer,
            dst_nb_samples,
            (const uint8_t**)frame->data,
            frame->nb_samples); // 1024

        if (out_samples < 0)
            return -1;

        int out_buffer_size = av_samples_get_buffer_size(NULL,
            2, out_samples, AV_SAMPLE_FMT_S16, 1); // 4096

        av_packet_unref(&pkt);
        return out_buffer_size;
    }
}

static void audio_callback(void* userdata, Uint8* stream, int len) {
    AVCodecContext* audio_codec_ctx = (AVCodecContext*)userdata;
    int len1, audio_size;

    static uint8_t audio_buf[(MAX_AUDIO_FRAME_SIZE * 3) / 2];
    static unsigned int audio_buf_size = 0;
    static unsigned int audio_buf_index = 0;

    while (len > 0) {
        if (audio_buf_index >= audio_buf_size) {
            // 버퍼에 데이터가 없으면 더 가져오기
            audio_size = audio_decode_frame(audio_codec_ctx, audio_buf, sizeof(audio_buf));
            if (audio_size < 0) {
                // 오류 발생시 무음으로 채우기
                audio_buf_size = 1024;
                memset(audio_buf, 0, audio_buf_size);
            }
            else {
                audio_buf_size = audio_size;
            }
            audio_buf_index = 0;
        }
        len1 = audio_buf_size - audio_buf_index;
        if (len1 > len)
            len1 = len;
        memcpy(stream, (uint8_t*)audio_buf + audio_buf_index, len1);
        len -= len1;
        stream += len1;
        audio_buf_index += len1;
        av_log(NULL, AV_LOG_INFO, "audio_callback() write %6d, left %6d\n", len1, len);
    }
    av_log(NULL, AV_LOG_INFO, "\n");
}

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " <file>\n";
        return -1;
    }

    const char* filename = argv[1];
    avformat_network_init();

    AVFormatContext* fmt_ctx = nullptr;
    if (avformat_open_input(&fmt_ctx, filename, nullptr, nullptr) != 0) {
        std::cerr << "Could not open file." << std::endl;
        return -1;
    }

    if (avformat_find_stream_info(fmt_ctx, nullptr) < 0) {
        std::cerr << "Failed to get stream info." << std::endl;
        return -1;
    }

    int videoStream = -1, audioStream = -1;
    for (unsigned int i = 0; i < fmt_ctx->nb_streams; i++) {
        if (fmt_ctx->streams[i]->codecpar->codec_type == AVMEDIA_TYPE_VIDEO && videoStream < 0)
            videoStream = i;
        if (fmt_ctx->streams[i]->codecpar->codec_type == AVMEDIA_TYPE_AUDIO && audioStream < 0)
            audioStream = i;
    }

    if (videoStream == -1 || audioStream == -1) {
        std::cerr << "Missing video or audio stream." << std::endl;
        return -1;
    }

    // 비디오 디코더 초기화
    AVCodecParameters* vCodecPar = fmt_ctx->streams[videoStream]->codecpar;
    const AVCodec* vCodec = avcodec_find_decoder(vCodecPar->codec_id);
    AVCodecContext* vCodecCtx = avcodec_alloc_context3(vCodec);
    avcodec_parameters_to_context(vCodecCtx, vCodecPar);
    avcodec_open2(vCodecCtx, vCodec, nullptr);

    // 오디오 디코더 초기화
    AVCodecParameters* aCodecPar = fmt_ctx->streams[audioStream]->codecpar;
    const AVCodec* aCodec = avcodec_find_decoder(aCodecPar->codec_id);
    AVCodecContext* aCodecCtx = avcodec_alloc_context3(aCodec);
    avcodec_parameters_to_context(aCodecCtx, aCodecPar);
    avcodec_open2(aCodecCtx, aCodec, nullptr);

    // 패킷 큐 초기화
    packet_queue_init(&audioq);

    SDL_Init(SDL_INIT_VIDEO | SDL_INIT_AUDIO | SDL_INIT_TIMER);
    SDL_Window* window = SDL_CreateWindow("A/V Sync Sample", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        vCodecCtx->width, vCodecCtx->height, SDL_WINDOW_SHOWN);
    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, 0);
    SDL_Texture* texture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_YV12,
        SDL_TEXTUREACCESS_STREAMING, vCodecCtx->width, vCodecCtx->height);

    SwsContext* sws_ctx = sws_getContext(vCodecCtx->width, vCodecCtx->height, vCodecCtx->pix_fmt,
        vCodecCtx->width, vCodecCtx->height, AV_PIX_FMT_YUV420P, SWS_BILINEAR, nullptr, nullptr, nullptr);

    SDL_AudioSpec wanted_spec, spec;
    wanted_spec.freq = aCodecCtx->sample_rate;
    wanted_spec.format = AUDIO_S16SYS;
    wanted_spec.channels = 2; //skc aCodecCtx->channels;
    wanted_spec.silence = 0;
    wanted_spec.samples = SDL_AUDIO_BUFFER_SIZE;
    wanted_spec.callback = audio_callback;
    wanted_spec.userdata = aCodecCtx;

    SDL_AudioDeviceID audio_dev;
    if (!(audio_dev = SDL_OpenAudioDevice(NULL, 0, &wanted_spec, &spec, 0))) {
        std::cerr << "Failed to open audio." << std::endl;
        return -1;
    }
    SDL_PauseAudioDevice(audio_dev, 0);

    AVPacket* packet = av_packet_alloc();
    AVFrame* frame = av_frame_alloc();

    while (av_read_frame(fmt_ctx, packet) >= 0) {
        if (packet->stream_index == videoStream) {
            avcodec_send_packet(vCodecCtx, packet);
            while (avcodec_receive_frame(vCodecCtx, frame) == 0) {
                SDL_UpdateYUVTexture(texture, nullptr,
                    frame->data[0], frame->linesize[0],
                    frame->data[1], frame->linesize[1],
                    frame->data[2], frame->linesize[2]);
                SDL_RenderClear(renderer);
                SDL_RenderCopy(renderer, texture, nullptr, nullptr);
                SDL_RenderPresent(renderer);
                SDL_Delay(33); // 간단한 프레임 딜레이 (30fps 기준)
            }
            av_packet_unref(packet);
        }
        else if (packet->stream_index == audioStream) {
#if 1
            // 큐를 사용하지 않으면 사운드가 늘어져서 재생되고 있다.
            packet_queue_put(&audioq, packet);
#else
            if (avcodec_send_packet(aCodecCtx, packet) < 0) {
                continue;
            }
            int err = avcodec_receive_frame(aCodecCtx, audio_frame);
            if (err == AVERROR(EAGAIN) || err == AVERROR_EOF) {
                continue;
            }
#endif
        }
        //av_packet_unref(packet);
    }

    // 정리
    av_frame_free(&frame);
    av_packet_free(&packet);
    sws_freeContext(sws_ctx);
    SDL_CloseAudioDevice(audio_dev);
    SDL_DestroyTexture(texture);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    avcodec_free_context(&aCodecCtx);
    avcodec_free_context(&vCodecCtx);
    avformat_close_input(&fmt_ctx);

    return 0;
}
