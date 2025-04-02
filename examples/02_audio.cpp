extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libswresample/swresample.h>
#include <libavutil/opt.h>
#include <SDL.h>
}
#include "../include/common.h"

#ifdef _WIN32
#undef main // SDL.h 에서 #define main SDL_main 부분 복구
#endif

#define SDL_AUDIO_BUFFER_SIZE 1024
#define MAX_AUDIO_FRAME_SIZE 192000

typedef struct PacketQueue {
    AVPacketList* first_pkt, * last_pkt;
    int nb_packets;
    int size;
    SDL_mutex* mutex;
    SDL_cond* cond;
} PacketQueue;

PacketQueue audioq;
int quit = 0;

void packet_queue_init(PacketQueue* q) {
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
            &audio_codec_ctx->ch_layout, AV_SAMPLE_FMT_S16, audio_codec_ctx->sample_rate/*skc 44100 도 재생됨(실제값 48000)*/,
            &audio_codec_ctx->ch_layout, audio_codec_ctx->sample_fmt, audio_codec_ctx->sample_rate,
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

        int dst_nb_samples = av_rescale_rnd(frame->nb_samples,
            audio_codec_ctx->sample_rate/*skc 44100 도 재생됨(실제값 48000)*/,
            audio_codec_ctx->sample_rate,
            AV_ROUND_UP);

        uint8_t* out_buffer = audio_buf;

        int out_samples = swr_convert(swr_ctx,
            &out_buffer,
            dst_nb_samples,
            (const uint8_t**)frame->data,
            frame->nb_samples);

        if (out_samples < 0)
            return -1;

        int out_buffer_size = av_samples_get_buffer_size(NULL,
            2,
            out_samples,
            AV_SAMPLE_FMT_S16,
            1);

        av_packet_unref(&pkt);
        return out_buffer_size;
    }
}

void audio_callback(void* userdata, Uint8* stream, int len) {
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
        fprintf(stderr, "사용법: %s <오디오_파일>\n", argv[0]);
        return -1;
    }

    // SDL 초기화
    if (SDL_Init(SDL_INIT_AUDIO | SDL_INIT_TIMER)) {
        fprintf(stderr, "SDL 초기화 오류: %s\n", SDL_GetError());
        return -1;
    }

    // FFmpeg 등록
    avformat_network_init();

    AVFormatContext* format_ctx = NULL;
    if (avformat_open_input(&format_ctx, argv[1], NULL, NULL) != 0) {
        fprintf(stderr, "파일을 열 수 없습니다: %s\n", argv[1]);
        return -1;
    }

    if (avformat_find_stream_info(format_ctx, NULL) < 0) {
        fprintf(stderr, "스트림 정보를 찾을 수 없습니다\n");
        return -1;
    }

    // 오디오 스트림 찾기
    int audio_stream_idx = -1;
    for (int i = 0; i < format_ctx->nb_streams; i++) {
        if (format_ctx->streams[i]->codecpar->codec_type == AVMEDIA_TYPE_AUDIO) {
            audio_stream_idx = i;
            break;
        }
    }

    if (audio_stream_idx == -1) {
        fprintf(stderr, "오디오 스트림을 찾을 수 없습니다\n");
        return -1;
    }

    // 코덱 가져오기
    const AVCodec* codec = avcodec_find_decoder(format_ctx->streams[audio_stream_idx]->codecpar->codec_id);
    if (!codec) {
        fprintf(stderr, "코덱을 찾을 수 없습니다\n");
        return -1;
    }

    // 코덱 컨텍스트 생성
    AVCodecContext* codec_ctx = avcodec_alloc_context3(codec);
    if (!codec_ctx) {
        fprintf(stderr, "코덱 컨텍스트를 할당할 수 없습니다\n");
        return -1;
    }

    if (avcodec_parameters_to_context(codec_ctx, format_ctx->streams[audio_stream_idx]->codecpar) < 0) {
        fprintf(stderr, "코덱 파라미터를 설정할 수 없습니다\n");
        return -1;
    }

    // 코덱 열기
    if (avcodec_open2(codec_ctx, codec, NULL) < 0) {
        fprintf(stderr, "코덱을 열 수 없습니다\n");
        return -1;
    }

    // 패킷 큐 초기화
    packet_queue_init(&audioq);

    // SDL 오디오 설정
    SDL_AudioSpec wanted_spec, spec;
    wanted_spec.freq = codec_ctx->sample_rate;  // 48000
    wanted_spec.format = AUDIO_S16SYS;          // 32784
    wanted_spec.channels = 2;
    wanted_spec.silence = 0;
    wanted_spec.samples = SDL_AUDIO_BUFFER_SIZE; //skc ffplay 2048
    wanted_spec.callback = audio_callback;
    wanted_spec.userdata = codec_ctx;

#if 1 //skc SDL_OpenAudio() 는 무음으로 재생되고 있음(레거시 함수)
    SDL_AudioDeviceID audio_dev;
    // spec.size = samples(1024) * AUDIO_S16SYS(2) * channels(2) = 4096
    if (!(audio_dev = SDL_OpenAudioDevice(NULL, 0, &wanted_spec, &spec, 0))) {
        fprintf(stderr, "SDL 오디오를 열 수 없습니다: %s\n", SDL_GetError());
        return -1;
    }
    // 오디오 재생 시작
    SDL_PauseAudioDevice(audio_dev, 0);
#else
    if (SDL_OpenAudio(&wanted_spec, &spec) < 0) {
        fprintf(stderr, "SDL 오디오를 열 수 없습니다: %s\n", SDL_GetError());
        return -1;
    }
    // 오디오 재생 시작
    SDL_PauseAudio(0);
#endif

    // SDL_QUIT 이벤트(윈도우 종료시 발생) 입력을 받기 위해 추가함
    SDL_Window* window = SDL_CreateWindow("Simple AudioPlayer",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 640, 480, SDL_WINDOW_SHOWN);

    // 패킷 읽기 및 큐에 넣기
    AVPacket packet;
    while (av_read_frame(format_ctx, &packet) >= 0) {
        if (packet.stream_index == audio_stream_idx) {
            packet_queue_put(&audioq, &packet);
        }
        else {
            av_packet_unref(&packet);
        }

        SDL_Event event;
        SDL_PollEvent(&event);
        if (event.type == SDL_QUIT) {
            quit = 1;
            break;
        }
    }

    // 종료할 때까지 대기
    while (!quit) {
        SDL_Delay(100);

        SDL_Event event;
        SDL_PollEvent(&event);
        if (event.type == SDL_QUIT ||
            event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_ESCAPE) {
            quit = 1;
        }
    }

    // 정리
    SDL_CloseAudioDevice(audio_dev);
    SDL_DestroyWindow(window);
    SDL_Quit();

    avcodec_free_context(&codec_ctx);
    avformat_close_input(&format_ctx);

    return 0;
}
