extern "C" {
#include "libavformat/avformat.h"
#include "libavutil/error.h"
#include "libavcodec/avcodec.h"
#include "libavutil/imgutils.h"     // for av_image_fill_arrays()
#include "libavutil/time.h"         // for av_gettime_relative()
#include "libswscale/swscale.h"
#include "SDL.h"
}

#include "../include/Perf.h"

#ifdef _WIN32
#undef main // SDL.h ���� #define main SDL_main �κ� ����
#endif

#define PERF_CHECK_INVERVAL_MS     200
#define PERF_CATEGORY   "cu_video"
#define PERF_INIT()      cu::Perf::PERF_INIT(10 * 2);
#define PERF_START(name) cu::Perf::PERF_START(PERF_CATEGORY, name)
#define PERF_END(name)   cu::Perf::PERF_END(PERF_CATEGORY, name)

static inline void print_error(const char* msg, int err)
{
#if 1
    // av_make_error_string((char[AV_ERROR_MAX_STRING_SIZE]){0}, AV_ERROR_MAX_STRING_SIZE, errnum)
    // (char[AV_ERROR_MAX_STRING_SIZE]){0}
    // --> error C4576: �̴ϼȶ����� ��� ���� ��ȣ�� ���� ������ ��ǥ�� ����� ���� ��ȯ �����Դϴ�.
    char errbuf[AV_ERROR_MAX_STRING_SIZE]{};
    char* errstr = av_make_error_string(errbuf, AV_ERROR_MAX_STRING_SIZE, err);
    av_log(NULL, AV_LOG_ERROR, "%s: %s\n", msg, errstr);
#else
    av_log(NULL, AV_LOG_ERROR, "%s: %s\n", msg, av_err2str(err));
#endif
}

int main(int argc, char* argv[]) {
    PERF_INIT();
    PERF_START("Init");
    if (argc < 2) {
        printf("Usage: %s <video_file>\n", argv[0]);
        return -1;
    }
    const char* filename = argv[1];
    AVFormatContext* fmt_ctx = NULL;

    // FFmpeg �ʱ�ȭ
    int err = avformat_open_input(&fmt_ctx, filename, NULL, NULL);
    if (err < 0) {
        print_error(filename, err);
        return -1;
    }

    // �Է� ������ ��Ʈ�� ������ �Ľ��ϰ� ä���(start_time, duration, bit_rate ��).
    // start_time = 0, duration = 10026667 / 2080000, bit_rate = 629116 / 746423
    err = avformat_find_stream_info(fmt_ctx, NULL);
    if (err < 0) {
        print_error("avformat_find_stream_info()", err);
        return -1;
    }

    // ��Ʈ�� ������ ȭ�鿡 ��� (������)
    av_dump_format(fmt_ctx, 0, argv[1], 0);

    // ���� ��Ʈ�� ã��
    // (���� ���Ͽ� ���� ���� ��Ʈ��(��: ���� 2��, ����� 3��)�� ���� ��,
    // ���� �Ϲ������� ����ؾ� �� ��Ʈ���� �ڵ� ����)
    // av_find_best_stream() ȣ�� �� ���������� avcodec_find_decoder() �� �ڵ� ȣ���
    const AVCodec* codec = NULL;
    int video_stream_index = av_find_best_stream(fmt_ctx, AVMEDIA_TYPE_VIDEO, -1, -1, &codec, 0);

    AVStream* stream = fmt_ctx->streams[video_stream_index];
    AVRational fr = stream->avg_frame_rate;
    double fps = av_q2d(fr);
    av_log(NULL, AV_LOG_INFO, "fps = %.f\n", fps);

    // �ڵ� ���ؽ�Ʈ(AVCodecContext)�� �������� �����ϰ� �ʱ�ȭ
    AVCodecContext* codec_ctx = avcodec_alloc_context3(codec);

    // AVFormatContext->streams[i]->codecpar �� ����� ��Ʈ�� ��Ÿ�����͸�
    // ���� ���ڵ��� ������ �� �ֵ��� AVCodecContext�� ����
    // (�̰� ȣ������� codec_ctx->pix_fmt = AV_PIX_FMT_YUV420P(0), codec_ctx->width ���� ä����)
    avcodec_parameters_to_context(codec_ctx, fmt_ctx->streams[video_stream_index]->codecpar);

    // ���ڴ� �Ǵ� ���ڴ��� ������ "�����ִ�" �Լ�
    if (err = avcodec_open2(codec_ctx, codec, NULL) < 0) {
        print_error("avcodec_open2", err); // �ڵ��� �� �� �����ϴ�.
        return -1;
    }

    AVPacket* pkt = av_packet_alloc();
    AVFrame* frame = av_frame_alloc();
    AVFrame* frame_rgb = av_frame_alloc();  // ���� ��ȯ ����� ����

    const int align = 32; // 32: SIMD ����ȭ ��ƾ���� �޸� misalignment�� ���ϰ� ���� ���
    AVPixelFormat pix_fmt = AV_PIX_FMT_RGB24;
    SDL_PixelFormatEnum sdl_pix_fmt = SDL_PIXELFORMAT_RGB24;

    // Ư�� �ȼ� ����, �ػ�, ���� ���ǿ� ���� �̹��� �ϳ��� �����ϱ� ���� �ʿ��� ��ü ���� ũ��(����Ʈ ����)�� ���
    int num_bytes = av_image_get_buffer_size(pix_fmt, codec_ctx->width, codec_ctx->height, align);
    // ����ڰ� �����͸� ä��� ���� �޸𸮸� Ȯ���� ����
    // �Ʒ� sws_scale() ȣ��� frame_rgb->data �� ���� buffer �޸𸮿� ��ȯ�� �̹����� ������
    uint8_t* buffer = (uint8_t*)av_malloc(num_bytes * sizeof(uint8_t));

    // buffer �� �� plane�� ���� ��ġ�� stride�� ����ؼ� frame_rgb->data[], linesize[]�� ����
    int ret = av_image_fill_arrays(frame_rgb->data,     // uint8_t *dst_data[4]: �� �̹��� plane(Y/U/V �Ǵ� RGB)�� ���� �ּҸ� ���� �迭
                                   frame_rgb->linesize, // int dst_linesize[4]: �� plane�� stride(�� �ٴ� ����Ʈ ��)
                                   buffer, pix_fmt, codec_ctx->width, codec_ctx->height, align);

    SwsContext* sws_ctx = sws_getContext(codec_ctx->width, codec_ctx->height, codec_ctx->pix_fmt/*srcFormat*/,
                                         codec_ctx->width, codec_ctx->height, pix_fmt/*dstFormat*/,
                                         SWS_BILINEAR, NULL, NULL, NULL);

    // SDL �ʱ�ȭ
    SDL_Init(SDL_INIT_VIDEO);
    SDL_Window* window = SDL_CreateWindow("Simple VideoPlayer", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,
        codec_ctx->width, codec_ctx->height, SDL_WINDOW_RESIZABLE); // SDL_WINDOWPOS_CENTERED, flags = SDL_WINDOW_SHOWN
    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, 0); // flags = SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC

    // SDL_TEXTUREACCESS_STREAMING �� CPU���� ���� �ؽ�ó ������ ������ �� �ֵ��� ���
    // SDL_UpdateTexture() �� ������ �� ���� ��忩�� ��
    SDL_Texture* texture = SDL_CreateTexture(renderer, sdl_pix_fmt, SDL_TEXTUREACCESS_STREAMING,
        codec_ctx->width, codec_ctx->height);

    PERF_END("Init");

    // ���ڵ� ����
    PERF_START("Loop");
    auto started = std::chrono::high_resolution_clock::now();
    while (true) { // 1190 loop
        PERF_START("Render");
        auto loop_start_time = std::chrono::high_resolution_clock::now();
        int err = av_read_frame(fmt_ctx, pkt);  // Return the next frame of a stream
        if (err < 0) break;

        bool quit = false;
        SDL_Event event;
        // returns 1 if there is a pending event
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) {
                quit = true;
                break;
            }
        }
        if (quit) break;
        if (pkt->stream_index == video_stream_index) {
            // Supply raw packet data as input to a decoder
            // AVERROR(EAGAIN) �� �����ϸ� ���ڴ� �Է�ť�� �� ã���Ƿ� avcodec_receive_frame() �� ȣ���������
            avcodec_send_packet(codec_ctx, pkt); // ���� ������ ���ڵ�

            // AVERROR(EAGAIN) �� �����ϸ� ���ڴ� ���ť�� ������Ƿ� avcodec_send_packet() �� ȣ���������
            int frameFinished = avcodec_receive_frame(codec_ctx, frame); // 0us
            if (frameFinished == 0) {
                // ���� ��ȯ (YUV -> RGB)
                sws_scale(sws_ctx, frame->data, frame->linesize,
                    0/*srcSliceY*/, codec_ctx->height/*srcSliceH*/,
                    frame_rgb->data, frame_rgb->linesize);

                // AV_PIX_FMT_RGB24 �� packed ����(�޸𸮻� �������� ����)�̹Ƿ� 
                // frame_rgb->data[0] �� ä��������, frame_rgb->data[0] �� ����ϸ� ��
                // CPU �޸��� �ȼ� �����͸� �ؽ�ó�� ���� (GPU �޸𸮷� ���ε�)
                SDL_UpdateTexture(texture, NULL/*rect*/, frame_rgb->data[0], frame_rgb->linesize[0]);

                // ũ�Ⱑ �۰ų� ���İ� ������ ���� ������ ������ ���� �� �����Ƿ� ȣ���ʿ�(���� ������)
                SDL_RenderClear(renderer);

                SDL_RenderCopy(renderer, texture, NULL/*srcrect*/, NULL/*dstrect*/);

                // ȭ�� ����(���� ���� ����)
                SDL_RenderPresent(renderer);

                PERF_END("Render"); // ~3ms

                auto elapsed = std::chrono::high_resolution_clock::now() - started;
                auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();
                if (elapsed_ms >= PERF_CHECK_INVERVAL_MS) {
                    started = std::chrono::high_resolution_clock::now();
                    std::string str = cu::Perf::PERF_GET_STRING(PERF_CATEGORY, { "Render" }, "");
                    static int cnt = 0;
                    av_log(NULL, AV_LOG_INFO, "[%4d] %s\n", ++cnt, str.c_str());
                }

                auto loop_elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::high_resolution_clock::now() - loop_start_time).count();
                double sleep_ms = 1000 / fps - loop_elapsed_time;
                //av_log(NULL, AV_LOG_INFO, "sleep %.fms (loop_elapsed_time %lldms)\n", sleep_ms, loop_elapsed_time);
                if (sleep_ms > 0) {
                    SDL_Delay(sleep_ms); // 1000 / 25 = 40ms
                }
            } else {
                // ret(frameFinished) = -11
                av_log(NULL, AV_LOG_ERROR, "[%d] avcodec_receive_frame() ret=%d\n", pkt->stream_index, frameFinished);
            }
        }
        av_packet_unref(pkt);
    }
    PERF_END("Loop");

    std::string str = cu::Perf::PERF_GET_STRING(PERF_CATEGORY, { "Init", "Loop" }, "Total -> ");
    av_log(NULL, AV_LOG_INFO, "%s\n", str.c_str());

    av_frame_free(&frame);
    av_frame_free(&frame_rgb);
    av_packet_free(&pkt);
    avcodec_free_context(&codec_ctx);
    avformat_close_input(&fmt_ctx);

    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}
