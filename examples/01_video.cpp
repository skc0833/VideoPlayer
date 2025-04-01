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
#undef main // SDL.h 에서 #define main SDL_main 부분 복구
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
    // --> error C4576: 이니셜라이저 목록 앞의 괄호로 묶인 형식은 비표준 명시적 형식 변환 구문입니다.
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

    // FFmpeg 초기화
    int err = avformat_open_input(&fmt_ctx, filename, NULL, NULL);
    if (err < 0) {
        print_error(filename, err);
        return -1;
    }

    // 입력 파일의 스트림 정보를 파싱하고 채운다(start_time, duration, bit_rate 등).
    // start_time = 0, duration = 10026667 / 2080000, bit_rate = 629116 / 746423
    err = avformat_find_stream_info(fmt_ctx, NULL);
    if (err < 0) {
        print_error("avformat_find_stream_info()", err);
        return -1;
    }

    // 스트림 정보를 화면에 출력 (디버깅용)
    av_dump_format(fmt_ctx, 0, argv[1], 0);

    // 비디오 스트림 찾기
    // (비디오 파일에 여러 개의 스트림(예: 비디오 2개, 오디오 3개)이 있을 때,
    // 가장 일반적으로 재생해야 할 스트림을 자동 선택)
    // av_find_best_stream() 호출 시 내부적으로 avcodec_find_decoder() 가 자동 호출됨
    const AVCodec* codec = NULL;
    int video_stream_index = av_find_best_stream(fmt_ctx, AVMEDIA_TYPE_VIDEO, -1, -1, &codec, 0);

    AVStream* stream = fmt_ctx->streams[video_stream_index];
    AVRational fr = stream->avg_frame_rate;
    double fps = av_q2d(fr);
    av_log(NULL, AV_LOG_INFO, "fps = %.f\n", fps);

    // 코덱 컨텍스트(AVCodecContext)를 동적으로 생성하고 초기화
    AVCodecContext* codec_ctx = avcodec_alloc_context3(codec);

    // AVFormatContext->streams[i]->codecpar 에 저장된 스트림 메타데이터를
    // 실제 디코딩을 수행할 수 있도록 AVCodecContext에 복사
    // (이걸 호출해줘야 codec_ctx->pix_fmt = AV_PIX_FMT_YUV420P(0), codec_ctx->width 등이 채워짐)
    avcodec_parameters_to_context(codec_ctx, fmt_ctx->streams[video_stream_index]->codecpar);

    // 디코더 또는 인코더를 실제로 "열어주는" 함수
    if (err = avcodec_open2(codec_ctx, codec, NULL) < 0) {
        print_error("avcodec_open2", err); // 코덱을 열 수 없습니다.
        return -1;
    }

    AVPacket* pkt = av_packet_alloc();
    AVFrame* frame = av_frame_alloc();
    AVFrame* frame_rgb = av_frame_alloc();  // 색상 변환 결과를 저장

    const int align = 32; // 32: SIMD 최적화 루틴에서 메모리 misalignment를 피하고 성능 향상
    AVPixelFormat pix_fmt = AV_PIX_FMT_RGB24;
    SDL_PixelFormatEnum sdl_pix_fmt = SDL_PIXELFORMAT_RGB24;

    // 특정 픽셀 포맷, 해상도, 정렬 조건에 따라 이미지 하나를 저장하기 위해 필요한 전체 버퍼 크기(바이트 단위)를 계산
    int num_bytes = av_image_get_buffer_size(pix_fmt, codec_ctx->width, codec_ctx->height, align);
    // 사용자가 데이터를 채우기 위해 메모리만 확보한 상태
    // 아래 sws_scale() 호출시 frame_rgb->data 를 통해 buffer 메모리에 변환된 이미지를 저장함
    uint8_t* buffer = (uint8_t*)av_malloc(num_bytes * sizeof(uint8_t));

    // buffer 내 각 plane의 시작 위치와 stride를 계산해서 frame_rgb->data[], linesize[]를 설정
    int ret = av_image_fill_arrays(frame_rgb->data,     // uint8_t *dst_data[4]: 각 이미지 plane(Y/U/V 또는 RGB)의 시작 주소를 담을 배열
                                   frame_rgb->linesize, // int dst_linesize[4]: 각 plane의 stride(한 줄당 바이트 수)
                                   buffer, pix_fmt, codec_ctx->width, codec_ctx->height, align);

    SwsContext* sws_ctx = sws_getContext(codec_ctx->width, codec_ctx->height, codec_ctx->pix_fmt/*srcFormat*/,
                                         codec_ctx->width, codec_ctx->height, pix_fmt/*dstFormat*/,
                                         SWS_BILINEAR, NULL, NULL, NULL);

    // SDL 초기화
    SDL_Init(SDL_INIT_VIDEO);
    SDL_Window* window = SDL_CreateWindow("Simple VideoPlayer", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,
        codec_ctx->width, codec_ctx->height, SDL_WINDOW_RESIZABLE); // SDL_WINDOWPOS_CENTERED, flags = SDL_WINDOW_SHOWN
    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, 0); // flags = SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC

    // SDL_TEXTUREACCESS_STREAMING 는 CPU에서 자주 텍스처 내용을 변경할 수 있도록 허용
    // SDL_UpdateTexture() 를 쓰려면 이 접근 모드여야 함
    SDL_Texture* texture = SDL_CreateTexture(renderer, sdl_pix_fmt, SDL_TEXTUREACCESS_STREAMING,
        codec_ctx->width, codec_ctx->height);

    PERF_END("Init");

    // 디코딩 루프
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
            // AVERROR(EAGAIN) 를 리턴하면 디코더 입력큐가 꽉 찾으므로 avcodec_receive_frame() 를 호출해줘야함
            avcodec_send_packet(codec_ctx, pkt); // 비디오 프레임 디코딩

            // AVERROR(EAGAIN) 를 리턴하면 디코더 출력큐가 비었으므로 avcodec_send_packet() 를 호출해줘야함
            int frameFinished = avcodec_receive_frame(codec_ctx, frame); // 0us
            if (frameFinished == 0) {
                // 색상 변환 (YUV -> RGB)
                sws_scale(sws_ctx, frame->data, frame->linesize,
                    0/*srcSliceY*/, codec_ctx->height/*srcSliceH*/,
                    frame_rgb->data, frame_rgb->linesize);

                // AV_PIX_FMT_RGB24 가 packed 포맷(메모리상에 연속으로 저장)이므로 
                // frame_rgb->data[0] 만 채워졌으며, frame_rgb->data[0] 만 출력하면 됨
                // CPU 메모리의 픽셀 데이터를 텍스처에 복사 (GPU 메모리로 업로드)
                SDL_UpdateTexture(texture, NULL/*rect*/, frame_rgb->data[0], frame_rgb->linesize[0]);

                // 크기가 작거나 알파가 있으면 이전 프레임 흔적이 남을 수 있으므로 호출필요(보통 검정색)
                SDL_RenderClear(renderer);

                SDL_RenderCopy(renderer, texture, NULL/*srcrect*/, NULL/*dstrect*/);

                // 화면 갱신(더블 버퍼 스왑)
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
