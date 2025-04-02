extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
#include <libavutil/time.h>
#include <SDL.h>
#include <SDL_ttf.h>
}
#include "../include/common.h"

#ifdef _WIN32
#undef main // SDL.h 에서 #define main SDL_main 부분 복구
#endif

#define SCREEN_WIDTH 1280
#define SCREEN_HEIGHT 720

typedef struct SubtitleContext {
    AVFormatContext* format_ctx;
    AVCodecContext* subtitle_codec_ctx;
    int subtitle_stream_idx;
    SDL_Renderer* renderer;
    SDL_Texture* texture;
    TTF_Font* font;
} SubtitleContext;

void render_text_subtitle(const char* text, SDL_Renderer* renderer, TTF_Font* font) {
    if (!text || strlen(text) == 0) return;

    // 화면 지우기
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
    SDL_RenderClear(renderer);

    // 텍스트 렌더링
    SDL_Color textColor = { 255, 255, 255, 255 }; // 흰색 텍스트
    SDL_Surface* textSurface = TTF_RenderUTF8_Blended_Wrapped(font, text, textColor, SCREEN_WIDTH - 100);
    if (!textSurface) {
        fprintf(stderr, "텍스트 렌더링 실패: %s\n", TTF_GetError());
        return;
    }

    SDL_Texture* textTexture = SDL_CreateTextureFromSurface(renderer, textSurface);
    if (!textTexture) {
        fprintf(stderr, "텍스처 생성 실패: %s\n", SDL_GetError());
        SDL_FreeSurface(textSurface);
        return;
    }

    // 텍스트 위치 지정 (화면 하단)
    SDL_Rect textRect;
    textRect.x = 50;
    textRect.y = SCREEN_HEIGHT - textSurface->h - 50;
    textRect.w = textSurface->w;
    textRect.h = textSurface->h;

    // 텍스트 배경 그리기 (검은색 반투명)
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 128);
    SDL_Rect bgRect = textRect;
    bgRect.x -= 10;
    bgRect.y -= 10;
    bgRect.w += 20;
    bgRect.h += 20;
    SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);
    SDL_RenderFillRect(renderer, &bgRect);

    // 텍스트 렌더링
    SDL_RenderCopy(renderer, textTexture, NULL, &textRect);
    SDL_RenderPresent(renderer);

    // 메모리 해제
    SDL_DestroyTexture(textTexture);
    SDL_FreeSurface(textSurface);
}

void render_subtitle(AVSubtitle* subtitle, SubtitleContext* ctx) {
    if (!subtitle || subtitle->num_rects <= 0) return;

    for (unsigned i = 0; i < subtitle->num_rects; i++) {
        AVSubtitleRect* rect = subtitle->rects[i];

        if (rect->type == SUBTITLE_BITMAP) {
            // 비트맵 자막 처리 (이전 버전과 동일)
            uint8_t* pixels;
            int pitch;
            SDL_LockTexture(ctx->texture, NULL, (void**)&pixels, &pitch);

            // 텍스처를 투명하게 초기화
            memset(pixels, 0, pitch * SCREEN_HEIGHT);

            // 자막 비트맵을 텍스처에 복사
            int rect_y = SCREEN_HEIGHT - rect->h - 50; // 화면 하단에 배치

            for (int y = 0; y < rect->h; y++) {
                for (int x = 0; x < rect->w; x++) {
                    int pos = (rect_y + y) * pitch + (x + (SCREEN_WIDTH - rect->w) / 2) * 4;
                    int src_pos = y * rect->linesize[0] + x;

                    if (rect->data[0][src_pos] > 0) {
                        // RGBA 형식으로 설정
                        pixels[pos + 0] = 255; // R
                        pixels[pos + 1] = 255; // G
                        pixels[pos + 2] = 255; // B
                        pixels[pos + 3] = rect->data[0][src_pos]; // A (투명도)
                    }
                }
            }

            SDL_UnlockTexture(ctx->texture);

            // 화면에 렌더링
            SDL_SetRenderDrawColor(ctx->renderer, 0, 0, 0, 255);
            SDL_RenderClear(ctx->renderer);
            SDL_RenderCopy(ctx->renderer, ctx->texture, NULL, NULL);
            SDL_RenderPresent(ctx->renderer);
        }
        else if (rect->type == SUBTITLE_TEXT) {
            // 텍스트 자막 처리 (SRT 등)
            printf("텍스트 자막: %s\n", rect->text);
            render_text_subtitle(rect->text, ctx->renderer, ctx->font);
        }
        else if (rect->type == SUBTITLE_ASS) {
            // ASS 자막 처리
            printf("ASS 자막: %s\n", rect->ass);
            // ASS 포맷에서 태그 제거하고 텍스트만 추출하는 간단한 처리
            char* text = rect->ass;
            char* clean_text = _strdup(rect->ass);

            // ASS 형식에서 "Dialogue:" 이후의 텍스트 추출 시도
            //char* dialogue = strstr(clean_text, "Dialogue:");
            char* dialogue = clean_text;
            if (dialogue) {
                // 9개의 쉼표 이후에 실제 텍스트가 시작됨
                // 0,0,Default,,0,0,0,,Hello!!! // 8개
                int comma_count = 0;
                char* p = dialogue;
                while (*p && comma_count < 8) { //skc org 9
                    if (*p == ',') comma_count++;
                    p++;
                }
                if (comma_count == 8) { //skc org 9
                    render_text_subtitle(p, ctx->renderer, ctx->font);
                }
                else {
                    render_text_subtitle(clean_text, ctx->renderer, ctx->font);
                }
            }
            else {
                render_text_subtitle(clean_text, ctx->renderer, ctx->font);
            }
            free(clean_text);
        }
    }
}

int main(int argc, char* argv[]) {
    if (argc < 4) {
        fprintf(stderr, "사용법: %s <동영상 파일> <자막 파일> <폰트파일 위치>\n", argv[0]);
        return -1;
    }

    // SDL 초기화
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        fprintf(stderr, "SDL 초기화 실패: %s\n", SDL_GetError());
        return -1;
    }

    // SDL_ttf 초기화
    if (TTF_Init() < 0) {
        fprintf(stderr, "SDL_ttf 초기화 실패: %s\n", TTF_GetError());
        return -1;
    }

    SDL_Window* window = SDL_CreateWindow("FFmpeg 자막 렌더링",
        SDL_WINDOWPOS_UNDEFINED,
        SDL_WINDOWPOS_UNDEFINED,
        SCREEN_WIDTH, SCREEN_HEIGHT,
        SDL_WINDOW_SHOWN);
    if (!window) {
        fprintf(stderr, "윈도우 생성 실패: %s\n", SDL_GetError());
        return -1;
    }

    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    if (!renderer) {
        fprintf(stderr, "렌더러 생성 실패: %s\n", SDL_GetError());
        return -1;
    }

    SDL_Texture* texture = SDL_CreateTexture(renderer,
        SDL_PIXELFORMAT_RGBA32,
        SDL_TEXTUREACCESS_STREAMING,
        SCREEN_WIDTH, SCREEN_HEIGHT);
    if (!texture) {
        fprintf(stderr, "텍스처 생성 실패: %s\n", SDL_GetError());
        return -1;
    }

    // SDL 텍스처를 투명하게 설정
    SDL_SetTextureBlendMode(texture, SDL_BLENDMODE_BLEND);

    // 폰트 로드
    //TTF_Font* font = TTF_OpenFont("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 24);
    TTF_Font* font = TTF_OpenFont(argv[3], 24);
    if (!font) {
        fprintf(stderr, "폰트 로드 실패: %s\n", TTF_GetError());
        fprintf(stderr, "시스템에 있는 다른 폰트 경로를 사용해보세요.\n");
        // 대체 폰트 경로 시도
        const char* fontPaths[] = {
            //"/usr/share/fonts/TTF/arial.ttf",
            //"/usr/share/fonts/truetype/freefont/FreeSans.ttf",
            //"/System/Library/Fonts/Helvetica.ttc",  // macOS
            "C:\\Windows\\Fonts\\arial.ttf"         // Windows
        };

        for (int i = 0; i < sizeof(fontPaths) / sizeof(fontPaths[0]); i++) {
            font = TTF_OpenFont(fontPaths[i], 24);
            if (font) break;
        }

        if (!font) {
            fprintf(stderr, "폰트를 찾을 수 없습니다. 시스템에 맞는 폰트 경로를 지정해주세요.\n");
            return -1;
        }
    }

    // FFmpeg 초기화
    avformat_network_init();

    SubtitleContext ctx = { 0 };
    ctx.renderer = renderer;
    ctx.texture = texture;
    ctx.font = font;

    // 자막 파일 열기
    ctx.format_ctx = avformat_alloc_context();
    if (avformat_open_input(&ctx.format_ctx, argv[2], NULL, NULL) != 0) {
        fprintf(stderr, "파일을 열 수 없습니다: %s\n", argv[2]);
        return -1;
    }

    if (avformat_find_stream_info(ctx.format_ctx, NULL) < 0) {
        fprintf(stderr, "스트림 정보를 찾을 수 없습니다\n");
        return -1;
    }

    // 자막 스트림 찾기
    ctx.subtitle_stream_idx = -1;
    for (unsigned i = 0; i < ctx.format_ctx->nb_streams; i++) {
        if (ctx.format_ctx->streams[i]->codecpar->codec_type == AVMEDIA_TYPE_SUBTITLE) {
            ctx.subtitle_stream_idx = i;
            break;
        }
    }

    if (ctx.subtitle_stream_idx == -1) {
        fprintf(stderr, "자막 스트림을 찾을 수 없습니다\n");
        return -1;
    }

    // 코덱 가져오기
    const AVCodec* subtitle_codec = avcodec_find_decoder(
        ctx.format_ctx->streams[ctx.subtitle_stream_idx]->codecpar->codec_id);
    if (!subtitle_codec) {
        fprintf(stderr, "자막 코덱을 찾을 수 없습니다\n");
        return -1;
    }

    // 코덱 컨텍스트 할당 및 설정
    ctx.subtitle_codec_ctx = avcodec_alloc_context3(subtitle_codec);
    if (!ctx.subtitle_codec_ctx) {
        fprintf(stderr, "자막 코덱 컨텍스트를 할당할 수 없습니다\n");
        return -1;
    }

    if (avcodec_parameters_to_context(ctx.subtitle_codec_ctx,
        ctx.format_ctx->streams[ctx.subtitle_stream_idx]->codecpar) < 0) {
        fprintf(stderr, "자막 코덱 파라미터를 설정할 수 없습니다\n");
        return -1;
    }

    // 코덱 열기
    if (avcodec_open2(ctx.subtitle_codec_ctx, subtitle_codec, NULL) < 0) {
        fprintf(stderr, "자막 코덱을 열 수 없습니다\n");
        return -1;
    }

    // 자막 처리 및 표시
    AVPacket packet;
    int ret;
    int got_subtitle;
    AVSubtitle subtitle;

    // 초기 배경 지우기
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
    SDL_RenderClear(renderer);
    SDL_RenderPresent(renderer);

    // 자막 읽기 및 디코딩
    while (av_read_frame(ctx.format_ctx, &packet) >= 0) {
        if (packet.stream_index == ctx.subtitle_stream_idx) {
            ret = avcodec_decode_subtitle2(ctx.subtitle_codec_ctx, &subtitle, &got_subtitle, &packet);
            if (ret < 0) {
                fprintf(stderr, "자막 디코딩 실패\n");
                continue;
            }

            if (got_subtitle) {
                printf("\n자막 타입: %d, 개수: %d\n",
                    subtitle.num_rects > 0 ? subtitle.rects[0]->type : -1,
                    subtitle.num_rects);

                render_subtitle(&subtitle, &ctx);

                // 자막 표시 시간
                if (subtitle.end_display_time > 0) {
                    int display_ms = subtitle.end_display_time - subtitle.start_display_time;
                    SDL_Delay(display_ms);
                }
                else {
                    SDL_Delay(2000); // 기본 2초 표시
                }

                // 화면 지우기
                SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
                SDL_RenderClear(renderer);
                SDL_RenderPresent(renderer);

                avsubtitle_free(&subtitle);
            }
        }

        av_packet_unref(&packet);

        // 이벤트 처리 (종료 등)
        SDL_Event event;
        if (SDL_PollEvent(&event) && event.type == SDL_QUIT) {
            break;
        }
    }

    // 정리
    TTF_CloseFont(font);
    TTF_Quit();
    avcodec_free_context(&ctx.subtitle_codec_ctx);
    avformat_close_input(&ctx.format_ctx);
    SDL_DestroyTexture(texture);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}

#if 0
extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
#include <libavutil/time.h>
#include <SDL.h>
}
#include "../include/common.h"

#ifdef _WIN32
#undef main // SDL.h 에서 #define main SDL_main 부분 복구
#endif

#define SCREEN_WIDTH 1280
#define SCREEN_HEIGHT 720

typedef struct SubtitleContext {
    AVFormatContext* format_ctx;
    AVCodecContext* subtitle_codec_ctx;
    int subtitle_stream_idx;
    SDL_Renderer* renderer;
    SDL_Texture* texture;
} SubtitleContext;

void render_subtitle(AVSubtitle* subtitle, SDL_Renderer* renderer, SDL_Texture* texture) {
    if (!subtitle || subtitle->num_rects <= 0) return;

    for (unsigned i = 0; i < subtitle->num_rects; i++) {
        AVSubtitleRect* rect = subtitle->rects[i];
        if (rect->type != SUBTITLE_BITMAP) continue;

        // 텍스처 업데이트
        uint8_t* pixels;
        int pitch;
        SDL_LockTexture(texture, NULL, (void**)&pixels, &pitch);

        // 텍스처를 투명하게 초기화
        memset(pixels, 0, pitch * SCREEN_HEIGHT);

        // 자막 비트맵을 텍스처에 복사
        int rect_y = SCREEN_HEIGHT - rect->h - 50; // 화면 하단에 배치

        for (int y = 0; y < rect->h; y++) {
            for (int x = 0; x < rect->w; x++) {
                int pos = (rect_y + y) * pitch + (x + (SCREEN_WIDTH - rect->w) / 2) * 4;
                int src_pos = y * rect->linesize[0] + x;

                if (rect->data[0][src_pos] > 0) {
                    // RGBA 형식으로 설정
                    pixels[pos + 0] = 255; // R
                    pixels[pos + 1] = 255; // G
                    pixels[pos + 2] = 255; // B
                    pixels[pos + 3] = rect->data[0][src_pos]; // A (투명도)
                }
            }
        }

        SDL_UnlockTexture(texture);

        // 화면에 렌더링
        SDL_SetRenderDrawColor(renderer, 0, 0, 0, 0);
        SDL_RenderClear(renderer);
        SDL_RenderCopy(renderer, texture, NULL, NULL);
        SDL_RenderPresent(renderer);
    }
}

int main(int argc, char* argv[]) {
    if (argc < 2) {
        fprintf(stderr, "사용법: %s <자막_파일>\n", argv[0]);
        return -1;
    }

    // SDL 초기화
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        fprintf(stderr, "SDL 초기화 실패: %s\n", SDL_GetError());
        return -1;
    }

    SDL_Window* window = SDL_CreateWindow("FFmpeg 자막 렌더링",
        SDL_WINDOWPOS_UNDEFINED,
        SDL_WINDOWPOS_UNDEFINED,
        SCREEN_WIDTH, SCREEN_HEIGHT,
        SDL_WINDOW_SHOWN);
    if (!window) {
        fprintf(stderr, "윈도우 생성 실패: %s\n", SDL_GetError());
        return -1;
    }

    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    if (!renderer) {
        fprintf(stderr, "렌더러 생성 실패: %s\n", SDL_GetError());
        return -1;
    }

    SDL_Texture* texture = SDL_CreateTexture(renderer,
        SDL_PIXELFORMAT_RGBA32,
        SDL_TEXTUREACCESS_STREAMING,
        SCREEN_WIDTH, SCREEN_HEIGHT);
    if (!texture) {
        fprintf(stderr, "텍스처 생성 실패: %s\n", SDL_GetError());
        return -1;
    }

    // SDL 텍스처를 투명하게 설정
    SDL_SetTextureBlendMode(texture, SDL_BLENDMODE_BLEND);

    // FFmpeg 초기화
    avformat_network_init();

    SubtitleContext ctx = { 0 };
    ctx.renderer = renderer;
    ctx.texture = texture;

    // 자막 파일 열기
    ctx.format_ctx = avformat_alloc_context();
    if (avformat_open_input(&ctx.format_ctx, argv[1], NULL, NULL) != 0) {
        fprintf(stderr, "파일을 열 수 없습니다: %s\n", argv[1]);
        return -1;
    }

    if (avformat_find_stream_info(ctx.format_ctx, NULL) < 0) {
        fprintf(stderr, "스트림 정보를 찾을 수 없습니다\n");
        return -1;
    }

    // 자막 스트림 찾기
    ctx.subtitle_stream_idx = -1;
    for (unsigned i = 0; i < ctx.format_ctx->nb_streams; i++) {
        if (ctx.format_ctx->streams[i]->codecpar->codec_type == AVMEDIA_TYPE_SUBTITLE) {
            ctx.subtitle_stream_idx = i;
            break;
        }
    }

    if (ctx.subtitle_stream_idx == -1) {
        fprintf(stderr, "자막 스트림을 찾을 수 없습니다\n");
        return -1;
    }

    // 코덱 가져오기
    const AVCodec* subtitle_codec = avcodec_find_decoder(
        ctx.format_ctx->streams[ctx.subtitle_stream_idx]->codecpar->codec_id);
    if (!subtitle_codec) {
        fprintf(stderr, "자막 코덱을 찾을 수 없습니다\n");
        return -1;
    }

    // 코덱 컨텍스트 할당 및 설정
    ctx.subtitle_codec_ctx = avcodec_alloc_context3(subtitle_codec);
    if (!ctx.subtitle_codec_ctx) {
        fprintf(stderr, "자막 코덱 컨텍스트를 할당할 수 없습니다\n");
        return -1;
    }

    if (avcodec_parameters_to_context(ctx.subtitle_codec_ctx,
        ctx.format_ctx->streams[ctx.subtitle_stream_idx]->codecpar) < 0) {
        fprintf(stderr, "자막 코덱 파라미터를 설정할 수 없습니다\n");
        return -1;
    }

    // 코덱 열기
    if (avcodec_open2(ctx.subtitle_codec_ctx, subtitle_codec, NULL) < 0) {
        fprintf(stderr, "자막 코덱을 열 수 없습니다\n");
        return -1;
    }

    // 자막 처리 및 표시
    AVPacket packet;
    int ret;
    int got_subtitle;
    AVSubtitle subtitle;

    // 자막 읽기 및 디코딩
    while (av_read_frame(ctx.format_ctx, &packet) >= 0) {
        if (packet.stream_index == ctx.subtitle_stream_idx) {
            ret = avcodec_decode_subtitle2(ctx.subtitle_codec_ctx, &subtitle, &got_subtitle, &packet);
            if (ret < 0) {
                fprintf(stderr, "자막 디코딩 실패\n");
                continue;
            }

            if (got_subtitle) {
                render_subtitle(&subtitle, renderer, texture);

                // 자막 표시 시간
                if (subtitle.end_display_time > 0) {
                    int display_ms = subtitle.end_display_time - subtitle.start_display_time;
                    SDL_Delay(display_ms);
                }
                else {
                    SDL_Delay(2000); // 기본 2초 표시
                }

                avsubtitle_free(&subtitle);
            }
        }

        av_packet_unref(&packet);

        // 이벤트 처리 (종료 등)
        SDL_Event event;
        if (SDL_PollEvent(&event) && event.type == SDL_QUIT) {
            break;
        }
    }

    // 정리
    avcodec_free_context(&ctx.subtitle_codec_ctx);
    avformat_close_input(&ctx.format_ctx);
    SDL_DestroyTexture(texture);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}
#endif
