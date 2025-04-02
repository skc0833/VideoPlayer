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
#undef main // SDL.h ���� #define main SDL_main �κ� ����
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

    // ȭ�� �����
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
    SDL_RenderClear(renderer);

    // �ؽ�Ʈ ������
    SDL_Color textColor = { 255, 255, 255, 255 }; // ��� �ؽ�Ʈ
    SDL_Surface* textSurface = TTF_RenderUTF8_Blended_Wrapped(font, text, textColor, SCREEN_WIDTH - 100);
    if (!textSurface) {
        fprintf(stderr, "�ؽ�Ʈ ������ ����: %s\n", TTF_GetError());
        return;
    }

    SDL_Texture* textTexture = SDL_CreateTextureFromSurface(renderer, textSurface);
    if (!textTexture) {
        fprintf(stderr, "�ؽ�ó ���� ����: %s\n", SDL_GetError());
        SDL_FreeSurface(textSurface);
        return;
    }

    // �ؽ�Ʈ ��ġ ���� (ȭ�� �ϴ�)
    SDL_Rect textRect;
    textRect.x = 50;
    textRect.y = SCREEN_HEIGHT - textSurface->h - 50;
    textRect.w = textSurface->w;
    textRect.h = textSurface->h;

    // �ؽ�Ʈ ��� �׸��� (������ ������)
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 128);
    SDL_Rect bgRect = textRect;
    bgRect.x -= 10;
    bgRect.y -= 10;
    bgRect.w += 20;
    bgRect.h += 20;
    SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);
    SDL_RenderFillRect(renderer, &bgRect);

    // �ؽ�Ʈ ������
    SDL_RenderCopy(renderer, textTexture, NULL, &textRect);
    SDL_RenderPresent(renderer);

    // �޸� ����
    SDL_DestroyTexture(textTexture);
    SDL_FreeSurface(textSurface);
}

void render_subtitle(AVSubtitle* subtitle, SubtitleContext* ctx) {
    if (!subtitle || subtitle->num_rects <= 0) return;

    for (unsigned i = 0; i < subtitle->num_rects; i++) {
        AVSubtitleRect* rect = subtitle->rects[i];

        if (rect->type == SUBTITLE_BITMAP) {
            // ��Ʈ�� �ڸ� ó�� (���� ������ ����)
            uint8_t* pixels;
            int pitch;
            SDL_LockTexture(ctx->texture, NULL, (void**)&pixels, &pitch);

            // �ؽ�ó�� �����ϰ� �ʱ�ȭ
            memset(pixels, 0, pitch * SCREEN_HEIGHT);

            // �ڸ� ��Ʈ���� �ؽ�ó�� ����
            int rect_y = SCREEN_HEIGHT - rect->h - 50; // ȭ�� �ϴܿ� ��ġ

            for (int y = 0; y < rect->h; y++) {
                for (int x = 0; x < rect->w; x++) {
                    int pos = (rect_y + y) * pitch + (x + (SCREEN_WIDTH - rect->w) / 2) * 4;
                    int src_pos = y * rect->linesize[0] + x;

                    if (rect->data[0][src_pos] > 0) {
                        // RGBA �������� ����
                        pixels[pos + 0] = 255; // R
                        pixels[pos + 1] = 255; // G
                        pixels[pos + 2] = 255; // B
                        pixels[pos + 3] = rect->data[0][src_pos]; // A (����)
                    }
                }
            }

            SDL_UnlockTexture(ctx->texture);

            // ȭ�鿡 ������
            SDL_SetRenderDrawColor(ctx->renderer, 0, 0, 0, 255);
            SDL_RenderClear(ctx->renderer);
            SDL_RenderCopy(ctx->renderer, ctx->texture, NULL, NULL);
            SDL_RenderPresent(ctx->renderer);
        }
        else if (rect->type == SUBTITLE_TEXT) {
            // �ؽ�Ʈ �ڸ� ó�� (SRT ��)
            printf("�ؽ�Ʈ �ڸ�: %s\n", rect->text);
            render_text_subtitle(rect->text, ctx->renderer, ctx->font);
        }
        else if (rect->type == SUBTITLE_ASS) {
            // ASS �ڸ� ó��
            printf("ASS �ڸ�: %s\n", rect->ass);
            // ASS ���˿��� �±� �����ϰ� �ؽ�Ʈ�� �����ϴ� ������ ó��
            char* text = rect->ass;
            char* clean_text = _strdup(rect->ass);

            // ASS ���Ŀ��� "Dialogue:" ������ �ؽ�Ʈ ���� �õ�
            //char* dialogue = strstr(clean_text, "Dialogue:");
            char* dialogue = clean_text;
            if (dialogue) {
                // 9���� ��ǥ ���Ŀ� ���� �ؽ�Ʈ�� ���۵�
                // 0,0,Default,,0,0,0,,Hello!!! // 8��
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
        fprintf(stderr, "����: %s <������ ����> <�ڸ� ����> <��Ʈ���� ��ġ>\n", argv[0]);
        return -1;
    }

    // SDL �ʱ�ȭ
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        fprintf(stderr, "SDL �ʱ�ȭ ����: %s\n", SDL_GetError());
        return -1;
    }

    // SDL_ttf �ʱ�ȭ
    if (TTF_Init() < 0) {
        fprintf(stderr, "SDL_ttf �ʱ�ȭ ����: %s\n", TTF_GetError());
        return -1;
    }

    SDL_Window* window = SDL_CreateWindow("FFmpeg �ڸ� ������",
        SDL_WINDOWPOS_UNDEFINED,
        SDL_WINDOWPOS_UNDEFINED,
        SCREEN_WIDTH, SCREEN_HEIGHT,
        SDL_WINDOW_SHOWN);
    if (!window) {
        fprintf(stderr, "������ ���� ����: %s\n", SDL_GetError());
        return -1;
    }

    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    if (!renderer) {
        fprintf(stderr, "������ ���� ����: %s\n", SDL_GetError());
        return -1;
    }

    SDL_Texture* texture = SDL_CreateTexture(renderer,
        SDL_PIXELFORMAT_RGBA32,
        SDL_TEXTUREACCESS_STREAMING,
        SCREEN_WIDTH, SCREEN_HEIGHT);
    if (!texture) {
        fprintf(stderr, "�ؽ�ó ���� ����: %s\n", SDL_GetError());
        return -1;
    }

    // SDL �ؽ�ó�� �����ϰ� ����
    SDL_SetTextureBlendMode(texture, SDL_BLENDMODE_BLEND);

    // ��Ʈ �ε�
    //TTF_Font* font = TTF_OpenFont("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 24);
    TTF_Font* font = TTF_OpenFont(argv[3], 24);
    if (!font) {
        fprintf(stderr, "��Ʈ �ε� ����: %s\n", TTF_GetError());
        fprintf(stderr, "�ý��ۿ� �ִ� �ٸ� ��Ʈ ��θ� ����غ�����.\n");
        // ��ü ��Ʈ ��� �õ�
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
            fprintf(stderr, "��Ʈ�� ã�� �� �����ϴ�. �ý��ۿ� �´� ��Ʈ ��θ� �������ּ���.\n");
            return -1;
        }
    }

    // FFmpeg �ʱ�ȭ
    avformat_network_init();

    SubtitleContext ctx = { 0 };
    ctx.renderer = renderer;
    ctx.texture = texture;
    ctx.font = font;

    // �ڸ� ���� ����
    ctx.format_ctx = avformat_alloc_context();
    if (avformat_open_input(&ctx.format_ctx, argv[2], NULL, NULL) != 0) {
        fprintf(stderr, "������ �� �� �����ϴ�: %s\n", argv[2]);
        return -1;
    }

    if (avformat_find_stream_info(ctx.format_ctx, NULL) < 0) {
        fprintf(stderr, "��Ʈ�� ������ ã�� �� �����ϴ�\n");
        return -1;
    }

    // �ڸ� ��Ʈ�� ã��
    ctx.subtitle_stream_idx = -1;
    for (unsigned i = 0; i < ctx.format_ctx->nb_streams; i++) {
        if (ctx.format_ctx->streams[i]->codecpar->codec_type == AVMEDIA_TYPE_SUBTITLE) {
            ctx.subtitle_stream_idx = i;
            break;
        }
    }

    if (ctx.subtitle_stream_idx == -1) {
        fprintf(stderr, "�ڸ� ��Ʈ���� ã�� �� �����ϴ�\n");
        return -1;
    }

    // �ڵ� ��������
    const AVCodec* subtitle_codec = avcodec_find_decoder(
        ctx.format_ctx->streams[ctx.subtitle_stream_idx]->codecpar->codec_id);
    if (!subtitle_codec) {
        fprintf(stderr, "�ڸ� �ڵ��� ã�� �� �����ϴ�\n");
        return -1;
    }

    // �ڵ� ���ؽ�Ʈ �Ҵ� �� ����
    ctx.subtitle_codec_ctx = avcodec_alloc_context3(subtitle_codec);
    if (!ctx.subtitle_codec_ctx) {
        fprintf(stderr, "�ڸ� �ڵ� ���ؽ�Ʈ�� �Ҵ��� �� �����ϴ�\n");
        return -1;
    }

    if (avcodec_parameters_to_context(ctx.subtitle_codec_ctx,
        ctx.format_ctx->streams[ctx.subtitle_stream_idx]->codecpar) < 0) {
        fprintf(stderr, "�ڸ� �ڵ� �Ķ���͸� ������ �� �����ϴ�\n");
        return -1;
    }

    // �ڵ� ����
    if (avcodec_open2(ctx.subtitle_codec_ctx, subtitle_codec, NULL) < 0) {
        fprintf(stderr, "�ڸ� �ڵ��� �� �� �����ϴ�\n");
        return -1;
    }

    // �ڸ� ó�� �� ǥ��
    AVPacket packet;
    int ret;
    int got_subtitle;
    AVSubtitle subtitle;

    // �ʱ� ��� �����
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
    SDL_RenderClear(renderer);
    SDL_RenderPresent(renderer);

    // �ڸ� �б� �� ���ڵ�
    while (av_read_frame(ctx.format_ctx, &packet) >= 0) {
        if (packet.stream_index == ctx.subtitle_stream_idx) {
            ret = avcodec_decode_subtitle2(ctx.subtitle_codec_ctx, &subtitle, &got_subtitle, &packet);
            if (ret < 0) {
                fprintf(stderr, "�ڸ� ���ڵ� ����\n");
                continue;
            }

            if (got_subtitle) {
                printf("\n�ڸ� Ÿ��: %d, ����: %d\n",
                    subtitle.num_rects > 0 ? subtitle.rects[0]->type : -1,
                    subtitle.num_rects);

                render_subtitle(&subtitle, &ctx);

                // �ڸ� ǥ�� �ð�
                if (subtitle.end_display_time > 0) {
                    int display_ms = subtitle.end_display_time - subtitle.start_display_time;
                    SDL_Delay(display_ms);
                }
                else {
                    SDL_Delay(2000); // �⺻ 2�� ǥ��
                }

                // ȭ�� �����
                SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
                SDL_RenderClear(renderer);
                SDL_RenderPresent(renderer);

                avsubtitle_free(&subtitle);
            }
        }

        av_packet_unref(&packet);

        // �̺�Ʈ ó�� (���� ��)
        SDL_Event event;
        if (SDL_PollEvent(&event) && event.type == SDL_QUIT) {
            break;
        }
    }

    // ����
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
#undef main // SDL.h ���� #define main SDL_main �κ� ����
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

        // �ؽ�ó ������Ʈ
        uint8_t* pixels;
        int pitch;
        SDL_LockTexture(texture, NULL, (void**)&pixels, &pitch);

        // �ؽ�ó�� �����ϰ� �ʱ�ȭ
        memset(pixels, 0, pitch * SCREEN_HEIGHT);

        // �ڸ� ��Ʈ���� �ؽ�ó�� ����
        int rect_y = SCREEN_HEIGHT - rect->h - 50; // ȭ�� �ϴܿ� ��ġ

        for (int y = 0; y < rect->h; y++) {
            for (int x = 0; x < rect->w; x++) {
                int pos = (rect_y + y) * pitch + (x + (SCREEN_WIDTH - rect->w) / 2) * 4;
                int src_pos = y * rect->linesize[0] + x;

                if (rect->data[0][src_pos] > 0) {
                    // RGBA �������� ����
                    pixels[pos + 0] = 255; // R
                    pixels[pos + 1] = 255; // G
                    pixels[pos + 2] = 255; // B
                    pixels[pos + 3] = rect->data[0][src_pos]; // A (����)
                }
            }
        }

        SDL_UnlockTexture(texture);

        // ȭ�鿡 ������
        SDL_SetRenderDrawColor(renderer, 0, 0, 0, 0);
        SDL_RenderClear(renderer);
        SDL_RenderCopy(renderer, texture, NULL, NULL);
        SDL_RenderPresent(renderer);
    }
}

int main(int argc, char* argv[]) {
    if (argc < 2) {
        fprintf(stderr, "����: %s <�ڸ�_����>\n", argv[0]);
        return -1;
    }

    // SDL �ʱ�ȭ
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        fprintf(stderr, "SDL �ʱ�ȭ ����: %s\n", SDL_GetError());
        return -1;
    }

    SDL_Window* window = SDL_CreateWindow("FFmpeg �ڸ� ������",
        SDL_WINDOWPOS_UNDEFINED,
        SDL_WINDOWPOS_UNDEFINED,
        SCREEN_WIDTH, SCREEN_HEIGHT,
        SDL_WINDOW_SHOWN);
    if (!window) {
        fprintf(stderr, "������ ���� ����: %s\n", SDL_GetError());
        return -1;
    }

    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    if (!renderer) {
        fprintf(stderr, "������ ���� ����: %s\n", SDL_GetError());
        return -1;
    }

    SDL_Texture* texture = SDL_CreateTexture(renderer,
        SDL_PIXELFORMAT_RGBA32,
        SDL_TEXTUREACCESS_STREAMING,
        SCREEN_WIDTH, SCREEN_HEIGHT);
    if (!texture) {
        fprintf(stderr, "�ؽ�ó ���� ����: %s\n", SDL_GetError());
        return -1;
    }

    // SDL �ؽ�ó�� �����ϰ� ����
    SDL_SetTextureBlendMode(texture, SDL_BLENDMODE_BLEND);

    // FFmpeg �ʱ�ȭ
    avformat_network_init();

    SubtitleContext ctx = { 0 };
    ctx.renderer = renderer;
    ctx.texture = texture;

    // �ڸ� ���� ����
    ctx.format_ctx = avformat_alloc_context();
    if (avformat_open_input(&ctx.format_ctx, argv[1], NULL, NULL) != 0) {
        fprintf(stderr, "������ �� �� �����ϴ�: %s\n", argv[1]);
        return -1;
    }

    if (avformat_find_stream_info(ctx.format_ctx, NULL) < 0) {
        fprintf(stderr, "��Ʈ�� ������ ã�� �� �����ϴ�\n");
        return -1;
    }

    // �ڸ� ��Ʈ�� ã��
    ctx.subtitle_stream_idx = -1;
    for (unsigned i = 0; i < ctx.format_ctx->nb_streams; i++) {
        if (ctx.format_ctx->streams[i]->codecpar->codec_type == AVMEDIA_TYPE_SUBTITLE) {
            ctx.subtitle_stream_idx = i;
            break;
        }
    }

    if (ctx.subtitle_stream_idx == -1) {
        fprintf(stderr, "�ڸ� ��Ʈ���� ã�� �� �����ϴ�\n");
        return -1;
    }

    // �ڵ� ��������
    const AVCodec* subtitle_codec = avcodec_find_decoder(
        ctx.format_ctx->streams[ctx.subtitle_stream_idx]->codecpar->codec_id);
    if (!subtitle_codec) {
        fprintf(stderr, "�ڸ� �ڵ��� ã�� �� �����ϴ�\n");
        return -1;
    }

    // �ڵ� ���ؽ�Ʈ �Ҵ� �� ����
    ctx.subtitle_codec_ctx = avcodec_alloc_context3(subtitle_codec);
    if (!ctx.subtitle_codec_ctx) {
        fprintf(stderr, "�ڸ� �ڵ� ���ؽ�Ʈ�� �Ҵ��� �� �����ϴ�\n");
        return -1;
    }

    if (avcodec_parameters_to_context(ctx.subtitle_codec_ctx,
        ctx.format_ctx->streams[ctx.subtitle_stream_idx]->codecpar) < 0) {
        fprintf(stderr, "�ڸ� �ڵ� �Ķ���͸� ������ �� �����ϴ�\n");
        return -1;
    }

    // �ڵ� ����
    if (avcodec_open2(ctx.subtitle_codec_ctx, subtitle_codec, NULL) < 0) {
        fprintf(stderr, "�ڸ� �ڵ��� �� �� �����ϴ�\n");
        return -1;
    }

    // �ڸ� ó�� �� ǥ��
    AVPacket packet;
    int ret;
    int got_subtitle;
    AVSubtitle subtitle;

    // �ڸ� �б� �� ���ڵ�
    while (av_read_frame(ctx.format_ctx, &packet) >= 0) {
        if (packet.stream_index == ctx.subtitle_stream_idx) {
            ret = avcodec_decode_subtitle2(ctx.subtitle_codec_ctx, &subtitle, &got_subtitle, &packet);
            if (ret < 0) {
                fprintf(stderr, "�ڸ� ���ڵ� ����\n");
                continue;
            }

            if (got_subtitle) {
                render_subtitle(&subtitle, renderer, texture);

                // �ڸ� ǥ�� �ð�
                if (subtitle.end_display_time > 0) {
                    int display_ms = subtitle.end_display_time - subtitle.start_display_time;
                    SDL_Delay(display_ms);
                }
                else {
                    SDL_Delay(2000); // �⺻ 2�� ǥ��
                }

                avsubtitle_free(&subtitle);
            }
        }

        av_packet_unref(&packet);

        // �̺�Ʈ ó�� (���� ��)
        SDL_Event event;
        if (SDL_PollEvent(&event) && event.type == SDL_QUIT) {
            break;
        }
    }

    // ����
    avcodec_free_context(&ctx.subtitle_codec_ctx);
    avformat_close_input(&ctx.format_ctx);
    SDL_DestroyTexture(texture);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}
#endif
