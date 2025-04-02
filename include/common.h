#pragma once

extern "C" {
#include "libavutil/error.h"
#include "libavutil/log.h"
}
#include "perf.h"

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
