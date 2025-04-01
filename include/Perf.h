#pragma once

#include <chrono>
#include <string>
#include <map>
#include <vector>

#define NS_CU_START namespace cu {
#define NS_CU_END   }
#define NS_CU_USING using namespace cu;

#define PERF_ENABLED            true
#define PERF_USE_MICRO_SECOND   true

NS_CU_START

struct PerfInfo {
    size_t called_cnt = 0;
    std::chrono::high_resolution_clock::time_point last_started;
    std::chrono::duration<double> last_elapsed;
    long long total_ms, everage_ms = 0;
};

static std::map<std::string, std::map<std::string, PerfInfo>> perf_counter;
static int perf_everage_interval_ms = 0;

class Perf {
public:
    static void PERF_INIT(int everage_interval_ms = 0) {
        perf_everage_interval_ms = everage_interval_ms;
#if PERF_USE_MICRO_SECOND
        perf_everage_interval_ms *= 1000; // milli-second(ms) -> micro-second(us)
#endif
    }
    static void PERF_START(const std::string& category, const std::string& name) {
#if PERF_ENABLED
        // 누적해야 하므로 여기서 called_cnt, total_ms 등은 초기화하면 안된다.
        perf_counter[category][name].last_started = std::chrono::high_resolution_clock::now();
#endif
    }
    static void PERF_END(const std::string& category, const std::string& name) {
#if PERF_ENABLED
        auto& p = perf_counter[category][name];
        p.last_elapsed = std::chrono::high_resolution_clock::now() - p.last_started;
#if PERF_USE_MICRO_SECOND
        auto last_elapsed_ms = std::chrono::duration_cast<std::chrono::microseconds>(p.last_elapsed).count();
#else
        auto last_elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(p.last_elapsed).count();
#endif
        p.total_ms += last_elapsed_ms;
        p.called_cnt++;
        if (p.total_ms >= perf_everage_interval_ms/* && p.called_cnt > 0*/) {
            p.everage_ms = p.total_ms / p.called_cnt;
            p.called_cnt = 0;
            p.total_ms = 0;
        } else {
            p.everage_ms = p.total_ms / p.called_cnt;
        }
#endif
    }
    static std::map<std::string, PerfInfo>& PERF_GET_MAP(const std::string& category) {
        return perf_counter[category];
    }
    static std::string PERF_GET_STRING(const std::string& category, std::vector<std::string> names, 
                                          std::string prefix = "PERF::") {
        std::string perf_info = prefix;
        if (!perf_counter[category].empty()) {
            auto& cu_perf = perf_counter[category];
            if (!cu_perf.empty()) {
                for (auto& name : names) {
                    if (cu_perf.find(name) != cu_perf.end()) {
                        perf_info += name + ": " + std::to_string(cu_perf.at(name).everage_ms);
                        perf_info += " (" + std::to_string(cu_perf.at(name).called_cnt + 1) + ")";
#if PERF_USE_MICRO_SECOND
                        perf_info += " us";
#else
                        perf_info += " ms";
#endif
                        perf_info += ", ";
                    }
                }
                size_t pos = perf_info.find_last_of(",");
                if (pos != std::string::npos) {
                    perf_info = perf_info.substr(0, pos); // remove last ,
                }
            }
        }
        return perf_info;
    }
};

NS_CU_END
