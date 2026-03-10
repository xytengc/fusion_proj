#include "thread_affinity.h"

#include <pthread.h>
#include <sched.h>
#include <unistd.h>
#include <iostream>

namespace {
int normalizeCoreId(int core_id) {
    long cpu_count = sysconf(_SC_NPROCESSORS_ONLN);
    if (cpu_count <= 0) {
        return -1;
    }
    if (core_id < 0) {
        return 0;
    }
    return core_id % static_cast<int>(cpu_count);
}

void logAffinityError(const char* who, int core_id, int ret) {
    std::cerr << "[WARN] bind " << (who ? who : "thread")
              << " to core " << core_id << " failed, ret=" << ret << "\n";
}
}

bool bindCurrentThreadToCore(int core_id, const char* tag) {
    int normalized_core = normalizeCoreId(core_id);
    if (normalized_core < 0) {
        std::cerr << "[WARN] unable to detect cpu core count\n";
        return false;
    }

    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(normalized_core, &cpuset);

    int ret = pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);
    if (ret != 0) {
        logAffinityError(tag, normalized_core, ret);
        return false;
    }
    return true;
}

bool bindThreadToCore(std::thread& thread, int core_id, const char* tag) {
    if (!thread.joinable()) {
        std::cerr << "[WARN] target thread is not joinable\n";
        return false;
    }

    int normalized_core = normalizeCoreId(core_id);
    if (normalized_core < 0) {
        std::cerr << "[WARN] unable to detect cpu core count\n";
        return false;
    }

    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(normalized_core, &cpuset);

    int ret = pthread_setaffinity_np(thread.native_handle(), sizeof(cpu_set_t), &cpuset);
    if (ret != 0) {
        logAffinityError(tag, normalized_core, ret);
        return false;
    }
    return true;
}
