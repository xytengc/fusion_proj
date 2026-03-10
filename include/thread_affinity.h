#ifndef THREAD_AFFINITY_H
#define THREAD_AFFINITY_H

#include <thread>

bool bindCurrentThreadToCore(int core_id, const char* tag = nullptr);
bool bindThreadToCore(std::thread& thread, int core_id, const char* tag = nullptr);

#endif // THREAD_AFFINITY_H
