#include "Timer.hpp"

#include <chrono>
#include <cstdio>
#include <string>

static auto timer_start = std::chrono::steady_clock::now();

void startTimer()
{
    timer_start = std::chrono::steady_clock::now();
}

void stopTimer()
{
    const auto timer_stop = std::chrono::steady_clock::now();
    const auto milliseconds =
        std::chrono::duration_cast<std::chrono::milliseconds>(timer_stop - timer_start);
    printf("Timer counted %.2f seconds.\n", milliseconds.count() * 0.001);
}

void stopTimer(const char name[])
{
    const auto timer_stop = std::chrono::steady_clock::now();
    const auto milliseconds =
        std::chrono::duration_cast<std::chrono::milliseconds>(timer_stop - timer_start);
    printf("Timer counted %.2f seconds for %s.\n", milliseconds.count() * 0.001, name);
}
