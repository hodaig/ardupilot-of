/*
 * TimeStatistics.h
 *
 *  Created on: Apr 27, 2017
 *      Author: hodai
 */

#ifndef TIMESTATISTICS_H_
#define TIMESTATISTICS_H_

#include <ctime>
#include <stdint.h> // uint32_t

using namespace std;


static struct {
    struct timespec start_time;
} state;

uint64_t millis64()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return 1.0e3*((ts.tv_sec + (ts.tv_nsec*1.0e-9)) -
                  (state.start_time.tv_sec +
                   (state.start_time.tv_nsec*1.0e-9)));
}


uint32_t millis()
{
    return millis64() & 0xFFFFFFFF;
}

uint64_t micros64()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return 1.0e6*((ts.tv_sec + (ts.tv_nsec*1.0e-9)) -
                  (state.start_time.tv_sec +
                   (state.start_time.tv_nsec*1.0e-9)));
}
uint32_t micros()
{
    return micros64() & 0xFFFFFFFF;
}

struct time_stat_t{
    bool initialized = false;

    const char* name;
    uint32_t iterations;
    uint32_t total_us;
    uint32_t max_us;
    float avg;

    /* temp */
    volatile uint32_t begin;
};

void STATS_SHOW(struct time_stat_t& stat){
    if (stat.initialized){
        printf("*** stat - %s: avg=%7.1f(us), max=%d(us), count=%d\n", stat.name, stat.avg, stat.max_us, stat.iterations);
    } else {
        printf("*** stat - not Initialized\n");
    }
}

void STAT_INIT(struct time_stat_t& stat, const char* name){
        stat.initialized = true;
        stat.iterations = 0;
        stat.max_us = 0;
        stat.total_us = 0;
        stat.avg = 0;
        stat.name = name;
}

/* define and init the stats */
#define STAT_INIT_HERE(name) \
        struct time_stat_t name;\
        STAT_INIT(name, #name);

void STATS_START(struct time_stat_t& stat){
    if (!stat.initialized){
        STAT_INIT(stat, "noName");
    }
    stat.begin = micros();
}
void STATS_END(struct time_stat_t& stat){
    int dif = micros() - stat.begin;
    if(dif < 0){
        return;
    }
    if (stat.begin != 0){
        stat.iterations++;
        stat.total_us += (uint32_t)dif;
        stat.avg = (float)stat.total_us / stat.iterations;
        if (stat.max_us < (uint32_t)dif){
            stat.max_us = (uint32_t)dif;
        }
    }
    stat.begin = 0;
}


#endif /* TIMESTATISTICS_H_ */
