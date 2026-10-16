// SPDX-License-Identifier: MIT
// Real allocation callbacks and monotonic clock for the native B cost fixture.
#ifndef SNAPSHOTLAB_COST_SUPPORT_H
#define SNAPSHOTLAB_COST_SUPPORT_H
#include <inttypes.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "box3d/base.h"
#if defined(_WIN32)
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#else
#include <time.h>
#endif

typedef struct SCCounters {
    uint64_t alloc_calls,free_calls,allocated_bytes,freed_bytes;
    uint64_t live_bytes,peak_bytes,live_backing,peak_backing;
} SCCounters;
typedef struct SCAllocation {
    uint64_t magic;
    void* base;
    size_t bytes,backing;
    unsigned domain;
} SCAllocation;
static SCCounters sc_counters[2]; // engine callback aligned sizes, host sizes
static uint64_t sc_combined_peak,sc_errors;
static uint64_t sc_engine_logical_peak;
static void sc_update_peak(void) {
    uint64_t sum=sc_counters[0].live_backing+sc_counters[1].live_backing;
    if(sum>sc_combined_peak)sc_combined_peak=sum;
}
static void* sc_allocate(size_t size,size_t alignment,unsigned domain) {
    if(!size)return NULL;
    if(domain>1||!alignment||(alignment&(alignment-1))||alignment>4096||size>SIZE_MAX-sizeof(SCAllocation)-alignment) {
        ++sc_errors;return NULL;
    }
    size_t backing=size+sizeof(SCAllocation)+alignment-1;
    void* base=malloc(backing);if(!base){++sc_errors;return NULL;}
    uintptr_t address=((uintptr_t)base+sizeof(SCAllocation)+alignment-1)&~((uintptr_t)alignment-1);
    SCAllocation* h=(SCAllocation*)(address-sizeof(SCAllocation));
    h->magic=UINT64_C(0x534b414c4c4f4331);h->base=base;h->bytes=size;h->backing=backing;h->domain=domain;
    SCCounters* c=sc_counters+domain;++c->alloc_calls;c->allocated_bytes+=size;c->live_bytes+=size;c->live_backing+=backing;
    if(c->live_bytes>c->peak_bytes)c->peak_bytes=c->live_bytes;
    if(c->live_backing>c->peak_backing)c->peak_backing=c->live_backing;
    sc_update_peak();return (void*)address;
}
static void sc_release(void* p,size_t size,unsigned domain) {
    if(!p)return;
    SCAllocation* h=(SCAllocation*)((uint8_t*)p-sizeof(SCAllocation));
    if(h->magic!=UINT64_C(0x534b414c4c4f4331)||h->bytes!=size||h->domain!=domain){++sc_errors;return;}
    SCCounters* c=sc_counters+domain;
    if(c->live_bytes<size||c->live_backing<h->backing){++sc_errors;return;}
    ++c->free_calls;c->freed_bytes+=size;c->live_bytes-=size;c->live_backing-=h->backing;
    void* base=h->base;h->magic=0;free(base);
}
static void* sc_engine_alloc(size_t size,int32_t alignment) {
    int64_t logical=b3GetByteCount();
    if(logical>0&&(uint64_t)logical>sc_engine_logical_peak)sc_engine_logical_peak=(uint64_t)logical;
    return sc_allocate(size,(size_t)alignment,0);
}
static void sc_engine_free(void* p,size_t size) { sc_release(p,size,0); }
static void* sc_host_alloc(size_t size) { return sc_allocate(size,16,1); }
static void sc_host_free(void* p,size_t size) { sc_release(p,size,1); }
static void sc_reset_peaks(void) {
    for(int i=0;i<2;++i){sc_counters[i].peak_bytes=sc_counters[i].live_bytes;sc_counters[i].peak_backing=sc_counters[i].live_backing;}
    sc_combined_peak=sc_counters[0].live_backing+sc_counters[1].live_backing;
    sc_engine_logical_peak=(uint64_t)b3GetByteCount();
}

static uint64_t sc_clock_frequency,sc_clock_resolution,sc_observed_increment;
static int sc_clock_ok=1;
static const char* sc_clock_name="unavailable";
static uint64_t sc_now(void) {
#if defined(_WIN32)
    if(!sc_clock_frequency){sc_clock_ok=0;return 0;}
    LARGE_INTEGER x;if(!QueryPerformanceCounter(&x)){sc_clock_ok=0;return 0;}
    // Conversion is intentionally not presented as sub-ns resolution.
    return (uint64_t)((long double)x.QuadPart*1000000000.0L/(long double)sc_clock_frequency);
#else
    struct timespec x;if(clock_gettime(CLOCK_MONOTONIC,&x)){sc_clock_ok=0;return 0;}
    return (uint64_t)x.tv_sec*UINT64_C(1000000000)+(uint64_t)x.tv_nsec;
#endif
}
static void sc_clock_init(void) {
#if defined(_WIN32)
    LARGE_INTEGER f;if(!QueryPerformanceFrequency(&f)||f.QuadPart<=0){sc_clock_ok=0;return;}
    sc_clock_frequency=(uint64_t)f.QuadPart;sc_clock_resolution=(UINT64_C(1000000000)+sc_clock_frequency-1)/sc_clock_frequency;
    sc_clock_name="QueryPerformanceCounter";
#else
    struct timespec r;if(clock_getres(CLOCK_MONOTONIC,&r)){sc_clock_ok=0;return;}
    sc_clock_resolution=(uint64_t)r.tv_sec*UINT64_C(1000000000)+(uint64_t)r.tv_nsec;
    sc_clock_frequency=UINT64_C(1000000000);sc_clock_name="clock_gettime_CLOCK_MONOTONIC";
#endif
    uint64_t previous=sc_now();sc_observed_increment=UINT64_MAX;
    for(int i=0;i<4096;++i){uint64_t next=sc_now();if(next<previous)sc_clock_ok=0;else if(next>previous&&next-previous<sc_observed_increment)sc_observed_increment=next-previous;previous=next;}
    if(sc_observed_increment==UINT64_MAX){sc_observed_increment=0;sc_clock_ok=0;}
}
static int sc_u64_compare(const void* a,const void* b) { uint64_t x=*(const uint64_t*)a,y=*(const uint64_t*)b;return x<y?-1:x>y?1:0; }
static void sc_percentiles(const uint64_t* samples,size_t count,uint64_t* median,uint64_t* p95) {
    uint64_t sorted[31];if(count!=31){*median=*p95=0;++sc_errors;return;}
    memcpy(sorted,samples,sizeof(sorted));qsort(sorted,31,sizeof(uint64_t),sc_u64_compare);*median=sorted[15];*p95=sorted[29];
}
static void sc_array_u64(FILE* f,const uint64_t* samples,size_t n) {
    fputc('[',f);for(size_t i=0;i<n;++i){if(i)fputc(',',f);fprintf(f,"%" PRIu64,samples[i]);}fputc(']',f);
}
#endif
