// SPDX-License-Identifier: MIT
// Native snapshot-authoritative adapter B cost. Native execution is CI-only.
#if !defined(_WIN32)
#define _POSIX_C_SOURCE 200809L
#endif
#include "snapshotlab/cost-support.h"
#include "snapshotlab/snapshot-core.h"
#include "simd.h"
#include <float.h>

_Static_assert(sizeof(float)==4&&FLT_RADIX==2,"float32 binary fixture required");

enum { SC_SAMPLES=31,SC_WARMUPS=8,SC_END=360,SC_MAX_SLOTS=121 };
typedef struct SCFixture {
    int bodies,window,first,slots;
    SKBlob* ring;
    uint8_t* hashes;
    uint64_t payload_bytes,capacity_bytes,registry_bytes;
    int awake,sleeping,contacts,joints,reuse,positive;
    uint64_t workload_peak,logical_engine_peak,aligned_engine_peak;
} SCFixture;
typedef struct SCMeasurement {
    uint64_t ns[SC_SAMPLES],engine_calls[SC_SAMPLES],host_calls[SC_SAMPLES];
    uint64_t engine_bytes[SC_SAMPLES],host_bytes[SC_SAMPLES],peaks[SC_SAMPLES];
    uint64_t engine_frees[SC_SAMPLES],host_frees[SC_SAMPLES],engine_freed[SC_SAMPLES],host_freed[SC_SAMPLES];
    uint64_t engine_live[SC_SAMPLES],host_live[SC_SAMPLES];
    int exact,finite;
} SCMeasurement;
static const char* sc_profile="unspecified";
static FILE* sc_trace;
static int sc_io_ok=1,sc_execution=1,sc_coverage=1,sc_results;
static const char* sc_bool(int v){return v?"true":"false";}
static const char* sc_backend(void) {
#if defined(BOX3D_DISABLE_SIMD)
    return "scalar";
#elif defined(__aarch64__) || defined(_M_ARM64)
    return "neon";
#else
    return "sse2";
#endif
}
static SKBlob* sc_at(SCFixture* f,int tick) {
    if(tick<f->first||tick>SC_END)return NULL;
    return f->ring+(tick-f->first);
}
static int sc_verify(SCFixture* f,int tick,const SKBlob* value) {
    SKBlob* expected=sc_at(f,tick);uint8_t digest[32];
    if(!expected||!sk_blob_equal(expected,value))return 0;
    sk_blob_hash(value,digest);return !memcmp(digest,f->hashes+32*(tick-f->first),32);
}
static void sc_fixture_free(SCFixture* f) {
    if(f->ring){for(int i=0;i<f->slots;++i)sk_blob_free(f->ring+i);sc_host_free(f->ring,(size_t)f->slots*sizeof(SKBlob));}
    if(f->hashes)sc_host_free(f->hashes,(size_t)f->slots*32);
    memset(f,0,sizeof(*f));
}
static int sc_fixture_build(SCFixture* f,int bodies,int window) {
    memset(f,0,sizeof(*f));f->bodies=bodies;f->window=window;f->slots=window+1;f->first=SC_END-window;
    f->ring=(SKBlob*)sc_host_alloc((size_t)f->slots*sizeof(SKBlob));
    f->hashes=(uint8_t*)sc_host_alloc((size_t)f->slots*32);
    if(f->ring)memset(f->ring,0,(size_t)f->slots*sizeof(SKBlob));
    if(!f->ring||!f->hashes)return 0;
    SKBlob current={0},next={0};int ok=sk_seed(bodies,240,&current);
    // Both windows have the identical seed and input history. Only retention differs.
    for(int tick=240;ok&&tick<=SC_END;++tick) {
        if(tick>=f->first){SKBlob* slot=sc_at(f,tick);ok=sk_blob_copy(slot,&current);if(ok)sk_blob_hash(slot,f->hashes+32*(tick-f->first));}
        if(ok&&tick<SC_END){ok=sk_step(&current,tick+1,0,&next);sk_blob_free(&current);current=next;memset(&next,0,sizeof(next));}
    }
    sk_blob_free(&current);sk_blob_free(&next);if(!ok)return 0;
    for(int i=0;i<f->slots;++i){f->payload_bytes+=f->ring[i].size;f->capacity_bytes+=f->ring[i].capacity;f->registry_bytes+=sk_u64(f->ring[i].data+32);}
    SKWorld state={0};ok=sk_import(sc_at(f,SC_END),&state);if(!ok)return 0;
    b3World* w=b3GetWorldFromId(state.world);
    ok=w->simdWidth==4;
    f->awake=b3World_GetAwakeBodyCount(state.world);f->sleeping=bodies-f->awake;
    f->contacts=b3World_GetCounters(state.world).contactCount;
    f->joints=b3World_GetCounters(state.world).jointCount;
    b3BodyId now=sk_actor(&state,bodies);uint16_t generation=b3Body_IsValid(now)?w->bodies.data[bodies].generation:0;
    SKBlob projection={0};ok=ok&&sk_project(&state,&projection);sk_blob_free(&projection);sk_close(&state);
    // Observe the actual last actor disappearance and slot/generation reuse.
    if(ok)ok=sk_import(sc_at(f,351),&state);
    uint16_t before=0;if(ok){w=b3GetWorldFromId(state.world);b3BodyId id=sk_actor(&state,bodies);if(b3Body_IsValid(id))before=w->bodies.data[bodies].generation;else ok=0;sk_close(&state);}
    if(ok)ok=sk_import(sc_at(f,352),&state);
    int absent=0;if(ok){absent=!b3Body_IsValid(sk_actor(&state,bodies));sk_close(&state);}
    f->reuse=ok&&absent&&generation>before;
    // Positive input is applied live to the same imported state, then body floats
    // are compared in the supplemental typed projection (not just a checksum).
    SKBlob positive={0},baseline_projection={0},positive_projection={0};
    if(ok)ok=sk_step(sc_at(f,359),360,1,&positive);
    if(ok)ok=sk_project_blob(sc_at(f,360),&baseline_projection)&&sk_project_blob(&positive,&positive_projection);
    f->positive=0;
    if(ok&&baseline_projection.size==positive_projection.size&&positive_projection.size>=16){
        for(size_t off=16;off+72<=positive_projection.size;off+=72){
            for(size_t k=8;k<64;k+=4){uint32_t a=sk_u32(baseline_projection.data+off+k),b=sk_u32(positive_projection.data+off+k);
                if(a!=b&&(a&0x7fffffffu)&&(b&0x7fffffffu))f->positive=1;
            }
        }
        uint8_t hash[32];sk_blob_hash(&positive,hash);f->positive=f->positive&&memcmp(hash,f->hashes+32*window,32)!=0;
    }
    sk_blob_free(&positive);sk_blob_free(&baseline_projection);sk_blob_free(&positive_projection);
    return ok&&f->positive&&f->reuse&&f->awake>0&&f->sleeping>0&&f->contacts>0&&f->joints==bodies/16;
}
static int sc_b_block(SCFixture* f,int horizon) {
    int start=SC_END-horizon;SKBlob temporary={0};
    for(int tick=start+1;tick<=SC_END;++tick){
        if(!sk_step(sc_at(f,tick-1),tick,0,&temporary)){sk_blob_free(&temporary);return 0;}
        // Full memcmp before replacing a slot preserves an independent expected
        // byte oracle. SHA covers the actual copied slot, every resimulated tick.
        if(!sc_verify(f,tick,&temporary)||!sk_blob_copy(sc_at(f,tick),&temporary)){sk_blob_free(&temporary);return 0;}
        uint8_t digest[32];sk_blob_hash(sc_at(f,tick),digest);
        if(memcmp(digest,f->hashes+32*(tick-f->first),32)){sk_blob_free(&temporary);return 0;}
        sk_blob_free(&temporary);
    }
    return 1;
}
static int sc_persistent_check(SCFixture* f,SKWorld* state) {
    SKBlob actual={0},expected={0};int ok=sk_project(state,&actual)&&sk_project_blob(sc_at(f,state->tick),&expected)&&sk_blob_equal(&actual,&expected);
    sk_blob_free(&actual);sk_blob_free(&expected);return ok;
}
// Metrics: step, capture, copy, hash, restore, persistent resim, full B.
static int sc_operation(SCFixture* f,int metric,int horizon,SCMeasurement* measured,int index,int timed) {
    SKWorld live={0};SKBlob output={0},copy={0};uint8_t hash[32];
    int tick=SC_END-(metric==5||metric==6?horizon:1),ok=1;
    SKBlob* source=sc_at(f,tick);
    if(metric==0||metric==1||metric==5)ok=sk_import(source,&live);
    if(metric==2)ok=sk_blob_reserve(&copy,source->size);
    // Preparation above is deliberately outside component timers. Full B does
    // not enter this preparation and includes its imports and intermediate frees.
    if(!ok)goto done;
    SCCounters before_engine=sc_counters[0],before_host=sc_counters[1];
    sc_reset_peaks();uint64_t begin=sc_now();
    switch(metric){
        case 0:ok=sk_command(&live,tick+1,0);if(ok){b3World_Step(live.world,1.0f/60.0f,4);live.tick=tick+1;}break;
        case 1:ok=sk_capture(&live,&output);break;
        case 2:ok=sk_blob_copy(&copy,source);break;
        case 3:sk_blob_hash(source,hash);break;
        case 4:ok=sk_import(source,&live);break;
        case 5:for(int t=tick+1;ok&&t<=SC_END;++t){ok=sk_command(&live,t,0);if(ok){b3World_Step(live.world,1.0f/60.0f,4);live.tick=t;}}break;
        case 6:ok=sc_b_block(f,horizon);break;
        default:ok=0;break;
    }
    uint64_t end=sc_now();if(end<begin){sc_clock_ok=0;ok=0;}
    if(sc_combined_peak>f->workload_peak)f->workload_peak=sc_combined_peak;
    if(sc_counters[0].peak_bytes>f->aligned_engine_peak)f->aligned_engine_peak=sc_counters[0].peak_bytes;
    if(sc_engine_logical_peak>f->logical_engine_peak)f->logical_engine_peak=sc_engine_logical_peak;
    if(timed){
        measured->ns[index]=end>=begin?end-begin:0;
        measured->engine_calls[index]=sc_counters[0].alloc_calls-before_engine.alloc_calls;
        measured->host_calls[index]=sc_counters[1].alloc_calls-before_host.alloc_calls;
        measured->engine_bytes[index]=sc_counters[0].allocated_bytes-before_engine.allocated_bytes;
        measured->host_bytes[index]=sc_counters[1].allocated_bytes-before_host.allocated_bytes;
        measured->peaks[index]=sc_combined_peak;
        measured->engine_frees[index]=sc_counters[0].free_calls-before_engine.free_calls;
        measured->host_frees[index]=sc_counters[1].free_calls-before_host.free_calls;
        measured->engine_freed[index]=sc_counters[0].freed_bytes-before_engine.freed_bytes;
        measured->host_freed[index]=sc_counters[1].freed_bytes-before_host.freed_bytes;
        measured->engine_live[index]=sc_counters[0].live_bytes;
        measured->host_live[index]=sc_counters[1].live_bytes;
    }
    if(ok){
        if(metric==0||metric==5)ok=sc_persistent_check(f,&live);
        else if(metric==1)ok=sc_verify(f,tick,&output);
        else if(metric==2)ok=sk_blob_equal(source,&copy);
        else if(metric==3)ok=!memcmp(hash,f->hashes+32*(tick-f->first),32);
        else if(metric==4)ok=sc_persistent_check(f,&live);
    }
done:
    sk_close(&live);sk_blob_free(&output);sk_blob_free(&copy);
    // Include preparation and verification storage in the workload footprint,
    // even where those operations are deliberately outside component timings.
    if(sc_combined_peak>f->workload_peak)f->workload_peak=sc_combined_peak;
    if(sc_counters[0].peak_bytes>f->aligned_engine_peak)f->aligned_engine_peak=sc_counters[0].peak_bytes;
    if(sc_engine_logical_peak>f->logical_engine_peak)f->logical_engine_peak=sc_engine_logical_peak;
    return ok&&sc_errors==0;
}
static const char* sc_metric(int metric) {
    static const char* names[]={"step_persistent","capture_complete","snapshot_copy","snapshot_hash","restore_fresh","resim_persistent","rollback_b"};
    return names[metric];
}
static void sc_emit_result(SCFixture* f,int metric,int horizon,SCMeasurement* m,int pass) {
    uint64_t median,p95;sc_percentiles(m->ns,SC_SAMPLES,&median,&p95);
    printf("{\"schema\":\"box3d-snapshotlab-v1\",\"driver\":\"cost\",\"type\":\"result\",\"profile\":\"%s\",\"policy\":\"%s\",\"result_index\":%d,\"bodies\":%d,\"window\":%d,\"slots\":%d,\"metric\":\"%s\",\"horizon\":%d,\"substeps\":4,\"workers_actual\":1,\"simd_actual\":4,\"backend_actual\":\"%s\",\"samples_ns\":",sc_profile,sk_policy(),sc_results++,f->bodies,f->window,f->slots,sc_metric(metric),horizon,sc_backend());
    sc_array_u64(stdout,m->ns,SC_SAMPLES);
    printf(",\"median_ns\":%" PRIu64 ",\"p95_ns\":%" PRIu64,median,p95);
    printf(",\"execution_pass\":%s,\"status\":\"%s\",\"timer_resolved\":%s,\"engine_aligned_alloc_calls\":",sc_bool(pass),pass?"complete":"gate_failed",sc_bool(median>=sc_observed_increment));
    sc_array_u64(stdout,m->engine_calls,SC_SAMPLES);printf(",\"host_alloc_calls\":");sc_array_u64(stdout,m->host_calls,SC_SAMPLES);
    printf(",\"engine_aligned_allocated_bytes\":");sc_array_u64(stdout,m->engine_bytes,SC_SAMPLES);
    printf(",\"host_allocated_bytes\":");sc_array_u64(stdout,m->host_bytes,SC_SAMPLES);
    printf(",\"engine_free_calls\":");sc_array_u64(stdout,m->engine_frees,SC_SAMPLES);
    printf(",\"host_free_calls\":");sc_array_u64(stdout,m->host_frees,SC_SAMPLES);
    printf(",\"engine_aligned_freed_bytes\":");sc_array_u64(stdout,m->engine_freed,SC_SAMPLES);
    printf(",\"host_freed_bytes\":");sc_array_u64(stdout,m->host_freed,SC_SAMPLES);
    printf(",\"engine_aligned_live_at_stop\":");sc_array_u64(stdout,m->engine_live,SC_SAMPLES);
    printf(",\"host_live_at_stop\":");sc_array_u64(stdout,m->host_live,SC_SAMPLES);
    printf(",\"combined_known_backing_peak_bytes\":");sc_array_u64(stdout,m->peaks,SC_SAMPLES);printf("}\n");
    if(sc_trace){
        for(int i=0;i<SC_SAMPLES;++i)fprintf(sc_trace,"{\"schema\":\"box3d-snapshotlab-cost-trace-v1\",\"type\":\"sample\",\"bodies\":%d,\"window\":%d,\"metric\":\"%s\",\"horizon\":%d,\"round\":%d,\"ns\":%" PRIu64 ",\"engine_aligned_calls\":%" PRIu64 ",\"host_calls\":%" PRIu64 ",\"known_backing_peak_bytes\":%" PRIu64 "}\n",f->bodies,f->window,sc_metric(metric),horizon,i,m->ns[i],m->engine_calls[i],m->host_calls[i],m->peaks[i]);
        if(ferror(sc_trace))sc_io_ok=0;
    }
}
static int sc_workload(int bodies,int window) {
    SCFixture f;int ok=sc_fixture_build(&f,bodies,window);
    if(!ok){sc_coverage=0;sc_fixture_free(&f);return 0;}
    sc_reset_peaks();f.workload_peak=sc_combined_peak;
    // >=64 complete adapter ticks, in addition to the 240+ fixture warmup ticks.
    for(int i=0;i<SC_WARMUPS&&ok;++i)ok=sc_b_block(&f,12);
    if(sc_combined_peak>f.workload_peak)f.workload_peak=sc_combined_peak;
    if(sc_engine_logical_peak>f.logical_engine_peak)f.logical_engine_peak=sc_engine_logical_peak;
    if(sc_counters[0].peak_bytes>f.aligned_engine_peak)f.aligned_engine_peak=sc_counters[0].peak_bytes;
    for(int metric=0;metric<7;++metric){
        int variants=metric>=5?4:1;static const int horizons[]={1,4,8,12};
        for(int j=0;j<variants;++j){
            int horizon=metric>=5?horizons[j]:metric==0?1:0;
            SCMeasurement m={0};int pass=ok;
            for(int warm=0;warm<SC_WARMUPS&&pass;++warm)pass=sc_operation(&f,metric,horizon,&m,0,0);
            for(int sample=0;sample<SC_SAMPLES;++sample){int row=sc_operation(&f,metric,horizon,&m,sample,1);pass=pass&&row;}
            sc_emit_result(&f,metric,horizon,&m,pass);ok=ok&&pass;
        }
    }
    char source_hash[65],end_hash[65];
    sk_sha256_hex(f.hashes+32*(348-f.first),source_hash);sk_sha256_hex(f.hashes+32*window,end_hash);
    printf("{\"schema\":\"box3d-snapshotlab-v1\",\"driver\":\"cost\",\"type\":\"workload\",\"profile\":\"%s\",\"policy\":\"%s\",\"bodies\":%d,\"window\":%d,\"slots\":%d,\"first_tick\":%d,\"last_tick\":360,\"substeps\":4,\"seed_tick\":240,\"warmup_adapter_ticks\":96,",sc_profile,sk_policy(),bodies,window,f.slots,f.first);
    printf("\"ring_payload_bytes\":%" PRIu64 ",\"ring_capacity_bytes\":%" PRIu64 ",\"registry_wire_bytes_included\":%" PRIu64 ",\"ring_metadata_bytes\":%zu,",f.payload_bytes,f.capacity_bytes,f.registry_bytes,(size_t)f.slots*(sizeof(SKBlob)+32));
    printf("\"known_backing_peak_bytes\":%" PRIu64 ",\"engine_logical_peak_bytes\":%" PRIu64 ",\"engine_aligned_peak_bytes\":%" PRIu64 ",",f.workload_peak,f.logical_engine_peak,f.aligned_engine_peak);
    printf("\"awake\":%d,\"sleeping\":%d,\"contacts\":%d,\"joints\":%d,\"reuse_observed\":%s,\"positive_body_response\":%s,\"coverage_pass\":true,\"execution_pass\":%s,\"secondary_stress\":%s,\"tick348_sha256\":\"%s\",\"tick360_sha256\":\"%s\"}\n",f.awake,f.sleeping,f.contacts,f.joints,sc_bool(f.reuse),sc_bool(f.positive),sc_bool(ok),sc_bool(bodies==1024),source_hash,end_hash);
    sc_fixture_free(&f);
    int clean=b3GetByteCount()==0&&sc_counters[0].live_bytes==0&&sc_counters[1].live_bytes==0;
    return ok&&clean;
}
int main(int argc,char** argv) {
    const char* trace_path=NULL;int workers=1,simd=4;
    for(int i=1;i<argc;++i){
        if(!strcmp(argv[i],"--profile")&&i+1<argc)sc_profile=argv[++i];
        else if(!strcmp(argv[i],"--workers")&&i+1<argc){if(strcmp(argv[++i],"1"))return 2;workers=1;}
        else if(!strcmp(argv[i],"--simd")&&i+1<argc){if(strcmp(argv[++i],"4"))return 2;simd=4;}
        else if(!strcmp(argv[i],"--trace")&&i+1<argc)trace_path=argv[++i];
        else return 2;
    }
    if(!sk_label(sc_profile)||workers!=1||simd!=4)return 2;
    int hash_ok=sk_sha256_selftest();if(!hash_ok)return 3;
#if defined(BOX3D_DISABLE_SIMD) || defined(BOX3D_DOUBLE_PRECISION)
    return 3;
#endif
    b3SetSIMDWidth(4);if(b3GetSIMDWidth()!=4||sizeof(void*)!=8||SNAPSHOTLAB_POLICY!=1)return 3;
    const uint32_t endian=1;if(*(const uint8_t*)&endian!=1)return 3;
    if(trace_path){sc_trace=fopen(trace_path,"wb");if(!sc_trace)return 3;}
    sc_clock_init();b3SetAllocator(sc_engine_alloc,sc_engine_free);sk_set_host_allocator(sc_host_alloc,sc_host_free);
    uint64_t empty[SC_SAMPLES];for(int i=0;i<SC_SAMPLES;++i){uint64_t a=sc_now(),b=sc_now();if(b<a)sc_clock_ok=0;empty[i]=b>=a?b-a:0;}
    printf("{\"schema\":\"box3d-snapshotlab-v1\",\"driver\":\"cost\",\"type\":\"criteria\",\"profile\":\"%s\",\"policy\":\"%s\",\"results_expected\":78,\"workloads_expected\":6,\"bodies\":[64,256,1024],\"windows\":[12,120],\"horizons\":[1,4,8,12],\"sample_count\":31,\"warmup_operations\":8,\"seed_tick\":240,\"end_tick\":360,\"substeps\":4,\"dt_bits\":\"3c888889\",\"hash\":\"SHA-256\",\"sha256_selftest\":true,\"percentile\":\"nearest_rank_30_of_31\",\"primary_pipeline\":\"fresh_import_command_live_step_capture_destroy_ring_copy_sha256_full_compare_per_tick\",\"managed_runtime_cost_included\":false,\"clock\":{\"name\":\"%s\",\"frequency\":%" PRIu64 ",\"nominal_resolution_ns\":%" PRIu64 ",\"observed_increment_ns\":%" PRIu64 ",\"empty_samples_ns\":",sc_profile,sk_policy(),sc_clock_name,sc_clock_frequency,sc_clock_resolution,sc_observed_increment);
    sc_array_u64(stdout,empty,SC_SAMPLES);printf("},\"allocator_scope\":\"engine_callback_aligned_sizes_and_all_core_host_buffers\",\"malloc_internal_overhead_known\":false,\"os_rss_measured\":false}\n");
    static const int bodies[]={64,256,1024},windows[]={12,120};
    for(int n=0;n<3;++n)for(int k=0;k<2;++k){int pass=sc_workload(bodies[n],windows[k]);sc_execution=sc_execution&&pass;}
    if(sc_trace){if(ferror(sc_trace))sc_io_ok=0;if(fclose(sc_trace))sc_io_ok=0;}
    int cleanup=b3GetByteCount()==0&&sc_counters[0].live_bytes==0&&sc_counters[1].live_bytes==0;
    sc_execution=sc_execution&&cleanup&&sc_clock_ok&&!sc_errors&&sc_results==78&&sc_io_ok;
    printf("{\"schema\":\"box3d-snapshotlab-v1\",\"driver\":\"cost\",\"type\":\"summary\",\"profile\":\"%s\",\"policy\":\"%s\",\"status\":\"%s\",\"execution_pass\":%s,\"coverage_pass\":%s,\"cleanup_pass\":%s,\"clock_valid\":%s,\"trace_io_ok\":%s,\"results\":%d,\"results_expected\":78,\"backend_actual\":\"%s\",\"simd_actual\":4,\"substeps\":4,\"workers_actual\":1,\"allocator_errors\":%" PRIu64 ",\"engine_logical_live_bytes\":%" PRId64 ",\"engine_aligned_live_bytes\":%" PRIu64 ",\"host_live_bytes\":%" PRIu64 "}\n",sc_profile,sk_policy(),sc_execution?"complete":"gate_failed",sc_bool(sc_execution),sc_bool(sc_coverage),sc_bool(cleanup),sc_bool(sc_clock_ok),sc_bool(sc_io_ok),sc_results,sc_backend(),sc_errors,b3GetByteCount(),sc_counters[0].live_bytes,sc_counters[1].live_bytes);
    return sc_execution?0:1;
}
