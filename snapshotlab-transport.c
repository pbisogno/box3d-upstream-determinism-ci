// SPDX-License-Identifier: MIT
// Synthetic foreign-seed import, new live inputs, and corrected-input rollback.
#ifndef _WIN32
#define _POSIX_C_SOURCE 200809L
#endif
#include "snapshotlab/snapshot-core.h"
#ifdef _WIN32
#include <io.h>
#define SK_DUP _dup
#define SK_DUP2 _dup2
#define SK_CLOSE _close
#define SK_FILENO _fileno
#else
#include <unistd.h>
#define SK_DUP dup
#define SK_DUP2 dup2
#define SK_CLOSE close
#define SK_FILENO fileno
#endif
enum { REF_HEADER=64, REF_ROW=160 };
typedef struct { int tick,branch;const char* kind;int offset; } Difference;
static Difference first={-1,-1,"none",-1};
static int trace_ok=1;
static size_t trace_bytes=0;
static int mark(int same,int branch,int tick,const char* kind) { if(!same&&first.tick<0)first=(Difference){tick,branch,kind,-1};return same; }
static void roundtrip_difference(const SKBlob* a,const SKBlob* b,int tick) { if(sk_blob_equal(a,b)||first.tick>=0)return;size_t n=a->size<b->size?a->size:b->size,i=SK_HEADER;while(i<n&&a->data[i]==b->data[i])++i;if(i==n){i=0;while(i<n&&a->data[i]==b->data[i])++i;}first=(Difference){tick,0,"seed_roundtrip_payload_bytes",(int)i}; }
static const char* truth(int n) { return n?"true":"false"; }
static int emit_trace(FILE* f,int branch,int tick,const SKBlob* p) {
    if(!f)return 0;uint8_t h[16]={0};sk_w32(h,(uint32_t)branch);sk_w32(h+4,(uint32_t)tick);sk_w32(h+8,(uint32_t)p->size);
    if(trace_bytes+sizeof(h)+p->size>SK_MAX_BYTES)return trace_ok=0;
    int ok=fwrite(h,1,sizeof(h),f)==sizeof(h)&&fwrite(p->data,1,p->size,f)==p->size;trace_bytes+=sizeof(h)+p->size;if(!ok)trace_ok=0;return ok;
}
static void body_hash(const SKBlob* observation,int bodies,uint8_t out[32]) { sk_sha256(observation->data+16,((size_t)bodies+1)*72,out); }
static int append_row(SKBlob* ref,int branch,int tick,const SKBlob* state,const SKBlob* obs,const SKCoverage* c) {
    size_t n=ref->size+REF_ROW;if(!sk_blob_reserve(ref,n))return 0;uint8_t* p=ref->data+ref->size;memset(p,0,REF_ROW);sk_w32(p,(uint32_t)branch);sk_w32(p+4,(uint32_t)tick);
    sk_blob_hash(state,p+8);sk_blob_hash(obs,p+40);body_hash(obs,(int)sk_u32(state->data+16),p+72);
    int values[13]={c->contacts,c->warm_points,c->asleep,c->awake,c->joints,c->body_reuses,c->shape_reuses,c->begin_events,c->end_events,c->hit_events,c->move_events,c->sleep_events,c->joint_events};
    for(int k=0;k<13;++k)sk_w32(p+104+4*k,(uint32_t)values[k]);ref->size=n;return 1;
}
static const uint8_t* find_row(const SKBlob* ref,int branch,int tick) {
    for(size_t i=REF_HEADER;i+REF_ROW<=ref->size;i+=REF_ROW){const uint8_t* p=ref->data+i;if(sk_u32(p)==(uint32_t)branch&&sk_u32(p+4)==(uint32_t)tick)return p;}return NULL;
}
static int validate_ref(const SKBlob* ref,const SKBlob* seed,int bodies,int checkpoint,int horizon) {
    if(ref->size<REF_HEADER||memcmp(ref->data,"SKREF1\0",8)||sk_u32(ref->data+8)!=1||sk_u32(ref->data+12)!=SNAPSHOTLAB_POLICY||sk_u32(ref->data+16)!=(uint32_t)bodies||sk_u32(ref->data+20)!=(uint32_t)checkpoint||sk_u32(ref->data+24)!=(uint32_t)horizon)return 0;
    uint32_t count=(uint32_t)(horizon-checkpoint+1+16);if(sk_u32(ref->data+28)!=count||ref->size!=REF_HEADER+(size_t)count*REF_ROW)return 0;
    uint8_t hash[32];sk_blob_hash(seed,hash);if(memcmp(hash,ref->data+32,32))return 0;
    size_t offset=REF_HEADER;for(int branch=0;branch<=12;branch=branch==0?4:branch==4?12:13){int start=branch?horizon-branch+1:checkpoint;
        for(int tick=start;tick<=horizon;++tick){const uint8_t* p=ref->data+offset;if(sk_u32(p)!=(uint32_t)branch||sk_u32(p+4)!=(uint32_t)tick||sk_u32(p+156))return 0;offset+=REF_ROW;}}
    return offset==ref->size;
}
static void accumulate(SKCoverage* a,const SKCoverage* b) {
    if(b->contacts>a->contacts)a->contacts=b->contacts;if(b->warm_points>a->warm_points)a->warm_points=b->warm_points;if(b->asleep>a->asleep)a->asleep=b->asleep;if(b->awake>a->awake)a->awake=b->awake;if(b->joints>a->joints)a->joints=b->joints;if(b->body_reuses>a->body_reuses)a->body_reuses=b->body_reuses;if(b->shape_reuses>a->shape_reuses)a->shape_reuses=b->shape_reuses;
    a->begin_events+=b->begin_events;a->end_events+=b->end_events;a->hit_events+=b->hit_events;a->move_events+=b->move_events;a->sleep_events+=b->sleep_events;a->joint_events+=b->joint_events;
}
static int make_reference(int bodies,int checkpoint,int horizon,SKBlob* seed,SKBlob* ref,FILE* trace,SKCoverage* coverage) {
    if(!sk_blob_reserve(ref,REF_HEADER))return 0;ref->size=REF_HEADER;memset(ref->data,0,REF_HEADER);memcpy(ref->data,"SKREF1",6);sk_w32(ref->data+8,1);sk_w32(ref->data+12,SNAPSHOTLAB_POLICY);sk_w32(ref->data+16,(uint32_t)bodies);sk_w32(ref->data+20,(uint32_t)checkpoint);sk_w32(ref->data+24,(uint32_t)horizon);sk_w32(ref->data+28,(uint32_t)(horizon-checkpoint+17));
    int ok=1;SKBlob state={0},obs={0};
    for(int branch=0;ok&&branch<=12;branch=branch==0?4:branch==4?12:13){
        SKWorld world={0};ok=sk_create(bodies,&world);int cut=branch?horizon-branch:checkpoint;
        for(int tick=1;ok&&tick<=horizon;++tick){
            int input=branch&&tick==cut+1;ok=sk_command(&world,tick,input);if(!ok)break;b3World_Step(world.world,1.0f/60.0f,4);world.tick=tick;
            if(!branch&&tick==checkpoint){ok=sk_capture(&world,seed);if(ok)sk_blob_hash(seed,ref->data+32);}
            if((!branch&&tick>=checkpoint)||(branch&&tick>cut)){
                SKCoverage c=sk_coverage(&world,tick!=checkpoint||branch!=0);accumulate(coverage,&c);
                ok=ok&&sk_capture(&world,&state)&&sk_observe(&world,tick!=checkpoint||branch!=0,&obs)&&append_row(ref,branch,tick,&state,&obs,&c)&&emit_trace(trace,branch,tick,&obs);
            }
        }sk_close(&world);
    }sk_blob_free(&state);sk_blob_free(&obs);return ok;
}
typedef struct { int state,observation,body; } Comparison;
static int compare_row(const SKBlob* ref,int branch,int tick,const SKBlob* state,const SKBlob* obs,Comparison* c) {
    const uint8_t* row=find_row(ref,branch,tick);if(!row)return 0;uint8_t h[32];sk_blob_hash(state,h);int a=memcmp(h,row+8,32)==0;sk_blob_hash(obs,h);int b=memcmp(h,row+40,32)==0;body_hash(obs,(int)sk_u32(state->data+16),h);int d=memcmp(h,row+72,32)==0;
    c->state&=mark(a,branch,tick,"authority_sha256");c->observation&=mark(b,branch,tick,"typed_observation_sha256");c->body&=mark(d,branch,tick,"body_sha256");return 1;
}
static int corrupt_controls(const SKBlob* seed,int* rejected) {
    SKWorld guard={0};SKBlob before={0},after={0},bad={0};if(!sk_import(seed,&guard))return 0;int ok=sk_capture(&guard,&before);*rejected=0;
    for(int i=0;ok&&i<9;++i){ok=sk_blob_copy(&bad,seed);if(!ok)break;
        if(i==0)bad.data[0]^=1;else if(i==1)sk_w32(bad.data+8,2);else if(i==2)--bad.size;else if(i==3)sk_w64(bad.data+24,UINT64_MAX);else if(i==4)bad.data[48]^=1;else if(i==7)sk_w32(bad.data+12,99);else if(i==8)sk_w32(bad.data+20,sk_u32(bad.data+20)+1);else{bad.data[SK_HEADER+(i==5?0:8)]^=1;sk_envelope_hash(bad.data,bad.size,bad.data+48);}
        uint8_t h1[32],h2[32];sk_blob_hash(&bad,h1);int64_t bytes=b3GetByteCount();uint64_t host=sk_host_live_bytes;SKWorld temp={0};int accepted=sk_import(&bad,&temp);if(accepted)sk_close(&temp);sk_blob_hash(&bad,h2);
        ok=!accepted&&!memcmp(h1,h2,32)&&b3GetByteCount()==bytes&&sk_host_live_bytes==host&&sk_capture(&guard,&after)&&sk_blob_equal(&before,&after);if(ok)++*rejected;
    }sk_close(&guard);sk_blob_free(&before);sk_blob_free(&after);sk_blob_free(&bad);return ok&&*rejected==9;
}
typedef struct { int h,steps,sham_steps,executed,state,observation,body,sham_state,sham_observation,positive_body,positive_checksum; } Rollback;
static int consume(const SKBlob* seed,const SKBlob* ref,int checkpoint,int horizon,FILE* trace,SKCoverage* coverage,Comparison* baseline,int* roundtrip,Rollback rb[2],int* steps,int* rejected) {
    SKWorld initial={0};SKBlob current={0},next={0},obs={0},captured={0},cuts[2]={{0},{0}};int ok=sk_import(seed,&initial);if(!ok)return 0;
    ok=sk_capture(&initial,&captured)&&sk_observe(&initial,0,&obs);*roundtrip=ok&&sk_blob_equal(seed,&captured);if(ok){roundtrip_difference(seed,&captured,checkpoint);ok=compare_row(ref,0,checkpoint,&captured,&obs,baseline)&&emit_trace(trace,0,checkpoint,&obs);}sk_close(&initial);
    ok=ok&&sk_blob_copy(&current,seed);int hs[2]={4,12};
    for(int tick=checkpoint+1;ok&&tick<=horizon;++tick){SKCoverage c={0};ok=sk_step_observed(&current,tick,0,&next,&obs,&c);if(!ok)break;++*steps;accumulate(coverage,&c);ok=compare_row(ref,0,tick,&next,&obs,baseline)&&emit_trace(trace,0,tick,&obs);
        for(int i=0;i<2;++i)if(tick==horizon-hs[i])ok=ok&&sk_blob_copy(cuts+i,&next);SKBlob tmp=current;current=next;next=tmp;
    }
    for(int i=0;ok&&i<2;++i){Rollback* b=rb+i;b->h=hs[i];b->state=b->observation=b->body=b->sham_state=b->sham_observation=1;int cut=horizon-b->h;
        for(int corrected=0;ok&&corrected<2;++corrected){ok=sk_blob_copy(&current,cuts+i);Comparison comparison={1,1,1};
            for(int tick=cut+1;ok&&tick<=horizon;++tick){ok=sk_step_observed(&current,tick,corrected&&tick==cut+1,&next,&obs,NULL);if(!ok)break;
                int branch=corrected?b->h:0;ok=compare_row(ref,branch,tick,&next,&obs,&comparison);
                if(corrected){++b->steps;ok=ok&&emit_trace(trace,branch,tick,&obs);if(tick==cut+1){const uint8_t* row=find_row(ref,0,tick);uint8_t hash[32];sk_blob_hash(&next,hash);b->positive_checksum=row&&memcmp(hash,row+8,32)!=0;body_hash(&obs,(int)sk_u32(seed->data+16),hash);b->positive_body=row&&memcmp(hash,row+72,32)!=0;}}
                else ++b->sham_steps;SKBlob tmp=current;current=next;next=tmp;
            }
            if(corrected){b->state=comparison.state;b->observation=comparison.observation;b->body=comparison.body;}
            else{b->sham_state=comparison.state;b->sham_observation=comparison.observation;}
        }b->executed=b->steps==b->h&&b->sham_steps==b->h;
    }
    if(ok)ok=corrupt_controls(seed,rejected);
    sk_blob_free(&current);sk_blob_free(&next);sk_blob_free(&obs);sk_blob_free(&captured);for(int i=0;i<2;++i)sk_blob_free(cuts+i);return ok;
}
int main(int argc,char** argv) {
    const char* mode=NULL,*profile=NULL,*seed_path=NULL,*ref_path=NULL,*trace_path=NULL;int bodies=0,checkpoint=0,horizon=0;
    if(argc>1)mode=argv[1];for(int i=2;i+1<argc;i+=2){const char* k=argv[i],*v=argv[i+1];if(!strcmp(k,"--profile"))profile=v;else if(!strcmp(k,"--bodies"))bodies=atoi(v);else if(!strcmp(k,"--checkpoint"))checkpoint=atoi(v);else if(!strcmp(k,"--horizon"))horizon=atoi(v);else if(!strcmp(k,"--seed"))seed_path=v;else if(!strcmp(k,"--reference"))ref_path=v;else if(!strcmp(k,"--trace"))trace_path=v;else return 64;}
    int producer=mode&&!strcmp(mode,"produce");if(!mode||(!producer&&strcmp(mode,"consume"))||!profile||!sk_label(profile)||!seed_path||!ref_path||!trace_path||(argc%2)!=0||(bodies!=64&&bodies!=256&&bodies!=1024)||(checkpoint!=60&&checkpoint!=120)||horizon!=256)return 64;
    fflush(stdout);int saved=SK_DUP(SK_FILENO(stdout));if(saved<0)return 5;if(SK_DUP2(SK_FILENO(stderr),SK_FILENO(stdout))<0){SK_CLOSE(saved);return 5;}
    int selftest=sk_sha256_selftest(),config=sk_runtime_valid(),ok=selftest&&config;SKBlob seed={0},ref={0};SKCoverage coverage={0};Comparison baseline={1,1,1};Rollback rb[2]={{0},{0}};int roundtrip=1,steps=0,rejected=0;
    FILE* trace=fopen(trace_path,"wb");if(!trace)ok=0;else{uint8_t h[16]={0};memcpy(h,"SKTRACE1",8);sk_w32(h+8,1);sk_w32(h+12,SNAPSHOTLAB_POLICY);ok=ok&&fwrite(h,1,sizeof(h),trace)==sizeof(h);trace_bytes=sizeof(h);}
    if(producer){if(ok)ok=make_reference(bodies,checkpoint,horizon,&seed,&ref,trace,&coverage);if(ok)ok=sk_validate(&seed)&&validate_ref(&ref,&seed,bodies,checkpoint,horizon)&&sk_write_file(seed_path,&seed)&&sk_write_file(ref_path,&ref);for(size_t off=REF_HEADER;off+REF_ROW<=ref.size;off+=REF_ROW)if(sk_u32(ref.data+off)==0&&sk_u32(ref.data+off+4)>(uint32_t)checkpoint)++steps;}
    else{if(ok)ok=sk_read_file(seed_path,&seed)&&sk_read_file(ref_path,&ref)&&sk_validate(&seed)&&validate_ref(&ref,&seed,bodies,checkpoint,horizon);if(ok)ok=consume(&seed,&ref,checkpoint,horizon,trace,&coverage,&baseline,&roundtrip,rb,&steps,&rejected);}
    if(producer&&ok){SKWorld temp={0};SKBlob captured={0};ok=sk_import(&seed,&temp);if(ok)ok=sk_capture(&temp,&captured);roundtrip=ok&&sk_blob_equal(&seed,&captured);if(ok)roundtrip_difference(&seed,&captured,checkpoint);sk_close(&temp);sk_blob_free(&captured);}
    if(trace&&fclose(trace))trace_ok=0;
    int covered=coverage.contacts>0&&coverage.warm_points>0&&coverage.asleep>0&&coverage.joints==bodies/16&&coverage.body_reuses>0&&coverage.shape_reuses>0;
    int rollback=producer||(rb[0].executed&&rb[1].executed);int positive=0;
    if(producer){positive=1;int hs[2]={4,12};for(int i=0;i<2;++i){const uint8_t* a=find_row(&ref,0,horizon-hs[i]+1),*b=find_row(&ref,hs[i],horizon-hs[i]+1);positive=positive&&a&&b&&memcmp(a+8,b+8,32)!=0&&memcmp(a+72,b+72,32)!=0;}}
    else positive=rb[0].positive_body&&rb[0].positive_checksum&&rb[1].positive_body&&rb[1].positive_checksum;
    int equality=roundtrip&&baseline.state&&baseline.observation&&baseline.body;
    if(!producer)for(int i=0;i<2;++i)equality=equality&&rb[i].state&&rb[i].observation&&rb[i].body&&rb[i].sham_state&&rb[i].sham_observation;
    size_t seed_bytes=seed.size,sn=seed.size>=SK_HEADER?(size_t)sk_u64(seed.data+24):0,rn=seed.size>=SK_HEADER?(size_t)sk_u64(seed.data+32):0;char seed_hash[65]={0};uint8_t hash[32];sk_blob_hash(&seed,hash);sk_hex(hash,32,seed_hash);
    config=config&&sk_seen_simd==4&&sk_seen_workers==1;
    sk_blob_free(&seed);sk_blob_free(&ref);int cleanup=b3GetByteCount()==0&&sk_host_live_bytes==0;int execution=ok&&selftest&&config&&trace_ok&&rollback&&positive&&covered&&cleanup;
    fflush(stdout);if(SK_DUP2(saved,SK_FILENO(stdout))<0){SK_CLOSE(saved);return 5;}SK_CLOSE(saved);
    printf("{\"schema\":\"box3d-snapshotlab-v1\",\"driver\":\"transport\",\"type\":\"result\",\"mode\":\"%s\",\"profile\":\"%s\",\"policy\":\"%s\",\"bodies\":%d,\"checkpoint\":%d,\"horizon\":%d,\"actual_workers\":%d,\"actual_simd\":%d,\"status\":\"%s\",\"execution_pass\":%s,\"live_steps\":%d,\"imported_recorded_frames\":%d,\"seed_bytes\":%zu,\"seed_snapshot_bytes\":%zu,\"seed_registry_bytes\":%zu,\"seed_sha256\":\"%s\",\"trace_bytes\":%zu,\"execution_gates\":{\"io_and_native_calls\":%s,\"sha_selftest\":%s,\"configuration\":%s,\"trace_io\":%s,\"coverage\":%s,\"rollback_executed\":%s,\"positive_body_and_checksum_control\":%s,\"cleanup\":%s},\"seed_roundtrip_equal\":%s,\"baseline_state_equal\":%s,\"baseline_observation_equal\":%s,\"baseline_body_equal\":%s,\"corrupt_controls_rejected\":%d,\"coverage\":{\"max_contacts\":%d,\"max_warm_points\":%d,\"max_asleep\":%d,\"max_awake\":%d,\"joints\":%d,\"body_reuses\":%d,\"shape_reuses\":%d,\"begin_events\":%d,\"end_events\":%d,\"hit_events\":%d,\"move_events\":%d,\"sleep_events\":%d,\"joint_events\":%d},\"rollback_cases\":[",mode,profile,sk_policy(),bodies,checkpoint,horizon,sk_seen_workers,sk_seen_simd,execution&&equality?"PASS":"FAIL",truth(execution),steps,sk_imported_frames,seed_bytes,sn,rn,seed_hash,trace_bytes,truth(ok),truth(selftest),truth(config),truth(trace_ok),truth(covered),truth(rollback),truth(positive),truth(cleanup),truth(roundtrip),truth(baseline.state),truth(baseline.observation),truth(baseline.body),rejected,coverage.contacts,coverage.warm_points,coverage.asleep,coverage.awake,coverage.joints,coverage.body_reuses,coverage.shape_reuses,coverage.begin_events,coverage.end_events,coverage.hit_events,coverage.move_events,coverage.sleep_events,coverage.joint_events);
    if(!producer)for(int i=0;i<2;++i){Rollback* b=rb+i;printf("%s{\"ticks\":%d,\"corrected_steps\":%d,\"sham_steps\":%d,\"state_equal\":%s,\"observation_equal\":%s,\"body_equal\":%s,\"sham_state_equal\":%s,\"sham_observation_equal\":%s,\"positive_body\":%s,\"positive_checksum\":%s}",i?",":"",b->h,b->steps,b->sham_steps,truth(b->state),truth(b->observation),truth(b->body),truth(b->sham_state),truth(b->sham_observation),truth(b->positive_body),truth(b->positive_checksum));}
    printf("],\"first_difference\":{\"tick\":%d,\"branch\":%d,\"kind\":\"%s\",\"byte_offset\":%d},\"scope\":\"full_authority_hash_each_tick;typed_body_contact_joint_new_event_projection;no_recorded_future_playback;seed_event_buffers_excluded\"}\n",first.tick,first.branch,first.kind,first.offset);
    return execution?(equality?0:1):5;
}
