// SPDX-License-Identifier: MIT
// Synthetic observations of the pinned public solver, with an uninstrumented twin.
#define main frontier_frozen_stress_main
#include "stress-replay.c"
#undef main
#include "physics_world.h"
#include "contact.h"
#include "frontier/frontier-hook.h"
#include <stddef.h>
#include <math.h>
#ifndef FRONTIER_INSTRUMENTED
#define FRONTIER_INSTRUMENTED 0
#endif

enum { FC_HORIZON=512, FC_HOOK_EVENTS=128 };
typedef struct { int index,manifold; uint32_t generation,before,after; b3Manifold* storage; } FCTarget;
typedef struct { int stage,width,lane,count; uint32_t bits[6]; } FCEvent;
typedef struct { int count,overflow; FCEvent events[FC_HOOK_EVENTS]; } FCLog;
static FCLog fc_logs[3];
static const b3World* fc_world;
static FCTarget fc_target;
static int fc_branch=-1;

static uint32_t fc_bits(float v) { uint32_t b; memcpy(&b,&v,4); return b; }
// Only a single contact/manifold is selected. Solver graph ownership serializes
// its stage accesses; the driver changes the selection only while Step is idle.
void frontierCacheObserve(int stage,const b3World* world,const b3Contact* contact,
    int manifold,int width,int lane,const float* values,int count)
{
    if (fc_branch<0 || world!=fc_world || contact->contactId!=fc_target.index ||
        contact->generation!=fc_target.generation || manifold!=fc_target.manifold) return;
    FCLog* log=fc_logs+fc_branch;
    if (count>6 || log->count==FC_HOOK_EVENTS) { log->overflow=1; return; }
    FCEvent* e=log->events+log->count++; e->stage=stage; e->width=width; e->lane=lane; e->count=count;
    for (int i=0;i<count;++i) e->bits[i]=fc_bits(values[i]);
}
static const FCEvent* fc_first(const FCLog* log,int stage)
{ for (int i=0;i<log->count;++i) if (log->events[i].stage==stage) return log->events+i; return NULL; }
static int fc_stage_count(const FCLog* log,int stage)
{ int n=0; for (int i=0;i<log->count;++i) n+=log->events[i].stage==stage; return n; }
static uint64_t fc_hash(uint64_t h,uint32_t word)
{ for (int i=0;i<4;++i) { h^=(word>>(8*i))&255u; h*=UINT64_C(1099511628211); } return h; }
static uint64_t fc_digest(uint64_t h,const Trace* t)
{ h=fc_hash(h,(uint32_t)t->count); for (int i=0;i<t->count;++i) { h=fc_hash(h,t->cells[i].tag); h=fc_hash(h,t->cells[i].value); } return h; }
static int fc_observe(const Scene* s,int tick,Trace* t)
{
    observationTick=tick; t->count=0; b3Counters c=b3World_GetCounters(s->world);
    int counts[]={tick,c.bodyCount,c.shapeCount,c.contactCount,c.jointCount,c.islandCount,b3World_GetAwakeBodyCount(s->world)};
    for (int j=0;j<7;++j) if (!scalar(t,tag(1,0,j+1),(uint32_t)counts[j])) return 0;
    for (int i=0;i<ROLES;++i) {
        b3BodyId id=s->body[i]; int valid=b3Body_IsValid(id); uint32_t base=tag(2,i,1);
        if (!scalar(t,base,(uint32_t)valid)) return 0; if (!valid) continue;
        b3WorldTransform x=b3Body_GetTransform(id);
        if (!scalar(t,base+1,(uint32_t)b3Body_GetType(id)) || !scalar(t,base+2,b3Body_IsAwake(id)) ||
            !position(t,base+3,x.p) || !vector(t,base+6,x.q.v) || !real(t,base+9,x.q.s) ||
            !vector(t,base+10,b3Body_GetLinearVelocity(id)) || !vector(t,base+13,b3Body_GetAngularVelocity(id))) return 0;
    }
    return 1;
}
static int fc_trace(FILE* f,int trial,int branch,int tick,const Trace* t)
{
    if (!f) return 1;
    return put32(f,(uint32_t)trial) && put32(f,(uint32_t)branch) && put32(f,(uint32_t)tick) && write_trace(f,t);
}
static int fc_equal(const Trace* a,const Trace* b)
{ return a->count==b->count && memcmp(a->cells,b->cells,(size_t)a->count*sizeof(Cell))==0; }
static int fc_differences(const Trace* a,const Trace* b,int bodiesOnly,int* zero)
{
    int n=0; if (a->count!=b->count) return -1;
    for (int i=0;i<a->count;++i) {
        if (a->cells[i].tag!=b->cells[i].tag) return -1;
        if (bodiesOnly && a->cells[i].tag>>24!=2) continue;
        uint32_t x=a->cells[i].value,y=b->cells[i].value;
        if (x!=y) { ++n; if ((x&0x7fffffffu)==0 && (y&0x7fffffffu)==0) ++*zero; }
    }
    return n;
}
static int fc_select(const Scene* s,int wantNonzero,int overflow,FCTarget* target)
{
    b3World* w=b3GetWorldFromId(s->world);
    for (int i=0;i<w->contacts.count;++i) {
        b3Contact* c=w->contacts.data+i;
        if (c->contactId!=i || c->setIndex!=b3_awakeSet || !(c->flags&b3_contactTouchingFlag) ||
            c->manifoldCount<1 || !c->manifolds) continue;
        if ((c->colorIndex==B3_OVERFLOW_INDEX)!=overflow) continue;
        for (int m=0;m<c->manifoldCount;++m) {
            b3Manifold* p=c->manifolds+m; uint32_t b=fc_bits(p->twistImpulse);
            int nonzero=(b&0x7fffffffu)!=0;
            if (p->pointCount<1 || nonzero!=wantNonzero || !isfinite(p->twistImpulse)) continue;
            *target=(FCTarget){i,m,c->generation,b,b,p}; return 1;
        }
    }
    return 0;
}
static int fc_make(Scene* s,int kind,int workers)
{
    Stats unused;
    if (kind<5) {
        if (!make_scene(s,kind==4?1:kind,workers,&unused)) return 0;
        if (kind==4) {
            b3World_EnableSleeping(s->world,false);
            b3Body_SetAngularVelocity(s->body[1],(b3Vec3){0,2,0});
        }
        return 1;
    }
    memset(s,0,sizeof(*s)); s->kind=5; s->workers=workers;
    b3WorldDef wd=b3DefaultWorldDef(); wd.workerCount=(uint32_t)workers; wd.enableSleep=false;
    wd.gravity=(b3Vec3){0,-10,0}; s->world=b3CreateWorld(&wd);
    if (!b3World_IsValid(s->world)) return 0;
    b3BodyDef bd=b3DefaultBodyDef(); s->body[0]=b3CreateBody(s->world,&bd); b3Body_SetName(s->body[0],names[0]);
    b3ShapeDef sd=b3DefaultShapeDef(); sd.baseMaterial.friction=0.5f;
    b3BoxHull floor=b3MakeBoxHull(2,0.5f,2);
    for (int i=0;i<30;++i) b3CreateHullShape(s->body[0],&sd,&floor.base);
    bd.type=b3_dynamicBody; bd.position=(b3Pos){0,0.84f,0}; bd.angularVelocity=(b3Vec3){0,2,0};
    s->body[1]=b3CreateBody(s->world,&bd); b3Body_SetName(s->body[1],names[1]);
    b3BoxHull box=b3MakeBoxHull(0.35f,0.35f,0.35f); b3CreateHullShape(s->body[1],&sd,&box.base);
    return 1;
}
static void fc_print_hooks(const FCLog* log)
{
    printf("{\"overflow\":%s,\"stage_counts\":[%d,%d,%d,%d],\"events\":[",log->overflow?"true":"false",
        fc_stage_count(log,1),fc_stage_count(log,2),fc_stage_count(log,3),fc_stage_count(log,4));
    for (int i=0;i<log->count;++i) {
        const FCEvent* e=log->events+i; if (i) putchar(',');
        printf("{\"stage\":%d,\"width\":%d,\"lane\":%d,\"bits\":[",e->stage,e->width,e->lane);
        for (int j=0;j<e->count;++j) { if (j) putchar(','); printf("\"%08x\"",e->bits[j]); }
        fputs("]}",stdout);
    }
    fputs("]}",stdout);
}
static int fc_run(int trial,int kind,int mode,int workers,const char* profile,FILE* output)
{
    const char* sceneNames[]={"none","contacts","joints","lifecycle","torsion","overflow"};
    const char* modes[]={"none","zero_sign","positive","ulp"};
    Scene s[3]={{0}}; Stats unused[3]={{0}}; Trace t[3]; FCTarget target[3]={{0}};
    uint64_t hashes[3]={UINT64_C(14695981039346656037),UINT64_C(14695981039346656037),UINT64_C(14695981039346656037)};
    int ok=1,selected=0,at=-1,written=0,local=1,control=1,prefix=1,width=0,actual=0;
    int firstBody=-1,bodyTicks=0,bodyCells=0,bodyZero=0,comparisonCount=0,firstProjection=-1;
    int fixed=kind==1?47:(kind==2?128:(kind==3?52:(kind==5?2:-1)));
    memset(fc_logs,0,sizeof(fc_logs)); fc_branch=-1;
    for (int b=0;b<3;++b) if (!fc_make(s+b,kind,workers)) ok=0;
    for (int tick=0;ok && tick<=FC_HORIZON;++tick) {
        if (tick>0) {
            for (int b=0;b<3;++b) {
                if (kind<=3 && !commands(s+b,tick,unused+b)) ok=0;
                fc_branch=selected && tick==at+1?b:-1;
                if (fc_branch>=0) { fc_world=b3GetWorldFromId(s[b].world); fc_target=target[b]; }
                b3World_Step(s[b].world,1.0f/60.0f,4); fc_branch=-1;
            }
        }
        for (int b=0;b<3;++b) {
            const b3World* w=b3GetWorldFromId(s[b].world); int measured=w->simdWidth;
            if (!width) width=measured;
            if (width!=measured || (width!=4 && width!=8) || b3World_GetWorkerCount(s[b].world)!=workers) ok=0;
            if (!fc_observe(s+b,tick,t+b)) ok=0;
            hashes[b]=fc_digest(hashes[b],t+b);
            if (!fc_trace(output,trial,b,tick,t+b)) ok=0;
        }
        ++comparisonCount;
        if (!fc_equal(t,t+1)) control=0;
        if (!selected && !fc_equal(t,t+2)) prefix=0;
        if (selected && tick>at) {
            int zero=0,n=fc_differences(t,t+2,1,&zero);
            if (n<0) { ok=0; break; }
            if (n>0) { if (firstBody<0) firstBody=tick; ++bodyTicks; bodyCells+=n; bodyZero+=zero; }
            if (!fc_equal(t,t+2) && firstProjection<0) firstProjection=tick;
        }
        int selectNow=!selected && (fixed>=0?tick==fixed:(tick>=32 && tick<=256));
        if (selectNow) {
            int nonzero=kind==4 || (kind==5 && mode!=1);
            // Overflow is used to prove the scalar path; its positive control
            // may replace any finite impulse, while torsion ULP uses nonzero.
            int found=fc_select(s,nonzero,kind==5,target);
            if (!found && kind==5) found=fc_select(s,!nonzero,1,target);
            if (!found) { if (fixed>=0) ok=0; continue; }
            for (int b=1;b<3;++b) {
                b3World* w=b3GetWorldFromId(s[b].world); FCTarget a=target[0];
                if (a.index>=w->contacts.count) { ok=0; continue; }
                b3Contact* c=w->contacts.data+a.index;
                if (c->generation!=a.generation || c->manifoldCount<=a.manifold || !c->manifolds) { ok=0; continue; }
                a.storage=c->manifolds+a.manifold; a.before=fc_bits(a.storage->twistImpulse); a.after=a.before;
                target[b]=a; if (a.before!=target[0].before) ok=0;
            }
            if (!ok) break;
            selected=1; at=tick; FCTarget* a=target+2; uint32_t bits=a->before;
            if (mode==1) { if ((bits&0x7fffffffu)!=0) { ok=0; break; } bits^=0x80000000u; }
            else if (mode==2) bits=fc_bits(0.03125f);
            else if (mode==3) {
                if ((bits&0x7fffffffu)==0) { ok=0; break; }
                bits=fc_bits(nextafterf(a->storage->twistImpulse,INFINITY));
            }
            a->after=bits;
            unsigned char before[sizeof(b3Manifold)],expected[sizeof(b3Manifold)];
            memcpy(before,a->storage,sizeof(before)); memcpy(expected,before,sizeof(expected));
            memcpy(expected+offsetof(b3Manifold,twistImpulse),&bits,4);
            memcpy(&a->storage->twistImpulse,&bits,4);
            written=fc_bits(a->storage->twistImpulse)==bits && (mode==0 || bits!=a->before);
            local=memcmp(expected,a->storage,sizeof(expected))==0;
        }
    }
    const FCEvent* prepared=fc_first(fc_logs+2,1); const FCEvent* used=fc_first(fc_logs+2,2);
    const FCEvent* apply=fc_first(fc_logs+2,3); const FCEvent* reference=fc_first(fc_logs,3);
    int rawMatch=prepared && prepared->bits[0]==target[2].after;
    int usedMatch=used && prepared && used->bits[0]==prepared->bits[1];
    int applyDiff=0,applyNonzero=0,completeStages=1;
    if (apply && reference && apply->count==reference->count) for (int j=0;j<apply->count;++j)
        if (apply->bits[j]!=reference->bits[j]) { ++applyDiff; if ((apply->bits[j]&0x7fffffffu)!=0 || (reference->bits[j]&0x7fffffffu)!=0) ++applyNonzero; }
    for (int b=0;b<3;++b) for (int stage=1;stage<=4;++stage)
        if (!fc_first(fc_logs+b,stage) || fc_logs[b].overflow) completeStages=0;
    int instrumentGate=!FRONTIER_INSTRUMENTED || (completeStages && rawMatch && usedMatch && (mode!=2 || applyNonzero>0));
    ok=ok && selected && written && local && control && prefix && comparisonCount==FC_HORIZON+1 && instrumentGate;
    if (s[0].world.index1) actual=b3World_GetWorkerCount(s[0].world);
    printf("{\"schema\":\"box3d-frontier-v1\",\"driver\":\"cache\",\"type\":\"trial\",\"profile\":\"%s\",\"case\":\"%s\",\"mode\":\"%s\",\"trial\":%d,\"status\":\"%s\",\"instrumented\":%s,\"workers_requested\":%d,\"workers_actual\":%d,\"simd_actual\":%d,\"horizon\":%d,\"intervention_tick\":%d,\"observed_states\":%d,\"target\":{\"contact_index\":%d,\"generation\":%u,\"manifold\":%d,\"before\":\"%08x\",\"after\":\"%08x\"},\"gates\":{\"activation\":%s,\"locality_and_readback\":%s,\"reference_sham_exact\":%s,\"prefix_exact\":%s,\"all_hook_stages\":%s,\"raw_loaded_matches\":%s,\"prepared_used_matches\":%s,\"instrumentation_contract\":%s},\"warm_application_differing_components\":%d,\"warm_application_nonzero_differences\":%d,\"future_body\":{\"first_tick\":%d,\"differing_ticks\":%d,\"cells\":%d,\"signed_zero_cells\":%d},\"first_projection_tick\":%d,\"body_projection_digests\":[\"%016llx\",\"%016llx\",\"%016llx\"],\"hooks\":[",
        profile,sceneNames[kind],modes[mode],trial,ok?"complete":"gate_failed",FRONTIER_INSTRUMENTED?"true":"false",workers,actual,width,FC_HORIZON,at,comparisonCount,
        target[2].index,target[2].generation,target[2].manifold,target[2].before,target[2].after,selected?"true":"false",written&&local?"true":"false",control?"true":"false",prefix?"true":"false",completeStages?"true":"false",rawMatch?"true":"false",usedMatch?"true":"false",instrumentGate?"true":"false",applyDiff,applyNonzero,firstBody,bodyTicks,bodyCells,bodyZero,firstProjection,(unsigned long long)hashes[0],(unsigned long long)hashes[1],(unsigned long long)hashes[2]);
    for (int b=0;b<3;++b) { if (b) putchar(','); fc_print_hooks(fc_logs+b); }
    puts("]}");
    for (int b=0;b<3;++b) if (b3World_IsValid(s[b].world)) b3DestroyWorld(s[b].world);
    return ok;
}
int main(int argc,char** argv)
{
    const char* profile="unspecified"; const char* path=NULL; int workers=1;
    for (int i=1;i<argc;i+=2) {
        if (i+1>=argc) return 64;
        if (!strcmp(argv[i],"--profile")) profile=argv[i+1];
        else if (!strcmp(argv[i],"--trace")) path=argv[i+1];
        else if (!strcmp(argv[i],"--workers")) { if (!number(argv[i+1],1,4,&workers) || workers==3) return 64; }
        else if (!strcmp(argv[i],"--case") && !strcmp(argv[i+1],"all")) { }
        else return 64;
    }
    FILE* file=path?fopen(path,"wb"):NULL; if (path && !file) return 2;
    if (file) put32(file,0x31484346u); // FCH1, little endian; repeated trial/branch/tick/Trace.
    int ok=1,trial=0;
    for (int kind=1;kind<=3;++kind) for (int mode=0;mode<=2;++mode) ok=fc_run(++trial,kind,mode,workers,profile,file) && ok;
    ok=fc_run(++trial,4,0,workers,profile,file) && ok;
    ok=fc_run(++trial,4,2,workers,profile,file) && ok;
    ok=fc_run(++trial,4,3,workers,profile,file) && ok;
    ok=fc_run(++trial,5,0,workers,profile,file) && ok;
    ok=fc_run(++trial,5,2,workers,profile,file) && ok;
    if (file && fclose(file)) ok=0;
    return ok?0:5;
}
