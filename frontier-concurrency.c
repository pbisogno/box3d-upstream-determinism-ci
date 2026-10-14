// SPDX-License-Identifier: MIT
// Generic synthetic piles and a distance-joint lattice, using published Box3D
// APIs and its unchanged in-tree scheduler. Diagnostics are not solver inputs.
#include "box3d/box3d.h"
#include "box3d/collision.h"
#include "physics_world.h"
#include "scheduler.h"
#include "platform.h"
#include "box3d/constants.h"
#if defined(__x86_64__) || defined(_M_X64) || defined(__i386__) || defined(_M_IX86)
#include <xmmintrin.h>
#endif
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stddef.h>
#ifdef BOX3D_DOUBLE_PRECISION
#error "This tagged observation format requires float32"
#endif
#if defined(_MSC_VER)
#define CN_TLS __declspec(thread)
#else
#define CN_TLS _Thread_local
#endif
enum { CN_BODIES=192, CN_STEPS=256, CN_CELLS=131072, CN_CONTACTS=4096, CN_THREADS=8 };
typedef struct { uint32_t tag,bits; } CNCell;
typedef struct { int count; CNCell* cells; } CNTrace;
typedef struct CNMetrics CNMetrics;
typedef struct { b3TaskCallback* callback; void* context; CNMetrics* metrics; int collide; } CNTask;
struct CNMetrics {
    b3Scheduler* scheduler; b3WorldId world; int epoch;
    b3AtomicInt slots,threadCount,invalid,activeCollide,maxCollide;
    b3AtomicI64 calls[CN_THREADS],sat[CN_THREADS],logicalSat[4];
    b3AtomicInt fpSeen[CN_THREADS]; uint64_t fpRaw[CN_THREADS],fpControl[CN_THREADS]; int fpKind[CN_THREADS];
    CNTask tasks[B3_MAX_TASKS];
};
typedef struct { b3WorldId world; b3BodyId bodies[CN_BODIES]; b3JointId joints[CN_BODIES]; int jointCount; CNMetrics* metrics; } CNScene;
static CN_TLS int cn_epoch=-1,cn_slot=-1;
static int cn_next_epoch=0;
static int cn_thread_slot(CNMetrics* m)
{
    if (cn_epoch!=m->epoch) { cn_epoch=m->epoch; cn_slot=b3AtomicFetchAddInt(&m->threadCount,1); }
    if (cn_slot<0 || cn_slot>=CN_THREADS) { b3AtomicStoreInt(&m->invalid,1); return -1; }
    return cn_slot;
}
// Published parallel_for.c declares this prefix. memcpy reads object bytes;
// no strict-aliasing cast to an unrelated struct and no engine field is written.
typedef struct { void* shared; int workerIndex; } CNParallelPrefix;
// Read only. MXCSR status flags are excluded from fpControl; fpRaw retains them.
static uint64_t cn_read_fp(int* kind)
{
#if defined(__x86_64__) || defined(_M_X64) || defined(__i386__) || defined(_M_IX86)
    *kind=1; return (uint64_t)_mm_getcsr();
#elif defined(__aarch64__) && (defined(__GNUC__) || defined(__clang__)) && !defined(_MSC_VER)
    uint64_t value; __asm__ volatile("mrs %0, fpcr" : "=r"(value)); *kind=2; return value;
#else
    *kind=0; return 0;
#endif
}
static void cn_task(void* opaque)
{
    CNTask* task=(CNTask*)opaque; CNMetrics* m=task->metrics; int slot=cn_thread_slot(m),logical=-1,before=0;
    if (slot>=0 && b3AtomicLoadInt(m->fpSeen+slot)==0) {
        m->fpRaw[slot]=cn_read_fp(m->fpKind+slot);
        m->fpControl[slot]=m->fpKind[slot]==1?m->fpRaw[slot]&0xffc0u:m->fpRaw[slot];
        b3AtomicStoreInt(m->fpSeen+slot,1);
    }
    b3World* world=b3GetWorldFromId(m->world);
    if (task->collide) {
        CNParallelPrefix prefix={0}; memcpy(&prefix,task->context,offsetof(CNParallelPrefix,workerIndex)+sizeof(int));
        logical=prefix.workerIndex;
        if (logical<0 || logical>=world->taskContexts.count || logical>=4) b3AtomicStoreInt(&m->invalid,1);
        else before=world->taskContexts.data[logical].satCallCount;
        int active=b3AtomicFetchAddInt(&m->activeCollide,1)+1,old=b3AtomicLoadInt(&m->maxCollide);
        while (active>old && !b3AtomicCompareExchangeInt(&m->maxCollide,old,active)) old=b3AtomicLoadInt(&m->maxCollide);
    }
    task->callback(task->context);
    if (slot>=0) b3AtomicFetchAddI64(m->calls+slot,1);
    if (task->collide) {
        if (logical>=0 && logical<world->taskContexts.count && logical<4) {
            int delta=world->taskContexts.data[logical].satCallCount-before;
            if (delta<0) b3AtomicStoreInt(&m->invalid,1);
            else { if (slot>=0) b3AtomicFetchAddI64(m->sat+slot,delta); b3AtomicFetchAddI64(m->logicalSat+logical,delta); }
        }
        b3AtomicFetchAddInt(&m->activeCollide,-1);
    }
}
static void* cn_enqueue(b3TaskCallback* task,void* context,void* user,const char* name)
{
    CNMetrics* m=(CNMetrics*)user; int index=b3AtomicFetchAddInt(&m->slots,1);
    if (index<0 || index>=B3_MAX_TASKS) { fputs("diagnostic task capacity exceeded\n",stderr); abort(); }
    CNTask* wrapper=m->tasks+index; *wrapper=(CNTask){task,context,m,name && !strcmp(name,"collide")};
    return b3SchedulerEnqueueTask(cn_task,wrapper,m->scheduler,name);
}
static void cn_finish(void* task,void* user)
{ CNMetrics* m=(CNMetrics*)user; b3SchedulerFinishTask(task,m->scheduler); }
static uint32_t cn_tag(int group,int item,int field)
{ return ((uint32_t)group<<24)|((uint32_t)item<<8)|(uint32_t)field; }
static int cn_uint(CNTrace* t,uint32_t tag,uint32_t value)
{ if (t->count>=CN_CELLS) return 0; t->cells[t->count++]=(CNCell){tag,value}; return 1; }
static int cn_real(CNTrace* t,uint32_t tag,float value)
{ uint32_t bits; memcpy(&bits,&value,4); if ((bits&0x7f800000u)==0x7f800000u) return 0; return cn_uint(t,tag,bits); }
static int cn_vec(CNTrace* t,uint32_t tag,b3Vec3 v)
{ return cn_real(t,tag,v.x)&&cn_real(t,tag+1,v.y)&&cn_real(t,tag+2,v.z); }
static int cn_pos(CNTrace* t,uint32_t tag,b3Pos v)
{ return cn_real(t,tag,v.x)&&cn_real(t,tag+1,v.y)&&cn_real(t,tag+2,v.z); }
static int cn_u32(FILE* f,uint32_t n)
{ unsigned char b[4]={(unsigned char)n,(unsigned char)(n>>8),(unsigned char)(n>>16),(unsigned char)(n>>24)}; return fwrite(b,1,4,f)==4; }
static int cn_write(FILE* f,int tick,const CNTrace* t)
{
    if (!cn_u32(f,(uint32_t)tick)||!cn_u32(f,(uint32_t)t->count)) return 0;
    for (int i=0;i<t->count;++i) if (!cn_u32(f,t->cells[i].tag)||!cn_u32(f,t->cells[i].bits)) return 0;
    return 1;
}
static int cn_make(CNScene* s,int kind,int workers,int measured)
{
    memset(s,0,sizeof(*s)); b3WorldDef wd=b3DefaultWorldDef(); wd.workerCount=(uint32_t)workers; wd.enableSleep=false;
    if (measured) {
        s->metrics=(CNMetrics*)calloc(1,sizeof(CNMetrics)); if (!s->metrics) return 0;
        CNMetrics* m=s->metrics; m->epoch=++cn_next_epoch; cn_thread_slot(m);
        m->scheduler=b3CreateScheduler(workers); if (!m->scheduler) return 0;
        wd.enqueueTask=cn_enqueue; wd.finishTask=cn_finish; wd.userTaskContext=m;
    }
    s->world=b3CreateWorld(&wd); if (!b3World_IsValid(s->world)) return 0;
    if (s->metrics) s->metrics->world=s->world;
    b3BodyDef bd=b3DefaultBodyDef(); b3BodyId floor=b3CreateBody(s->world,&bd);
    b3ShapeDef sd=b3DefaultShapeDef(); b3BoxHull ground=b3MakeBoxHull(40,0.5f,40);
    b3CreateHullShape(floor,&sd,&ground.base); b3Body_SetName(floor,"floor");
    b3BoxHull box=b3MakeBoxHull(0.48f,0.48f,0.48f); sd.density=1; sd.baseMaterial.friction=0.5f;
    for (int i=0;i<CN_BODIES;++i) {
        bd=b3DefaultBodyDef(); bd.type=b3_dynamicBody;
        if (kind==10) { int stack=i/3,level=i%3; bd.position=(b3Pos){-14.0f+4.0f*(stack%8),1.2f+1.0f*level,-14.0f+4.0f*(stack/8)}; }
        else bd.position=(b3Pos){-3.5f+(float)(i%8),1.2f+(float)(i/48),-2.5f+(float)((i/8)%6)};
        s->bodies[i]=b3CreateBody(s->world,&bd); b3CreateHullShape(s->bodies[i],&sd,&box.base);
        char name[16]; snprintf(name,sizeof(name),"body%03d",i); b3Body_SetName(s->bodies[i],name);
        if (kind==11 && i>0) {
            int parent=i%48==0?i-48:(i%8==0?i-8:i-1);
            b3DistanceJointDef jd=b3DefaultDistanceJointDef(); jd.base.bodyIdA=s->bodies[parent]; jd.base.bodyIdB=s->bodies[i];
            jd.base.collideConnected=true; jd.length=1; jd.enableSpring=true; jd.hertz=5; jd.dampingRatio=0.8f;
            s->joints[s->jointCount++]=b3CreateDistanceJoint(s->world,&jd);
        }
    }
    return 1;
}
static void cn_destroy(CNScene* s)
{
    if (b3World_IsValid(s->world)) b3DestroyWorld(s->world);
    if (s->metrics) { if (s->metrics->scheduler) b3DestroyScheduler(s->metrics->scheduler); free(s->metrics); }
}
static void cn_step(CNScene* s,int tick)
{
    if (tick==96 || tick==192) for (int i=0;i<CN_BODIES;i+=17)
        b3Body_ApplyLinearImpulseToCenter(s->bodies[i],(b3Vec3){0.3f,0.5f,0.1f},true);
    if (s->metrics) { b3ResetScheduler(s->metrics->scheduler); b3AtomicStoreInt(&s->metrics->slots,0); }
    b3World_Step(s->world,1.0f/60.0f,4);
}

static int cn_contact_order(const void* a,const void* b)
{
    const b3ContactData* x=(const b3ContactData*)a; const b3ContactData* y=(const b3ContactData*)b;
    return x->contactId.index1<y->contactId.index1?-1:x->contactId.index1>y->contactId.index1;
}
// Exact fields: tick/world counts; every dynamic body's slot/generation,type,
// awake,position3,quaternion4,linear3,angular3; sorted touching-contact ids and
// shape ids; manifold normals and twist/friction/rolling impulses; live point
// separation/normalImpulse/totalNormalImpulse/normalVelocity/baseSeparation,
// featureId/triangleIndex/persisted; all lattice-joint forces/torques; counts
// of body/contact/sensor/joint buffered events. Full event arrays are NOT covered.
static int cn_observe(const CNScene* s,int tick,CNTrace* t)
{
    t->count=0; b3Counters c=b3World_GetCounters(s->world);
    int counts[7]={tick,c.bodyCount,c.shapeCount,c.contactCount,c.jointCount,c.islandCount,b3World_GetAwakeBodyCount(s->world)};
    for (int i=0;i<7;++i) if (!cn_uint(t,cn_tag(1,0,i+1),(uint32_t)counts[i])) return 0;
    b3ContactData* contacts=(b3ContactData*)malloc(sizeof(b3ContactData)*CN_CONTACTS); if (!contacts) return 0;
    int contactCount=0,ok=1;
    for (int i=0;i<CN_BODIES && ok;++i) {
        b3BodyId id=s->bodies[i]; b3WorldTransform xf=b3Body_GetTransform(id); uint32_t k=cn_tag(2,i,1);
        ok=cn_uint(t,k,id.index1)&&cn_uint(t,k+1,id.generation)&&cn_uint(t,k+2,b3Body_GetType(id))&&
            cn_uint(t,k+3,b3Body_IsAwake(id))&&cn_pos(t,k+4,xf.p)&&cn_vec(t,k+7,xf.q.v)&&cn_real(t,k+10,xf.q.s)&&
            cn_vec(t,k+11,b3Body_GetLinearVelocity(id))&&cn_vec(t,k+14,b3Body_GetAngularVelocity(id));
        if (b3Body_GetContactCapacity(id)>64) { ok=0; break; }
        b3ContactData local[64]; int n=b3Body_GetContactData(id,local,64);
        for (int j=0;j<n && ok;++j) {
            int seen=0; for (int p=0;p<contactCount;++p) if (B3_ID_EQUALS(contacts[p].contactId,local[j].contactId)) { seen=1; break; }
            if (!seen) { if (contactCount>=CN_CONTACTS) ok=0; else contacts[contactCount++]=local[j]; }
        }
    }
    qsort(contacts,(size_t)contactCount,sizeof(b3ContactData),cn_contact_order);
    if (ok) ok=cn_uint(t,cn_tag(3,0,0),(uint32_t)contactCount);
    for (int i=0;i<contactCount && ok;++i) {
        const b3ContactData* c=contacts+i; uint32_t k=cn_tag(3,i,1);
        ok=cn_uint(t,k,c->contactId.index1)&&cn_uint(t,k+1,c->contactId.generation)&&
            cn_uint(t,k+2,c->shapeIdA.index1)&&cn_uint(t,k+3,c->shapeIdA.generation)&&
            cn_uint(t,k+4,c->shapeIdB.index1)&&cn_uint(t,k+5,c->shapeIdB.generation)&&cn_uint(t,k+6,c->manifoldCount);
        if (c->manifoldCount<0 || c->manifoldCount>4) { ok=0; break; }
        for (int m=0;m<c->manifoldCount && ok;++m) {
            const b3Manifold* manifold=c->manifolds+m; int item=i*4+m; k=cn_tag(4,item,1);
            ok=cn_uint(t,k,manifold->pointCount)&&cn_vec(t,k+1,manifold->normal)&&cn_real(t,k+4,manifold->twistImpulse)&&
                cn_vec(t,k+5,manifold->frictionImpulse)&&cn_vec(t,k+8,manifold->rollingImpulse);
            if (manifold->pointCount<0 || manifold->pointCount>B3_MAX_MANIFOLD_POINTS) { ok=0; break; }
            for (int p=0;p<manifold->pointCount && ok;++p) {
                const b3ManifoldPoint* v=manifold->points+p; k=cn_tag(5,item*B3_MAX_MANIFOLD_POINTS+p,1);
                ok=cn_real(t,k,v->separation)&&cn_real(t,k+1,v->normalImpulse)&&cn_real(t,k+2,v->totalNormalImpulse)&&
                    cn_real(t,k+3,v->normalVelocity)&&cn_real(t,k+4,v->baseSeparation)&&cn_uint(t,k+5,v->featureId)&&
                    cn_uint(t,k+6,(uint32_t)v->triangleIndex)&&cn_uint(t,k+7,v->persisted);
            }
        }
    }
    free(contacts);
    for (int i=0;i<s->jointCount && ok;++i) {
        b3JointId id=s->joints[i]; uint32_t k=cn_tag(6,i,1);
        ok=cn_uint(t,k,id.index1)&&cn_uint(t,k+1,id.generation)&&cn_uint(t,k+2,b3Joint_IsAwake(id))&&
            cn_vec(t,k+3,b3Joint_GetConstraintForce(id))&&cn_vec(t,k+6,b3Joint_GetConstraintTorque(id));
    }
    b3ContactEvents ce=b3World_GetContactEvents(s->world); b3BodyEvents be=b3World_GetBodyEvents(s->world);
    b3SensorEvents se=b3World_GetSensorEvents(s->world); b3JointEvents je=b3World_GetJointEvents(s->world);
    int events[7]={ce.beginCount,ce.endCount,ce.hitCount,be.moveCount,se.beginCount,se.endCount,je.count};
    for (int i=0;ok && i<7;++i) ok=cn_uint(t,cn_tag(7,0,i+1),events[i]);
    return ok;
}
typedef struct { int tick; uint32_t tag,expected,actual; const char* branch; } CNDifference;
static int cn_equal(const CNTrace* a,const CNTrace* b,int tick,const char* branch,CNDifference* d)
{
    int n=a->count<b->count?a->count:b->count;
    for (int i=0;i<n;++i) if (a->cells[i].tag!=b->cells[i].tag || a->cells[i].bits!=b->cells[i].bits) {
        if (d->tick<0) *d=(CNDifference){tick,a->cells[i].tag,a->cells[i].bits,b->cells[i].bits,branch}; return 0;
    }
    if (a->count!=b->count) { if (d->tick<0) *d=(CNDifference){tick,0,(uint32_t)a->count,(uint32_t)b->count,branch}; return 0; }
    return 1;
}
static void cn_array(const b3AtomicI64* a,int n)
{ putchar('['); for (int i=0;i<n;++i) printf("%s%lld",i?",":"",(long long)b3AtomicLoadI64((b3AtomicI64*)(a+i))); putchar(']'); }
static int cn_run(int kind,int workers,const char* profile,FILE* trace)
{
    CNScene scenes[3]={{0}}; CNTrace observations[3]={{0}}; CNDifference difference={-1,0,0,0,"none"};
    int ok=1,valid=1,equal=1,sham=1,configuration=1,steps=0,compared=0,width=0,denseSteps=0,independentSteps=0,maxContacts=0;
    for (int i=0;i<3;++i) { observations[i].cells=(CNCell*)malloc(sizeof(CNCell)*CN_CELLS);
        if (!observations[i].cells || !cn_make(scenes+i,kind,i==0?1:workers,i==1)) ok=0; }
    if (trace && (!cn_u32(trace,0x31545246u)||!cn_u32(trace,1)||!cn_u32(trace,(uint32_t)kind)||!cn_u32(trace,CN_STEPS))) ok=0;
    for (int tick=0;ok && tick<=CN_STEPS;++tick) {
        if (tick) { for (int i=0;i<3;++i) cn_step(scenes+i,tick); steps=tick; }
        for (int i=0;i<3;++i) {
            b3World* w=b3GetWorldFromId(scenes[i].world);
            if (!width) width=w->simdWidth;
            if ((width!=4 && width!=8) || w->simdWidth!=width || b3World_GetWorkerCount(scenes[i].world)!=(i==0?1:workers)) configuration=0;
            if (!cn_observe(scenes+i,tick,observations+i)) valid=0;
        }
        if (!valid) { ok=0; break; }
        if (!cn_equal(observations,observations+1,tick,"requested_vs_w1",&difference)) equal=0;
        if (!cn_equal(observations+2,observations+1,tick,"instrumented_vs_uninstrumented",&difference)) sham=0;
        ++compared;
        b3Counters c=b3World_GetCounters(scenes[1].world); if (c.contactCount>maxContacts) maxContacts=c.contactCount;
        if (tick>64 && c.islandCount>=16) ++independentSteps;
        if (tick>0 && c.islandCount==1 && c.jointCount==CN_BODIES-1) ++denseSteps;
        if (trace && !cn_write(trace,tick,observations+1)) ok=0;
        if (!equal || !sham || !configuration) ok=0;
    }
    CNMetrics* m=scenes[1].metrics; int usefulThreads=0,usefulWorkers=0,threadCount=m?b3AtomicLoadInt(&m->threadCount):0;
    if (threadCount>CN_THREADS) threadCount=CN_THREADS;
    if (m) { for (int i=0;i<threadCount;++i) if (b3AtomicLoadI64(m->sat+i)>0) ++usefulThreads;
        for (int i=0;i<workers;++i) if (b3AtomicLoadI64(m->logicalSat+i)>0) ++usefulWorkers; }
    int activation=m && !b3AtomicLoadInt(&m->invalid) && usefulThreads>=(workers>1?2:1) && usefulWorkers>=(workers>1?2:1);
    int topology=kind==10?independentSteps>0:denseSteps>0;
    ok=ok && valid && equal && sham && configuration && activation && topology && steps==CN_STEPS && compared==CN_STEPS+1 && maxContacts>0;
    printf("{\"schema\":\"box3d-frontier-v1\",\"frontier\":\"concurrency\",\"profile\":\"%s\",\"case\":\"%s\",\"status\":\"%s\",\"workers_requested\":%d,\"simd_width_actual\":%d,\"horizon\":%d,\"steps_completed\":%d,\"states_compared\":%d,\"dynamic_bodies\":%d,\"gates\":{\"observations_valid\":%s,\"requested_equals_w1\":%s,\"instrumentation_control_exact\":%s,\"configuration\":%s,\"useful_parallel_work\":%s,\"topology\":%s,\"complete_horizon\":%s},\"thread_slots_observed\":%d,\"threads_with_sat_work\":%d,\"logical_workers_with_sat_work\":%d,\"max_overlapping_collide_callbacks\":%d,\"per_thread_callbacks\":",
        profile,kind==10?"islands":"dense",ok?"complete":"gate_failed",workers,width,CN_STEPS,steps,compared,CN_BODIES,
        valid?"true":"false",equal?"true":"false",sham?"true":"false",configuration?"true":"false",activation?"true":"false",topology?"true":"false",steps==CN_STEPS&&compared==CN_STEPS+1?"true":"false",
        threadCount,usefulThreads,usefulWorkers,m?b3AtomicLoadInt(&m->maxCollide):0);
    if (m) cn_array(m->calls,threadCount); else fputs("[]",stdout);
    fputs(",\"per_thread_raw_sat_calls\":",stdout); if (m) cn_array(m->sat,threadCount); else fputs("[]",stdout);
    fputs(",\"per_logical_worker_raw_sat_calls\":",stdout); if (m) cn_array(m->logicalSat,workers); else fputs("[]",stdout);
    fputs(",\"thread_fp_environment\":[",stdout);
    for (int i=0;i<threadCount;++i) {
        int seen=m?b3AtomicLoadInt(m->fpSeen+i):0,kind=m?m->fpKind[i]:0;
        printf("%s{\"slot\":%d,\"captured\":%s,\"register\":\"%s\",\"raw\":%llu,\"control\":%llu}",
            i?",":"",i,seen?"true":"false",kind==1?"mxcsr":kind==2?"fpcr":"unavailable",
            (unsigned long long)(m?m->fpRaw[i]:0),(unsigned long long)(m?m->fpControl[i]:0));
    }
    putchar(']');
    printf(",\"independent_island_steps\":%d,\"dense_connected_steps\":%d,\"peak_contacts\":%d,\"first_difference\":{\"tick\":%d,\"tag\":%u,\"expected_bits\":%u,\"actual_bits\":%u,\"branch\":\"%s\"},\"projection_only\":true}\n",
        independentSteps,denseSteps,maxContacts,difference.tick,difference.tag,difference.expected,difference.actual,difference.branch);
    for (int i=0;i<3;++i) { cn_destroy(scenes+i); free(observations[i].cells); }
    return ok;
}
int main(int argc,char** argv)
{
    const char* profile="unspecified"; const char* path=NULL; int workers=1,kind=0;
    for (int i=1;i<argc;i+=2) {
        if (i+1>=argc) goto usage;
        if (!strcmp(argv[i],"--profile")) profile=argv[i+1];
        else if (!strcmp(argv[i],"--workers")) { if (strlen(argv[i+1])!=1) goto usage; workers=argv[i+1][0]-'0'; if (workers!=1&&workers!=2&&workers!=4) goto usage; }
        else if (!strcmp(argv[i],"--case")) { if (!strcmp(argv[i+1],"all")) kind=0; else if (!strcmp(argv[i+1],"islands")) kind=10; else if (!strcmp(argv[i+1],"dense")) kind=11; else goto usage; }
        else if (!strcmp(argv[i],"--trace")) path=argv[i+1]; else goto usage;
    }
    if (!*profile || strlen(profile)>120) goto usage;
    for (const char* q=profile;*q;++q) if (!((*q>='a'&&*q<='z')||(*q>='A'&&*q<='Z')||(*q>='0'&&*q<='9')||strchr("_./:+-",*q))) goto usage;
    FILE* trace=path?fopen(path,"wb"):NULL; if (path&&!trace) { fputs("cannot open trace output\n",stderr); return 5; }
    int ok=1;
    if (!kind || kind==10) if (!cn_run(10,workers,profile,trace)) ok=0;
    if (!kind || kind==11) if (!cn_run(11,workers,profile,trace)) ok=0;
    if (trace && fclose(trace)!=0) ok=0;
    return ok?0:5;
usage:
    fputs("Usage: frontier-concurrency --profile LABEL --workers 1|2|4 [--case all|islands|dense] [--trace FILE]\n",stderr); return 64;
}
