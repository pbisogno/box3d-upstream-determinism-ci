#if !defined(_WIN32) && !defined(_POSIX_C_SOURCE)
#define _POSIX_C_SOURCE 200809L
#endif
// SPDX-License-Identifier: MIT
// Synthetic cache lifecycle experiment; upstream arithmetic is unchanged.
#define main contactlab_frozen_stress_main
#include "stress-replay.c"
#undef main
#include "physics_world.h"
#include "contact.h"
#include "simd.h"
#include "contactlab/zero-hook.h"
#include <stddef.h>
#include <math.h>
#if defined(_WIN32)
#include <io.h>
#define z_fileno _fileno
#define z_dup _dup
#define z_dup2 _dup2
#define z_close _close
#else
#include <unistd.h>
#define z_fileno fileno
#define z_dup dup
#define z_dup2 dup2
#define z_close close
#endif
#ifndef CONTACTLAB_INSTRUMENTED
#define CONTACTLAB_INSTRUMENTED 0
#endif
enum { Z_HORIZON=1536, Z_ARMS=4, Z_LOG_CAP=8192 };
typedef struct { int begin,end,hit,move,sleep,joint; } ZEvents;
static int z_id(Trace* t,uint32_t base,uint32_t index,uint32_t generation)
{ return scalar(t,base,index) && scalar(t,base+1,generation); }
// These are buffered PUBLIC event arrays, not callbacks. Order is preserved.
// World-id provenance and userData pointers are excluded; slot/generation stay exact.
static int z_events(b3WorldId world,Trace* t,ZEvents* totals)
{
    b3ContactEvents c=b3World_GetContactEvents(world); b3BodyEvents b=b3World_GetBodyEvents(world);
    b3JointEvents j=b3World_GetJointEvents(world); b3SensorEvents se=b3World_GetSensorEvents(world);
    if (c.beginCount>32 || c.endCount>32 || c.hitCount>32 || b.moveCount>8 || j.count>8 || se.beginCount || se.endCount) return 0;
    if (!scalar(t,tag(8,0,1),c.beginCount) || !scalar(t,tag(8,0,2),c.endCount) ||
        !scalar(t,tag(8,0,3),c.hitCount) || !scalar(t,tag(8,0,4),b.moveCount) ||
        !scalar(t,tag(8,0,5),j.count) || !scalar(t,tag(8,0,6),se.beginCount) || !scalar(t,tag(8,0,7),se.endCount)) return 0;
    for (int i=0;i<c.beginCount;++i) {
        b3ContactBeginTouchEvent e=c.beginEvents[i]; uint32_t k=tag(9,i,1);
        if (!z_id(t,k,e.shapeIdA.index1,e.shapeIdA.generation) || !z_id(t,k+2,e.shapeIdB.index1,e.shapeIdB.generation) ||
            !z_id(t,k+4,e.contactId.index1,e.contactId.generation)) return 0;
    }
    for (int i=0;i<c.endCount;++i) {
        b3ContactEndTouchEvent e=c.endEvents[i]; uint32_t k=tag(10,i,1);
        if (!z_id(t,k,e.shapeIdA.index1,e.shapeIdA.generation) || !z_id(t,k+2,e.shapeIdB.index1,e.shapeIdB.generation) ||
            !z_id(t,k+4,e.contactId.index1,e.contactId.generation)) return 0;
    }
    for (int i=0;i<c.hitCount;++i) {
        b3ContactHitEvent e=c.hitEvents[i]; uint32_t k=tag(11,i,1);
        if (!z_id(t,k,e.shapeIdA.index1,e.shapeIdA.generation) || !z_id(t,k+2,e.shapeIdB.index1,e.shapeIdB.generation) ||
            !z_id(t,k+4,e.contactId.index1,e.contactId.generation) || !position(t,k+6,e.point) || !vector(t,k+9,e.normal) ||
            !real(t,k+12,e.approachSpeed) || !scalar(t,k+13,(uint32_t)e.userMaterialIdA) ||
            !scalar(t,k+14,(uint32_t)(e.userMaterialIdA>>32)) || !scalar(t,k+15,(uint32_t)e.userMaterialIdB) ||
            !scalar(t,k+16,(uint32_t)(e.userMaterialIdB>>32))) return 0;
    }
    for (int i=0;i<b.moveCount;++i) {
        b3BodyMoveEvent e=b.moveEvents[i]; uint32_t k=tag(12,i,1);
        if (!z_id(t,k,e.bodyId.index1,e.bodyId.generation) || !scalar(t,k+2,e.fellAsleep) || !position(t,k+3,e.transform.p) ||
            !vector(t,k+6,e.transform.q.v) || !real(t,k+9,e.transform.q.s)) return 0;
        if (totals && e.fellAsleep) ++totals->sleep;
    }
    for (int i=0;i<j.count;++i) if (!z_id(t,tag(13,i,1),j.jointEvents[i].jointId.index1,j.jointEvents[i].jointId.generation)) return 0;
    if (totals) { totals->begin+=c.beginCount; totals->end+=c.endCount; totals->hit+=c.hitCount; totals->move+=b.moveCount; totals->joint+=j.count; }
    return 1;
}
typedef struct { int index,manifold; uint32_t generation,before,after; b3Manifold* p; } ZTarget;
typedef struct { int tick,arm,stage,index,manifold,width,lane,count; uint32_t generation,bits[12]; } ZHook;
static ZHook z_log[Z_LOG_CAP];
static int z_log_count,z_overflow,z_tick,z_arm=-1;
static int z_restore_arm,z_restore_tick=-1;
static uint32_t z_restore_tag,z_restore_expected,z_restore_actual;
static const b3World* z_world;
static ZTarget z_targets[Z_ARMS];
static uint32_t z_bits(float f) { uint32_t b; memcpy(&b,&f,4); return b; }
// W1 is required. Logging stops after the initial origin window, except for
// the actual intervention's first consumption and authored lifecycle windows.
void contactlabZeroObserve(int stage,const b3World* world,const b3Contact* contact,
    int manifold,int width,int lane,const float* values,int count)
{
    if (z_arm<0 || world!=z_world) return;
    int target=contact->contactId==z_targets[z_arm].index &&
        contact->generation==z_targets[z_arm].generation && manifold==z_targets[z_arm].manifold;
    int consume=target && (z_tick==48 || z_tick==53);
    int origin=z_arm==0 && z_tick<=64;
    int lifecycle=z_arm==0 && (z_tick==768 || z_tick==769 || z_tick==1024 || z_tick==1025 || z_tick==1280);
    if (!origin && !consume && !lifecycle) return;
    if (count>12 || z_log_count==Z_LOG_CAP) { z_overflow=1; return; }
    ZHook* h=z_log+z_log_count++;
    *h=(ZHook){z_tick,z_arm,stage,contact->contactId,manifold,width,lane,count,contact->generation,{0}};
    for (int i=0;i<count;++i) h->bits[i]=z_bits(values[i]);
}
static int z_equal(const Trace* a,const Trace* b,int seed)
{
    int i=0,j=0;
    for (;;) {
        while (seed && i<a->count && a->cells[i].tag>>24>=8 && a->cells[i].tag>>24<=13) ++i;
        while (seed && j<b->count && b->cells[j].tag>>24>=8 && b->cells[j].tag>>24<=13) ++j;
        if (i==a->count || j==b->count) return i==a->count && j==b->count;
        if (a->cells[i].tag!=b->cells[j].tag || a->cells[i].value!=b->cells[j].value) return 0;
        ++i; ++j;
    }
}
// Group 15: live contact slot/generation/endpoints/count; group 16: raw cached
// twist by slot/manifold, including sleeping and non-touching retained caches.
// Public projections in groups 1..14 retain the frozen helpers' field contract.
static int z_observe(Scene* s,int tick,Trace* t,int events,ZEvents* totals)
{
    if (!observe(s->world,s->body,tick,t)) return 0;
    if (events && !z_events(s->world,t,totals)) return 0;
    for (int r=0;r<ROLES;++r) if (b3Body_IsValid(s->body[r]) &&
        !z_id(t,tag(14,r,1),s->body[r].index1,s->body[r].generation)) return 0;
    b3World* w=b3GetWorldFromId(s->world); int n=0;
    for (int i=0;i<w->contacts.count;++i) {
        b3Contact* c=w->contacts.data+i;
        if (c->contactId!=i || c->setIndex==B3_NULL_INDEX) continue;
        if (i>=256 || c->manifoldCount<0 || c->manifoldCount>4) return 0;
        ++n; uint32_t base=tag(15,i,1);
        if (!scalar(t,base,i) || !scalar(t,base+1,c->generation) ||
            !scalar(t,base+2,c->shapeIdA) || !scalar(t,base+3,c->shapeIdB) ||
            !scalar(t,base+4,c->manifoldCount)) return 0;
        for (int m=0;m<c->manifoldCount;++m)
            if (!real(t,tag(16,i*4+m,1),c->manifolds[m].twistImpulse)) return 0;
    }
    return scalar(t,tag(15,255,0),n);
}
static int z_trace(FILE* f,int trial,int arm,int tick,const Trace* t)
{ return !f || (put32(f,trial) && put32(f,arm) && put32(f,tick) && write_trace(f,t)); }
static b3JointId z_joint(Scene* s)
{
    b3DistanceJointDef d=b3DefaultDistanceJointDef();
    d.base.bodyIdA=s->body[1]; d.base.bodyIdB=s->body[2];
    d.base.forceThreshold=0.01f; d.base.torqueThreshold=0.01f; d.base.collideConnected=true;
    d.length=1.6f; d.enableSpring=true; d.hertz=3; d.dampingRatio=1;
    d.enableLimit=true; d.minLength=1.1f; d.maxLength=2.1f;
    return b3CreateDistanceJoint(s->world,&d);
}
static int z_make(Scene* s,int kind,b3JointId* joint)
{
    Stats unused;
    if (!make_scene(s,kind,1,&unused)) return 0;
    *joint=b3_nullJointId;
    for (int r=0;r<ROLES;++r) if (b3Body_IsValid(s->body[r])) {
        b3ShapeId ids[8]; int n=b3Body_GetShapes(s->body[r],ids,8);
        for (int j=0;j<n;++j) { b3Shape_EnableContactEvents(ids[j],true); b3Shape_EnableHitEvents(ids[j],true); }
    }
    if (kind==3) { *joint=z_joint(s); if (!b3Joint_IsValid(*joint)) return 0; }
    return 1;
}
static int z_commands(Scene* s,int tick,int targetRole,b3JointId* joint,int* reused)
{
    if (tick==768 || tick==1280)
        b3Body_ApplyLinearImpulseToCenter(s->body[targetRole],(b3Vec3){0.2f,2.0f,0.1f},true);
    if (tick==1024) {
        b3BodyId old=s->body[targetRole]; b3ShapeId before[1],after[1];
        if (b3Body_GetShapes(old,before,1)!=1) return 0;
        b3DestroyBody(old);
        if (b3Body_IsValid(old) || b3Shape_IsValid(before[0])) return 0;
        s->body[targetRole]=make_actor(s,targetRole,tick);
        if (b3Body_GetShapes(s->body[targetRole],after,1)!=1) return 0;
        b3Shape_EnableContactEvents(after[0],true); b3Shape_EnableHitEvents(after[0],true);
        if (s->kind==3) *joint=z_joint(s);
        *reused=old.index1==s->body[targetRole].index1 && old.generation!=s->body[targetRole].generation &&
            before[0].index1==after[0].index1 && before[0].generation!=after[0].generation;
        if (!*reused) return 0;
    }
    return 1;
}
static int z_select(Scene* s,ZTarget* out,int* targetRole)
{
    b3World* w=b3GetWorldFromId(s->world);
    for (int i=0;i<w->contacts.count;++i) {
        b3Contact* c=w->contacts.data+i;
        if (c->contactId!=i || c->setIndex!=b3_awakeSet || !(c->flags&b3_contactTouchingFlag)) continue;
        for (int m=0;m<c->manifoldCount;++m) {
            b3Manifold* p=c->manifolds+m;
            if (!p->pointCount || (z_bits(p->twistImpulse)&0x7fffffffu)) continue;
            for (int r=1;r<ROLES;++r) if (b3Body_IsValid(s->body[r]) &&
                (c->edges[0].bodyId==(int)s->body[r].index1-1 || c->edges[1].bodyId==(int)s->body[r].index1-1)) {
                *out=(ZTarget){i,m,c->generation,z_bits(p->twistImpulse),z_bits(p->twistImpulse),p};
                *targetRole=r; return 1;
            }
        }
    }
    return 0;
}
static const ZHook* z_first(int arm,int stage,int tick)
{
    for (int i=0;i<z_log_count;++i) {
        ZHook* h=z_log+i; ZTarget* t=z_targets+arm;
        if (h->arm==arm && h->stage==stage && h->tick==tick && h->index==t->index &&
            h->generation==t->generation && h->manifold==t->manifold) return h;
    }
    return NULL;
}
static int z_check_player(b3RecPlayer* p,const Trace* states,int checkpoint,int frame,int width,long long* compared)
{
    if (b3RecPlayer_HasDiverged(p) || b3RecPlayer_GetFrame(p)!=frame) return 0;
    Scene s={0}; s.world=b3RecPlayer_GetWorldId(p); player_bodies(p,s.body);
    if (b3GetWorldFromId(s.world)->simdWidth!=width || b3World_GetWorkerCount(s.world)!=1) return 0;
    Trace t; if (!z_observe(&s,checkpoint+frame,&t,frame>0,NULL)) return 0;
    ++*compared; const Trace* expected=states+checkpoint+frame;
    int equal=z_equal(expected,&t,frame==0);
    if (!equal && z_restore_tick<0) {
        z_restore_tick=checkpoint+frame; int i=0,j=0;
        for (;;) {
            while (!frame && i<expected->count && expected->cells[i].tag>>24>=8 && expected->cells[i].tag>>24<=13) ++i;
            while (!frame && j<t.count && t.cells[j].tag>>24>=8 && t.cells[j].tag>>24<=13) ++j;
            if (i==expected->count || j==t.count || expected->cells[i].tag!=t.cells[j].tag || expected->cells[i].value!=t.cells[j].value) {
                z_restore_tag=i<expected->count?expected->cells[i].tag:(j<t.count?t.cells[j].tag:0);
                z_restore_expected=i<expected->count?expected->cells[i].value:0;
                z_restore_actual=j<t.count?t.cells[j].value:0; break;
            }
            ++i; ++j;
        }
    }
    return equal;
}
static int z_replay(b3Recording* rec,const Trace* states,int checkpoint,int width,long long* compared,
    int* seedEqual,int* restartEqual,int* seekEqual,const ZTarget* target,uint32_t* loadedBits)
{
    // Some upstream replay diagnostics use printf directly. Preserve them on
    // stderr so even a failed restore leaves the public JSONL parseable.
    if (fflush(stdout)) return 0;
    int saved=z_dup(z_fileno(stdout)); if (saved<0) return 0;
    if (z_dup2(z_fileno(stderr),z_fileno(stdout))<0) { z_close(saved); return 0; }
    b3RecPlayer* p=b3CreatePlayer(b3Recording_GetData(rec),b3Recording_GetSize(rec),1);
    int ok=p!=NULL; z_arm=-1;
    if (ok) {
        b3World* w=b3GetWorldFromId(b3RecPlayer_GetWorldId(p));
        if (target->index>=w->contacts.count) ok=0;
        else {
            b3Contact* c=w->contacts.data+target->index;
            if (c->contactId!=target->index || c->generation!=target->generation || c->manifoldCount<=target->manifold) ok=0;
            else *loadedBits=z_bits(c->manifolds[target->manifold].twistImpulse);
        }
    }
    if (ok) { *seedEqual=z_check_player(p,states,checkpoint,0,width,compared); ok=*seedEqual; }
    for (int frame=1;ok && frame<=Z_HORIZON-checkpoint;++frame)
        ok=b3RecPlayer_StepFrame(p) && z_check_player(p,states,checkpoint,frame,width,compared);
    if (ok) {
        b3RecPlayer_SeekFrame(p,(Z_HORIZON-checkpoint)/2);
        *seekEqual=z_check_player(p,states,checkpoint,(Z_HORIZON-checkpoint)/2,width,compared); ok=*seekEqual;
    }
    if (ok) {
        b3RecPlayer_Restart(p); *restartEqual=z_check_player(p,states,checkpoint,0,width,compared); ok=*restartEqual;
        for (int frame=1;ok && frame<=Z_HORIZON-checkpoint;++frame)
            ok=b3RecPlayer_StepFrame(p) && z_check_player(p,states,checkpoint,frame,width,compared);
    }
    if (p) b3DestroyPlayer(p);
    if (fflush(stdout)) ok=0;
    if (z_dup2(saved,z_fileno(stdout))<0) ok=0;
    if (z_close(saved)) ok=0;
    return ok;
}
static void z_print_hooks(void)
{
    putchar('[');
    for (int i=0;i<z_log_count;++i) {
        ZHook* h=z_log+i;
        printf("%s{\"tick\":%d,\"arm\":%d,\"stage\":%d,\"contact_index\":%d,\"generation\":%u,\"manifold\":%d,\"width\":%d,\"lane\":%d,\"bits\":[",
            i?",":"",h->tick,h->arm,h->stage,h->index,h->generation,h->manifold,h->width,h->lane);
        for (int j=0;j<h->count;++j) printf("%s\"%08x\"",j?",":"",h->bits[j]);
        fputs("]}",stdout);
    }
    putchar(']');
}
typedef struct { int firstBody,firstId,firstEvent,firstCache,bodyCells,idCells,eventCells,zeroCells,otherCells,bodyFloatCells; } ZDiff;
static int z_category(uint32_t id)
{
    int g=(int)(id>>24);
    if (g==2) return 1;
    if (g==14 || g==15) return 2;
    if (g>=8 && g<=13) return 3;
    if (g==16 || (g==4 && (id&65535u)==5)) return 4;
    return 0;
}
static void z_diff(const Trace* a,const Trace* b,int tick,ZDiff* d)
{
    // Compare each typed category in its original order. A changed event array
    // length must not falsely label later, identical body handles as different.
    // No matching by value, reordering, epsilon or zero normalization occurs.
    for (int category=1;category<=4;++category) {
        int i=0,j=0;
        for (;;) {
            while (i<a->count && z_category(a->cells[i].tag)!=category) ++i;
            while (j<b->count && z_category(b->cells[j].tag)!=category) ++j;
            if (i==a->count && j==b->count) break;
            uint32_t at=i<a->count?a->cells[i].tag:0,bt=j<b->count?b->cells[j].tag:0;
            uint32_t av=i<a->count?a->cells[i].value:0,bv=j<b->count?b->cells[j].value:0;
            if (at!=bt || av!=bv) {
                if (category==1) {
                    if (d->firstBody<0) d->firstBody=tick; ++d->bodyCells;
                    unsigned field=at&65535u;
                    if (at==bt && field>=4 && field<=16 && ((av|bv)&0x7fffffffu)) ++d->bodyFloatCells;
                }
                else if (category==2) { if (d->firstId<0) d->firstId=tick; ++d->idCells; }
                else if (category==3) { if (d->firstEvent<0) d->firstEvent=tick; ++d->eventCells; }
                else {
                    if (d->firstCache<0) d->firstCache=tick;
                    if (at==bt && (av&0x7fffffffu)==0 && (bv&0x7fffffffu)==0) ++d->zeroCells;
                    else ++d->otherCells;
                }
            }
            if (i<a->count) ++i;
            if (j<b->count) ++j;
        }
    }
}
static const char* z_backend(int requested)
{
#if defined(BOX3D_DISABLE_SIMD)
    (void)requested; return "scalar";
#else
    if (requested==8) return "avx2";
#if defined(__aarch64__) || defined(_M_ARM64)
    return "neon";
#else
    return "sse2";
#endif
#endif
}
static int z_trial(int trial,int kind,int sleepSnapshot,const char* profile,int requested,FILE* file)
{
    Scene s[Z_ARMS]={{0}}; b3JointId joints[Z_ARMS]={{0}}; b3Recording* records[Z_ARMS]={0};
    Trace* states[Z_ARMS]={0}; ZEvents events[Z_ARMS]={{0}};
    uint32_t savedBits[Z_ARMS]={0},loadedBits[Z_ARMS]={0};
    ZDiff diffs[Z_ARMS]; int reused[Z_ARMS]={0}; int seed[Z_ARMS]={0},restart[Z_ARMS]={0},seek[Z_ARMS]={0};
    int targetGone[Z_ARMS]={0},contactReused[Z_ARMS]={0};
    int ok=1,selected=0,at=kind==1?47:52,targetRole=1,checkpoint=-1,locality=1,prefix=1,sham=1;
    int width=0,backendGate=1,persistent=0,sleepTicks=0,hookStages=1,loaded=1,used=1,positiveWarm=0;
    int replay=1,io=1,updates=0; long long compared=0; z_log_count=0; z_overflow=0; z_arm=-1;
    z_restore_tick=-1; z_restore_tag=0; z_restore_expected=0; z_restore_actual=0;
    memset(z_targets,0,sizeof(z_targets));
    for (int b=0;b<Z_ARMS;++b) {
        diffs[b]=(ZDiff){-1,-1,-1,-1,0,0,0,0,0};
        states[b]=(Trace*)calloc(Z_HORIZON+1,sizeof(Trace));
        if (!states[b] || !z_make(s+b,kind,joints+b)) ok=0;
    }
    for (int tick=0;ok && tick<=Z_HORIZON;++tick) {
        if (tick) for (int b=0;b<Z_ARMS;++b) {
            if (!z_commands(s+b,tick,targetRole,joints+b,reused+b)) { ok=0; break; }
            z_tick=tick; z_arm=b; z_world=b3GetWorldFromId(s[b].world);
            b3World_Step(s[b].world,1.0f/60.0f,4); z_arm=-1; ++updates;
        }
        if (!ok) break;
        if (tick==at) {
            selected=z_select(s,z_targets,&targetRole); if (!selected) { ok=0; break; }
            Trace before[Z_ARMS];
            for (int b=0;b<Z_ARMS;++b) {
                if (!z_observe(s+b,tick,before+b,1,NULL)) ok=0;
                if (b && !z_equal(before,before+b,0)) prefix=0;
            }
            for (int b=0;b<Z_ARMS;++b) {
                b3World* w=b3GetWorldFromId(s[b].world); ZTarget t=z_targets[0];
                if (t.index>=w->contacts.count) { ok=0; break; }
                b3Contact* c=w->contacts.data+t.index;
                if (c->generation!=t.generation || c->manifoldCount<=t.manifold) { ok=0; break; }
                t.p=c->manifolds+t.manifold;
                if (z_bits(t.p->twistImpulse)!=t.before) prefix=0;
                t.after=b==2?t.before^0x80000000u:(b==3?z_bits(0.03125f):t.before);
                unsigned char expected[sizeof(b3Manifold)]; memcpy(expected,t.p,sizeof(expected));
                memcpy(expected+offsetof(b3Manifold,twistImpulse),&t.after,4);
                memcpy(&t.p->twistImpulse,&t.after,4);
                if (memcmp(expected,t.p,sizeof(expected)) || z_bits(t.p->twistImpulse)!=t.after) locality=0;
                z_targets[b]=t;
            }
        }
        int allSleeping=1;
        for (int b=0;b<Z_ARMS;++b) {
            b3World* w=b3GetWorldFromId(s[b].world);
            if (!width) width=w->simdWidth;
            if (w->simdWidth!=width || width!=(requested==8?8:4) || b3World_GetWorkerCount(s[b].world)!=1) backendGate=0;
            if (b3World_GetAwakeBodyCount(s[b].world)>0) allSleeping=0;
            if (selected && tick>=1024) {
                ZTarget* target=z_targets+b;
                if (target->index>=w->contacts.count) targetGone[b]=1;
                else {
                    b3Contact* c=w->contacts.data+target->index;
                    if (c->contactId!=target->index || c->generation!=target->generation || c->setIndex==B3_NULL_INDEX) targetGone[b]=1;
                    if (c->contactId==target->index && c->generation!=target->generation && c->setIndex!=B3_NULL_INDEX) contactReused[b]=1;
                }
            }
            if (!z_observe(s+b,tick,states[b]+tick,1,events+b)) { ok=0; break; }
            if (!z_trace(file,trial,b,tick,states[b]+tick)) io=0;
        }
        if (!ok) break;
        if (!z_equal(states[0]+tick,states[1]+tick,0)) sham=0;
        if (tick<at) for (int b=2;b<Z_ARMS;++b) if (!z_equal(states[0]+tick,states[b]+tick,0)) prefix=0;
        if (selected && tick>=at) for (int b=2;b<Z_ARMS;++b) z_diff(states[0]+tick,states[b]+tick,tick,diffs+b);
        if (tick>at && tick<768 && b3World_GetAwakeBodyCount(s[0].world)==0) ++sleepTicks;
        if (selected) {
            b3World* w=b3GetWorldFromId(s[0].world); ZTarget* t=z_targets;
            if (t->index<w->contacts.count) {
                b3Contact* c=w->contacts.data+t->index;
                if (c->contactId==t->index && c->generation==t->generation && c->manifoldCount>t->manifold) ++persistent;
            }
        }
        if (selected && checkpoint<0 && ((!sleepSnapshot && tick==at) || (sleepSnapshot && tick>at+1 && allSleeping))) {
            checkpoint=tick;
            for (int b=0;b<Z_ARMS;++b) {
                b3World* w=b3GetWorldFromId(s[b].world); ZTarget* t=z_targets+b;
                b3Contact* c=w->contacts.data+t->index;
                if (c->contactId!=t->index || c->generation!=t->generation || c->manifoldCount<=t->manifold) { ok=0; break; }
                savedBits[b]=z_bits(c->manifolds[t->manifold].twistImpulse);
                records[b]=b3CreateRecording(0); if (!records[b]) { ok=0; break; }
                b3World_StartRecording(s[b].world,records[b]);
            }
        }
    }
    z_arm=-1;
    for (int b=0;b<Z_ARMS;++b) {
        if (records[b]) b3World_StopRecording(s[b].world);
        if (b3World_IsValid(s[b].world)) b3DestroyWorld(s[b].world);
    }
    if (CONTACTLAB_INSTRUMENTED) {
        for (int b=0;b<Z_ARMS;++b) {
            for (int stage=1;stage<=5;++stage) if (!z_first(b,stage,at+1)) hookStages=0;
            if (!z_first(b,7,at+1)) hookStages=0;
            const ZHook* prep=z_first(b,1,at+1); const ZHook* warm=z_first(b,2,at+1);
            if (!prep || prep->bits[0]!=z_targets[b].after) loaded=0;
            if (!prep || !warm || prep->bits[1]!=warm->bits[0]) used=0;
        }
        const ZHook* ref=z_first(0,3,at+1); const ZHook* pos=z_first(3,3,at+1);
        if (ref && pos) for (int j=0;j<ref->count;++j) if (ref->bits[j]!=pos->bits[j] &&
            ((ref->bits[j]|pos->bits[j])&0x7fffffffu)) ++positiveWarm;
    }
    int coverage=selected && checkpoint>=at && checkpoint<768 && sleepTicks>0 && persistent>4 &&
        events[0].sleep>0 && events[0].begin>0 && events[0].end>0 && reused[0] && reused[1] && reused[2] && reused[3] &&
        targetGone[0] && targetGone[1] && targetGone[2] && targetGone[3];
    int positive=diffs[3].bodyFloatCells>0 && (!CONTACTLAB_INSTRUMENTED || positiveWarm>0);
    if (ok && checkpoint>=0) {
        for (int b=0;b<Z_ARMS;++b) {
            if (z_restore_tick<0) z_restore_arm=b;
            if (!z_replay(records[b],states[b],checkpoint,width,&compared,seed+b,restart+b,seek+b,z_targets+b,loadedBits+b) ||
                savedBits[b]!=loadedBits[b]) replay=0;
        }
    } else replay=0;
    int instrument=!CONTACTLAB_INSTRUMENTED || (hookStages && loaded && used && !z_overflow);
    ok=ok && coverage && positive && locality && prefix && sham && backendGate && replay && io && instrument &&
        updates==Z_HORIZON*Z_ARMS && nonfiniteTick<0;
    printf("{\"schema\":\"box3d-contactlab-v1\",\"type\":\"result\",\"driver\":\"zero\",\"profile\":\"%s\",\"trial\":%d,\"case\":\"%s-%s\",\"instrumented\":%s,\"status\":\"%s\",\"execution_pass\":%s,\"backend_actual\":\"%s\",\"simd_requested\":%d,\"simd_actual\":%d,\"workers_requested\":1,\"workers_actual\":1,\"horizon\":%d,\"intervention_tick\":%d,\"checkpoint\":%d,\"target\":{\"contact_index\":%d,\"generation\":%u,\"manifold\":%d,\"body_role\":%d,\"before\":\"%08x\",\"flip\":\"%08x\",\"positive\":\"3d000000\"},\"gates\":{\"coverage\":%s,\"finite\":%s,\"locality\":%s,\"prefix_exact\":%s,\"reference_sham_exact\":%s,\"backend\":%s,\"positive_response\":%s,\"save_load_exact\":%s,\"instrumentation_contract\":%s},\"observation_coverage\":%s,\"hook_stages_complete\":%s,\"raw_loaded_matches\":%s,\"prepared_used_matches\":%s,\"hook_overflow\":%s,\"positive_warm_components\":%d,\"persistent_target_epochs\":%d,\"natural_sleep_epochs\":%d,\"save_load_states_compared\":%lld,\"trace_io_ok\":%s,\"replay\":[",
        profile,trial,kind==1?"contacts":"lifecycle",sleepSnapshot?"sleep":"cache",CONTACTLAB_INSTRUMENTED?"true":"false",ok?"complete":"gate_failed",ok?"true":"false",z_backend(requested),requested,width,Z_HORIZON,at,checkpoint,
        z_targets[0].index,z_targets[0].generation,z_targets[0].manifold,targetRole,z_targets[0].before,z_targets[2].after,
        coverage?"true":"false",nonfiniteTick<0?"true":"false",locality?"true":"false",prefix?"true":"false",sham?"true":"false",backendGate?"true":"false",positive?"true":"false",replay?"true":"false",instrument?"true":"false",
        CONTACTLAB_INSTRUMENTED?"true":"false",CONTACTLAB_INSTRUMENTED&&hookStages?"true":"false",CONTACTLAB_INSTRUMENTED&&loaded?"true":"false",CONTACTLAB_INSTRUMENTED&&used?"true":"false",z_overflow?"true":"false",positiveWarm,persistent,sleepTicks,compared,io?"true":"false");
    for (int b=0;b<Z_ARMS;++b) printf("%s{\"arm\":%d,\"recording_bytes\":%d,\"cache_before_save\":\"%08x\",\"cache_after_load\":\"%08x\",\"seed_exact\":%s,\"restart_exact\":%s,\"seek_exact\":%s}",
        b?",":"",b,records[b]?b3Recording_GetSize(records[b]):0,savedBits[b],loadedBits[b],seed[b]?"true":"false",restart[b]?"true":"false",seek[b]?"true":"false");
    fputs("],\"differences\":[",stdout);
    for (int b=2;b<Z_ARMS;++b) { ZDiff* d=diffs+b;
        printf("%s{\"arm\":%d,\"first_body_tick\":%d,\"body_cells\":%d,\"body_nonzero_float_cells\":%d,\"first_id_tick\":%d,\"id_cells\":%d,\"first_event_tick\":%d,\"event_cells\":%d,\"first_cache_tick\":%d,\"cache_signed_zero_cells\":%d,\"cache_other_cells\":%d}",
            b==2?"":",",b,d->firstBody,d->bodyCells,d->bodyFloatCells,d->firstId,d->idCells,d->firstEvent,d->eventCells,d->firstCache,d->zeroCells,d->otherCells);
    }
    printf("],\"target_contact_destroyed_arms\":%d,\"target_contact_slot_reused_arms\":%d,\"restore_first_difference\":{\"arm\":%d,\"tick\":%d,\"tag\":%u,\"expected\":\"%08x\",\"actual\":\"%08x\"},\"seed_event_buffers_compared\":false,\"future_event_order_compared\":true,\"cross_host_recording_transport_tested\":false,\"hooks\":",
        targetGone[0]+targetGone[1]+targetGone[2]+targetGone[3],contactReused[0]+contactReused[1]+contactReused[2]+contactReused[3],
        z_restore_arm,z_restore_tick,z_restore_tag,z_restore_expected,z_restore_actual);
    z_print_hooks(); puts("}");
    for (int b=0;b<Z_ARMS;++b) { if (records[b]) b3DestroyRecording(records[b]); free(states[b]); }
    return ok;
}
static void z_log_stderr(const char* s) { fprintf(stderr,"Box3D: %s\n",s); }
int main(int argc,char** argv)
{
    const char* profile="unspecified"; const char* path=NULL; int requested=4,workers=1;
    for (int i=1;i<argc;i+=2) {
        if (i+1>=argc) goto usage;
        if (!strcmp(argv[i],"--profile")) profile=argv[i+1];
        else if (!strcmp(argv[i],"--trace")) path=argv[i+1];
        else if (!strcmp(argv[i],"--workers")) { if (!number(argv[i+1],1,1,&workers)) goto usage; }
        else if (!strcmp(argv[i],"--simd")) { if (!number(argv[i+1],0,8,&requested) || (requested!=0 && requested!=4 && requested!=8)) goto usage; }
        else goto usage;
    }
    if (!*profile || strlen(profile)>120) goto usage;
    for (const char* q=profile;*q;++q) if (!((*q>='a'&&*q<='z')||(*q>='A'&&*q<='Z')||(*q>='0'&&*q<='9')||strchr("_./:+-",*q))) goto usage;
#if defined(BOX3D_DISABLE_SIMD)
    if (requested!=0) goto usage;
#else
    if (requested==0) goto usage;
#endif
    b3SetLogFcn(z_log_stderr); b3SetSIMDWidth(requested==8?8:4);
    FILE* file=path?fopen(path,"wb"):NULL; if (path && !file) return 2;
    int io=!file || put32(file,0x314c435au),ok=io,trial=0;
    for (int sleep=0;sleep<=1;++sleep) for (int kind=1;kind<=3;kind+=2)
        if (!z_trial(++trial,kind,sleep,profile,requested,file)) ok=0;
    if (file && fclose(file)) { io=0; ok=0; }
    printf("{\"schema\":\"box3d-contactlab-v1\",\"type\":\"summary\",\"driver\":\"zero\",\"profile\":\"%s\",\"instrumented\":%s,\"status\":\"%s\",\"execution_pass\":%s,\"trace_io_ok\":%s,\"quality_status\":\"not_applicable\",\"trials\":4,\"backend_actual\":\"%s\",\"simd_actual\":%d,\"simd_requested\":%d}\n",
        profile,CONTACTLAB_INSTRUMENTED?"true":"false",ok?"complete":"gate_failed",ok?"true":"false",io?"true":"false",z_backend(requested),b3GetSIMDWidth(),requested);
    return ok?0:5;
usage:
    fputs("Usage: contactlab-zero --profile LABEL --workers 1 --simd 0|4|8 [--trace FILE]\n",stderr); return 64;
}
