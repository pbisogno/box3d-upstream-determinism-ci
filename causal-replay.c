// SPDX-License-Identifier: MIT
// A generic counterfactual experiment using the published Box3D source pin.
// The frozen public example supplies scenes, authored commands and observers.
// Only this diagnostic translation unit may write the experimental cache field.
#define main frozen_public_stress_entrypoint
#include "stress-replay.c"
#undef main
#include "physics_world.h"
#include "contact.h"
#include <stddef.h>

enum { TARGETS = 64 };
typedef struct { int tick; uint32_t tag, a,b; const char* field; const char* classification; int bits; } Finding;
typedef struct {
    long long differingTicks,zero,nonzero,integer,structure;
    long long bodyTicks,bodyZero,bodyNonzero,bodyInteger,bodyStructure;
    Finding first,firstNonzero,firstBodyNonzero;
} Effects;
typedef struct { int contactIndex,manifoldIndex; uint32_t generation,before,after; b3Manifold* storage; } Target;
typedef struct {
    Target targets[TARGETS]; int count,positive,negative,awake,sleeping,other;
    int aliasVerified,valid;
} Selection;
typedef struct {
    int flipped,confirmed,bytesVerified,nextRetained,nextAwake,afterRetained,afterSignRetained;
    int warm,nextWarm,impulseReadback;
} Intervention;
static Finding no_finding(void) { return (Finding){-1,0,0,0,"none","none",0}; }
static void init_effects(Effects* e)
{ memset(e,0,sizeof(*e)); e->first=e->firstNonzero=e->firstBodyNonzero=no_finding(); }
static uint32_t float_bits(float value) { uint32_t b; memcpy(&b,&value,4); return b; }
static int is_float_tag(uint32_t id)
{
    unsigned group=id>>24,field=id&0xffffu;
    return (group==2 && field>=4 && field<=16) || (group==4 && field>=2 && field<=11) ||
        (group==5 && field>=1 && field<=5) || (group==6 && field>=5 && field<=10) ||
        (group==7 && field>=3 && field<=9);
}
static const Cell* find_cell(const Trace* t,uint32_t id)
{ for (int i=0;i<t->count;++i) if (t->cells[i].tag==id) return t->cells+i; return NULL; }
static void record_difference(Effects* e,int tick,uint32_t id,uint32_t a,uint32_t b,const char* kind,int bits)
{
    Finding f={tick,id,a,b,field_name(id),kind,bits}; int body=(id>>24)==2;
    if (e->first.tick<0) e->first=f;
    if (strcmp(kind,"signed_zero")==0) { ++e->zero; if (body) ++e->bodyZero; }
    else {
        if (strcmp(kind,"nonzero_float")==0) { ++e->nonzero; if (body) ++e->bodyNonzero; }
        else if (strcmp(kind,"integer")==0) { ++e->integer; if (body) ++e->bodyInteger; }
        else { ++e->structure; if (body) ++e->bodyStructure; }
        if (e->firstNonzero.tick<0) e->firstNonzero=f;
        if (body && e->firstBodyNonzero.tick<0) e->firstBodyNonzero=f;
    }
}
static int compare_observations(const Trace* a,const Trace* b,int tick,Effects* e)
{
    long long old=e->zero+e->nonzero+e->integer+e->structure;
    long long oldBody=e->bodyZero+e->bodyNonzero+e->bodyInteger+e->bodyStructure;
    int sameTags=a->count==b->count;
    for (int i=0;sameTags && i<a->count;++i) if (a->cells[i].tag!=b->cells[i].tag) sameTags=0;
    int sameOrder=sameTags;
    for (int i=0;i<a->count;++i) {
        const Cell* left=a->cells+i; const Cell* right=sameTags?b->cells+i:find_cell(b,left->tag);
        if (!right) { record_difference(e,tick,left->tag,0,0,"membership",0); continue; }
        if (i>=b->count || b->cells[i].tag!=left->tag) sameOrder=0;
        if (left->value==right->value) continue;
        const char* kind=!is_float_tag(left->tag)?"integer":
            ((left->value&0x7fffffffu)==0 && (right->value&0x7fffffffu)==0?"signed_zero":"nonzero_float");
        record_difference(e,tick,left->tag,left->value,right->value,kind,1);
    }
    for (int i=0;!sameTags && i<b->count;++i) if (!find_cell(a,b->cells[i].tag))
        record_difference(e,tick,b->cells[i].tag,0,0,"membership",0);
    if (!sameOrder && a->count==b->count) record_difference(e,tick,0,0,0,"order",0);
    long long now=e->zero+e->nonzero+e->integer+e->structure;
    long long nowBody=e->bodyZero+e->bodyNonzero+e->bodyInteger+e->bodyStructure;
    if (now!=old) ++e->differingTicks; if (nowBody!=oldBody) ++e->bodyTicks;
    return now==old;
}
static int collect_public_contacts(const Scene* scene,Contact out[CONTACTS])
{
    int count=0;
    for (int i=0;i<ROLES;++i) {
        if (!b3Body_IsValid(scene->body[i])) continue;
        if (b3Body_GetContactCapacity(scene->body[i])>CONTACTS) return -1;
        b3ContactData data[CONTACTS]; int n=b3Body_GetContactData(scene->body[i],data,CONTACTS);
        for (int j=0;j<n;++j) {
            int seen=0; for (int k=0;k<count;++k) if (B3_ID_EQUALS(out[k].data.contactId,data[j].contactId)) seen=1;
            if (seen) continue; if (count==CONTACTS) return -1;
            int a=role(b3Shape_GetBody(data[j].shapeIdA)),b=role(b3Shape_GetBody(data[j].shapeIdB));
            if (a<0 || b<0) return -1; out[count++]=(Contact){data[j],a,b};
        }
    }
    qsort(out,(size_t)count,sizeof(Contact),contact_order); return count;
}
static int awake_graph_member(const b3World* world,const b3Contact* contact)
{
    if (contact->setIndex!=b3_awakeSet || contact->colorIndex<0 || contact->colorIndex>=B3_GRAPH_COLOR_COUNT) return 0;
    const b3GraphColor* color=world->constraintGraph.colors+contact->colorIndex;
    int local=contact->localIndex;
    if ((contact->flags&b3_simMeshContact) || contact->colorIndex==B3_OVERFLOW_INDEX)
        return local>=0 && local<color->contacts.count && color->contacts.data[local].contactId==contact->contactId;
    return local>=0 && local<color->convexContacts.count && color->convexContacts.data[local]==contact->contactId;
}
static int sleeping_member(const b3World* world,const b3Contact* contact)
{
    int set=contact->setIndex,local=contact->localIndex;
    if (set<b3_firstSleepingSet || set>=world->solverSets.count) return 0;
    const b3SolverSet* sleeping=world->solverSets.data+set;
    return local>=0 && local<sleeping->contactIndices.count && sleeping->contactIndices.data[local]==contact->contactId;
}
static Selection select_cache(const Scene* scene,int scope)
{
    Selection out={0}; out.valid=1;
    b3World* world=b3GetWorldFromId(scene->world);
    if (!world || world->locked) { out.valid=0; return out; }
    int wanted=-1; uint32_t generation=0;
    if (scope==1) {
        Contact contacts[CONTACTS]; int count=collect_public_contacts(scene,contacts); int index=scene->kind==1?1:0;
        if (count<=index) { out.valid=0; return out; }
        b3ContactData publicData=contacts[index].data;
        wanted=publicData.contactId.index1-1; generation=publicData.contactId.generation;
        if (wanted<0 || wanted>=world->contacts.count || publicData.manifoldCount<1 ||
            world->contacts.data[wanted].manifolds!=publicData.manifolds) { out.valid=0; return out; }
        out.aliasVerified=1;
    }
    // The stable contact owns the persistent manifolds used by graph/sleep sets.
    // In the pinned prepare path, contactLanes[lane]->manifolds is read directly.
    for (int i=0;i<world->contacts.count;++i) {
        b3Contact* contact=world->contacts.data+i;
        if (contact->setIndex==B3_NULL_INDEX || contact->contactId!=i) continue;
        if (scope==1 && (i!=wanted || contact->generation!=generation)) continue;
        if (!contact->manifolds || contact->manifoldCount<1 || !(contact->flags&b3_contactTouchingFlag)) continue;
        int awake=awake_graph_member(world,contact),sleeping=sleeping_member(world,contact);
        for (int m=0;m<contact->manifoldCount;++m) {
            if (scope==1 && m!=0) continue;
            b3Manifold* manifold=contact->manifolds+m;
            if (manifold->pointCount<=0) continue;
            uint32_t bits=float_bits(manifold->twistImpulse);
            if ((bits&0x7fffffffu)!=0) continue;
            if (!awake && !sleeping) { ++out.other; continue; }
            if (out.count==TARGETS) { out.valid=0; return out; }
            out.targets[out.count++]=(Target){i,m,contact->generation,bits,bits,manifold};
            if (bits==0) ++out.positive; else ++out.negative;
            if (awake) ++out.awake; else ++out.sleeping;
        }
    }
    return out;
}
static int selection_equal(const Selection* a,const Selection* b)
{
    if (!a->valid || !b->valid || a->count!=b->count || a->positive!=b->positive || a->negative!=b->negative ||
        a->awake!=b->awake || a->sleeping!=b->sleeping || a->other!=b->other) return 0;
    for (int i=0;i<a->count;++i) if (a->targets[i].contactIndex!=b->targets[i].contactIndex ||
        a->targets[i].manifoldIndex!=b->targets[i].manifoldIndex || a->targets[i].generation!=b->targets[i].generation || a->targets[i].before!=b->targets[i].before) return 0;
    return 1;
}
static int flip_cache(Selection* selected,Intervention* intervention)
{
    for (int i=0;i<selected->count;++i) {
        Target* target=selected->targets+i; b3Manifold* manifold=target->storage;
        uint32_t before=float_bits(manifold->twistImpulse);
        if ((before&0x7fffffffu)!=0 || before!=target->before) return 0;
        unsigned char original[sizeof(b3Manifold)],expected[sizeof(b3Manifold)];
        memcpy(original,manifold,sizeof(original)); memcpy(expected,original,sizeof(expected));
        uint32_t after=before^0x80000000u;
        memcpy(expected+offsetof(b3Manifold,twistImpulse),&after,4);
        memcpy(&manifold->twistImpulse,&after,4); ++intervention->flipped; target->after=after;
        if (float_bits(manifold->twistImpulse)==after) ++intervention->confirmed; else return 0;
        if (memcmp(expected,manifold,sizeof(expected))==0) ++intervention->bytesVerified; else return 0;
    }
    return 1;
}
static void cache_retention(const Scene* scene,const Selection* selected,Intervention* intervention,int beforeStep)
{
    b3World* world=b3GetWorldFromId(scene->world);
    if (beforeStep) intervention->nextWarm=b3World_IsWarmStartingEnabled(scene->world);
    for (int i=0;i<selected->count;++i) {
        const Target* target=selected->targets+i;
        if (target->contactIndex<0 || target->contactIndex>=world->contacts.count) continue;
        const b3Contact* contact=world->contacts.data+target->contactIndex;
        if (contact->contactId!=target->contactIndex || contact->setIndex==B3_NULL_INDEX ||
            contact->generation!=target->generation || contact->manifoldCount<=target->manifoldIndex || !contact->manifolds) continue;
        const b3Manifold* manifold=contact->manifolds+target->manifoldIndex;
        if (manifold!=target->storage || manifold->pointCount<=0) continue;
        if (beforeStep) {
            if (float_bits(manifold->twistImpulse)==target->after) ++intervention->nextRetained;
            if (awake_graph_member(world,contact)) ++intervention->nextAwake;
        } else {
            ++intervention->afterRetained;
            if (float_bits(manifold->twistImpulse)==target->after) ++intervention->afterSignRetained;
        }
    }
}
typedef struct {
    int sleepTick,wakeTick,preSleep[3],sleepCalls[3],sleepingBodies[3],wakeCalls[3],awakeBodies[3],preserved;
} BalancedCycle;
static int set_scene_awake(const Scene* scene,int awake,int* calls,int* observed)
{
    *calls=*observed=0;
    for (int i=1;i<ROLES;++i) if (b3Body_IsValid(scene->body[i])) {
        b3Body_SetAwake(scene->body[i],awake!=0); ++*calls;
    }
    for (int i=1;i<ROLES;++i) if (b3Body_IsValid(scene->body[i]) &&
        b3Body_IsAwake(scene->body[i])==(awake!=0)) ++*observed;
    return *calls>0 && *calls==*observed;
}
static int same_cache_targets(const Selection* before,const Selection* after)
{
    if (!before->valid || !after->valid || before->count<=0 || before->count!=after->count) return 0;
    for (int i=0;i<before->count;++i) {
        const Target* a=before->targets+i; const Target* b=after->targets+i;
        if (a->contactIndex!=b->contactIndex || a->manifoldIndex!=b->manifoldIndex ||
            a->generation!=b->generation || a->before!=b->before || a->storage!=b->storage) return 0;
    }
    return 1;
}
static void print_finding(const Finding* f)
{
    char a[24]="null",b[24]="null";
    if (f->bits) { snprintf(a,sizeof(a),"\"0x%08X\"",f->a); snprintf(b,sizeof(b),"\"0x%08X\"",f->b); }
    printf("{\"tick\":%d,\"tag\":%u,\"field\":\"%s\",\"expected_bits\":%s,\"actual_bits\":%s,\"classification\":\"%s\"}",
        f->tick,f->tag,f->field,a,b,f->classification);
}
static void print_effects(const Effects* e)
{
    printf("{\"differing_ticks\":%lld,\"signed_zero_cells\":%lld,\"nonzero_float_cells\":%lld,\"integer_cells\":%lld,\"structure_cells\":%lld,\"body_differing_ticks\":%lld,\"body_signed_zero_cells\":%lld,\"body_nonzero_float_cells\":%lld,\"body_integer_cells\":%lld,\"body_structure_cells\":%lld,\"first_difference\":",
        e->differingTicks,e->zero,e->nonzero,e->integer,e->structure,e->bodyTicks,e->bodyZero,e->bodyNonzero,e->bodyInteger,e->bodyStructure);
    print_finding(&e->first); fputs(",\"first_nonzero_or_discrete\":",stdout); print_finding(&e->firstNonzero);
    fputs(",\"first_body_nonzero_or_discrete\":",stdout); print_finding(&e->firstBodyNonzero); putchar('}');
}
static int observe_causal(const Scene* scenes,int index,int tick,Trace* out,int widths[3],int* gate)
{
    const Scene* scene=scenes+index;
    const b3World* measuredWorld=b3GetWorldFromId(scene->world);
    int width=measuredWorld?measuredWorld->simdWidth:0;
    if (width!=4 && width!=8) *gate=0;
    if (widths[index]==0) widths[index]=width;
    else if (widths[index]!=width) *gate=0;
    for (int i=0;i<3;++i) if (widths[i]!=0 && widths[i]!=width) *gate=0;
    return observe(scene->world,scene->body,tick,out);
}
static int run_pair(int kind,int scope,int tick,int horizon,int workers,int sleepContext)
{
    Scene scenes[3]={{0}}; Stats stats[3]={{0}}; Trace traces[3]; Selection selected[3]={{0}};
    Intervention injection={0}; Effects control,prefix,immediate,future;
    init_effects(&control); init_effects(&prefix); init_effects(&immediate); init_effects(&future);
    int ok=1,workerGate=1,warmGate=1,steps=0,controlCount=0,prefixCount=0,futureCount=0,activation=0,readback=0,onlySign=0;
    int actualWorkers[3]={0},actualSimd[3]={0},simdGate=1,sleepGate=!sleepContext,wakeGate=!sleepContext;
    BalancedCycle cycle={0}; cycle.sleepTick=cycle.wakeTick=-1;
    cycle.preserved=1;
    for (int i=0;i<3;++i) if (!make_scene(scenes+i,kind,workers,stats+i)) ok=0;
    for (int now=0;ok && now<=horizon;++now) {
        if (now>0) {
            for (int i=0;i<3;++i) if (!commands(scenes+i,now,stats+i)) ok=0;
            if (!ok) break;
            if (now==tick+1 && sleepContext) {
                cycle.wakeTick=now; wakeGate=1;
                for (int i=0;i<3;++i) if (!set_scene_awake(scenes+i,1,cycle.wakeCalls+i,cycle.awakeBodies+i)) wakeGate=0;
                if (cycle.wakeCalls[0]!=cycle.wakeCalls[1] || cycle.wakeCalls[0]!=cycle.wakeCalls[2] ||
                    cycle.wakeCalls[0]!=cycle.sleepCalls[0]) wakeGate=0;
            }
            if (now==tick+1 && scope!=3) cache_retention(scenes+2,selected+2,&injection,1);
            for (int i=0;i<3;++i) b3World_Step(scenes[i].world,1.0f/60.0f,4);
            steps=now;
            if (now==tick+1 && scope!=3) cache_retention(scenes+2,selected+2,&injection,0);
        }
        for (int i=0;i<3;++i) {
            actualWorkers[i]=b3World_GetWorkerCount(scenes[i].world);
            if (actualWorkers[i]!=workers) workerGate=0;
            if (!b3World_IsWarmStartingEnabled(scenes[i].world)) warmGate=0;
            if (!observe_causal(scenes,i,now,traces+i,actualSimd,&simdGate)) ok=0;
        }
        if (actualSimd[0]!=actualSimd[1] || actualSimd[0]!=actualSimd[2]) simdGate=0;
        if (!ok) break;
        ++controlCount; if (!compare_observations(traces,traces+1,now,&control)) ok=0;
        if (now<=tick) { ++prefixCount; if (!compare_observations(traces,traces+2,now,&prefix)) ok=0; }
        else { ++futureCount; compare_observations(traces,traces+2,now,&future); }
        if (!ok) break;
        if (now==tick) {
            if (sleepContext) {
                Selection beforeSleep[3];
                for (int i=0;i<3;++i) {
                    beforeSleep[i]=select_cache(scenes+i,scope==1?1:2);
                    cycle.preSleep[i]=beforeSleep[i].count;
                }
                sleepGate=selection_equal(beforeSleep,beforeSleep+1) && selection_equal(beforeSleep,beforeSleep+2);
                cycle.sleepTick=now;
                for (int i=0;i<3;++i) if (!set_scene_awake(scenes+i,0,cycle.sleepCalls+i,cycle.sleepingBodies+i)) sleepGate=0;
                if (cycle.sleepCalls[0]!=cycle.sleepCalls[1] || cycle.sleepCalls[0]!=cycle.sleepCalls[2]) sleepGate=0;
                // Observe the balanced sleep transition before the experimental write.
                for (int i=0;i<3;++i) if (!observe_causal(scenes,i,now,traces+i,actualSimd,&simdGate)) ok=0;
                if (!ok) break;
                ++controlCount; if (!compare_observations(traces,traces+1,now,&control)) ok=0;
                ++prefixCount; if (!compare_observations(traces,traces+2,now,&prefix)) ok=0;
                for (int i=0;i<3;++i) {
                    selected[i]=select_cache(scenes+i,scope==1?1:2);
                    if (!same_cache_targets(beforeSleep+i,selected+i)) cycle.preserved=0;
                    if (selected[i].count<=0 || selected[i].sleeping!=selected[i].count || selected[i].awake!=0) sleepGate=0;
                }
                sleepGate=sleepGate && cycle.preserved;
                if (!ok || !sleepGate) { ok=0; break; }
            }
            for (int i=0;i<3;++i) selected[i]=select_cache(scenes+i,scope==1?1:2);
            ok=selection_equal(selected,selected+1) && selection_equal(selected,selected+2);
            injection.warm=b3World_IsWarmStartingEnabled(scenes[2].world);
            if (ok && scope==3) {
                b3Vec3 before=b3Body_GetLinearVelocity(scenes[2].body[1]);
                b3Body_ApplyLinearImpulseToCenter(scenes[2].body[1],(b3Vec3){1.0f,0.75f,0.4f},true);
                b3Vec3 after=b3Body_GetLinearVelocity(scenes[2].body[1]);
                injection.impulseReadback=before.x!=after.x || before.y!=after.y || before.z!=after.z;
                activation=readback=onlySign=injection.impulseReadback;
            } else if (ok) {
                activation=selected[2].count>0 && (scope!=1 || selected[2].count==1 && selected[2].aliasVerified);
                ok=activation && flip_cache(selected+2,&injection);
                readback=injection.confirmed==injection.flipped && injection.flipped==selected[2].count;
                onlySign=injection.bytesVerified==injection.flipped;
            }
            if (!ok) break;
            // Repeat ALL three observations equally, including the fresh ray.
            for (int i=0;i<3;++i) if (!observe_causal(scenes,i,now,traces+i,actualSimd,&simdGate)) ok=0;
            if (!ok) break;
            ++controlCount; if (!compare_observations(traces,traces+1,now,&control)) ok=0;
            compare_observations(traces,traces+2,now,&immediate);
            if (scope!=3) {
                readback=readback && immediate.zero==injection.flipped && immediate.nonzero==0 &&
                    immediate.integer==0 && immediate.structure==0 &&
                    (scope!=1 || immediate.first.tag==(kind==1?0x04040005u:0x04000005u));
            } else {
                injection.impulseReadback=injection.impulseReadback && immediate.bodyNonzero>0;
                readback=injection.impulseReadback;
            }
            if (!readback) ok=0;
        }
    }
    int counts=controlCount==horizon+2+sleepContext && prefixCount==tick+1+sleepContext && futureCount==horizon-tick && steps==horizon;
    int sensitive=scope!=3 || injection.impulseReadback && future.bodyNonzero>0;
    int eligible=scope==3 || injection.nextRetained==injection.flipped && injection.nextAwake==injection.flipped && injection.afterRetained==injection.flipped && injection.nextWarm;
    ok=ok && workerGate && warmGate && simdGate && control.differingTicks==0 && prefix.differingTicks==0 &&
        activation && readback && onlySign && counts && sensitive && eligible && sleepGate && wakeGate && nonfiniteTick<0;
    const char* effect=!ok?"not_established":(future.nonzero || future.integer || future.structure)?"nonzero_or_discrete_effect":
        (future.zero?"signed_zero_only":"no_selected_effect");
    const char* scopes[4]={"unknown","single","all","impulse"};
    printf("{\"schema\":\"box3d-causal-pair-v2\",\"status\":\"%s\",\"effect\":\"%s\",\"case\":\"%s\",\"context\":\"%s\",\"scope\":\"%s\",\"intervention_kind\":\"%s\",\"intervention_tick\":%d,\"horizon\":%d,\"steps_completed\":%d,\"workers_requested\":%d,\"workers_actual\":[%d,%d,%d],\"simd_width_actual\":[%d,%d,%d],\"control_states_compared\":%d,\"pre_intervention_states_compared\":%d,\"future_states_compared\":%d,\"gate_applicability\":{\"only_zero_sign_changed\":%s,\"solver_eligibility\":%s,\"positive_control_sensitive\":%s,\"sleep_context\":%s,\"balanced_wake\":%s},\"gates\":{\"workers\":%s,\"warmstarting\":%s,\"finite_observations\":%s,\"sham_exact\":%s,\"prefix_exact\":%s,\"activation\":%s,\"readback\":%s,\"only_zero_sign_changed\":%s,\"expected_comparison_counts\":%s,\"solver_eligibility\":%s,\"positive_control_sensitive\":%s,\"sleep_context\":%s,\"balanced_wake\":%s,\"simd_width_stable_and_equal\":%s},\"intervention\":{\"baseline_eligible\":%d,\"sham_eligible\":%d,\"flip_eligible\":%d,\"flipped\":%d,\"confirmed\":%d,\"object_bytes_verified\":%d,\"positive_before\":%d,\"negative_before\":%d,\"awake_graph\":%d,\"sleeping\":%d,\"other\":%d,\"target_field_tag\":%u,\"public_alias_verified\":%s,\"warmstarting\":%s,\"next_step_retained\":%d,\"next_step_awake_graph\":%d,\"next_step_warmstarting\":%s,\"after_step_retained\":%d,\"after_step_sign_retained\":%d,\"impulse_readback_nonzero\":%s},\"immediate\":",
        ok?"complete":"gate_failed",effect,cases[kind],sleepContext?"sleep_wake":"ordinary",scopes[scope],scope==3?"positive_control":"cache_zero_sign",tick,horizon,steps,
        workers,actualWorkers[0],actualWorkers[1],actualWorkers[2],actualSimd[0],actualSimd[1],actualSimd[2],controlCount,prefixCount,futureCount,
        scope!=3?"true":"false",scope!=3?"true":"false",scope==3?"true":"false",sleepContext?"true":"false",sleepContext?"true":"false",
        workerGate?"true":"false",warmGate?"true":"false",nonfiniteTick<0?"true":"false",control.differingTicks==0?"true":"false",
        prefix.differingTicks==0?"true":"false",activation?"true":"false",readback?"true":"false",onlySign?"true":"false",counts?"true":"false",
        eligible?"true":"false",sensitive?"true":"false",sleepGate?"true":"false",wakeGate?"true":"false",simdGate?"true":"false",selected[0].count,selected[1].count,selected[2].count,injection.flipped,injection.confirmed,
        injection.bytesVerified,selected[2].positive,selected[2].negative,selected[2].awake,selected[2].sleeping,selected[2].other,
        scope==1?(kind==1?0x04040005u:0x04000005u):0u,selected[2].aliasVerified?"true":"false",injection.warm?"true":"false",
        injection.nextRetained,injection.nextAwake,injection.nextWarm?"true":"false",injection.afterRetained,injection.afterSignRetained,
        injection.impulseReadback?"true":"false");
    print_effects(&immediate);
    printf(",\"balanced_cycle\":{\"sleep_tick\":%d,\"wake_tick\":%d,\"pre_sleep_eligible\":[%d,%d,%d],\"sleep_body_calls\":[%d,%d,%d],\"sleeping_bodies\":[%d,%d,%d],\"wake_body_calls\":[%d,%d,%d],\"awake_bodies_before_step\":[%d,%d,%d],\"cache_preserved_across_sleep\":%s}",
        cycle.sleepTick,cycle.wakeTick,cycle.preSleep[0],cycle.preSleep[1],cycle.preSleep[2],
        cycle.sleepCalls[0],cycle.sleepCalls[1],cycle.sleepCalls[2],cycle.sleepingBodies[0],cycle.sleepingBodies[1],cycle.sleepingBodies[2],
        cycle.wakeCalls[0],cycle.wakeCalls[1],cycle.wakeCalls[2],cycle.awakeBodies[0],cycle.awakeBodies[1],cycle.awakeBodies[2],cycle.preserved?"true":"false");
    fputs(",\"future\":",stdout); print_effects(&future);
    fputs(",\"control_first_difference\":",stdout); print_finding(&control.first);
    fputs(",\"prefix_first_difference\":",stdout); print_finding(&prefix.first);
    printf(",\"nonfinite_tick\":%d,\"nonfinite_field_tag\":%u}\n",nonfiniteTick,nonfiniteTag);
    for (int i=0;i<3;++i) if (b3World_IsValid(scenes[i].world)) b3DestroyWorld(scenes[i].world);
    return ok?0:5;
}
int main(int argc,char** argv)
{
    int kind=0,scope=0,tick=-1,horizon=8192,workers=1,sleepContext=0;
    for (int i=1;i<argc;i+=2) {
        if (i+1==argc) goto usage;
        if (strcmp(argv[i],"--case")==0) { kind=0; for (int k=1;k<=3;++k) if (strcmp(argv[i+1],cases[k])==0) kind=k; if (!kind) goto usage; }
        else if (strcmp(argv[i],"--scope")==0) {
            const char* scopes[4]={"unknown","single","all","impulse"}; scope=0;
            for (int k=1;k<=3;++k) if (strcmp(argv[i+1],scopes[k])==0) scope=k; if (!scope) goto usage;
        } else if (strcmp(argv[i],"--context")==0) {
            if (strcmp(argv[i+1],"ordinary")==0) sleepContext=0;
            else if (strcmp(argv[i+1],"sleep_wake")==0) sleepContext=1;
            else goto usage;
        } else if (strcmp(argv[i],"--tick")==0) { if (!number(argv[i+1],1,32767,&tick)) goto usage; }
        else if (strcmp(argv[i],"--horizon")==0) { if (!number(argv[i+1],2,32768,&horizon)) goto usage; }
        else if (strcmp(argv[i],"--workers")==0) { if (!number(argv[i+1],1,4,&workers) || workers==3) goto usage; }
        else goto usage;
    }
    if (!kind || !scope || tick<1 || tick>=horizon || scope==3 && (kind!=1 || tick!=129 || sleepContext)) goto usage;
    return run_pair(kind,scope,tick,horizon,workers,sleepContext);
usage:
    fputs("Usage: causal-replay --case contacts|joints|lifecycle --scope single|all|impulse --tick N [--context ordinary|sleep_wake] [--workers 1|2|4] [--horizon 8192]\n",stderr);
    return 64;
}
