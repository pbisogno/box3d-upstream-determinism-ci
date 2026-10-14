#if !defined(_WIN32) && !defined(_POSIX_C_SOURCE)
#define _POSIX_C_SOURCE 200809L
#endif
// SPDX-License-Identifier: MIT
// Generic public recording experiments; scene helpers derive from upstream
// test/test_recording.c through the unchanged public stress-replay example.
#define main public_stress_example_main
#include "stress-replay.c"
#undef main
#include "physics_world.h"
#if defined(_WIN32)
#include <io.h>
#define rb_fileno _fileno
#define rb_dup _dup
#define rb_dup2 _dup2
#define rb_close _close
#else
#include <unistd.h>
#define rb_fileno fileno
#define rb_dup dup
#define rb_dup2 dup2
#define rb_close close
#endif
enum { RB_HORIZON=2048, RB_CHECKPOINTS=3, RB_CYCLES=8 };
typedef struct { Scene scene; b3JointId joint; int bodyReuse,shapeReuse,jointReuse,stale,creates,destroys; } RBWorld;
typedef struct { int begin,end,hit,move,sleep,joint; } RBEvents;
static b3JointId rb_joint(Scene* scene)
{
    b3DistanceJointDef jd=b3DefaultDistanceJointDef();
    jd.base.bodyIdA=scene->body[1]; jd.base.bodyIdB=scene->body[2];
    jd.base.forceThreshold=0.01f; jd.base.torqueThreshold=0.01f; jd.base.collideConnected=true;
    jd.length=1.6f; jd.enableSpring=true; jd.hertz=3; jd.dampingRatio=1;
    jd.enableLimit=true; jd.minLength=1.1f; jd.maxLength=2.1f;
    return b3CreateDistanceJoint(scene->world,&jd);
}
static int rb_make(RBWorld* out,int workers)
{
    memset(out,0,sizeof(*out)); Stats ignored;
    if (!make_scene(&out->scene,3,workers,&ignored)) return 0;
    b3ShapeId ids[8];
    for (int b=0;b<3;++b) {
        int n=b3Body_GetShapes(out->scene.body[b],ids,8);
        for (int i=0;i<n;++i) { b3Shape_EnableContactEvents(ids[i],true); b3Shape_EnableHitEvents(ids[i],true); }
    }
    out->joint=rb_joint(&out->scene); out->creates=3;
    return b3Joint_IsValid(out->joint);
}
static int rb_commands(RBWorld* world,int tick)
{
    Scene* s=&world->scene;
    if (tick==1024) b3Body_ApplyLinearImpulseToCenter(s->body[1],(b3Vec3){1,2,0.1f},true);
    if (tick==1280) {
        b3BodyId old=s->body[2]; b3JointId oldJoint=world->joint;
        b3ShapeId oldShape[1],newShape[1];
        if (b3Body_GetShapes(old,oldShape,1)!=1) return 0;
        b3DestroyBody(old); ++world->destroys;
        if (b3Body_IsValid(old) || b3Shape_IsValid(oldShape[0]) || b3Joint_IsValid(oldJoint)) return 0;
        world->stale+=3;
        s->body[2]=make_actor(s,2,tick); ++world->creates;
        if (b3Body_GetShapes(s->body[2],newShape,1)!=1) return 0;
        b3Shape_EnableContactEvents(newShape[0],true); b3Shape_EnableHitEvents(newShape[0],true);
        world->joint=rb_joint(s);
        world->bodyReuse+=old.index1==s->body[2].index1 && old.generation!=s->body[2].generation;
        world->shapeReuse+=oldShape[0].index1==newShape[0].index1 && oldShape[0].generation!=newShape[0].generation;
        world->jointReuse+=oldJoint.index1==world->joint.index1 && oldJoint.generation!=world->joint.generation;
    }
    if (tick==1536) b3Body_ApplyLinearImpulseToCenter(s->body[2],(b3Vec3){0.3f,3,0},true);
    return 1;
}
static int rb_id(Trace* t,uint32_t base,uint32_t index,uint32_t generation)
{ return scalar(t,base,index) && scalar(t,base+1,generation); }
// These are buffered PUBLIC event arrays, not callbacks. Order is preserved.
// World-id provenance and userData pointers are excluded; slot/generation stay exact.
static int rb_events(b3WorldId world,Trace* t,RBEvents* totals)
{
    b3ContactEvents c=b3World_GetContactEvents(world); b3BodyEvents b=b3World_GetBodyEvents(world);
    b3JointEvents j=b3World_GetJointEvents(world); b3SensorEvents se=b3World_GetSensorEvents(world);
    if (c.beginCount>32 || c.endCount>32 || c.hitCount>32 || b.moveCount>8 || j.count>8 || se.beginCount || se.endCount) return 0;
    if (!scalar(t,tag(8,0,1),c.beginCount) || !scalar(t,tag(8,0,2),c.endCount) ||
        !scalar(t,tag(8,0,3),c.hitCount) || !scalar(t,tag(8,0,4),b.moveCount) ||
        !scalar(t,tag(8,0,5),j.count) || !scalar(t,tag(8,0,6),se.beginCount) || !scalar(t,tag(8,0,7),se.endCount)) return 0;
    for (int i=0;i<c.beginCount;++i) {
        b3ContactBeginTouchEvent e=c.beginEvents[i]; uint32_t k=tag(9,i,1);
        if (!rb_id(t,k,e.shapeIdA.index1,e.shapeIdA.generation) || !rb_id(t,k+2,e.shapeIdB.index1,e.shapeIdB.generation) ||
            !rb_id(t,k+4,e.contactId.index1,e.contactId.generation)) return 0;
    }
    for (int i=0;i<c.endCount;++i) {
        b3ContactEndTouchEvent e=c.endEvents[i]; uint32_t k=tag(10,i,1);
        if (!rb_id(t,k,e.shapeIdA.index1,e.shapeIdA.generation) || !rb_id(t,k+2,e.shapeIdB.index1,e.shapeIdB.generation) ||
            !rb_id(t,k+4,e.contactId.index1,e.contactId.generation)) return 0;
    }
    for (int i=0;i<c.hitCount;++i) {
        b3ContactHitEvent e=c.hitEvents[i]; uint32_t k=tag(11,i,1);
        if (!rb_id(t,k,e.shapeIdA.index1,e.shapeIdA.generation) || !rb_id(t,k+2,e.shapeIdB.index1,e.shapeIdB.generation) ||
            !rb_id(t,k+4,e.contactId.index1,e.contactId.generation) || !position(t,k+6,e.point) || !vector(t,k+9,e.normal) ||
            !real(t,k+12,e.approachSpeed) || !scalar(t,k+13,(uint32_t)e.userMaterialIdA) ||
            !scalar(t,k+14,(uint32_t)(e.userMaterialIdA>>32)) || !scalar(t,k+15,(uint32_t)e.userMaterialIdB) ||
            !scalar(t,k+16,(uint32_t)(e.userMaterialIdB>>32))) return 0;
    }
    for (int i=0;i<b.moveCount;++i) {
        b3BodyMoveEvent e=b.moveEvents[i]; uint32_t k=tag(12,i,1);
        if (!rb_id(t,k,e.bodyId.index1,e.bodyId.generation) || !scalar(t,k+2,e.fellAsleep) || !position(t,k+3,e.transform.p) ||
            !vector(t,k+6,e.transform.q.v) || !real(t,k+9,e.transform.q.s)) return 0;
        if (totals && e.fellAsleep) ++totals->sleep;
    }
    for (int i=0;i<j.count;++i) if (!rb_id(t,tag(13,i,1),j.jointEvents[i].jointId.index1,j.jointEvents[i].jointId.generation)) return 0;
    if (totals) { totals->begin+=c.beginCount; totals->end+=c.endCount; totals->hit+=c.hitCount; totals->move+=b.moveCount; totals->joint+=j.count; }
    return 1;
}
static int rb_observe(Scene* scene,int tick,Trace* t,int includeEvents,RBEvents* events)
{
    if (!observe(scene->world,scene->body,tick,t)) return 0;
    for (int i=0;i<ROLES;++i) if (b3Body_IsValid(scene->body[i]) &&
        !rb_id(t,tag(14,i,1),scene->body[i].index1,scene->body[i].generation)) return 0;
    return !includeEvents || rb_events(scene->world,t,events);
}
static int rb_expected_width=0;
static int rb_same(const Trace* a,const Trace* b,int tick,int includeEvents)
{
    Trace left=*a,right=*b;
    if (!includeEvents) {
        while (left.count && left.cells[left.count-1].tag>>24>=8 && left.cells[left.count-1].tag>>24<=13) --left.count;
        while (right.count && right.cells[right.count-1].tag>>24>=8 && right.cells[right.count-1].tag>>24<=13) --right.count;
    }
    Difference previous=difference; int result=equal_trace(&left,&right,tick);
    if (previous.tick>=0) difference=previous;
    return result;
}
static int rb_trace_header(FILE* f)
{ return put32(f,0x31545246u) && put32(f,1) && put32(f,1) && put32(f,RB_HORIZON); }
static int rb_trace_state(FILE* f,int tick,const Trace* t)
{ return put32(f,(uint32_t)tick) && write_trace(f,t); }
static int rb_warm(Scene* scene)
{
    b3ContactData data[CONTACTS];
    for (int i=1;i<ROLES;++i) if (b3Body_IsValid(scene->body[i])) {
        int n=b3Body_GetContactData(scene->body[i],data,CONTACTS);
        for (int c=0;c<n;++c) for (int m=0;m<data[c].manifoldCount;++m)
            for (int p=0;p<data[c].manifolds[m].pointCount;++p) {
                const b3ManifoldPoint* point=data[c].manifolds[m].points+p;
                if (point->persisted && point->normalImpulse!=0) return 1;
            }
    }
    return 0;
}
static int rb_check_player(b3RecPlayer* p,const Trace* reference,int checkpoint,int frame,int workers,long long* compared)
{
    if (b3RecPlayer_GetFrame(p)!=frame || b3RecPlayer_HasDiverged(p)) return 0;
    Scene scene={0}; scene.world=b3RecPlayer_GetWorldId(p); player_bodies(p,scene.body);
    if (b3World_GetWorkerCount(scene.world)!=workers || b3GetWorldFromId(scene.world)->simdWidth!=rb_expected_width) return 0;
    Trace actual; if (!rb_observe(&scene,checkpoint+frame,&actual,frame>0,NULL)) return 0;
    ++*compared; return rb_same(reference+checkpoint+frame,&actual,checkpoint+frame,frame>0);
}

typedef struct { int tick,frames,seedEqual,validator,restarts,seeks,fresh,negativeDetected,magicRejected; long long compared; size_t peak; } RBCheckpoint;
static int rb_trial(int checkpoint,int workers,const Trace* reference,RBCheckpoint* result)
{
    memset(result,0,sizeof(*result)); result->tick=checkpoint; result->frames=RB_HORIZON-checkpoint;
    RBWorld source={0}; b3Recording* recording=NULL; b3RecPlayer* player=NULL; int ok=rb_make(&source,workers);
    for (int tick=1;ok && tick<=checkpoint;++tick) {
        ok=rb_commands(&source,tick); if (ok) b3World_Step(source.scene.world,1.0f/60.0f,4);
    }
    Trace actual;
    if (ok) ok=rb_observe(&source.scene,checkpoint,&actual,0,NULL) && rb_same(reference+checkpoint,&actual,checkpoint,0);
    if (ok) { recording=b3CreateRecording(0); ok=recording!=NULL; }
    if (ok) b3World_StartRecording(source.scene.world,recording);
    for (int tick=checkpoint+1;ok && tick<=RB_HORIZON;++tick) {
        ok=rb_commands(&source,tick); if (ok) b3World_Step(source.scene.world,1.0f/60.0f,4);
        if (ok) ok=rb_observe(&source.scene,tick,&actual,1,NULL) && rb_same(reference+tick,&actual,tick,1);
    }
    if (recording) b3World_StopRecording(source.scene.world);
    if (b3World_IsValid(source.scene.world)) b3DestroyWorld(source.scene.world);
    const uint8_t* bytes=recording?b3Recording_GetData(recording):NULL;
    int size=recording?b3Recording_GetSize(recording):0;
    if (ok) { result->validator=b3ValidateReplay(bytes,size,workers); ok=result->validator; }
    if (ok) {
        uint8_t* bad=(uint8_t*)malloc((size_t)size);
        if (!bad) ok=0;
        else { memcpy(bad,bytes,(size_t)size); bad[0]^=0xff; b3RecPlayer* rejected=b3CreatePlayer(bad,size,workers);
            result->magicRejected=rejected==NULL; if (rejected) b3DestroyPlayer(rejected); free(bad); ok=result->magicRejected; }
    }
    if (ok) { player=b3CreatePlayer(bytes,size,workers); ++result->fresh; ok=player!=NULL; }
    if (ok) {
        b3RecPlayer_SetKeyframePolicy(player,1024u*1024u,32); b3RecPlayer_Restart(player); ++result->restarts;
        result->seedEqual=rb_check_player(player,reference,checkpoint,0,workers,&result->compared); ok=result->seedEqual;
    }
    for (int frame=1;ok && frame<=result->frames;++frame) {
        ok=b3RecPlayer_StepFrame(player) && rb_check_player(player,reference,checkpoint,frame,workers,&result->compared);
        size_t held=b3RecPlayer_GetKeyframeBytes(player); if (held>result->peak) result->peak=held;
        if (held>b3RecPlayer_GetKeyframeBudget(player)) ok=0;
    }
    for (int cycle=0;ok && cycle<RB_CYCLES;++cycle) {
        int target=(cycle*73+17)%(result->frames+1);
        b3RecPlayer_SeekFrame(player,target); ++result->seeks;
        // Seek/restart can rebuild the world. Resolve its id and names again.
        ok=rb_check_player(player,reference,checkpoint,target,workers,&result->compared);
        for (int frame=target+1;ok && frame<=result->frames;++frame)
            ok=b3RecPlayer_StepFrame(player) && rb_check_player(player,reference,checkpoint,frame,workers,&result->compared);
        if (ok) { b3RecPlayer_Restart(player); ++result->restarts;
            ok=rb_check_player(player,reference,checkpoint,0,workers,&result->compared); }
        for (int frame=1;ok && frame<=result->frames;++frame)
            ok=b3RecPlayer_StepFrame(player) && rb_check_player(player,reference,checkpoint,frame,workers,&result->compared);
    }
    if (player) b3DestroyPlayer(player); player=NULL;
    if (ok) {
        player=b3CreatePlayer(bytes,size,workers); ++result->fresh; ok=player!=NULL;
        if (ok) {
            Scene changed={0}; changed.world=b3RecPlayer_GetWorldId(player); player_bodies(player,changed.body);
            b3Body_ApplyLinearImpulseToCenter(changed.body[1],(b3Vec3){1,0.75f,0.4f},true);
            if (!b3RecPlayer_StepFrame(player)) ok=0;
            else {
                changed.world=b3RecPlayer_GetWorldId(player); player_bodies(player,changed.body);
                Trace altered; ok=rb_observe(&changed,checkpoint+1,&altered,1,NULL);
                // A deliberately changed input must alter a BODY float, not merely a hash/flag.
                const Trace* expected=reference+checkpoint+1;
                for (int i=0;ok && i<altered.count;++i) if (altered.cells[i].tag>>24==2) {
                    uint32_t id=altered.cells[i].tag; unsigned field=id&0xffffu;
                    if (field<4 || field>16) continue;
                    for (int j=0;j<expected->count;++j) if (expected->cells[j].tag==id && expected->cells[j].value!=altered.cells[i].value &&
                        ((expected->cells[j].value|altered.cells[i].value)&0x7fffffffu)!=0) result->negativeDetected=1;
                }
                ok=ok && result->negativeDetected;
            }
        }
    }
    if (player) b3DestroyPlayer(player);
    if (recording) b3DestroyRecording(recording);
    return ok;
}
static int rb_capture_upstream_stdout(int* saved)
{
    *saved=-1;
    if (fflush(stdout)!=0) return 0;
    int descriptor=rb_dup(rb_fileno(stdout));
    if (descriptor<0) return 0;
    if (rb_dup2(rb_fileno(stderr),rb_fileno(stdout))<0) { rb_close(descriptor); return 0; }
    *saved=descriptor; return 1;
}
static int rb_restore_stdout(int saved)
{
    int ok=fflush(stdout)==0;
    if (rb_dup2(saved,rb_fileno(stdout))<0) ok=0;
    if (rb_close(saved)!=0) ok=0;
    if (fflush(stdout)!=0) ok=0;
    return ok;
}
static void rb_log_stderr(const char* message)
{
    // Preserve the upstream default prefix/message/newline; only change stream.
    fprintf(stderr,"Box3D: %s\n",message);
}
int main(int argc,char** argv)
{
    b3SetLogFcn(rb_log_stderr);
    const char* profile="unspecified"; const char* path=NULL; int workers=1;
    for (int i=1;i<argc;i+=2) {
        if (i+1>=argc) goto usage;
        if (!strcmp(argv[i],"--profile")) profile=argv[i+1];
        else if (!strcmp(argv[i],"--workers")) { if (!number(argv[i+1],1,4,&workers) || workers==3) goto usage; }
        else if (!strcmp(argv[i],"--trace")) path=argv[i+1];
        else if (!strcmp(argv[i],"--case")) { if (strcmp(argv[i+1],"all") && strcmp(argv[i+1],"mixed")) goto usage; }
        else goto usage;
    }
    if (!*profile || strlen(profile)>120) goto usage;
    for (const char* q=profile;*q;++q) if (!((*q>='a'&&*q<='z')||(*q>='A'&&*q<='Z')||(*q>='0'&&*q<='9')||strchr("_./:+-",*q))) goto usage;
    int saved_stdout=-1;
    if (!rb_capture_upstream_stdout(&saved_stdout)) { fputs("cannot redirect upstream stdout\n",stderr); return 5; }
    Trace* reference=(Trace*)calloc(RB_HORIZON+1,sizeof(Trace)); RBWorld baseline={0}; RBEvents events={0};
    RBCheckpoint checkpoints[RB_CHECKPOINTS]={{0}}; int checkpointTicks[3]={-1,-1,1281};
    int ok=reference && rb_make(&baseline,workers),updates=0,width=0,widthGate=1,warmTicks=0,sleepTicks=0;
    FILE* traceFile=path?fopen(path,"wb"):NULL; if (path && !traceFile) ok=0;
    if (traceFile && !rb_trace_header(traceFile)) ok=0;
    for (int tick=0;ok && tick<=RB_HORIZON;++tick) {
        if (tick) { ok=rb_commands(&baseline,tick); if (ok) { b3World_Step(baseline.scene.world,1.0f/60.0f,4); ++updates; } }
        if (!ok) break;
        const b3World* world=b3GetWorldFromId(baseline.scene.world); int measured=world->simdWidth;
        if (measured!=4 && measured!=8) widthGate=0; if (!width) width=measured; else if (width!=measured) widthGate=0;
        ok=rb_observe(&baseline.scene,tick,reference+tick,1,&events);
        int warm=rb_warm(&baseline.scene); if (warm) { ++warmTicks; if (checkpointTicks[0]<0) checkpointTicks[0]=tick; }
        if (tick>0 && tick<1024 && b3World_GetAwakeBodyCount(baseline.scene.world)==0) {
            ++sleepTicks; if (checkpointTicks[1]<0) checkpointTicks[1]=tick;
        }
        if (traceFile && (!rb_trace_state(traceFile,tick,reference+tick))) ok=0;
    }
    if (traceFile && fclose(traceFile)!=0) ok=0;
    if (b3World_IsValid(baseline.scene.world)) b3DestroyWorld(baseline.scene.world);
    int coverage=checkpointTicks[0]>0 && checkpointTicks[1]>0 && sleepTicks>0 && events.sleep>0 && warmTicks>0 &&
        events.begin>0 && events.end>0 && events.hit>0 && events.move>0 && events.joint>0 &&
        baseline.bodyReuse>0 && baseline.shapeReuse>0 && baseline.jointReuse>0 && baseline.stale==3;
    ok=ok && coverage && widthGate && updates==RB_HORIZON && nonfiniteTick<0;
    int trialGate=ok; rb_expected_width=width;
    for (int i=0;i<RB_CHECKPOINTS;++i) if (ok && checkpointTicks[i]>0 && reference) {
        if (!rb_trial(checkpointTicks[i],workers,reference,checkpoints+i)) trialGate=0;
    } else trialGate=0;
    ok=ok && trialGate;
    if (!rb_restore_stdout(saved_stdout)) { fputs("cannot restore JSON stdout\n",stderr); free(reference); return 5; }
    printf("{\"schema\":\"box3d-frontier-v1\",\"frontier\":\"rollback\",\"profile\":\"%s\",\"case\":\"mixed\",\"status\":\"%s\",\"workers_requested\":%d,\"simd_width_actual\":%d,\"horizon\":%d,\"updates\":%d,\"gates\":{\"coverage\":%s,\"finite\":%s,\"simd_stable\":%s,\"restore_and_controls\":%s},\"natural_sleep_ticks\":%d,\"warm_contact_ticks\":%d,\"events\":{\"contact_begin\":%d,\"contact_end\":%d,\"contact_hit\":%d,\"body_move\":%d,\"fell_asleep\":%d,\"joint_threshold\":%d,\"sensor_coverage\":false},\"reused_body_slots\":%d,\"reused_shape_slots\":%d,\"reused_joint_slots\":%d,\"stale_handles_rejected\":%d,\"checkpoints\":[",
        profile,ok?"complete":"gate_failed",workers,width,RB_HORIZON,updates,coverage?"true":"false",nonfiniteTick<0?"true":"false",widthGate?"true":"false",trialGate?"true":"false",
        sleepTicks,warmTicks,events.begin,events.end,events.hit,events.move,events.sleep,events.joint,baseline.bodyReuse,baseline.shapeReuse,baseline.jointReuse,baseline.stale);
    const char* labels[3]={"warm_contact","natural_sleep","post_reuse"};
    for (int i=0;i<3;++i) { RBCheckpoint* c=checkpoints+i;
        printf("%s{\"name\":\"%s\",\"tick\":%d,\"frames\":%d,\"states_compared\":%lld,\"seed_persistent_equal\":%s,\"embedded_validator\":%s,\"restarts\":%d,\"seeks\":%d,\"fresh_players\":%d,\"changed_input_control_detected\":%s,\"bad_magic_rejected\":%s,\"peak_keyframe_bytes\":%llu}",
            i?",":"",labels[i],c->tick,c->frames,c->compared,c->seedEqual?"true":"false",c->validator?"true":"false",c->restarts,c->seeks,c->fresh,
            c->negativeDetected?"true":"false",c->magicRejected?"true":"false",(unsigned long long)c->peak);
    }
    printf("],\"first_difference\":{\"tick\":%d,\"tag\":%u,\"expected_bits\":%u,\"actual_bits\":%u},\"seed_event_buffers_compared\":false,\"future_event_order_compared\":true,\"nonfinite_tick\":%d}\n",difference.tick,difference.tag,difference.expected,difference.actual,nonfiniteTick);
    free(reference); return ok?0:5;
usage:
    fputs("Usage: frontier-rollback --profile LABEL --workers 1|2|4 [--case all|mixed] [--trace FILE]\n",stderr); return 64;
}
