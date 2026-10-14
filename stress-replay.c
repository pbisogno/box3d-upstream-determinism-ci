// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Public example scenes adapted from test/test_recording.c (MidStreamContacts,
// SphereRoundTrip, AllOps) at upstream 5d83df83ab47172c2c745b44315e7538ead02397.
// This measures explicitly selected PUBLIC observations, not all causal state.
#include "box3d/box3d.h"
#include "box3d/collision.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#ifdef BOX3D_DOUBLE_PRECISION
#error "This public trace format measures the float profile only"
#endif

enum { TOTAL = 4096, WINDOW = 128, ROLES = 4, CELLS = 2048, CONTACTS = 16,
       JOINTS = 8, RAW_LIMIT = 300 * 1024, REF_MAGIC = 0x31525453u };
_Static_assert(sizeof(float)==4, "Trace values require 32-bit float");
static const char* names[ROLES] = { "floor", "actor0", "actor1", "actor2" };
static const char* cases[4] = { "unknown", "contacts", "joints", "lifecycle" };
typedef struct { uint32_t tag, value; } Cell;
typedef struct { int count; Cell cells[CELLS]; } Trace;
typedef struct { int kind, checkpoint, frames; Trace* states; } Reference;
typedef struct { b3WorldId world; b3BodyId body[ROLES]; int kind, workers; } Scene;
typedef struct { int warm, joints, sleep, wake, creates, destroys, stale, workerChecks;
                 int exists[ROLES], awake[ROLES], actualWorkers; } Stats;
typedef struct { int tick; uint32_t tag, expected, actual; const char* field; } Difference;
static Difference difference = { -1, 0, 0, 0, "none" };
static int observationTick = -1, nonfiniteTick = -1;
static uint32_t nonfiniteTag = 0, nonfiniteBits = 0;
static const char* field_name(uint32_t id);

static uint32_t tag(int group, int item, int field)
{ return ((uint32_t)group << 24) | ((uint32_t)item << 16) | (uint32_t)field; }
static int scalar(Trace* trace, uint32_t id, uint32_t value)
{
    if (trace->count == CELLS) return 0;
    trace->cells[trace->count++] = (Cell){ id, value }; return 1;
}
static int real(Trace* trace, uint32_t id, float value)
{
    uint32_t bits; memcpy(&bits, &value, 4);
    // Reject both infinities and every NaN payload; valid float bits stay exact.
    if ((bits & 0x7f800000u) == 0x7f800000u) {
        if (nonfiniteTick < 0) { nonfiniteTick=observationTick; nonfiniteTag=id; nonfiniteBits=bits; }
        return 0;
    }
    return scalar(trace, id, bits);
}
static int vector(Trace* trace, uint32_t id, b3Vec3 value)
{ return real(trace,id,value.x) && real(trace,id+1,value.y) && real(trace,id+2,value.z); }
static int position(Trace* trace, uint32_t id, b3Pos value)
{ return real(trace,id,value.x) && real(trace,id+1,value.y) && real(trace,id+2,value.z); }
static int put32(FILE* file, uint32_t value)
{
    unsigned char bytes[4] = { (unsigned char)value, (unsigned char)(value>>8),
        (unsigned char)(value>>16), (unsigned char)(value>>24) };
    return fwrite(bytes,1,4,file) == 4;
}
static int get32(FILE* file, uint32_t* value)
{
    unsigned char bytes[4]; if (fread(bytes,1,4,file) != 4) return 0;
    *value = (uint32_t)bytes[0] | ((uint32_t)bytes[1]<<8) |
        ((uint32_t)bytes[2]<<16) | ((uint32_t)bytes[3]<<24); return 1;
}
static int header(FILE* file, int kind, int checkpoint, int frames)
{ return put32(file,REF_MAGIC) && put32(file,(uint32_t)kind) &&
    put32(file,(uint32_t)checkpoint) && put32(file,(uint32_t)frames); }
static int write_trace(FILE* file, const Trace* trace)
{
    if (!put32(file,(uint32_t)trace->count)) return 0;
    for (int i=0;i<trace->count;++i)
        if (!put32(file,trace->cells[i].tag) || !put32(file,trace->cells[i].value)) return 0;
    return 1;
}
static long file_bytes(const char* path)
{
    FILE* file=fopen(path,"rb"); if (!file) return -1;
    int ok=fseek(file,0,SEEK_END)==0; long size=ok?ftell(file):-1; fclose(file); return size;
}
static char* suffix(const char* path, const char* ending)
{
    size_t n=strlen(path)+strlen(ending)+1; char* result=(char*)malloc(n);
    if (result) { strcpy(result,path); strcat(result,ending); } return result;
}
static int read_reference(const char* path, Reference* ref)
{
    long size=file_bytes(path); if (size<16 || size>RAW_LIMIT) return 0;
    FILE* file=fopen(path,"rb"); if (!file) return 0;
    uint32_t magic,kind,checkpoint,frames; int ok=get32(file,&magic) && get32(file,&kind)
        && get32(file,&checkpoint) && get32(file,&frames);
    ok=ok && magic==REF_MAGIC && kind>=1 && kind<=3 && frames==WINDOW && checkpoint<=TOTAL-WINDOW;
    if (ok) { ref->kind=(int)kind; ref->checkpoint=(int)checkpoint; ref->frames=(int)frames;
        ref->states=(Trace*)calloc(WINDOW+1,sizeof(Trace)); ok=ref->states!=NULL; }
    for (int frame=0;ok && frame<=WINDOW;++frame) {
        uint32_t count; ok=get32(file,&count) && count<=CELLS && count>0;
        if (!ok) break; ref->states[frame].count=(int)count;
        for (uint32_t i=0;ok && i<count;++i)
            ok=get32(file,&ref->states[frame].cells[i].tag) && get32(file,&ref->states[frame].cells[i].value);
    }
    if (ok) ok=fgetc(file)==EOF; fclose(file);
    if (!ok) { free(ref->states); ref->states=NULL; } return ok;
}
static int role(b3BodyId body)
{
    if (!b3Body_IsValid(body)) return -1;
    const char* name=b3Body_GetName(body);
    for (int i=0;name && i<ROLES;++i) if (strcmp(name,names[i])==0) return i;
    return -1;
}
static void player_bodies(b3RecPlayer* player, b3BodyId bodies[ROLES])
{
    memset(bodies,0,ROLES*sizeof(b3BodyId));
    int count=b3RecPlayer_GetBodyCount(player);
    for (int i=0;i<count;++i) {
        b3BodyId id=b3RecPlayer_GetBodyId(player,i); int index=role(id);
        if (index>=0) bodies[index]=id;
    }
}
typedef struct { b3ContactData data; int a,b; } Contact;
typedef struct { b3JointId id; int a,b,type; } Joint;
static int contact_order(const void* av,const void* bv)
{
    const Contact* a=(const Contact*)av; const Contact* b=(const Contact*)bv;
    return a->a!=b->a ? a->a-b->a : a->b-b->b;
}
static int joint_order(const void* av,const void* bv)
{
    const Joint* a=(const Joint*)av; const Joint* b=(const Joint*)bv;
    if (a->a!=b->a) return a->a-b->a; if (a->b!=b->b) return a->b-b->b; return a->type-b->type;
}

// Trace tag groups: 1 world; 2 named body; 3 touching contact endpoints;
// 4 manifold; 5 point; 6 joint; 7 freshly executed post-step closest ray.
// Bodies: exists,type,awake,position3,rotation4,linear3,angular3.
// World: absolute tick,body/shape/contact/joint/island counts,awake body count.
// Manifolds: pointCount,normal3,twistImpulse,frictionImpulse3,rollingImpulse3.
// Points: separation,normalImpulse,totalNormalImpulse,normalVelocity,
// baseSeparation,featureId,triangleIndex,persisted. Anchors/unused points omitted.
// Joints: named endpoints,type,awake,constraintForce3,constraintTorque3.
// Ray: hit,named body,point3,normal3,fraction,materialId64,triangle/child indices.
// Opaque IDs/pointers, struct padding, BVH diagnostic visits, worker telemetry
// and every field not enumerated above are excluded, not claimed comparable.
static int observe(b3WorldId world, const b3BodyId bodies[ROLES], int tick, Trace* out)
{
    observationTick=tick; out->count=0; b3Counters counters=b3World_GetCounters(world);
    int values[7]={tick,counters.bodyCount,counters.shapeCount,counters.contactCount,
        counters.jointCount,counters.islandCount,b3World_GetAwakeBodyCount(world)};
    for (int i=0;i<7;++i) if (!scalar(out,tag(1,0,i+1),(uint32_t)values[i])) return 0;
    Contact contacts[CONTACTS]; Joint joints[JOINTS]; int contactCount=0,jointCount=0;
    for (int i=0;i<ROLES;++i) {
        b3BodyId body=bodies[i]; int exists=b3Body_IsValid(body); uint32_t base=tag(2,i,1);
        if (!scalar(out,base,(uint32_t)exists)) return 0; if (!exists) continue;
        b3WorldTransform transform=b3Body_GetTransform(body);
        if (!scalar(out,base+1,(uint32_t)b3Body_GetType(body)) ||
            !scalar(out,base+2,(uint32_t)b3Body_IsAwake(body)) ||
            !position(out,base+3,transform.p) || !real(out,base+6,transform.q.v.x) ||
            !real(out,base+7,transform.q.v.y) || !real(out,base+8,transform.q.v.z) ||
            !real(out,base+9,transform.q.s) || !vector(out,base+10,b3Body_GetLinearVelocity(body)) ||
            !vector(out,base+13,b3Body_GetAngularVelocity(body))) return 0;
        if (b3Body_GetContactCapacity(body)>CONTACTS || b3Body_GetJointCount(body)>JOINTS) return 0;
        b3ContactData temp[CONTACTS]; int count=b3Body_GetContactData(body,temp,CONTACTS);
        for (int j=0;j<count;++j) {
            int seen=0; for (int k=0;k<contactCount;++k)
                if (B3_ID_EQUALS(contacts[k].data.contactId,temp[j].contactId)) seen=1;
            if (seen) continue; if (contactCount==CONTACTS) return 0;
            int a=role(b3Shape_GetBody(temp[j].shapeIdA)), b=role(b3Shape_GetBody(temp[j].shapeIdB));
            if (a<0 || b<0) return 0; contacts[contactCount++]=(Contact){temp[j],a,b};
        }
        b3JointId jt[JOINTS]; count=b3Body_GetJoints(body,jt,JOINTS);
        for (int j=0;j<count;++j) {
            int seen=0; for (int k=0;k<jointCount;++k) if (B3_ID_EQUALS(joints[k].id,jt[j])) seen=1;
            if (seen) continue; if (jointCount==JOINTS) return 0;
            int a=role(b3Joint_GetBodyA(jt[j])), b=role(b3Joint_GetBodyB(jt[j]));
            if (a<0 || b<0) return 0; joints[jointCount++]=(Joint){jt[j],a,b,(int)b3Joint_GetType(jt[j])};
        }
    }
    qsort(contacts,(size_t)contactCount,sizeof(Contact),contact_order);
    if (!scalar(out,tag(3,0,0),(uint32_t)contactCount)) return 0;
    for (int c=0;c<contactCount;++c) {
        const Contact* contact=contacts+c; int mc=contact->data.manifoldCount;
        if (mc<0 || mc>4) return 0;
        if (!scalar(out,tag(3,c,1),(uint32_t)contact->a) || !scalar(out,tag(3,c,2),(uint32_t)contact->b) ||
            !scalar(out,tag(3,c,3),(uint32_t)mc)) return 0;
        for (int m=0;m<mc;++m) {
            const b3Manifold* manifold=contact->data.manifolds+m; int item=c*4+m;
            if (manifold->pointCount<0 || manifold->pointCount>B3_MAX_MANIFOLD_POINTS) return 0;
            uint32_t base=tag(4,item,1);
            if (!scalar(out,base,(uint32_t)manifold->pointCount) || !vector(out,base+1,manifold->normal) ||
                !real(out,base+4,manifold->twistImpulse) || !vector(out,base+5,manifold->frictionImpulse) ||
                !vector(out,base+8,manifold->rollingImpulse)) return 0;
            for (int p=0;p<manifold->pointCount;++p) {
                const b3ManifoldPoint* point=manifold->points+p; base=tag(5,item*B3_MAX_MANIFOLD_POINTS+p,1);
                if (!real(out,base,point->separation) || !real(out,base+1,point->normalImpulse) ||
                    !real(out,base+2,point->totalNormalImpulse) || !real(out,base+3,point->normalVelocity) ||
                    !real(out,base+4,point->baseSeparation) || !scalar(out,base+5,point->featureId) ||
                    !scalar(out,base+6,(uint32_t)point->triangleIndex) || !scalar(out,base+7,(uint32_t)point->persisted)) return 0;
            }
        }
    }
    qsort(joints,(size_t)jointCount,sizeof(Joint),joint_order);
    if (!scalar(out,tag(6,0,0),(uint32_t)jointCount)) return 0;
    for (int j=0;j<jointCount;++j) {
        uint32_t base=tag(6,j,1); b3JointId id=joints[j].id;
        if (!scalar(out,base,(uint32_t)joints[j].a) || !scalar(out,base+1,(uint32_t)joints[j].b) ||
            !scalar(out,base+2,(uint32_t)joints[j].type) || !scalar(out,base+3,(uint32_t)b3Joint_IsAwake(id)) ||
            !vector(out,base+4,b3Joint_GetConstraintForce(id)) || !vector(out,base+7,b3Joint_GetConstraintTorque(id))) return 0;
    }
    b3RayResult ray=b3World_CastRayClosest(world,(b3Pos){0,12,0},(b3Vec3){0,-20,0},b3DefaultQueryFilter());
    if (!scalar(out,tag(7,0,1),(uint32_t)ray.hit)) return 0;
    if (ray.hit) {
        int index=role(b3Shape_GetBody(ray.shapeId)); if (index<0) return 0;
        if (!scalar(out,tag(7,0,2),(uint32_t)index) || !position(out,tag(7,0,3),ray.point) ||
            !vector(out,tag(7,0,6),ray.normal) || !real(out,tag(7,0,9),ray.fraction) ||
            !scalar(out,tag(7,0,10),(uint32_t)ray.userMaterialId) ||
            !scalar(out,tag(7,0,11),(uint32_t)(ray.userMaterialId>>32)) ||
            !scalar(out,tag(7,0,12),(uint32_t)ray.triangleIndex) ||
            !scalar(out,tag(7,0,13),(uint32_t)ray.childIndex)) return 0;
    }
    return 1;
}

static b3BodyId make_actor(Scene* scene,int index,int tick)
{
    b3BodyDef bd=b3DefaultBodyDef(); bd.type=b3_dynamicBody;
    float x=scene->kind==2 ? 1.25f*(float)(index-2) : (scene->kind==3 ? (index==1?-0.8f:0.8f) : 0.0f);
    float y=scene->kind==1 ? 3.5f+1.5f*(float)(index-1) : 5.0f;
    bd.position=(b3Pos){x,y,0}; b3BodyId body=b3CreateBody(scene->world,&bd);
    b3Body_SetName(body,names[index]);
    b3ShapeDef sd=b3DefaultShapeDef(); sd.density=1; sd.enableHitEvents=true;
    sd.baseMaterial.rollingResistance=0.2f;
    if (scene->kind==3 && index==2 && (tick/512)%2==0) {
        b3Sphere sphere={b3Vec3_zero,0.35f}; b3CreateSphereShape(body,&sd,&sphere);
    } else {
        float half=scene->kind==2?0.25f:0.35f;
        b3BoxHull box=b3MakeBoxHull(half,half,half); b3CreateHullShape(body,&sd,&box.base);
    }
    return body;
}
static int make_scene(Scene* scene,int kind,int workers,Stats* stats)
{
    memset(scene,0,sizeof(*scene)); memset(stats,0,sizeof(*stats));
    scene->kind=kind; scene->workers=workers;
    b3WorldDef wd=b3DefaultWorldDef(); wd.workerCount=(uint32_t)workers;
    scene->world=b3CreateWorld(&wd); if (!b3World_IsValid(scene->world)) return 0;
    b3World_SetGravity(scene->world,(b3Vec3){0,-10,0});
    b3BodyDef bd=b3DefaultBodyDef(); scene->body[0]=b3CreateBody(scene->world,&bd);
    b3Body_SetName(scene->body[0],names[0]); b3BoxHull floor=b3MakeBoxHull(8,1,8);
    b3ShapeDef sd=b3DefaultShapeDef(); sd.enableHitEvents=true;
    b3CreateHullShape(scene->body[0],&sd,&floor.base); ++stats->creates;
    int count=kind==2?3:2;
    for (int i=1;i<=count;++i) { scene->body[i]=make_actor(scene,i,0); ++stats->creates; }
    if (kind==2) {
        for (int i=1;i<3;++i) {
            b3DistanceJointDef jd=b3DefaultDistanceJointDef();
            jd.base.bodyIdA=scene->body[i]; jd.base.bodyIdB=scene->body[i+1];
            jd.length=1.25f; jd.enableSpring=true; jd.hertz=4; jd.dampingRatio=0.7f;
            jd.enableLimit=true; jd.minLength=1; jd.maxLength=1.5f;
            b3CreateDistanceJoint(scene->world,&jd);
        }
    }
    return 1;
}
static int commands(Scene* scene,int tick,Stats* stats)
{
    // Authored public commands, not network/gameplay inputs. The quiet interval
    // explicitly requests sleep, and the wake command is equally explicit.
    int phase=tick%512;
    if (phase==128 && b3Body_IsValid(scene->body[1])) {
        float sign=(tick/512)%2 ? -1.0f : 1.0f;
        b3Body_ApplyLinearImpulseToCenter(scene->body[1],(b3Vec3){0.1f*sign,0.3f,0.02f},true);
    }
    if (phase==320 || phase==400)
        for (int i=1;i<ROLES;++i) if (b3Body_IsValid(scene->body[i])) b3Body_SetAwake(scene->body[i],phase==400);
    if (scene->kind==3 && phase==432 && b3Body_IsValid(scene->body[2])) {
        b3BodyId old=scene->body[2]; b3DestroyBody(old); scene->body[2]=b3_nullBodyId;
        ++stats->destroys; if (b3Body_IsValid(old)) return 0; ++stats->stale;
    }
    if (scene->kind==3 && phase==440) { scene->body[2]=make_actor(scene,2,tick); ++stats->creates; }
    return 1;
}
static int statistics(Scene* scene,Stats* stats)
{
    ++stats->workerChecks;
    stats->actualWorkers=b3World_GetWorkerCount(scene->world);
    if (stats->actualWorkers!=scene->workers) return 0;
    b3Counters counts=b3World_GetCounters(scene->world); if (counts.jointCount>0) ++stats->joints;
    int warm=0;
    for (int i=1;i<ROLES;++i) {
        int exists=b3Body_IsValid(scene->body[i]); int awake=exists ? b3Body_IsAwake(scene->body[i]) : 0;
        if (exists && stats->exists[i]) {
            if (stats->awake[i] && !awake) ++stats->sleep;
            if (!stats->awake[i] && awake) ++stats->wake;
        }
        stats->exists[i]=exists; stats->awake[i]=awake;
        if (!exists) continue;
        b3ContactData data[CONTACTS]; int count=b3Body_GetContactData(scene->body[i],data,CONTACTS);
        for (int c=0;c<count;++c) for (int m=0;m<data[c].manifoldCount;++m) {
            const b3Manifold* manifold=data[c].manifolds+m;
            for (int p=0;p<manifold->pointCount;++p)
                if (manifold->points[p].persisted && manifold->points[p].normalImpulse!=0) warm=1;
        }
    }
    if (warm) ++stats->warm; return 1;
}
static const char* field_name(uint32_t id)
{
    switch (id>>24) {
        case 1:return "world.tick_or_count"; case 2:return "named_body.pose_velocity_or_awake";
        case 3:return "touching_contact.endpoints_or_count"; case 4:return "manifold.normal_or_impulse";
        case 5:return "manifold_point.impulse_or_matching_field";
        case 6:return "joint.endpoint_force_or_torque"; case 7:return "fresh_closest_ray.result";
        default:return "trace.structure";
    }
}
static int equal_trace(const Trace* expected,const Trace* actual,int tick)
{
    int count=expected->count<actual->count?expected->count:actual->count;
    for (int i=0;i<count;++i) if (expected->cells[i].tag!=actual->cells[i].tag || expected->cells[i].value!=actual->cells[i].value) {
        difference=(Difference){tick,expected->cells[i].tag,expected->cells[i].value,actual->cells[i].value,field_name(expected->cells[i].tag)};
        return 0;
    }
    if (expected->count!=actual->count) {
        difference=(Difference){tick,0,(uint32_t)expected->count,(uint32_t)actual->count,"trace.cell_count"}; return 0;
    }
    return 1;
}
static int capture(const char* path,int kind,int checkpoint,int workers)
{
    char* referencePath=suffix(path,".ref"); char* fullPath=suffix(path,".full-trace");
    FILE* refFile=NULL; FILE* fullFile=NULL; b3Recording* recording=NULL;
    Scene scene={0}; Stats stats={0}; int ok=referencePath && fullPath;
    if (ok) { refFile=fopen(referencePath,"wb"); fullFile=fopen(fullPath,"wb"); ok=refFile && fullFile; }
    if (ok) ok=header(refFile,kind,checkpoint,WINDOW) && header(fullFile,kind,0,TOTAL) && make_scene(&scene,kind,workers,&stats);
    Trace trace; int recordingStarted=0,recordingStopped=0,updates=0,recordedFrames=0,refStates=0,fullStates=0;
    for (int tick=0;ok && tick<=TOTAL;++tick) {
        if (tick>0) { ok=commands(&scene,tick,&stats); if (ok) { b3World_Step(scene.world,1.0f/60.0f,4); ++updates; if (recordingStarted && !recordingStopped) ++recordedFrames; } }
        if (!ok) break;
        if (tick==checkpoint) {
            recording=b3CreateRecording(0); if (!recording) { ok=0; break; }
            b3World_StartRecording(scene.world,recording); recordingStarted=1;
        }
        ok=observe(scene.world,scene.body,tick,&trace) && write_trace(fullFile,&trace);
        if (ok) { ++fullStates; ok=statistics(&scene,&stats); }
        if (ok && tick>=checkpoint && tick<=checkpoint+WINDOW) { ok=write_trace(refFile,&trace); if (ok) ++refStates; }
        if (tick==checkpoint+WINDOW && recordingStarted) { b3World_StopRecording(scene.world); recordingStopped=1; }
    }
    if (recordingStarted && !recordingStopped) b3World_StopRecording(scene.world);
    if (b3World_IsValid(scene.world)) b3DestroyWorld(scene.world);
    if (refFile && fclose(refFile)!=0) ok=0; if (fullFile && fclose(fullFile)!=0) ok=0;
    int recordingBytes=recording ? b3Recording_GetSize(recording) : 0;
    long referenceBytes=referencePath ? file_bytes(referencePath) : -1;
    int validated=0;
    if (ok && recordingBytes>0 && recordingBytes<=RAW_LIMIT && referenceBytes>0 && referenceBytes<=RAW_LIMIT)
        validated=b3ValidateReplay(b3Recording_GetData(recording),recordingBytes,1);
    ok=ok && validated && b3SaveRecordingToFile(recording,path);
    b3DestroyRecording(recording);
    int lifecycle=kind!=3 || (stats.destroys>0 && stats.stale==stats.destroys && stats.creates>stats.destroys);
    int jointGate=kind!=2 || stats.joints>0;
    ok=ok && stats.warm>0 && stats.sleep>0 && stats.wake>0 && lifecycle && jointGate && stats.workerChecks==TOTAL+1;
    char finiteBits[24]="null";
    if (nonfiniteTick>=0) snprintf(finiteBits,sizeof(finiteBits),"\"0x%08X\"",nonfiniteBits);
    printf("{\"schema\":\"box3d-stress-capture-v1\",\"status\":\"%s\",\"case\":\"%s\",\"total_ticks\":%d,\"checkpoint\":%d,\"frames\":%d,\"recording_bytes\":%d,\"reference_bytes\":%ld,\"reference_states\":%d,\"full_trace_states\":%d,\"workers_requested\":%d,\"workers_actual\":%d,\"worker_checks\":%d,\"recording_validated\":%s,\"observations\":{\"warm_contact_ticks\":%d,\"joint_ticks\":%d,\"sleep_transitions\":%d,\"wake_transitions\":%d,\"creates\":%d,\"destroys\":%d,\"stale_handles_invalid\":%d},\"gates\":{\"workers\":%s,\"warm_contacts\":%s,\"sleep_wake\":%s,\"lifecycle_commands\":%s,\"joints\":%s},\"first_divergent_tick\":-1,\"nonfinite_tick\":%d,\"nonfinite_field_tag\":%u,\"nonfinite_field\":\"%s\",\"nonfinite_bits\":%s}\n",
        ok?"pass":"gate_failed",cases[kind],updates,checkpoint,recordedFrames,recordingBytes,referenceBytes,refStates,fullStates,
        workers,stats.actualWorkers,stats.workerChecks,validated?"true":"false",stats.warm,stats.joints,stats.sleep,stats.wake,stats.creates,
        stats.destroys,stats.stale,stats.workerChecks==TOTAL+1?"true":"false",stats.warm>0?"true":"false",
        stats.sleep>0 && stats.wake>0?"true":"false",lifecycle?"true":"false",jointGate?"true":"false",nonfiniteTick,nonfiniteTag,
        nonfiniteTick>=0?field_name(nonfiniteTag):"none",finiteBits);
    free(referencePath); free(fullPath); return ok?0:5;
}

typedef struct { long states; int fresh,restarts,seeks,workerChecks,actualWorkers,observedStates; size_t peakKeyframeBytes; } VerifyStats;
static b3RecPlayer* new_player(const b3Recording* recording,int workers,VerifyStats* stats)
{
    b3RecPlayer* player=b3CreatePlayer(b3Recording_GetData(recording),b3Recording_GetSize(recording),workers);
    // NULL means CreatePlayer rejection only; execution policies are checked below.
    if (!player) return NULL; ++stats->fresh;
    b3RecPlayer_SetWorkerCount(player,workers);
    b3RecPlayer_SetKeyframePolicy(player,64*1024,8);
    b3RecPlayer_Restart(player); ++stats->restarts;
    return player;
}
static int compare_player(b3RecPlayer* player,const Reference* ref,int frame,int workers,VerifyStats* stats,FILE* observed)
{
    b3WorldId world=b3RecPlayer_GetWorldId(player); ++stats->workerChecks;
    stats->actualWorkers=b3World_GetWorkerCount(world);
    size_t keyframeBytes=b3RecPlayer_GetKeyframeBytes(player);
    if (keyframeBytes>stats->peakKeyframeBytes) stats->peakKeyframeBytes=keyframeBytes;
    if (stats->actualWorkers!=workers || b3RecPlayer_GetFrame(player)!=frame ||
        b3RecPlayer_GetKeyframeBudget(player)!=64*1024 || b3RecPlayer_GetKeyframeMinInterval(player)!=8 || keyframeBytes>64*1024) return 0;
    if (b3RecPlayer_HasDiverged(player)) {
        int first=b3RecPlayer_GetDivergeFrame(player);
        difference=(Difference){first<0?-1:ref->checkpoint+first,0,0,0,"native_state_hash"}; return 0;
    }
    b3BodyId bodies[ROLES]; player_bodies(player,bodies); Trace actual;
    if (!observe(world,bodies,ref->checkpoint+frame,&actual)) return 0;
    ++stats->states;
    if (!equal_trace(ref->states+frame,&actual,ref->checkpoint+frame)) return 0;
    if (observed) { if (!write_trace(observed,&actual)) return 0; ++stats->observedStates; }
    return 1;
}
static int complete(b3RecPlayer* player)
{
    // Consume trailing queries/bounds/end marker, without manufacturing a step.
    if (b3RecPlayer_StepFrame(player)) return 0;
    return b3RecPlayer_IsAtEnd(player) && !b3RecPlayer_HasDiverged(player);
}
static int rejected(const char* status,int code)
{
    printf("{\"schema\":\"box3d-stress-verify-v1\",\"status\":\"%s\",\"first_divergent_tick\":-1}\n",status);
    return code;
}
static int verify(const char* path,const char* referencePath,int workers,int cycles)
{
    // Load first: a bad-magic negative control must remain a Load rejection.
    b3Recording* recording=b3LoadRecordingFromFile(path);
    if (!recording) return rejected("load_rejected",2);
    Reference ref={0}; if (!read_reference(referencePath,&ref)) {
        b3DestroyRecording(recording); return rejected("reference_rejected",6);
    }
    VerifyStats stats={0}; b3RecPlayer* player=new_player(recording,workers,&stats);
    if (!player) { free(ref.states); b3DestroyRecording(recording); return rejected("player_rejected",3); }
    char* observedPath=suffix(path,".observed-trace"); FILE* observed=observedPath?fopen(observedPath,"wb"):NULL;
    int ok=observed && header(observed,ref.kind,ref.checkpoint,WINDOW);
    b3RecPlayerInfo info=b3RecPlayer_GetInfo(player);
    ok=ok && info.frameCount==WINDOW && info.timeStep==1.0f/60.0f && info.subStepCount==4;
    if (ok) ok=compare_player(player,&ref,0,workers,&stats,observed);
    for (int frame=1;ok && frame<=WINDOW;++frame)
        ok=b3RecPlayer_StepFrame(player) && compare_player(player,&ref,frame,workers,&stats,observed);
    if (ok) ok=complete(player);
    if (observed && fclose(observed)!=0) ok=0; observed=NULL;
    for (int cycle=0;ok && cycle<cycles;++cycle) {
        if (cycle%16==0) {
            b3DestroyPlayer(player); player=new_player(recording,workers,&stats); if (!player) { ok=0; break; }
        } else { b3RecPlayer_Restart(player); ++stats.restarts; }
        ok=compare_player(player,&ref,0,workers,&stats,NULL);
        for (int frame=1;ok && frame<=WINDOW;++frame)
            ok=b3RecPlayer_StepFrame(player) && compare_player(player,&ref,frame,workers,&stats,NULL);
        if (ok) ok=complete(player);
        if (ok) {
            b3RecPlayer_SeekFrame(player,64); ++stats.seeks;
            // Always reacquire world and named handles after restoration.
            ok=compare_player(player,&ref,64,workers,&stats,NULL);
        }
        for (int frame=65;ok && frame<=WINDOW;++frame)
            ok=b3RecPlayer_StepFrame(player) && compare_player(player,&ref,frame,workers,&stats,NULL);
        if (ok) ok=complete(player);
    }
    int nativeDiverged=player && b3RecPlayer_HasDiverged(player);
    if (nativeDiverged && difference.tick<0) {
        int frame=b3RecPlayer_GetDivergeFrame(player);
        difference=(Difference){frame<0?-1:ref.checkpoint+frame,0,0,0,"native_state_hash"};
    }
    if (player) b3DestroyPlayer(player);
    int validated=ok && b3ValidateReplay(b3Recording_GetData(recording),b3Recording_GetSize(recording),1);
    ok=ok && validated && stats.states==129L+194L*cycles && stats.restarts==1+cycles &&
        stats.seeks==cycles && stats.fresh==1+(cycles+15)/16;
    char expectedBits[24]="null",actualBits[24]="null";
    if (difference.tick>=0 && strcmp(difference.field,"native_state_hash")!=0) {
        snprintf(expectedBits,sizeof(expectedBits),"\"0x%08X\"",difference.expected);
        snprintf(actualBits,sizeof(actualBits),"\"0x%08X\"",difference.actual);
    }
    char finiteBits[24]="null";
    if (nonfiniteTick>=0) snprintf(finiteBits,sizeof(finiteBits),"\"0x%08X\"",nonfiniteBits);
    int diverged=nonfiniteTick<0 && (nativeDiverged || difference.tick>=0);
    printf("{\"schema\":\"box3d-stress-verify-v1\",\"status\":\"%s\",\"case\":\"%s\",\"checkpoint\":%d,\"frames\":%d,\"cycles\":%d,\"fresh_players\":%d,\"restart_count\":%d,\"seek_count\":%d,\"states_compared\":%ld,\"workers_requested\":%d,\"workers_actual\":%d,\"worker_checks\":%d,\"recording_validated\":%s,\"observed_trace_states\":%d,\"first_divergent_tick\":%d,\"first_divergent_field_tag\":%u,\"field\":\"%s\",\"expected_bits\":%s,\"actual_bits\":%s,\"keyframe_budget_bytes\":65536,\"keyframe_peak_bytes\":%zu,\"nonfinite_tick\":%d,\"nonfinite_field_tag\":%u,\"nonfinite_field\":\"%s\",\"nonfinite_bits\":%s}\n",
        ok?"pass":(diverged?"diverged":"gate_failed"),cases[ref.kind],ref.checkpoint,WINDOW,cycles,
        stats.fresh,stats.restarts,stats.seeks,stats.states,workers,stats.actualWorkers,stats.workerChecks,
        validated?"true":"false",stats.observedStates,difference.tick,difference.tag,difference.field,expectedBits,actualBits,stats.peakKeyframeBytes,nonfiniteTick,nonfiniteTag,
        nonfiniteTick>=0?field_name(nonfiniteTag):"none",finiteBits);
    free(ref.states); free(observedPath); b3DestroyRecording(recording);
    return ok?0:(diverged?4:5);
}
static int number(const char* text,int low,int high,int* value)
{
    char* end; long n=strtol(text,&end,10);
    if (!*text || *end || n<low || n>high) return 0; *value=(int)n; return 1;
}
int main(int argc,char** argv)
{
    if (argc<3) goto usage;
    int captureMode=strcmp(argv[1],"capture")==0;
    int verifyMode=strcmp(argv[1],"verify")==0 || strcmp(argv[1],"replay")==0;
    if (!captureMode && !verifyMode) goto usage;
    int kind=0,checkpoint=-1,workers=1,cycles=100; const char* referencePath=NULL;
    for (int i=3;i<argc;i+=2) {
        if (i+1==argc) goto usage;
        if (strcmp(argv[i],"--case")==0 && captureMode) {
            for (int k=1;k<=3;++k) if (strcmp(argv[i+1],cases[k])==0) kind=k;
            if (!kind) goto usage;
        } else if (strcmp(argv[i],"--checkpoint")==0 && captureMode) {
            if (!number(argv[i+1],0,TOTAL-WINDOW,&checkpoint)) goto usage;
        } else if (strcmp(argv[i],"--workers")==0) {
            if (!number(argv[i+1],1,4,&workers) || workers==3) goto usage;
        } else if (strcmp(argv[i],"--cycles")==0 && verifyMode) {
            if (!number(argv[i+1],0,1000,&cycles)) goto usage;
        } else if (strcmp(argv[i],"--reference")==0 && verifyMode) referencePath=argv[i+1];
        else goto usage;
    }
    if (captureMode) {
        if (!kind) goto usage;
        if (checkpoint<0) checkpoint=kind==1?64:(kind==2?2048:3968);
        return capture(argv[2],kind,checkpoint,workers);
    }
    if (!referencePath) goto usage;
    return verify(argv[2],referencePath,workers,cycles);
usage:
    fputs("Usage: stress-replay capture FILE --case contacts|joints|lifecycle [--checkpoint N] [--workers 1|2|4]\n"
          "       stress-replay verify FILE --reference FILE.ref [--workers 1|2|4] [--cycles N]\n",stderr);
    return 64;
}
