// SPDX-License-Identifier: MIT
// Synthetic snapshot experiments against publicly available Box3D source.
// Internal source access is diagnostic and is not a supported Box3D API contract.
#pragma once
#include "box3d/box3d.h"
#include "box3d/collision.h"
#include "body.h"
#include "contact.h"
#include "joint.h"
#include "shape.h"
#include "physics_world.h"
#include "recording.h"
#include "sha256.h"
#include <inttypes.h>
#include <limits.h>
#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef SNAPSHOTLAB_POLICY
#define SNAPSHOTLAB_POLICY 0
#endif
enum { SK_HEADER=96, SK_MAX_BYTES=64*1024*1024 };
typedef struct { uint8_t* data; size_t size,capacity; } SKBlob;
typedef struct { b3WorldId world; b3RecPlayer* owner; int bodies,tick; } SKWorld;
typedef void*(*SKAlloc)(size_t);
typedef void(*SKFree)(void*,size_t);
static void* sk_default_alloc(size_t n) { return malloc(n); }
static void sk_default_free(void* p,size_t n) { (void)n;free(p); }
static SKAlloc sk_host_alloc=sk_default_alloc;
static SKFree sk_host_free=sk_default_free;
static uint64_t sk_host_live_bytes=0,sk_host_peak_bytes=0,sk_host_alloc_calls=0,sk_host_free_calls=0;
static void* sk_alloc(size_t n) { void* p=sk_host_alloc(n);if(p){sk_host_live_bytes+=n;++sk_host_alloc_calls;if(sk_host_live_bytes>sk_host_peak_bytes)sk_host_peak_bytes=sk_host_live_bytes;}return p; }
static void sk_free(void* p,size_t n) { if(p){sk_host_free(p,n);sk_host_live_bytes-=n;++sk_host_free_calls;} }
static int sk_seen_simd=0,sk_seen_workers=0;
static int sk_imported_frames=-1;
static int sk_configuration(b3WorldId id) { int workers=b3World_GetWorkerCount(id),simd=b3GetWorldFromId(id)->simdWidth;if((sk_seen_simd&&sk_seen_simd!=simd)||(sk_seen_workers&&sk_seen_workers!=workers))return 0;sk_seen_simd=simd;sk_seen_workers=workers;return simd==4&&workers==1; }
static void sk_set_host_allocator(SKAlloc a,SKFree f) { sk_host_alloc=a?a:sk_default_alloc;sk_host_free=f?f:sk_default_free; }
static void sk_blob_free(SKBlob* b) { if(b->data)sk_free(b->data,b->capacity);memset(b,0,sizeof(*b)); }
static int sk_blob_reserve(SKBlob* b,size_t n) {
    if(n>SK_MAX_BYTES)return 0;
    if(n<=b->capacity)return 1;
    uint8_t* p=(uint8_t*)sk_alloc(n);if(!p)return 0;
    if(b->size)memcpy(p,b->data,b->size);
    if(b->data)sk_free(b->data,b->capacity);
    b->data=p;b->capacity=n;return 1;
}
static int sk_blob_copy(SKBlob* to,const SKBlob* from) { if(to==from)return 1;if(!sk_blob_reserve(to,from->size))return 0;memcpy(to->data,from->data,from->size);to->size=from->size;return 1; }
static uint32_t sk_u32(const uint8_t* p) { return (uint32_t)p[0]|(uint32_t)p[1]<<8|(uint32_t)p[2]<<16|(uint32_t)p[3]<<24; }
static uint64_t sk_u64(const uint8_t* p) { return (uint64_t)sk_u32(p)|(uint64_t)sk_u32(p+4)<<32; }
static void sk_w32(uint8_t* p,uint32_t v) { for(int i=0;i<4;++i)p[i]=(uint8_t)(v>>(8*i)); }
static void sk_w64(uint8_t* p,uint64_t v) { for(int i=0;i<8;++i)p[i]=(uint8_t)(v>>(8*i)); }
static uint32_t sk_fb(float f) { uint32_t u;memcpy(&u,&f,4);return u; }
static void sk_hex(const uint8_t* p,size_t n,char* out) { static const char h[]="0123456789abcdef";for(size_t i=0;i<n;++i){out[2*i]=h[p[i]>>4];out[2*i+1]=h[p[i]&15];}out[2*n]=0; }
static void sk_blob_hash(const SKBlob* b,uint8_t out[32]) { sk_sha256(b->data,b->size,out); }
static void sk_envelope_hash(const uint8_t* p,size_t n,uint8_t out[32]) { SKSHA256 c;sk_sha_init(&c);sk_sha_update(&c,p,48);sk_sha_update(&c,p+80,n-80);sk_sha_final(&c,out); }
static int sk_blob_equal(const SKBlob* a,const SKBlob* b) { return a->size==b->size && !memcmp(a->data,b->data,a->size); }
static const char* sk_policy(void) { return SNAPSHOTLAB_POLICY==0?"stock":SNAPSHOTLAB_POLICY==1?"wire":"wirezero"; }
static int sk_runtime_valid(void) { uint32_t x=1;return sizeof(void*)==8&&sizeof(b3Pos)==12&&*(uint8_t*)&x==1; }
static int sk_label(const char* s) { size_t n=strlen(s);if(!n||n>120)return 0;for(size_t i=0;i<n;++i)if(!((s[i]>='a'&&s[i]<='z')||(s[i]>='A'&&s[i]<='Z')||(s[i]>='0'&&s[i]<='9')||strchr("_./:+-",s[i])))return 0;return 1; }
static void sk_close(SKWorld* s) { if(s->owner)b3DestroyPlayer(s->owner);else if(B3_IS_NON_NULL(s->world))b3DestroyWorld(s->world);memset(s,0,sizeof(*s)); }
static b3BodyId sk_actor(const SKWorld* s,int slot) {
    b3World* w=b3GetWorldFromId(s->world);
    if(slot<0||slot>=w->bodies.count||w->bodies.data[slot].id!=slot)return b3_nullBodyId;
    return b3MakeBodyId(w,slot);
}
static b3BodyId sk_add_actor(SKWorld* s,int ordinal) {
    b3BodyDef bd=b3DefaultBodyDef();bd.type=b3_dynamicBody;
    bd.position=(b3Pos){(float)((ordinal-1)%32)*2.0f,0.505f,(float)((ordinal-1)/32)*2.0f};
    b3BodyId id=b3CreateBody(s->world,&bd);b3BoxHull box=b3MakeBoxHull(.5f,.5f,.5f);
    b3ShapeDef sd=b3DefaultShapeDef();sd.density=1.0f;sd.enableContactEvents=true;
    b3CreateHullShape(id,&sd,&box.base);return id;
}
static int sk_create(int bodies,SKWorld* s) {
    if(bodies<16||bodies>4096||!sk_runtime_valid())return 0;
    memset(s,0,sizeof(*s));s->bodies=bodies;
    b3WorldDef wd=b3DefaultWorldDef();wd.workerCount=1;s->world=b3CreateWorld(&wd);
    b3BodyDef bd=b3DefaultBodyDef();bd.position=(b3Pos){30,-.5f,30};
    b3BodyId floor=b3CreateBody(s->world,&bd);b3ShapeDef sd=b3DefaultShapeDef();
    b3BoxHull hull=b3MakeBoxHull(128,.5f,128);b3CreateHullShape(floor,&sd,&hull.base);
    for(int i=1;i<=bodies;++i)sk_add_actor(s,i);
    for(int j=0;j<bodies/16;++j){
        b3DistanceJointDef jd=b3DefaultDistanceJointDef();
        jd.base.bodyIdA=sk_actor(s,j*16+1);jd.base.bodyIdB=sk_actor(s,j*16+2);
        jd.length=2.0f;b3CreateDistanceJoint(s->world,&jd);
    }
    if(!b3World_IsValid(s->world)||!sk_configuration(s->world)){sk_close(s);return 0;}return 1;
}
// Logical commands are external inputs. No future operations are stored in a seed.
static int sk_command(SKWorld* s,int tick,int input) {
    if(tick!=s->tick+1||input<0||input>1)return 0;
    if(tick==132||tick==352){b3BodyId id=sk_actor(s,s->bodies);if(!b3Body_IsValid(id))return 0;b3DestroyBody(id);}
    if(tick==136||tick==356){if(B3_IS_NON_NULL(sk_actor(s,s->bodies)))return 0;b3BodyId id=sk_add_actor(s,s->bodies);if(!b3Body_IsValid(id))return 0;}
    b3BodyId a=sk_actor(s,1);
    if(!b3Body_IsValid(a))return 0;
    if(tick%37==0)b3Body_ApplyLinearImpulseToCenter(a,(b3Vec3){.02f,.03f,.01f},true);
    if(input==1)b3Body_ApplyLinearImpulseToCenter(a,(b3Vec3){1.0f,.75f,.4f},true);
    return 1;
}
static int sk_capture(const SKWorld* s,SKBlob* out) {
    b3Recording* rec=b3CreateRecording(0);if(!rec)return 0;
    b3World_StartRecording(s->world,rec);b3World_StopRecording(s->world);
    int n=b3Recording_GetSize(rec);const uint8_t* p=b3Recording_GetData(rec);
    b3RecHeader h={0};if(n>=(int)sizeof(h))memcpy(&h,p,sizeof(h));
    int ok=n>=(int)sizeof(h)&&h.snapshotSize>=16&&h.registryOffset<=((uint64_t)n)&&h.registryByteCount<=((uint64_t)n)-h.registryOffset;
    uint64_t total=SK_HEADER+h.snapshotSize+h.registryByteCount;
    ok=ok&&h.snapshotSize<SK_MAX_BYTES&&h.registryByteCount<SK_MAX_BYTES&&total<=SK_MAX_BYTES;
    if(ok)ok=sk_blob_reserve(out,(size_t)total);
    if(ok){
        memset(out->data,0,SK_HEADER);memcpy(out->data,"SKSEED1",7);sk_w32(out->data+8,1);sk_w32(out->data+12,SNAPSHOTLAB_POLICY);
        sk_w32(out->data+16,(uint32_t)s->bodies);sk_w32(out->data+20,(uint32_t)s->tick);
        sk_w64(out->data+24,h.snapshotSize);sk_w64(out->data+32,h.registryByteCount);sk_w32(out->data+40,sk_fb(h.lengthScale));
        memcpy(out->data+SK_HEADER,p+sizeof(h),(size_t)h.snapshotSize);
        memcpy(out->data+SK_HEADER+h.snapshotSize,p+h.registryOffset,(size_t)h.registryByteCount);
        sk_envelope_hash(out->data,(size_t)total,out->data+48);out->size=(size_t)total;
    }
    b3DestroyRecording(rec);return ok;
}
static int sk_validate(const SKBlob* b) {
    if(!b||!b->data||b->size<SK_HEADER||b->size>SK_MAX_BYTES)return 0;
    const uint8_t* p=b->data;
    if(memcmp(p,"SKSEED1",8)||sk_u32(p+8)!=1||sk_u32(p+12)!=SNAPSHOTLAB_POLICY||sk_u32(p+16)<16||sk_u32(p+16)>4096||sk_u32(p+20)>1000000||sk_u32(p+44)!=0)return 0;
    for(int i=80;i<SK_HEADER;++i)if(p[i])return 0;
    uint64_t sn=sk_u64(p+24),rn=sk_u64(p+32);
    if(sn<16||sn>SK_MAX_BYTES||rn>SK_MAX_BYTES||sn+rn+SK_HEADER!=b->size||sk_u32(p+40)!=sk_fb(1.0f))return 0;
    uint8_t hash[32];sk_envelope_hash(p,b->size,hash);return memcmp(hash,p+48,32)==0;
}
static int sk_import(const SKBlob* b,SKWorld* out) {
    if(!sk_validate(b))return 0;
    size_t sn=(size_t)sk_u64(b->data+24),rn=(size_t)sk_u64(b->data+32),n=sizeof(b3RecHeader)+sn+rn;
    uint8_t* rec=(uint8_t*)sk_alloc(n);if(!rec)return 0;
    b3RecHeader h={0};h.magic=B3_REC_MAGIC;h.versionMajor=B3_REC_VERSION_MAJOR;h.versionMinor=B3_REC_VERSION_MINOR;
    h.pointerWidth=8;h.lengthScale=1.0f;h.snapshotSize=sn;h.registryOffset=sizeof(h)+sn;h.registryByteCount=rn;
    memcpy(rec,&h,sizeof(h));memcpy(rec+sizeof(h),b->data+SK_HEADER,sn+rn);
    b3RecPlayer* p=b3CreatePlayer(rec,(int)n,1);sk_free(rec,n);if(!p)return 0;
    SKWorld temp={0};temp.owner=p;temp.world=b3RecPlayer_GetWorldId(p);temp.bodies=(int)sk_u32(b->data+16);temp.tick=(int)sk_u32(b->data+20);
    b3World* w=b3GetWorldFromId(temp.world);
    sk_imported_frames=b3RecPlayer_GetInfo(p).frameCount;
    int ok=sk_imported_frames==0&&b3World_IsValid(temp.world)&&sk_configuration(temp.world)&&w->bodies.count==temp.bodies+1;
    if(!ok){sk_close(&temp);return 0;}
    *out=temp;return 1;
}
static int sk_step(const SKBlob* in,int tick,int input,SKBlob* out) {
    if(in==out||(in->data&&in->data==out->data))return 0;
    SKWorld s={0};if(!sk_import(in,&s))return 0;
    int ok=sk_command(&s,tick,input);
    if(ok){b3World_Step(s.world,1.0f/60.0f,4);s.tick=tick;ok=sk_capture(&s,out);}
    sk_close(&s);return ok;
}
static int sk_seed(int bodies,int checkpoint,SKBlob* out) {
    SKWorld s={0};if(checkpoint<0||checkpoint>1000000||!sk_create(bodies,&s))return 0;
    int ok=1;for(int t=1;ok&&t<=checkpoint;++t){ok=sk_command(&s,t,0);if(ok){b3World_Step(s.world,1.0f/60.0f,4);s.tick=t;}}
    if(ok)ok=sk_capture(&s,out);sk_close(&s);return ok;
}
// Explicit supplemental public projection; the complete snapshot remains primary.
static int sk_project(const SKWorld* s,SKBlob* out) {
    b3World* w=b3GetWorldFromId(s->world);size_t n=16+(size_t)w->bodies.count*72;
    if(!sk_blob_reserve(out,n))return 0;uint8_t* p=out->data;memset(p,0,n);
    sk_w32(p,(uint32_t)s->tick);sk_w32(p+4,(uint32_t)w->bodies.count);sk_w32(p+8,(uint32_t)b3World_GetAwakeBodyCount(s->world));sk_w32(p+12,(uint32_t)b3World_GetCounters(s->world).contactCount);
    for(int i=0;i<w->bodies.count;++i){uint8_t* q=p+16+(size_t)i*72;b3Body* b=w->bodies.data+i;sk_w32(q,(uint32_t)b->id);sk_w32(q+4,b->generation);
        if(b->id!=i)continue;b3BodyId id=b3MakeBodyId(w,i);b3Pos x=b3Body_GetPosition(id);b3Quat r=b3Body_GetRotation(id);b3Vec3 v=b3Body_GetLinearVelocity(id),a=b3Body_GetAngularVelocity(id);
        float f[14]={(float)x.x,(float)x.y,(float)x.z,r.v.x,r.v.y,r.v.z,r.s,v.x,v.y,v.z,a.x,a.y,a.z,b3Body_GetMass(id)};
        for(int k=0;k<14;++k){uint32_t bits=sk_fb(f[k]);if((bits&0x7f800000u)==0x7f800000u)return 0;sk_w32(q+8+4*k,bits);}
        sk_w32(q+64,b->type);sk_w32(q+68,(uint32_t)b3Body_IsAwake(id));
    }out->size=n;return 1;
}
static int sk_project_blob(const SKBlob* state,SKBlob* out) { SKWorld s={0};if(!sk_import(state,&s))return 0;int ok=sk_project(&s,out);sk_close(&s);return ok; }
typedef struct { int contacts,warm_points,asleep,awake,joints,body_reuses,shape_reuses;int begin_events,end_events,hit_events,move_events,sleep_events,joint_events; } SKCoverage;
static SKCoverage sk_coverage(const SKWorld* s,int events) {
    SKCoverage c={0};b3World* w=b3GetWorldFromId(s->world);c.awake=b3World_GetAwakeBodyCount(s->world);
    for(int i=0;i<w->bodies.count;++i){b3Body* b=w->bodies.data+i;if(b->id!=i)continue;if(b->generation>1)c.body_reuses+=b->generation-1;if(b->type==b3_dynamicBody&&!b3Body_IsAwake(b3MakeBodyId(w,i)))++c.asleep;}
    for(int i=0;i<w->shapes.count;++i)if(w->shapes.data[i].id==i&&w->shapes.data[i].generation>1)c.shape_reuses+=w->shapes.data[i].generation-1;
    for(int i=0;i<w->contacts.count;++i){b3Contact* t=w->contacts.data+i;if(t->contactId!=i)continue;++c.contacts;for(int m=0;m<t->manifoldCount;++m)for(int p=0;p<t->manifolds[m].pointCount;++p)if(t->manifolds[m].points[p].normalImpulse>0)++c.warm_points;}
    for(int i=0;i<w->joints.count;++i)if(w->joints.data[i].jointId==i)++c.joints;
    if(events){b3ContactEvents e=b3World_GetContactEvents(s->world);b3BodyEvents b=b3World_GetBodyEvents(s->world);c.begin_events=e.beginCount;c.end_events=e.endCount;c.hit_events=e.hitCount;c.move_events=b.moveCount;c.joint_events=b3World_GetJointEvents(s->world).count;for(int i=0;i<b.moveCount;++i)c.sleep_events+=b.moveEvents[i].fellAsleep;}
    return c;
}
typedef struct { SKBlob* out;int ok; } SKWriter;
static void sk_put(SKWriter* w,uint32_t x) { if(!w->ok)return;size_t n=w->out->size+4;if(n>w->out->capacity){size_t cap=w->out->capacity?w->out->capacity*2:1024;if(cap<n)cap=n;if(!sk_blob_reserve(w->out,cap)){w->ok=0;return;}}sk_w32(w->out->data+w->out->size,x);w->out->size=n; }
static void sk_float(SKWriter* w,float f) { uint32_t u=sk_fb(f);if((u&0x7f800000u)==0x7f800000u)w->ok=0;sk_put(w,u); }
static void sk_shape_handle(SKWriter* w,b3ShapeId id) { sk_put(w,(uint32_t)id.index1);sk_put(w,id.generation); }
static void sk_contact_handle(SKWriter* w,b3ContactId id) { sk_put(w,(uint32_t)id.index1);sk_put(w,id.generation); }
// Bodies then contacts/caches then joints then selected NEW-step events, in engine order.
// Local world-handle provenance and host pointers are deliberately not observations.
static int sk_observe(const SKWorld* s,int events,SKBlob* out) {
    if(!sk_project(s,out))return 0;SKWriter wr={out,1};b3World* w=b3GetWorldFromId(s->world);
    sk_put(&wr,0x31544343u);sk_put(&wr,(uint32_t)w->contacts.count);
    for(int i=0;i<w->contacts.count;++i){b3Contact* c=w->contacts.data+i;sk_put(&wr,(uint32_t)c->contactId);sk_put(&wr,c->generation);if(c->contactId!=i)continue;
        sk_put(&wr,c->flags);sk_put(&wr,(uint32_t)c->shapeIdA);sk_put(&wr,(uint32_t)c->shapeIdB);sk_put(&wr,(uint32_t)c->setIndex);sk_put(&wr,(uint32_t)c->colorIndex);sk_put(&wr,(uint32_t)c->localIndex);sk_put(&wr,(uint32_t)c->manifoldCount);
        for(int m=0;m<c->manifoldCount;++m){b3Manifold* mm=c->manifolds+m;float f[10]={mm->normal.x,mm->normal.y,mm->normal.z,mm->twistImpulse,mm->frictionImpulse.x,mm->frictionImpulse.y,mm->frictionImpulse.z,mm->rollingImpulse.x,mm->rollingImpulse.y,mm->rollingImpulse.z};for(int k=0;k<10;++k)sk_float(&wr,f[k]);sk_put(&wr,(uint32_t)mm->pointCount);
            for(int p=0;p<mm->pointCount;++p){b3ManifoldPoint* v=mm->points+p;float q[11]={v->anchorA.x,v->anchorA.y,v->anchorA.z,v->anchorB.x,v->anchorB.y,v->anchorB.z,v->separation,v->normalImpulse,v->totalNormalImpulse,v->normalVelocity,v->baseSeparation};for(int k=0;k<11;++k)sk_float(&wr,q[k]);sk_put(&wr,v->featureId);sk_put(&wr,(uint32_t)v->triangleIndex);sk_put(&wr,v->persisted);}
        }
    }
    sk_put(&wr,(uint32_t)w->joints.count);
    for(int i=0;i<w->joints.count;++i){b3Joint* j=w->joints.data+i;sk_put(&wr,(uint32_t)j->jointId);sk_put(&wr,j->generation);if(j->jointId!=i)continue;sk_put(&wr,(uint32_t)j->type);sk_put(&wr,(uint32_t)j->setIndex);sk_put(&wr,(uint32_t)j->colorIndex);sk_put(&wr,(uint32_t)j->localIndex);sk_put(&wr,j->collideConnected);
        b3JointId id={i+1,(uint16_t)w->worldId,j->generation};b3Vec3 f=b3Joint_GetConstraintForce(id),t=b3Joint_GetConstraintTorque(id);float q[8]={f.x,f.y,f.z,t.x,t.y,t.z,b3Joint_GetLinearSeparation(id),b3Joint_GetAngularSeparation(id)};for(int k=0;k<8;++k)sk_float(&wr,q[k]);
    }
    sk_put(&wr,events?1u:0u);
    if(events){b3ContactEvents c=b3World_GetContactEvents(s->world);b3BodyEvents b=b3World_GetBodyEvents(s->world);b3JointEvents j=b3World_GetJointEvents(s->world);
        sk_put(&wr,(uint32_t)c.beginCount);for(int i=0;i<c.beginCount;++i){sk_shape_handle(&wr,c.beginEvents[i].shapeIdA);sk_shape_handle(&wr,c.beginEvents[i].shapeIdB);sk_contact_handle(&wr,c.beginEvents[i].contactId);}
        sk_put(&wr,(uint32_t)c.endCount);for(int i=0;i<c.endCount;++i){sk_shape_handle(&wr,c.endEvents[i].shapeIdA);sk_shape_handle(&wr,c.endEvents[i].shapeIdB);sk_contact_handle(&wr,c.endEvents[i].contactId);}
        sk_put(&wr,(uint32_t)c.hitCount);for(int i=0;i<c.hitCount;++i){b3ContactHitEvent* e=c.hitEvents+i;sk_shape_handle(&wr,e->shapeIdA);sk_shape_handle(&wr,e->shapeIdB);sk_contact_handle(&wr,e->contactId);float q[7]={(float)e->point.x,(float)e->point.y,(float)e->point.z,e->normal.x,e->normal.y,e->normal.z,e->approachSpeed};for(int k=0;k<7;++k)sk_float(&wr,q[k]);sk_put(&wr,(uint32_t)e->userMaterialIdA);sk_put(&wr,(uint32_t)(e->userMaterialIdA>>32));sk_put(&wr,(uint32_t)e->userMaterialIdB);sk_put(&wr,(uint32_t)(e->userMaterialIdB>>32));}
        sk_put(&wr,(uint32_t)b.moveCount);for(int i=0;i<b.moveCount;++i){b3BodyMoveEvent* e=b.moveEvents+i;sk_put(&wr,(uint32_t)e->bodyId.index1);sk_put(&wr,e->bodyId.generation);sk_put(&wr,e->fellAsleep);float q[7]={(float)e->transform.p.x,(float)e->transform.p.y,(float)e->transform.p.z,e->transform.q.v.x,e->transform.q.v.y,e->transform.q.v.z,e->transform.q.s};for(int k=0;k<7;++k)sk_float(&wr,q[k]);}
        sk_put(&wr,(uint32_t)j.count);for(int i=0;i<j.count;++i){sk_put(&wr,(uint32_t)j.jointEvents[i].jointId.index1);sk_put(&wr,j.jointEvents[i].jointId.generation);}
    }
    return wr.ok;
}
static int sk_step_observed(const SKBlob* in,int tick,int input,SKBlob* out,SKBlob* observation,SKCoverage* coverage) {
    if(in==out||(in->data&&in->data==out->data))return 0;SKWorld s={0};if(!sk_import(in,&s))return 0;
    int ok=sk_command(&s,tick,input);if(ok){b3World_Step(s.world,1.0f/60.0f,4);s.tick=tick;if(coverage)*coverage=sk_coverage(&s,1);if(observation)ok=sk_observe(&s,1,observation);if(ok)ok=sk_capture(&s,out);}sk_close(&s);return ok;
}
static int sk_write_file(const char* path,const SKBlob* b) { FILE* f=fopen(path,"wb");if(!f)return 0;int ok=fwrite(b->data,1,b->size,f)==b->size;return fclose(f)==0&&ok; }
static int sk_read_file(const char* path,SKBlob* b) { FILE* f=fopen(path,"rb");if(!f)return 0;if(fseek(f,0,SEEK_END)){fclose(f);return 0;}long n=ftell(f);if(n<0||n>SK_MAX_BYTES||fseek(f,0,SEEK_SET)||!sk_blob_reserve(b,(size_t)n)){fclose(f);return 0;}int ok=fread(b->data,1,(size_t)n,f)==(size_t)n;b->size=ok?(size_t)n:0;return fclose(f)==0&&ok; }
