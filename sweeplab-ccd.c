// SPDX-License-Identifier: MIT
// Matched synthetic CCD strata. Pinned upstream remains untouched; optional
// hooks copy diagnostics after existing calculations, never solver inputs.
#include "box3d/box3d.h"
#include "box3d/collision.h"
#include "physics_world.h"
#include "sweep-hook.h"
#include <inttypes.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#ifndef SWEEPLAB_INSTRUMENTED
#define SWEEPLAB_INSTRUMENTED 0
#endif
#ifdef BOX3D_DOUBLE_PRECISION
#error "float32 observation contract required"
#endif
_Static_assert(sizeof(float)==4 && sizeof(double)==8,"binary32/64 required");
enum { STAGES=15, CASES=864, CONTACT_CAP=4, HOOK_CAP=1024, HOOK_WINDOW=8 };
typedef struct { unsigned count; int first,last,minAux,maxAux; float minValue,maxValue; } Stage;
typedef struct { const char* series; int family,targetDynamic,bullet,ccd,di,sub,it,neighbor; } Spec;
typedef struct { b3WorldId world; b3BodyId bodies[2]; int id,frame,finite,configuration; double maxPen,manifoldPen,maxImpulse,maxEnergy,maxMomentum,maxAngular,rebound; int firstContact,firstImpulse,firstGeometry,maxGeometryFrame; uint64_t digest; } Scene;
static const char* profile="unspecified";
static int requestedSimd=4,workers=1,caseCount=0,qualityFails=0,executionFails=0,traceBad=0,hookBad=0;
static int activeFrame=0,activeCase=-1,activatedControls[4]={0};
static const void* activeWorld=NULL;
static Stage stage[STAGES];
static FILE* trace=NULL;
static uint64_t traceStates=0;
static const char* families[]={"fixed_geometry","phase_held"};
static uint32_t fb(float v) { uint32_t b;memcpy(&b,&v,4);return b; }
static uint64_t db(double v) { uint64_t b;memcpy(&b,&v,8);return b; }
static int finite32(float v) { return (fb(v)&0x7f800000u)!=0x7f800000u; }
static int finite64(double v) { return (db(v)&UINT64_C(0x7ff0000000000000))!=UINT64_C(0x7ff0000000000000); }
static double maximum(double a,double b) { return a>b?a:b; }
static double length(b3Vec3 v) { return sqrt((double)v.x*v.x+(double)v.y*v.y+(double)v.z*v.z); }
static void hash32(uint64_t* h,uint32_t b) { for(int j=0;j<4;++j) { *h^=(b>>(8*j))&255u;*h*=UINT64_C(1099511628211); } }
static const char* truth(int v) { return v?"true":"false"; }
static const char* backend(int width)
{
#if defined(BOX3D_DISABLE_SIMD)
    (void)width;return "scalar";
#elif defined(__aarch64__) || defined(_M_ARM64)
    (void)width;return "neon";
#else
    return width==8?"avx2":"sse2";
#endif
}
// Workers=1 is a deliberate scope restriction. These counters are NOT a
// concurrent observer. The engine joins its work before the driver reads them.
void sweeplabCcdObserve(int event,const void* world,int key,float value,int aux)
{
    (void)key;
    if(activeCase<0)return; // World setup/teardown is outside the measured case.
    if(world!=activeWorld || event<1 || event>STAGES || !finite32(value)) {hookBad=1;return;}
    Stage* s=stage+event-1;
    if(s->count==0) {s->first=activeFrame;s->minValue=s->maxValue=value;s->minAux=s->maxAux=aux;}
    if(s->count==1000000u) {hookBad=1;return;}
    ++s->count;s->last=activeFrame;
    if(value<s->minValue)s->minValue=value;if(value>s->maxValue)s->maxValue=value;
    if(aux<s->minAux)s->minAux=aux;if(aux>s->maxAux)s->maxAux=aux;
}
static int safe_label(const char* s)
{ if(!*s || strlen(s)>120)return 0;for(;*s;++s)if(!((*s>='a'&&*s<='z')||(*s>='A'&&*s<='Z')||(*s>='0'&&*s<='9')||strchr("_./:+-",*s)))return 0;return 1; }
static void prefix(const char* type)
{ printf("{\"schema\":\"box3d-sweeplab-v1\",\"driver\":\"ccd\",\"type\":\"%s\",\"profile\":\"%s\",\"instrumented\":%s",type,profile,truth(SWEEPLAB_INSTRUMENTED)); }
static void metric(const char* key,double v)
{ printf(",\"%s\":",key);if(finite64(v))printf("%.17g",v);else printf("null"); }
static void body_values(b3BodyId id,float values[13])
{
    b3WorldTransform t=b3Body_GetTransform(id);b3Vec3 v=b3Body_GetLinearVelocity(id),w=b3Body_GetAngularVelocity(id);
    float a[13]={t.p.x,t.p.y,t.p.z,t.q.v.x,t.q.v.y,t.q.v.z,t.q.s,v.x,v.y,v.z,w.x,w.y,w.z};memcpy(values,a,sizeof(a));
}
static void floats(Scene* s,const float* v,int n)
{
    for(int j=0;j<n;++j){if(!finite32(v[j]))s->finite=0;hash32(&s->digest,fb(v[j]));if(trace)fprintf(trace,"%s\"%08" PRIx32 "\"",j?",":"",fb(v[j]));}
}
static int contact_order(const void* av,const void* bv)
{const b3ContactData* a=av;const b3ContactData* b=bv;return (a->contactId.index1>b->contactId.index1)-(a->contactId.index1<b->contactId.index1);}
static void observe(Scene* s)
{
    int actual=b3GetWorldFromId(s->world)->simdWidth;
    if(b3World_GetWorkerCount(s->world)!=workers || actual!=(requestedSimd==8?8:4))s->configuration=0;
    hash32(&s->digest,(uint32_t)s->frame);
    if(trace)fprintf(trace,"{\"case_id\":%d,\"frame\":%d,\"bodies\":[",s->id,s->frame);
    for(int b=0;b<2;++b){float v[13];body_values(s->bodies[b],v);if(trace)fprintf(trace,"%s[",b?",":"");floats(s,v,13);if(trace)fputc(']',trace);}
    b3Pos a=b3Body_GetPosition(s->bodies[0]),b=b3Body_GetPosition(s->bodies[1]);
    double x=(double)b.x-a.x,y=(double)b.y-a.y,z=(double)b.z-a.z,d=sqrt(x*x+y*y+z*z),pen=1-d;
    if(!finite64(d)||!finite64(pen))s->finite=0;
    if(pen>0 && s->firstGeometry<0)s->firstGeometry=s->frame;
    if(pen>s->maxPen){s->maxPen=pen;s->maxGeometryFrame=s->frame;}
    if(trace)fprintf(trace,"],\"geometry_bits\":[\"%016" PRIx64 "\",\"%016" PRIx64 "\"],\"contacts\":[",db(d),db(pen));
    b3ContactData contacts[CONTACT_CAP];int count=b3Body_GetContactData(s->bodies[0],contacts,CONTACT_CAP);
    if(count<0 || count>=CONTACT_CAP){s->configuration=0;count=0;}
    qsort(contacts,(size_t)count,sizeof(contacts[0]),contact_order);hash32(&s->digest,(uint32_t)count);
    if(count && s->firstContact<0)s->firstContact=s->frame;
    for(int c=0;c<count;++c){b3ContactData* data=contacts+c;
        if(trace)fprintf(trace,"%s{\"id\":[%d,%u],\"shapes\":[%d,%u,%d,%u],\"manifolds\":[",c?",":"",data->contactId.index1,(unsigned)data->contactId.generation,data->shapeIdA.index1,(unsigned)data->shapeIdA.generation,data->shapeIdB.index1,(unsigned)data->shapeIdB.generation);
        hash32(&s->digest,(uint32_t)data->contactId.index1);hash32(&s->digest,(uint32_t)data->contactId.generation);hash32(&s->digest,(uint32_t)data->manifoldCount);
        if(data->manifoldCount<0 || data->manifoldCount>4){s->configuration=0;continue;}
        for(int m=0;m<data->manifoldCount;++m){const b3Manifold* mm=data->manifolds+m;
            float v[10]={mm->normal.x,mm->normal.y,mm->normal.z,mm->twistImpulse,mm->frictionImpulse.x,mm->frictionImpulse.y,mm->frictionImpulse.z,mm->rollingImpulse.x,mm->rollingImpulse.y,mm->rollingImpulse.z};
            if(trace)fprintf(trace,"%s{\"bits\":[",m?",":"");floats(s,v,10);if(trace)fprintf(trace,"],\"points\":[");
            if(mm->pointCount<0 || mm->pointCount>B3_MAX_MANIFOLD_POINTS){s->configuration=0;continue;}
            hash32(&s->digest,(uint32_t)mm->pointCount);
            for(int k=0;k<mm->pointCount;++k){const b3ManifoldPoint* pp=mm->points+k;
                float v2[11]={pp->anchorA.x,pp->anchorA.y,pp->anchorA.z,pp->anchorB.x,pp->anchorB.y,pp->anchorB.z,pp->separation,pp->normalImpulse,pp->totalNormalImpulse,pp->normalVelocity,pp->baseSeparation};
                s->manifoldPen=maximum(s->manifoldPen,-(double)pp->separation);s->maxImpulse=maximum(s->maxImpulse,pp->totalNormalImpulse);
                if(pp->totalNormalImpulse>0 && s->firstImpulse<0)s->firstImpulse=s->frame;
                if(trace)fprintf(trace,"%s{\"bits\":[",k?",":"");floats(s,v2,11);
                hash32(&s->digest,pp->featureId);hash32(&s->digest,(uint32_t)pp->triangleIndex);hash32(&s->digest,(uint32_t)pp->persisted);
                if(trace)fprintf(trace,"],\"feature\":%u,\"triangle\":%d,\"persisted\":%s}",pp->featureId,pp->triangleIndex,truth(pp->persisted));
            }
            if(trace)fprintf(trace,"]}");
        }
        if(trace)fprintf(trace,"]}");
    }
    if(trace){fprintf(trace,"]}\n");if(ferror(trace))traceBad=1;}
    ++traceStates;
}
static b3BodyId sphere(Scene* s,int index,float x,float vx,int dynamic,int bullet)
{
    b3BodyDef bd=b3DefaultBodyDef();bd.type=dynamic?b3_dynamicBody:b3_staticBody;bd.position=(b3Pos){x,0,0};bd.linearVelocity=(b3Vec3){vx,0,0};
    bd.linearDamping=0;bd.angularDamping=0;bd.enableSleep=false;bd.isBullet=bullet!=0;
    b3BodyId body=b3CreateBody(s->world,&bd);s->bodies[index]=body;return body;
}
static void shape(b3BodyId body,float restitution)
{
    b3ShapeDef sd=b3DefaultShapeDef();sd.density=1;sd.baseMaterial.friction=0;sd.baseMaterial.restitution=restitution;
    b3Sphere sphereDef={{0,0,0},0.5f};b3CreateSphereShape(body,&sd,&sphereDef);
}

typedef struct { int frame,stage,key,aux,ordinal,count; uint32_t bits[14]; } Hook;
static Hook hooks[HOOK_CAP];
static int hookCount=0,hookOverflow=0,excludedHooks=0,integrationSelected=0;
static int pendingShape=-1,pendingShapeOrdinal=0,shapeOrdinal[2],integrationOrdinal[2],integrationPending[2];
static int predicateGate=1,integrationGate=1,matchedGate=1,activeCcd=0,bodyOrdinal[2];
static uint32_t previousInitial[28];static int previousPair=-1;
static uint64_t totalHooks=0,totalExcluded=0;
static void keep_hook(int event,int key,int aux,int ordinal,const float* values,int count)
{
    if(activeFrame<1 || activeFrame>HOOK_WINDOW){++excludedHooks;return;}
    if(count<0 || count>14 || hookCount>=HOOK_CAP){hookOverflow=1;hookBad=1;return;}
    Hook* h=hooks+hookCount++;*h=(Hook){activeFrame,event,key,aux,ordinal,count,{0}};
    for(int j=0;j<count;++j){h->bits[j]=fb(values[j]);if(!finite32(values[j]))hookBad=1;}
}
void sweeplabCcdPredicate(int event,int shapeId,int result,const float* values,int count)
{
    if(activeCase<0)return;
    if(shapeId<0 || shapeId>1){predicateGate=0;hookBad=1;return;}
    if(event==10){
        if(pendingShape!=-1 || count!=4 || result!=-1){predicateGate=0;hookBad=1;return;}
        pendingShape=shapeId;pendingShapeOrdinal=++shapeOrdinal[shapeId];
        sweeplabCcdObserve(event,activeWorld,shapeId,values[0],-1);
        keep_hook(event,shapeId,-1,pendingShapeOrdinal,values,count);
    } else if(event==11){
        if(pendingShape!=shapeId || count!=0 || (result!=0 && result!=1)){predicateGate=0;hookBad=1;return;}
        sweeplabCcdObserve(event,activeWorld,shapeId,0.0f,result);
        keep_hook(event,shapeId,result,pendingShapeOrdinal,NULL,0);pendingShape=-1;
    } else if(event==12){
        if(pendingShape!=-1 || count!=0 || result!=-1){predicateGate=0;hookBad=1;return;}
        sweeplabCcdObserve(event,activeWorld,shapeId,0.0f,-1);
        keep_hook(event,shapeId,-1,++shapeOrdinal[shapeId],NULL,0);
    } else {predicateGate=0;hookBad=1;}
}
void sweeplabCcdIntegration(int event,const void* world,int bodyId,const float* values,int count)
{
    if(activeCase<0 || !integrationSelected)return;
    if(world!=activeWorld || bodyId<0 || bodyId>1 || count!=14 || (event!=13 && event!=14)){integrationGate=0;hookBad=1;return;}
    if(event==13){
        if(integrationPending[bodyId]){integrationGate=0;hookBad=1;}
        integrationPending[bodyId]=1;++integrationOrdinal[bodyId];
    } else {
        if(!integrationPending[bodyId]){integrationGate=0;hookBad=1;}
        integrationPending[bodyId]=0;
    }
    if(activeFrame<=HOOK_WINDOW)sweeplabCcdObserve(event,world,bodyId,values[0],event==13?0:1);
    keep_hook(event,bodyId,event==13?0:1,integrationOrdinal[bodyId],values,count);
}
void sweeplabCcdBodyDecision(const void* world,int bodyId,int actualFast,const float* values,int count)
{
    if(activeCase<0)return;
    if(world!=activeWorld || bodyId<0 || bodyId>1 || count!=7 || (actualFast!=0 && actualFast!=1)){hookBad=1;return;}
    if(!activeCcd && actualFast)hookBad=1;
    sweeplabCcdObserve(15,world,bodyId,values[0],actualFast);
    keep_hook(15,bodyId,actualFast,++bodyOrdinal[bodyId],values,count);
}
static void print_hooks(int caseId)
{
    int parts=(hookCount+31)/32;
    for(int part=0;part<parts;++part){int offset=part*32,count=hookCount-offset;if(count>32)count=32;
        prefix("hooks");printf(",\"case_id\":%d,\"part\":%d,\"parts\":%d,\"offset\":%d,\"count\":%d,\"total_count\":%d,\"hooks\":[",caseId,part,parts,offset,count,hookCount);
        for(int i=offset;i<offset+count;++i){Hook* h=hooks+i;
            printf("%s{\"frame\":%d,\"stage\":%d,\"key\":%d,\"aux\":%d,\"ordinal\":%d,\"bits\":[",i>offset?",":"",h->frame,h->stage,h->key,h->aux,h->ordinal);
            for(int k=0;k<h->count;++k)printf("%s\"%08" PRIx32 "\"",k?",":"",h->bits[k]);fputs("]}",stdout);
        }
        puts("]}");
    }
}
static void run_case(Spec spec)
{
    const float dts[]={1.0f/30,1.0f/60,1.0f/120};float dt=dts[spec.di],restitution=0.5f;
    Scene s={0};s.id=caseCount++;s.finite=1;s.configuration=1;s.firstContact=s.firstImpulse=s.firstGeometry=s.maxGeometryFrame=-1;s.digest=UINT64_C(14695981039346656037);
    memset(stage,0,sizeof(stage));hookBad=0;activeCase=-1;activeWorld=NULL;activeFrame=0;
    hookCount=hookOverflow=excludedHooks=0;predicateGate=integrationGate=matchedGate=1;pendingShape=-1;
    integrationSelected=!strcmp(spec.series,"core") && spec.it==2;activeCcd=spec.ccd;
    b3WorldDef wd=b3DefaultWorldDef();wd.gravity=(b3Vec3){0,0,0};wd.workerCount=(uint32_t)workers;wd.enableSleep=false;wd.enableContinuous=spec.ccd!=0;wd.restitutionIterations=spec.it;
    s.world=b3CreateWorld(&wd);activeWorld=b3GetWorldFromId(s.world);activeCase=s.id;
    float va=60.0f,vb=0.0f,relative=60.0f;
    float basis=spec.family==0?-3.500000238418579f:-0.5f-(60.0f*1.5f*dt);
    float below=nextafterf(basis,-INFINITY),above=nextafterf(basis,INFINITY);
    float xa=spec.neighbor<0?below:spec.neighbor>0?above:basis,xb=0.5f;
    int coordinateGate=finite32(basis)&&finite32(below)&&finite32(above)&&below<basis&&above>basis&&fb(below)!=fb(basis)&&fb(above)!=fb(basis)&&fb(below)!=fb(above);
    b3BodyId a=sphere(&s,0,xa,va,1,spec.bullet),b=sphere(&s,1,xb,vb,spec.targetDynamic,0);shape(a,restitution);shape(b,restitution);
    if(fb(b3Body_GetPosition(a).x)!=fb(xa))coordinateGate=0;
    if(b3Body_IsBullet(a)!=(spec.bullet!=0) || b3Body_IsBullet(b) || b3World_IsContinuousEnabled(s.world)!=(spec.ccd!=0))s.configuration=0;
    double ma=b3Body_GetMass(a),mb=b3Body_GetMass(b),energy0=0.5*ma*va*va+0.5*mb*vb*vb,p0=ma*va+mb*vb,scale=ma*fabs(va)+mb*fabs(vb);
    uint32_t initial[28];
    for(int body=0;body<2;++body){float vals[13];body_values(s.bodies[body],vals);for(int j=0;j<13;++j)initial[13*body+j]=fb(vals[j]);}
    initial[26]=fb((float)ma);initial[27]=fb((float)mb);
    if(spec.bullet){if(previousPair!=s.id/2 || memcmp(previousInitial,initial,sizeof(initial)))matchedGate=0;}
    else{previousPair=s.id/2;memcpy(previousInitial,initial,sizeof(initial));}
    double effectiveGap=(double)xb-xa-1,effectivePhase=effectiveGap/relative/dt-1.0;
    int frames=(int)lround(0.75/(double)dt);observe(&s);
    for(int f=1;f<=frames;++f){
        s.frame=f;activeFrame=f;memset(bodyOrdinal,0,sizeof(bodyOrdinal));memset(shapeOrdinal,0,sizeof(shapeOrdinal));memset(integrationOrdinal,0,sizeof(integrationOrdinal));memset(integrationPending,0,sizeof(integrationPending));
        b3World_Step(s.world,dt,spec.sub);
        if(pendingShape!=-1)predicateGate=0;
        if(SWEEPLAB_INSTRUMENTED && (bodyOrdinal[0]!=1 || bodyOrdinal[1]!=(spec.targetDynamic?1:0)))hookBad=1;
        if(SWEEPLAB_INSTRUMENTED && integrationSelected){
            if(integrationPending[0] || integrationOrdinal[0]!=spec.sub)integrationGate=0;
            if(spec.targetDynamic){if(integrationPending[1] || integrationOrdinal[1]!=spec.sub)integrationGate=0;}
            else if(integrationPending[1] || integrationOrdinal[1])integrationGate=0;
        }
        observe(&s);
        b3Vec3 av=b3Body_GetLinearVelocity(a),bv=b3Body_GetLinearVelocity(b),aw=b3Body_GetAngularVelocity(a),bw=b3Body_GetAngularVelocity(b);
        double energy=0.5*ma*length(av)*length(av)+0.5*mb*length(bv)*length(bv);s.maxEnergy=maximum(s.maxEnergy,energy/energy0);
        double px=ma*av.x+mb*bv.x-p0,py=ma*av.y+mb*bv.y,pz=ma*av.z+mb*bv.z;
        if(spec.targetDynamic)s.maxMomentum=maximum(s.maxMomentum,sqrt(px*px+py*py+pz*pz)/scale);
        s.maxAngular=maximum(s.maxAngular,maximum(length(aw),length(bw)));
        if(s.maxImpulse>0)s.rebound=maximum(s.rebound,((double)bv.x-av.x)/relative);
    }
    if(SWEEPLAB_INSTRUMENTED){
        if(stage[5].count>stage[4].count || stage[4].count>stage[3].count || stage[3].count>stage[2].count)hookBad=1;
        if(stage[9].count!=stage[10].count)predicateGate=0;
        if(stage[14].count!=(unsigned)(frames*(spec.targetDynamic?2:1)))hookBad=1;
        if(!spec.ccd)for(int j=0;j<12;++j)if(j<6 || j>=9)if(stage[j].count)hookBad=1;
        if(integrationSelected){unsigned expected=(unsigned)(HOOK_WINDOW*spec.sub*(spec.targetDynamic?2:1));if(stage[12].count!=expected || stage[13].count!=expected)integrationGate=0;}
    }else for(int j=0;j<STAGES;++j)if(stage[j].count)hookBad=1;
    int actual=b3GetWorldFromId(s.world)->simdWidth;
    int touch=s.firstContact>=0,impulse=s.maxImpulse>0;
    int penOK=s.maxPen<=0.02,restOK=fabs(s.rebound-restitution)<=0.05,momentumOK=!spec.targetDynamic || s.maxMomentum<=0.0001,energyOK=s.maxEnergy<=1.01,angularOK=s.maxAngular<=1e-6;
    int quality=touch&&impulse&&penOK&&restOK&&momentumOK&&energyOK&&angularOK&&s.finite;
    int execution=s.finite&&s.configuration&&!traceBad&&!hookBad&&!hookOverflow&&predicateGate&&integrationGate&&coordinateGate&&matchedGate;
    if(!quality)++qualityFails;if(!execution)++executionFails;
    int stratum=2*spec.targetDynamic+spec.bullet;
    if(SWEEPLAB_INSTRUMENTED && spec.ccd && stage[5].count>0)activatedControls[stratum]=1;

    prefix("result");printf(",\"case_id\":%d,\"bullet_pair_id\":%d,\"series\":\"%s\",\"family\":\"%s\",\"target_dynamic\":%s,\"bullet_enabled\":%s,\"ccd_enabled\":%s,\"dt_bits\":\"%08" PRIx32 "\",\"substeps\":%d,\"restitution_iterations\":%d,\"restitution_bits\":\"%08" PRIx32 "\",\"coordinate_neighbor\":%d,\"basis_xa_bits\":\"%08" PRIx32 "\",\"initial_xa_bits\":\"%08" PRIx32 "\",\"initial_gap64_bits\":\"%016" PRIx64 "\",\"effective_phase_bits\":\"%016" PRIx64 "\",\"frames\":%d,\"workers_requested\":%d,\"actual_workers\":%d,\"simd_actual\":%d,\"backend_actual\":\"%s\",\"status\":\"%s\",\"execution_gates\":{\"finite\":%s,\"configuration\":%s,\"trace_io\":%s,\"hook_integrity\":%s,\"predicate_pairing\":%s,\"integration_complete\":%s,\"coordinate_neighbors_distinct\":%s,\"matched_initial_state\":%s},\"quality_gates\":{\"touch\":%s,\"impulse\":%s,\"penetration\":%s,\"restitution\":%s,\"momentum\":%s,\"energy\":%s,\"angular\":%s},\"momentum_applicable\":%s,\"first_public_contact_step\":%d,\"first_normal_impulse_step\":%d,\"first_geometry_overlap_step\":%d,\"max_geometry_overlap_step\":%d",
        s.id,s.id/2,spec.series,families[spec.family],truth(spec.targetDynamic),truth(spec.bullet),truth(spec.ccd),fb(dt),spec.sub,spec.it,fb(restitution),spec.neighbor,fb(basis),fb(xa),db(effectiveGap),db(effectivePhase),frames,workers,b3World_GetWorkerCount(s.world),actual,backend(actual),quality?"PASS":"FAIL",truth(s.finite),truth(s.configuration),truth(!traceBad),truth(!hookBad&&!hookOverflow),truth(predicateGate),truth(integrationGate),truth(coordinateGate),truth(matchedGate),truth(touch),truth(impulse),truth(penOK),truth(restOK),truth(momentumOK),truth(energyOK),truth(angularOK),truth(spec.targetDynamic),s.firstContact,s.firstImpulse,s.firstGeometry,s.maxGeometryFrame);
    metric("max_sampled_geometry_penetration_m",s.maxPen);metric("max_reported_manifold_penetration_m",s.manifoldPen);metric("max_normal_impulse",s.maxImpulse);metric("observed_restitution",s.rebound);metric("max_relative_momentum_error",s.maxMomentum);metric("max_energy_ratio",s.maxEnergy);metric("max_angular_speed_radps",s.maxAngular);
    printf(",\"projection_digest\":\"%016" PRIx64 "\",\"activation\":{\"status\":\"%s\",\"expected_control\":%s,\"stages\":[",s.digest,!SWEEPLAB_INSTRUMENTED?"unobserved":stage[5].count?"active":"inactive",truth(spec.ccd&&(!spec.targetDynamic||spec.bullet)));
    for(int j=0;j<STAGES;++j){Stage* st=stage+j;printf("%s{\"stage\":%d,\"count\":%u,\"first_step\":%d,\"last_step\":%d,\"min_value_bits\":\"%08" PRIx32 "\",\"max_value_bits\":\"%08" PRIx32 "\",\"min_aux\":%d,\"max_aux\":%d}",j?",":"",j+1,st->count,st->count?st->first:-1,st->count?st->last:-1,fb(st->minValue),fb(st->maxValue),st->minAux,st->maxAux);}
    printf("]},\"integration_selected\":%s,\"hook_window_first\":1,\"hook_window_last\":8,\"hooks_count\":%d,\"hook_parts\":%d,\"hooks_outside_window_excluded\":%d,\"hook_overflow\":%s}\n",truth(integrationSelected),hookCount,(hookCount+31)/32,excludedHooks,truth(hookOverflow));
    print_hooks(s.id);totalHooks+=(uint64_t)hookCount;totalExcluded+=(uint64_t)excludedHooks;
    activeCase=-1;activeWorld=NULL;b3DestroyWorld(s.world);
}
int main(int argc,char** argv)
{
    const char* tracePath=NULL;
    for(int i=1;i<argc;++i){if(!strcmp(argv[i],"--profile")&&i+1<argc)profile=argv[++i];else if(!strcmp(argv[i],"--workers")&&i+1<argc){const char* w=argv[++i];if(strcmp(w,"1"))return 64;workers=1;}else if(!strcmp(argv[i],"--simd")&&i+1<argc){const char* v=argv[++i];if(strcmp(v,"0")&&strcmp(v,"4")&&strcmp(v,"8"))return 64;requestedSimd=atoi(v);}else if(!strcmp(argv[i],"--trace")&&i+1<argc)tracePath=argv[++i];else{fputs("usage: sweeplab-ccd --profile LABEL --workers 1 --simd 0|4|8 --trace FILE\n",stderr);return 64;}}
    if(!safe_label(profile))return 64;
#if defined(BOX3D_DISABLE_SIMD)
    if(requestedSimd!=0)return 64;
#else
    if(requestedSimd==0)return 64;
#endif
    if(tracePath){trace=fopen(tracePath,"wb");if(!trace){perror("trace");return 5;}}
    prefix("criteria");printf(",\"results_expected\":864,\"workers_scope\":1,\"penetration_limit_m\":0.02,\"absolute_restitution_error_limit\":0.05,\"relative_momentum_error_limit\":0.0001,\"relative_energy_gain_limit\":0.01,\"angular_speed_limit_radps\":0.000001,\"observed_penetration_scope\":\"external_step_endpoints_not_continuous_maximum\",\"trace_scope\":\"exact_public_body_geometry_contacts_no_hooks_or_world_provenance\",\"projection_digest_scope\":\"FNV_body_and_selected_contact_fields_not_complete_JSONL_trace_use_trace_SHA_for_parity\",\"predicate_scope\":\"existing_operands_before_original_return_and_actual_caller_boolean_threshold_product_is_diagnostic_repeat\",\"integration_scope\":\"actual_local_accumulated_deltas_and_limited_velocities_before_after_each_position_integration_not_worldposes\",\"ordinal_scope\":\"derived_by_counting_observed_source_calls_per_body_per_external_step\",\"integration_selected_cases\":144,\"hook_window_first\":1,\"hook_window_last\":8,\"hook_cap_per_case\":1024,\"hook_chunk_events\":32,\"detailed_hook_upper_bound\":39168,\"trace_states_expected\":46080,\"ulp_scope\":\"nextafterf_actual_xa_positions_3distinct_binaries_core_and_ulp_centers_repeat_controls\",\"upstream\":\"5d83df83ab47172c2c745b44315e7538ead02397\"}\n");
    const int subs[]={1,4,8},iters[]={1,2,4};
    for(int family=0;family<2;++family)for(int target=0;target<2;++target)for(int ccd=0;ccd<2;++ccd)for(int d=0;d<3;++d)for(int sub=0;sub<3;++sub)for(int it=0;it<3;++it)for(int bullet=0;bullet<2;++bullet)
        run_case((Spec){"core",family,target,bullet,ccd,d,subs[sub],iters[it],0});
    for(int family=0;family<2;++family)for(int target=0;target<2;++target)for(int ccd=0;ccd<2;++ccd)for(int d=0;d<3;++d)for(int sub=0;sub<3;++sub)for(int neighbor=-1;neighbor<=1;++neighbor)for(int bullet=0;bullet<2;++bullet)
        run_case((Spec){"ulp",family,target,bullet,ccd,d,subs[sub],2,neighbor});
    if(trace&&fclose(trace))traceBad=1;trace=NULL;
    int activation=!SWEEPLAB_INSTRUMENTED || (activatedControls[0]&&activatedControls[1]&&activatedControls[3]);
    int execution=caseCount==CASES&&!executionFails&&!traceBad&&activation&&traceStates==46080&&totalHooks<=39168;
    prefix("summary");printf(",\"status\":\"%s\",\"results\":%d,\"quality_passes\":%d,\"quality_failures\":%d,\"execution_pass\":%s,\"execution_failures\":%d,\"activation_controls_pass\":%s,\"static_nonbullet_applied_toi\":%s,\"static_bullet_applied_toi\":%s,\"dynamic_nonbullet_applied_toi\":%s,\"dynamic_bullet_applied_toi\":%s,\"trace_io_ok\":%s,\"trace_states\":%" PRIu64 ",\"hooks_retained\":%" PRIu64 ",\"hooks_outside_window_excluded\":%" PRIu64 "}\n",qualityFails?"FAIL":"PASS",caseCount,caseCount-qualityFails,qualityFails,truth(execution),executionFails,truth(activation),truth(activatedControls[0]),truth(activatedControls[1]),truth(activatedControls[2]),truth(activatedControls[3]),truth(!traceBad),traceStates,totalHooks,totalExcluded);
    if(!execution)return 5;return qualityFails?1:0;
}
