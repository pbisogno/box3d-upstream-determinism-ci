// SPDX-License-Identifier: MIT
// Synthetic isolated contacts. Pinned upstream remains untouched; optional
// hooks copy diagnostics after existing calculations, never solver inputs.
#include "box3d/box3d.h"
#include "box3d/collision.h"
#include "physics_world.h"
#include "physics-hook.h"
#include <inttypes.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#ifndef CONTACTLAB_INSTRUMENTED
#define CONTACTLAB_INSTRUMENTED 0
#endif
#ifdef BOX3D_DOUBLE_PRECISION
#error "float32 observation contract required"
#endif
_Static_assert(sizeof(float)==4 && sizeof(double)==8,"binary32/64 required");
enum { STAGES=9, CASES=408, CONTACT_CAP=4 };
typedef struct { unsigned count; int first,last,minAux,maxAux; float minValue,maxValue; } Stage;
typedef struct { const char* series; int scene,ccd,di,sub,it,phaseIndex,original; float phase,restitution; } Spec;
typedef struct { b3WorldId world; b3BodyId bodies[2]; int id,frame,finite,configuration; double maxPen,manifoldPen,maxImpulse,maxEnergy,maxMomentum,maxAngular,rebound; int firstContact,firstImpulse; uint64_t digest; } Scene;
static const char* profile="unspecified";
static int requestedSimd=4,workers=1,caseCount=0,qualityFails=0,executionFails=0,traceBad=0,hookBad=0;
static int activeFrame=0,activeCase=-1,activatedControls[3]={0};
static const void* activeWorld=NULL;
static Stage stage[STAGES];
static FILE* trace=NULL;
static uint64_t traceStates=0;
static const char* names[]={"original_dynamic","fast_static","bullet_dynamic"};
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
void contactlabPhysicsObserve(int event,const void* world,int key,float value,int aux)
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
{ printf("{\"schema\":\"box3d-contactlab-v1\",\"driver\":\"physics\",\"type\":\"%s\",\"profile\":\"%s\",\"instrumented\":%s",type,profile,truth(CONTACTLAB_INSTRUMENTED)); }
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
    s->maxPen=maximum(s->maxPen,pen);
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
static void run_case(Spec spec)
{
    const float dts[]={1.0f/30,1.0f/60,1.0f/120};float dt=dts[spec.di];
    Scene s={0};s.id=caseCount++;s.finite=1;s.configuration=1;s.firstContact=s.firstImpulse=-1;s.digest=UINT64_C(14695981039346656037);
    memset(stage,0,sizeof(stage));hookBad=0;activeCase=-1;activeWorld=NULL;activeFrame=0;
    b3WorldDef wd=b3DefaultWorldDef();wd.gravity=(b3Vec3){0,0,0};wd.workerCount=(uint32_t)workers;wd.enableSleep=false;wd.enableContinuous=spec.ccd!=0;wd.restitutionIterations=spec.it;
    s.world=b3CreateWorld(&wd);activeWorld=b3GetWorldFromId(s.world);activeCase=s.id;
    float va=spec.scene==0?2.0f:60.0f,vb=spec.scene==0?-2.0f:0.0f,relative=va-vb;
    float gap=spec.original?0.5f:relative*((spec.scene==0?4.0f:1.0f)+spec.phase)*dt;
    float xa=spec.original?-0.75f:-0.5f-gap,xb=spec.original?0.75f:0.5f;
    b3BodyId a=sphere(&s,0,xa,va,1,spec.scene==2),b=sphere(&s,1,xb,vb,spec.scene!=1,0);shape(a,spec.restitution);shape(b,spec.restitution);
    double ma=b3Body_GetMass(a),mb=b3Body_GetMass(b),energy0=0.5*ma*va*va+0.5*mb*vb*vb,p0=ma*va+mb*vb,scale=ma*fabs(va)+mb*fabs(vb);
    double effectiveGap=(double)xb-xa-1,phaseEffective=effectiveGap/relative/dt-(spec.scene==0?4.0:1.0);
    int frames=(int)lround(0.75/(double)dt);observe(&s);
    for(int f=1;f<=frames;++f){s.frame=f;activeFrame=f;b3World_Step(s.world,dt,spec.sub);observe(&s);
        b3Vec3 av=b3Body_GetLinearVelocity(a),bv=b3Body_GetLinearVelocity(b),aw=b3Body_GetAngularVelocity(a),bw=b3Body_GetAngularVelocity(b);
        double energy=0.5*ma*length(av)*length(av)+0.5*mb*length(bv)*length(bv);s.maxEnergy=maximum(s.maxEnergy,energy/energy0);
        double px=ma*av.x+mb*bv.x-p0,py=ma*av.y+mb*bv.y,pz=ma*av.z+mb*bv.z;
        if(spec.scene!=1)s.maxMomentum=maximum(s.maxMomentum,sqrt(px*px+py*py+pz*pz)/scale);
        s.maxAngular=maximum(s.maxAngular,maximum(length(aw),length(bw)));
        if(s.maxImpulse>0)s.rebound=maximum(s.rebound,((double)bv.x-av.x)/relative);
    }
    if(CONTACTLAB_INSTRUMENTED) {
        if(stage[5].count>stage[4].count || stage[4].count>stage[3].count || stage[3].count>stage[2].count)hookBad=1;
        if(stage[5].count && (!stage[0].count || (spec.scene==2 && !stage[1].count)))hookBad=1;
        if(!spec.ccd)for(int j=0;j<6;++j)if(stage[j].count)hookBad=1;
    } else for(int j=0;j<STAGES;++j)if(stage[j].count)hookBad=1;
    int actual=b3GetWorldFromId(s.world)->simdWidth;
    int touch=s.firstContact>=0,impulse=s.maxImpulse>0;
    int penOK=s.maxPen<=0.02,restOK=fabs(s.rebound-spec.restitution)<=0.05,momentumOK=spec.scene==1 || s.maxMomentum<=0.0001,energyOK=s.maxEnergy<=1.01,angularOK=s.maxAngular<=1e-6;
    int quality=touch&&impulse&&penOK&&restOK&&momentumOK&&energyOK&&angularOK&&s.finite;
    int execution=s.finite&&s.configuration&&!traceBad&&!hookBad;
    if(!quality)++qualityFails;if(!execution)++executionFails;
    if(CONTACTLAB_INSTRUMENTED && spec.ccd && stage[5].count>0)activatedControls[spec.scene]=1;
    prefix("result");printf(",\"case_id\":%d,\"series\":\"%s\",\"scene\":\"%s\",\"ccd_enabled\":%s,\"dt_bits\":\"%08" PRIx32 "\",\"substeps\":%d,\"restitution_iterations\":%d,\"restitution_bits\":\"%08" PRIx32 "\",\"phase_index\":%d,\"phase_bits\":\"%08" PRIx32 "\",\"initial_gap_bits\":\"%08" PRIx32 "\",\"effective_phase_bits\":\"%016" PRIx64 "\",\"frames\":%d,\"workers_requested\":%d,\"actual_workers\":%d,\"simd_actual\":%d,\"backend_actual\":\"%s\",\"status\":\"%s\",\"execution_gates\":{\"finite\":%s,\"configuration\":%s,\"trace_io\":%s,\"hook_integrity\":%s},\"quality_gates\":{\"touch\":%s,\"impulse\":%s,\"penetration\":%s,\"restitution\":%s,\"momentum\":%s,\"energy\":%s,\"angular\":%s},\"momentum_applicable\":%s,\"first_public_contact_step\":%d,\"first_normal_impulse_step\":%d",
        s.id,spec.series,names[spec.scene],truth(spec.ccd),fb(dt),spec.sub,spec.it,fb(spec.restitution),spec.phaseIndex,fb(spec.phase),fb(gap),db(phaseEffective),frames,workers,b3World_GetWorkerCount(s.world),actual,backend(actual),quality?"PASS":"FAIL",truth(s.finite),truth(s.configuration),truth(!traceBad),truth(!hookBad),truth(touch),truth(impulse),truth(penOK),truth(restOK),truth(momentumOK),truth(energyOK),truth(angularOK),truth(spec.scene!=1),s.firstContact,s.firstImpulse);
    metric("max_sampled_geometry_penetration_m",s.maxPen);metric("max_reported_manifold_penetration_m",s.manifoldPen);metric("max_normal_impulse",s.maxImpulse);metric("observed_restitution",s.rebound);metric("max_relative_momentum_error",s.maxMomentum);metric("max_energy_ratio",s.maxEnergy);metric("max_angular_speed_radps",s.maxAngular);
    printf(",\"projection_digest\":\"%016" PRIx64 "\",\"activation\":{\"status\":\"%s\",\"expected_control\":%s,\"stages\":[",s.digest,!CONTACTLAB_INSTRUMENTED?"unobserved":stage[5].count?"active":"inactive",truth(spec.scene!=0&&spec.ccd));
    for(int j=0;j<STAGES;++j){Stage* st=stage+j;printf("%s{\"stage\":%d,\"count\":%u,\"first_step\":%d,\"last_step\":%d,\"min_value_bits\":\"%08" PRIx32 "\",\"max_value_bits\":\"%08" PRIx32 "\",\"min_aux\":%d,\"max_aux\":%d}",j?",":"",j+1,st->count,st->count?st->first:-1,st->count?st->last:-1,fb(st->minValue),fb(st->maxValue),st->minAux,st->maxAux);}
    printf("]}}\n");activeCase=-1;activeWorld=NULL;b3DestroyWorld(s.world);
}
int main(int argc,char** argv)
{
    const char* tracePath=NULL;
    for(int i=1;i<argc;++i){if(!strcmp(argv[i],"--profile")&&i+1<argc)profile=argv[++i];else if(!strcmp(argv[i],"--workers")&&i+1<argc){const char* w=argv[++i];if(strcmp(w,"1"))return 64;workers=1;}else if(!strcmp(argv[i],"--simd")&&i+1<argc){const char* v=argv[++i];if(strcmp(v,"0")&&strcmp(v,"4")&&strcmp(v,"8"))return 64;requestedSimd=atoi(v);}else if(!strcmp(argv[i],"--trace")&&i+1<argc)tracePath=argv[++i];else {fputs("usage: contactlab-physics --profile LABEL --workers 1 --simd 0|4|8 --trace FILE\n",stderr);return 64;}}
    if(!safe_label(profile))return 64;
#if defined(BOX3D_DISABLE_SIMD)
    if(requestedSimd!=0)return 64;
#else
    if(requestedSimd==0)return 64;
#endif
    if(tracePath){trace=fopen(tracePath,"wb");if(!trace){perror("trace");return 5;}}
    prefix("criteria");printf(",\"results_expected\":408,\"workers_scope\":1,\"penetration_limit_m\":0.02,\"absolute_restitution_error_limit\":0.05,\"relative_momentum_error_limit\":0.0001,\"relative_energy_gain_limit\":0.01,\"angular_speed_limit_radps\":0.000001,\"observed_penetration_scope\":\"external_step_endpoints_not_continuous_maximum\",\"trace_scope\":\"exact_public_body_geometry_contacts_no_hooks_or_world_provenance\",\"projection_digest_scope\":\"FNV_body_and_selected_contact_fields_not_the_complete_JSONL_trace_use_trace_SHA_for_parity\",\"phase_scope\":\"requested_float_neighbors_may_collapse_to_same_initial_gap\",\"continuous_activation_scope\":\"actual_hook_stages_not_ccd_flag_alone\",\"upstream\":\"5d83df83ab47172c2c745b44315e7538ead02397\"}\n");
    const int subs[]={1,4,8},iters[]={1,2,4};const float restitutions[]={0,0.5f,1};
    for(int ccd=0;ccd<2;++ccd)for(int d=0;d<3;++d)for(int sub=0;sub<3;++sub)for(int it=0;it<3;++it)for(int e=0;e<3;++e)run_case((Spec){"original",0,ccd,d,subs[sub],iters[it],-1,1,0,restitutions[e]});
    float phases[14]={0,nextafterf(0,1),nextafterf(0.25f,0),0.25f,nextafterf(0.25f,1),nextafterf(0.5f,0),0.5f,nextafterf(0.5f,1),nextafterf(0.75f,0),0.75f,nextafterf(0.75f,1),0,0,1};phases[12]=nextafterf(1,0);phases[11]=nextafterf(phases[12],0);
    for(int scene=0;scene<3;++scene)for(int ccd=0;ccd<2;++ccd)for(int ph=0;ph<14;++ph)run_case((Spec){"phase",scene,ccd,1,4,2,ph,0,phases[ph],0.5f});
    for(int scene=0;scene<3;++scene)for(int ccd=0;ccd<2;++ccd)for(int d=0;d<3;++d)for(int sub=0;sub<3;++sub)for(int it=0;it<3;++it)run_case((Spec){"factorial",scene,ccd,d,subs[sub],iters[it],6,0,0.5f,0.5f});
    if(trace && fclose(trace))traceBad=1;trace=NULL;
    int activation=!CONTACTLAB_INSTRUMENTED || (activatedControls[1]&&activatedControls[2]);
    int execution=caseCount==CASES && !executionFails && !traceBad && activation;
    prefix("summary");printf(",\"status\":\"%s\",\"results\":%d,\"quality_passes\":%d,\"quality_failures\":%d,\"execution_pass\":%s,\"execution_failures\":%d,\"activation_controls_pass\":%s,\"fast_static_applied_toi\":%s,\"bullet_dynamic_applied_toi\":%s,\"trace_io_ok\":%s,\"trace_states\":%" PRIu64 "}\n",qualityFails?"FAIL":"PASS",caseCount,caseCount-qualityFails,qualityFails,truth(execution),executionFails,truth(activation),truth(activatedControls[1]),truth(activatedControls[2]),truth(!traceBad),traceStates);
    if(!execution)return 5;return qualityFails?1:0;
}
