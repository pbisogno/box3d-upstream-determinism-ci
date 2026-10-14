// SPDX-License-Identifier: MIT
// Synthetic public-API quality checks. Criteria are declared before any stepping.
// Units: meters, kilograms, seconds; default length scale (one unit per meter).
// No physical oracle is inferred from matching determinism hashes.
#if !defined(_WIN32)
#define _POSIX_C_SOURCE 200809L
#endif
#include "box3d/box3d.h"
#include "box3d/collision.h"
#include "box3d/constants.h"
#include "physics_world.h" // Read-only actual engine SIMD selection.
#include <float.h>
#include <inttypes.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#if defined(_WIN32)
#include <windows.h>
#endif
#ifdef BOX3D_DOUBLE_PRECISION
#error "This diagnostic contract requires float positions"
#endif
_Static_assert(sizeof(float)==4,"binary32 required");
enum { MAX_CONTACTS=8, MAX_RESULTS=128 };
// Prospective engineering acceptance criteria, not upstream guarantees. Failure
// is a recorded quality result, never a reason to hide output or retune limits.
static const double FLIGHT_POSITION=0.001, FLIGHT_VELOCITY=0.0005;
static const double PENETRATION=0.02, RESTITUTION_ERROR=0.05;
static const double MOMENTUM_RELATIVE=0.0001, ENERGY_GAIN=0.01;
static const double DISTANCE_ERROR=0.01, FRICTION_VELOCITY=0.05;
static const char* profile="unspecified";
static int workers=1,timing=0,failures=0,results=0,traceFailed=0;
static FILE* trace=NULL;
static uint64_t totalDigest=UINT64_C(14695981039346656037);
typedef struct { b3WorldId world; b3BodyId bodies[2]; int count,grid,frames,nonfinite; const char* name; uint64_t digest; } Scene;
static uint32_t bits(float x) { uint32_t u; memcpy(&u,&x,4); return u; }
static int finite_bits(float x) { return (bits(x)&0x7f800000u)!=0x7f800000u; }
static void hash32(uint64_t* h,uint32_t x) { for(int k=0;k<4;++k) { *h^=(x>>(8*k))&255u; *h*=UINT64_C(1099511628211); } }
static double maxd(double a,double b) { return a>b?a:b; }
static double norm(b3Vec3 v) { return sqrt((double)v.x*v.x+(double)v.y*v.y+(double)v.z*v.z); }
static void metric(const char* name,double x)
{
    // Keep JSON valid if a failed simulation produced nonfinite diagnostics.
    // Raw body bit patterns remain untouched in observations and the trace.
    uint64_t u; _Static_assert(sizeof(double)==8,"binary64 metrics required"); memcpy(&u,&x,8);
    printf(",\"%s\":",name); if((u&UINT64_C(0x7ff0000000000000))==UINT64_C(0x7ff0000000000000)) printf("null"); else printf("%.17g",x);
}
static void prefix(const char* type,const char* name)
{ printf("{\"schema\":\"box3d-frontier-v1\",\"driver\":\"quality\",\"profile\":\"%s\",\"type\":\"%s\",\"case\":\"%s\"",profile,type,name); }
static int safe_label(const char* s)
{ if(!*s||strlen(s)>120) return 0; for(;*s;++s) if(!((*s>='a'&&*s<='z')||(*s>='A'&&*s<='Z')||(*s>='0'&&*s<='9')||strchr("_./:+-",*s))) return 0; return 1; }
static void state_values(b3BodyId body,float out[13])
{
    b3WorldTransform t=b3Body_GetTransform(body); b3Vec3 v=b3Body_GetLinearVelocity(body),w=b3Body_GetAngularVelocity(body);
    float a[13]={t.p.x,t.p.y,t.p.z,t.q.v.x,t.q.v.y,t.q.v.z,t.q.s,v.x,v.y,v.z,w.x,w.y,w.z}; memcpy(out,a,sizeof(a));
}
static void observe(Scene* s,int frame)
{
    hash32(&s->digest,(uint32_t)frame);
    for(int b=0;b<s->count;++b) {
        float values[13]; state_values(s->bodies[b],values); hash32(&s->digest,(uint32_t)b);
        if(trace) fprintf(trace,"{\"case\":\"%s\",\"grid\":%d,\"frame\":%d,\"body\":%d,\"bits\":[",s->name,s->grid,frame,b);
        for(int k=0;k<13;++k) { uint32_t u=bits(values[k]); if(!finite_bits(values[k])) s->nonfinite=1; hash32(&s->digest,u); if(trace) fprintf(trace,"%s\"%08" PRIx32 "\"",k?",":"",u); }
        if(trace) { fprintf(trace,"]}\n"); if(ferror(trace)) traceFailed=1; }
    }
}
static void result_prefix(Scene* s,float dt,int substeps,int iterations,int ok)
{
    ++results; if(!ok||s->nonfinite) ++failures; hash32(&totalDigest,(uint32_t)s->grid); hash32(&totalDigest,(uint32_t)s->digest); hash32(&totalDigest,(uint32_t)(s->digest>>32));
    prefix("result",s->name); printf(",\"grid\":%d,\"dt_bits\":\"%08" PRIx32 "\",\"substeps\":%d,\"restitution_iterations\":%d,\"frames\":%d,\"workers_requested\":%d,\"actual_workers\":%d,\"simd_actual\":%d,\"status\":\"%s\",\"finite\":%s,\"digest\":\"%016" PRIx64 "\",\"final_body_bits\":[",s->grid,bits(dt),substeps,iterations,s->frames,workers,b3World_GetWorkerCount(s->world),b3GetWorldFromId(s->world)->simdWidth,(ok&&!s->nonfinite)?"PASS":"FAIL",s->nonfinite?"false":"true",s->digest);
    for(int b=0;b<s->count;++b) { float a[13]; state_values(s->bodies[b],a); printf("%s[",b?",":""); for(int k=0;k<13;++k) printf("%s\"%08" PRIx32 "\"",k?",":"",bits(a[k])); printf("]"); }
    printf("]");
}
static Scene make_scene(const char* name,int grid,b3Vec3 gravity,int iterations)
{
    b3WorldDef wd=b3DefaultWorldDef(); wd.gravity=gravity; wd.workerCount=(uint32_t)workers;
    wd.enableSleep=false; wd.enableContinuous=false; wd.restitutionIterations=iterations;
    Scene s={0}; s.name=name; s.grid=grid; s.world=b3CreateWorld(&wd); s.digest=UINT64_C(14695981039346656037); return s;
}
static b3BodyId sphere(Scene* s,b3Pos p,b3Vec3 v,float radius,float restitution,int dynamic)
{
    b3BodyDef bd=b3DefaultBodyDef(); bd.type=dynamic?b3_dynamicBody:b3_staticBody; bd.position=p; bd.linearVelocity=v;
    bd.linearDamping=0; bd.angularDamping=0; bd.enableSleep=false;
    b3BodyId body=b3CreateBody(s->world,&bd); b3ShapeDef sd=b3DefaultShapeDef(); sd.density=1; sd.baseMaterial.friction=0; sd.baseMaterial.restitution=restitution;
    b3Sphere sp={{0,0,0},radius}; b3CreateSphereShape(body,&sd,&sp); s->bodies[s->count++]=body; return body;
}
static int contact_measure(b3BodyId body,double* penetration,double* impulse)
{
    b3ContactData contacts[MAX_CONTACTS]; int n=b3Body_GetContactData(body,contacts,MAX_CONTACTS);
    for(int c=0;c<n;++c) for(int m=0;m<contacts[c].manifoldCount;++m) for(int p=0;p<contacts[c].manifolds[m].pointCount;++p) {
        const b3ManifoldPoint* pt=contacts[c].manifolds[m].points+p;
        *penetration=maxd(*penetration,-(double)pt->separation); *impulse=maxd(*impulse,pt->totalNormalImpulse);
    }
    return n;
}
static void free_flight(int grid,float dt,int substeps)
{
    Scene s=make_scene("free_flight",grid,(b3Vec3){0,-10,0},2);
    b3BodyId b=sphere(&s,(b3Pos){0,2,0},(b3Vec3){1,3,-2},0.5f,0,1);
    int frames=(int)lround(1.0/(double)dt); s.frames=frames;
    // src/solver.c integrates velocity then position each substep. Its h is a
    // binary32 dt/substeps. Closed form below uses that same h, in double.
    float hf=dt/(float)substeps; double h=hf,maxP=0,maxV=0,maxAnalytic=0,maxBias=0;
    observe(&s,0);
    for(int f=1;f<=frames;++f) {
        b3World_Step(s.world,dt,substeps); observe(&s,f);
        double n=(double)f*substeps,t=n*h;
        b3Pos p=b3Body_GetPosition(b); b3Vec3 v=b3Body_GetLinearVelocity(b);
        double py=2+3*t-5*h*h*n*(n+1),vy=3-10*t,analytic=2+3*t-5*t*t;
        maxP=maxd(maxP,maxd(fabs((double)p.x-t),maxd(fabs((double)p.y-py),fabs((double)p.z+2*t))));
        maxV=maxd(maxV,maxd(fabs((double)v.x-1),maxd(fabs((double)v.y-vy),fabs((double)v.z+2))));
        maxAnalytic=maxd(maxAnalytic,fabs((double)p.y-analytic)); maxBias=maxd(maxBias,5*t*h);
    }
    int ok=maxP<=FLIGHT_POSITION&&maxV<=FLIGHT_VELOCITY&&maxAnalytic<=maxBias+FLIGHT_POSITION;
    result_prefix(&s,dt,substeps,2,ok);
    metric("max_discrete_position_error_m",maxP); metric("max_discrete_velocity_error_mps",maxV);
    metric("max_analytic_position_error_m",maxAnalytic); metric("predicted_semi_implicit_bias_m",maxBias); puts("}");
    b3DestroyWorld(s.world);
}
static void isolated_impact(int grid,float dt,int substeps,int iterations,float restitution)
{
    Scene s=make_scene("isolated_sphere_impact",grid,(b3Vec3){0,0,0},iterations);
    b3BodyId a=sphere(&s,(b3Pos){-0.75f,0,0},(b3Vec3){2,0,0},0.5f,restitution,1);
    b3BodyId b=sphere(&s,(b3Pos){0.75f,0,0},(b3Vec3){-2,0,0},0.5f,restitution,1);
    double ma=b3Body_GetMass(a),mb=b3Body_GetMass(b),initialEnergy=2*(ma+mb),momentumScale=2*(ma+mb);
    double maxPen=0,manifoldPen=0,maxImpulse=0,maxEnergy=0,maxMomentum=0,rebound=0,maxAngular=0;
    int touched=0,frames=(int)lround(0.75/(double)dt); s.frames=frames; observe(&s,0);
    for(int f=1;f<=frames;++f) {
        b3World_Step(s.world,dt,substeps); observe(&s,f);
        b3Pos pa=b3Body_GetPosition(a),pb=b3Body_GetPosition(b); b3Vec3 va=b3Body_GetLinearVelocity(a),vb=b3Body_GetLinearVelocity(b);
        b3Vec3 wa=b3Body_GetAngularVelocity(a),wb=b3Body_GetAngularVelocity(b);
        double dx=(double)pb.x-pa.x,dy=(double)pb.y-pa.y,dz=(double)pb.z-pa.z;
        maxPen=maxd(maxPen,1-sqrt(dx*dx+dy*dy+dz*dz));
        touched|=contact_measure(a,&manifoldPen,&maxImpulse)>0;
        double px=ma*va.x+mb*vb.x,py=ma*va.y+mb*vb.y,pz=ma*va.z+mb*vb.z;
        maxMomentum=maxd(maxMomentum,sqrt(px*px+py*py+pz*pz)/momentumScale);
        double energy=0.5*ma*norm(va)*norm(va)+0.5*mb*norm(vb)*norm(vb);
        maxEnergy=maxd(maxEnergy,energy/initialEnergy); maxAngular=maxd(maxAngular,maxd(norm(wa),norm(wb)));
        if(maxImpulse>0) rebound=maxd(rebound,((double)vb.x-va.x)/4);
    }
    // Central collinear impact of nonrotating equal spheres: angular velocity
    // must remain zero; only then translational energy is total kinetic energy.
    int ok=touched&&maxImpulse>0&&maxPen<=PENETRATION&&fabs(rebound-restitution)<=RESTITUTION_ERROR&&maxMomentum<=MOMENTUM_RELATIVE&&maxEnergy<=1+ENERGY_GAIN&&maxAngular<=1e-6;
    result_prefix(&s,dt,substeps,iterations,ok);
    metric("target_restitution",restitution); metric("observed_restitution",rebound); metric("max_geometry_penetration_m",maxPen);
    metric("max_reported_manifold_penetration_m",manifoldPen); metric("max_energy_ratio",maxEnergy);
    metric("max_relative_momentum_error",maxMomentum); metric("max_angular_speed_radps",maxAngular); metric("max_normal_impulse",maxImpulse);
    printf(",\"touched\":%s}\n",touched?"true":"false");
    b3DestroyWorld(s.world);
}
static void distance_constraint(int grid,float dt,int substeps)
{
    Scene s=make_scene("rigid_distance_pendulum",grid,(b3Vec3){0,-10,0},2);
    b3BodyDef bd=b3DefaultBodyDef(); b3BodyId anchor=b3CreateBody(s.world,&bd);
    b3BodyId b=sphere(&s,(b3Pos){1,0,0},(b3Vec3){0,0,2},0.1f,0,1);
    b3DistanceJointDef jd=b3DefaultDistanceJointDef(); jd.base.bodyIdA=anchor; jd.base.bodyIdB=b; jd.length=1; jd.enableSpring=false;
    b3JointId joint=b3CreateDistanceJoint(s.world,&jd); (void)joint;
    int frames=(int)lround(2.0/(double)dt); s.frames=frames; double error=0; observe(&s,0);
    for(int f=1;f<=frames;++f) {
        b3World_Step(s.world,dt,substeps); observe(&s,f); b3Pos p=b3Body_GetPosition(b);
        error=maxd(error,fabs(sqrt((double)p.x*p.x+(double)p.y*p.y+(double)p.z*p.z)-1));
    }
    result_prefix(&s,dt,substeps,2,error<=DISTANCE_ERROR);
    metric("max_distance_error_m",error); puts(",\"energy_gate\":\"none_constraint_stabilization_and_gravity_do_work\"}"); b3DestroyWorld(s.world);
}
static void sliding_friction(int grid,float dt,int substeps)
{
    Scene s=make_scene("locked_box_sliding",grid,(b3Vec3){0,-10,0},2);
    b3BodyDef bd=b3DefaultBodyDef(); bd.position=(b3Pos){0,-0.5f,0}; b3BodyId floor=b3CreateBody(s.world,&bd);
    b3ShapeDef sd=b3DefaultShapeDef(); sd.density=1; sd.baseMaterial.friction=0.5f; sd.baseMaterial.restitution=0;
    b3BoxHull ground=b3MakeBoxHull(20,0.5f,20); b3CreateHullShape(floor,&sd,&ground.base);
    bd=b3DefaultBodyDef(); bd.type=b3_dynamicBody; bd.position=(b3Pos){0,0.5f,0}; bd.enableSleep=false;
    bd.motionLocks.angularX=true; bd.motionLocks.angularY=true; bd.motionLocks.angularZ=true;
    b3BodyId b=b3CreateBody(s.world,&bd); b3BoxHull box=b3MakeBoxHull(0.5f,0.5f,0.5f); b3CreateHullShape(b,&sd,&box.base);
    s.bodies[s.count++]=b;
    // Settle vertical contact before the experiment. No horizontal drive and no
    // angular work. Warmup is fixed one second for every dt, not output-tuned.
    int warm=(int)lround(1.0/(double)dt); for(int f=0;f<warm;++f) b3World_Step(s.world,dt,substeps);
    b3Vec3 seed=b3Body_GetLinearVelocity(b); b3Body_SetLinearVelocity(b,(b3Vec3){3,seed.y,0});
    double initialHeight=b3Body_GetPosition(b).y,maxPen=0,manifoldPen=0,maxImpulse=0,maxSpeedIncrease=0;
    double maxEnergyGain=0,mass=b3Body_GetMass(b),initialEnergy=0.5*mass*(9+(double)seed.y*seed.y);
    int frames=(int)lround(0.4/(double)dt),touched=0; s.frames=frames; observe(&s,0);
    for(int f=1;f<=frames;++f) {
        b3World_Step(s.world,dt,substeps); observe(&s,f); b3Pos p=b3Body_GetPosition(b); b3Vec3 v=b3Body_GetLinearVelocity(b);
        maxPen=maxd(maxPen,0.5-(double)p.y); touched|=contact_measure(b,&manifoldPen,&maxImpulse)>0;
        maxSpeedIncrease=maxd(maxSpeedIncrease,fabs(v.x)-3);
        // Potential measured relative to seed; static floor and angular locks
        // do no work. Numerical contact stabilization can create energy -> gate.
        double energy=0.5*mass*norm(v)*norm(v)+mass*10*((double)p.y-initialHeight);
        maxEnergyGain=maxd(maxEnergyGain,(energy-initialEnergy)/initialEnergy);
    }
    b3Vec3 v=b3Body_GetLinearVelocity(b); double time=(double)frames*(double)dt;
    double expected=maxd(0,3-0.5*10*time),speedError=fabs((double)v.x-expected);
    int ok=touched&&maxImpulse>0&&maxPen<=PENETRATION&&speedError<=FRICTION_VELOCITY&&maxSpeedIncrease<=0.001&&maxEnergyGain<=ENERGY_GAIN;
    result_prefix(&s,dt,substeps,2,ok);
    metric("coulomb_expected_vx_mps",expected); metric("actual_vx_mps",v.x); metric("coulomb_velocity_error_mps",speedError);
    metric("max_geometry_penetration_m",maxPen); metric("max_reported_manifold_penetration_m",manifoldPen);
    metric("max_relative_energy_gain",maxEnergyGain); metric("max_horizontal_speed_increase_mps",maxSpeedIncrease);
    printf(",\"touched\":%s,\"warmup_frames\":%d}\n",touched?"true":"false",warm); b3DestroyWorld(s.world);
}
static double monotonic_seconds(void)
{
#if defined(_WIN32)
    LARGE_INTEGER q,f; if(!QueryPerformanceFrequency(&f)||!QueryPerformanceCounter(&q)) return -1; return (double)q.QuadPart/(double)f.QuadPart;
#else
    struct timespec ts; if(clock_gettime(CLOCK_MONOTONIC,&ts)!=0) return -1; return (double)ts.tv_sec+1e-9*(double)ts.tv_nsec;
#endif
}
static double elapsed_since(double start)
{ double end=monotonic_seconds(); return start<0||end<start?-1:end-start; }
static void exploratory_timing(void)
{
    Scene s=make_scene("timing_tiny_free_flight",0,(b3Vec3){0,0,0},2); sphere(&s,(b3Pos){0,0,0},(b3Vec3){1,0,0},0.5f,0,1);
    enum { COUNT=64 }; for(int i=0;i<16;++i) b3World_Step(s.world,1.0f/60,4);
    double start=monotonic_seconds(); for(int i=0;i<COUNT;++i) b3World_Step(s.world,1.0f/60,4); double step=elapsed_since(start);
    b3Recording* rec=b3CreateRecording(0); int bytes=0,restored=0; double capture=-1,restore=-1;
    if(rec) {
        start=monotonic_seconds(); b3World_StartRecording(s.world,rec); b3World_StopRecording(s.world); capture=elapsed_since(start);
        bytes=b3Recording_GetSize(rec); start=monotonic_seconds();
        for(int i=0;i<COUNT;++i) { b3RecPlayer* p=b3CreatePlayer(b3Recording_GetData(rec),bytes,workers); if(p) { ++restored; b3DestroyPlayer(p); } }
        restore=elapsed_since(start); b3DestroyRecording(rec);
    }
    prefix("timing","tiny_free_flight");
    printf(",\"status\":\"EXPLORATORY\",\"clock\":\"monotonic_wall\",\"clock_available\":%s,\"unavailable_seconds_sentinel\":-1,\"samples\":%d,\"step_total_seconds\":%.17g,\"snapshot_capture_stop_seconds\":%.17g,\"player_create_destroy_total_seconds\":%.17g,\"successful_restores\":%d,\"snapshot_bytes\":%d,\"allocation_parse_and_destroy_included\":true,\"in_digest\":false,\"performance_gate\":false,\"player_hardware_prediction\":false}\n",step>=0&&capture>=0&&restore>=0?"true":"false",COUNT,step,capture,restore,restored,bytes); b3DestroyWorld(s.world);
}
static void criteria(void)
{
    prefix("criteria","contract"); printf(",\"comparison\":\"exact_body_observation_bits_within_same_compiler_fp_simd_contract_only\",\"quality_tolerances_are_not_determinism_tolerances\":true,\"length_units_per_meter\":%.9g,\"output_limit_lines\":128,\"trace_body_fields\":\"position_xyz_quaternion_xyzs_linear_xyz_angular_xyz\",\"trace_limit_body_records\":14000,\"upstream\":\"5d83df83ab47172c2c745b44315e7538ead02397\"}\n",b3GetLengthUnitsPerMeter());
    prefix("criteria","free_flight"); printf(",\"oracle\":\"double_closed_form_semi_implicit_with_binary32_h_and_separate_continuous_analytic_bias\",\"position_error_limit_m\":%.9g,\"velocity_error_limit_mps\":%.9g,\"assumptions\":\"no_contacts_no_damping_no_sleep_no_speed_cap_no_rotation\"}\n",FLIGHT_POSITION,FLIGHT_VELOCITY);
    prefix("criteria","isolated_sphere_impact"); printf(",\"penetration_limit_m\":%.9g,\"absolute_restitution_error_limit\":%.9g,\"relative_momentum_error_limit\":%.9g,\"relative_energy_gain_limit\":%.9g,\"angular_speed_limit_radps\":1e-6,\"assumptions\":\"equal_masses_central_collinear_no_gravity_no_friction_no_damping_no_sleep\"}\n",PENETRATION,RESTITUTION_ERROR,MOMENTUM_RELATIVE,ENERGY_GAIN);
    prefix("criteria","rigid_distance_pendulum"); printf(",\"max_distance_error_limit_m\":%.9g,\"assumptions\":\"unit_length_rigid_center_anchor_no_collision_no_motor_no_spring\"}\n",DISTANCE_ERROR);
    prefix("criteria","locked_box_sliding"); printf(",\"coulomb_velocity_error_limit_mps\":%.9g,\"penetration_limit_m\":%.9g,\"relative_energy_gain_limit\":%.9g,\"horizontal_speed_increase_limit_mps\":0.001,\"assumptions\":\"mu0.5_on_horizontal_static_floor_angular_locked_seed_after_1s_settle_no_motor_no_damping\",\"criteria_origin\":\"prospective_meter_scale_engineering_requirements_not_fit_to_observed_results\"}\n",FRICTION_VELOCITY,PENETRATION,ENERGY_GAIN);
}
int main(int argc,char** argv)
{
    const char* tracePath=NULL;
    for(int i=1;i<argc;++i) {
        if(!strcmp(argv[i],"--profile")&&i+1<argc&&safe_label(argv[i+1])) profile=argv[++i];
        else if(!strcmp(argv[i],"--workers")&&i+1<argc) { const char* n=argv[++i]; if(strcmp(n,"1")&&strcmp(n,"2")&&strcmp(n,"4")) return 64; workers=atoi(n); }
        else if(!strcmp(argv[i],"--case")&&i+1<argc&&!strcmp(argv[i+1],"all")) ++i;
        else if(!strcmp(argv[i],"--mode")&&i+1<argc&&!strcmp(argv[i+1],"probe")) ++i;
        else if(!strcmp(argv[i],"--trace")&&i+1<argc) tracePath=argv[++i];
        else if(!strcmp(argv[i],"--timing")) timing=1;
        else { fputs("Usage: frontier-quality [--profile LABEL] [--workers 1|2|4] [--case all] [--mode probe] [--trace FILE] [--timing]\n",stderr); return 64; }
    }
    criteria();
    if(tracePath) { trace=fopen(tracePath,"wb"); if(!trace) { prefix("summary","all"); puts(",\"status\":\"FAIL\",\"reason\":\"trace_open\"}"); return 1; } }
    const float dts[]={1.0f/30,1.0f/60,1.0f/120}; const int subs[]={1,4,8},iterations[]={1,2,4}; const float bounce[]={0,0.5f,1};
    int grid=0;
    for(int d=0;d<3;++d) for(int u=0;u<3;++u) free_flight(grid++,dts[d],subs[u]);
    grid=0; for(int d=0;d<3;++d) for(int u=0;u<3;++u) for(int i=0;i<3;++i) for(int e=0;e<3;++e) isolated_impact(grid++,dts[d],subs[u],iterations[i],bounce[e]);
    grid=0; for(int d=0;d<3;++d) for(int u=0;u<3;++u) distance_constraint(grid++,dts[d],subs[u]);
    grid=0; for(int d=0;d<3;++d) for(int u=0;u<3;++u) sliding_friction(grid++,dts[d],subs[u]);
    if(timing) exploratory_timing();
    if(trace) { if(fclose(trace)!=0) traceFailed=1; trace=NULL; }
    if(traceFailed) ++failures;
    prefix("summary","all"); printf(",\"status\":\"%s\",\"results\":%d,\"failures\":%d,\"trace_io_ok\":%s,\"digest\":\"%016" PRIx64 "\",\"digest_scope\":\"ordered_per_case_per_frame_public_body_bits_excludes_metrics_and_timing\"}\n",failures?"FAIL":"PASS",results,failures,traceFailed?"false":"true",totalDigest);
    return failures?1:0;
}
