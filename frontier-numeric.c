// SPDX-License-Identifier: MIT
// Synthetic numerical frontiers for upstream Box3D, pinned by the build wrapper.
// Includes unchanged upstream SIMD helpers; does not patch their semantics.
#include "box3d/box3d.h"
#include "box3d/collision.h"
#include "box3d/constants.h"
#include "simd.h"
#include "physics_world.h" // Read-only actual engine SIMD selection.
#include <fenv.h>
#include <float.h>
#include <inttypes.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#if defined(_WIN32)
#include <windows.h>
#else
#include <pthread.h>
#endif
#if defined(__x86_64__) || defined(_M_X64) || defined(__i386__)
#include <xmmintrin.h>
#define HAVE_MXCSR 1
#endif
#if defined(__aarch64__) && (defined(__clang__) || defined(__GNUC__))
#define HAVE_FPCR 1
#endif
#if defined(_MSC_VER)
#define NOINLINE __declspec(noinline)
#else
#define NOINLINE __attribute__((noinline))
#endif
#ifdef BOX3D_DOUBLE_PRECISION
#error "This diagnostic contract requires float positions"
#endif
_Static_assert(sizeof(float) == 4, "binary32 required");
static const char* profile = "unspecified";
static int workers = 1, failures = 0, observations = 0;
static FILE* trace = NULL;
static int traceFailed = 0;
static uint64_t digest = UINT64_C(14695981039346656037);
static uint32_t bits(float x) { uint32_t u; memcpy(&u,&x,4); return u; }
static float value(uint32_t u) { float x; memcpy(&x,&u,4); return x; }
static NOINLINE float runtime_float(uint32_t u) { volatile float x=value(u); return x; }
static void hash32(uint32_t u) { for(int i=0;i<4;++i) { digest^=(u>>(8*i))&255u; digest*=UINT64_C(1099511628211); } }
static int finite_bits(float x) { return (bits(x)&0x7f800000u)!=0x7f800000u; }
static void prefix(const char* type,const char* name)
{ printf("{\"schema\":\"box3d-frontier-v1\",\"driver\":\"numeric\",\"profile\":\"%s\",\"type\":\"%s\",\"case\":\"%s\"",profile,type,name); }
static int safe_label(const char* s)
{ if(!*s || strlen(s)>120) return 0; for(;*s;++s) if(!((*s>='a'&&*s<='z')||(*s>='A'&&*s<='Z')||(*s>='0'&&*s<='9')||strchr("_./:+-",*s))) return 0; return 1; }
static void observation(const char* name,int index,const float* data,int n)
{
    prefix("observation",name); printf(",\"index\":%d,\"bits\":[",index); hash32((uint32_t)index); hash32((uint32_t)n);
    if(trace) fprintf(trace,"{\"case\":\"%s\",\"index\":%d,\"bits\":[",name,index);
    for(int i=0;i<n;++i) { uint32_t u=bits(data[i]); printf("%s\"%08" PRIx32 "\"",i?",":"",u); if(trace) fprintf(trace,"%s\"%08" PRIx32 "\"",i?",":"",u); hash32(u); if(!finite_bits(data[i])) ++failures; }
    if(trace) { fprintf(trace,"]}\n"); if(ferror(trace)) traceFailed=1; }
    puts("]}"); ++observations;
}
typedef struct { uint64_t control; int round,exceptions,available; uint32_t subInput,subOutput; } Environment;
static Environment environment(void)
{
    Environment e={0}; e.round=fegetround(); e.exceptions=fetestexcept(FE_ALL_EXCEPT);
#if HAVE_MXCSR
    e.control=_mm_getcsr(); e.available=1;
#elif HAVE_FPCR
    __asm__ volatile("mrs %0, fpcr":"=r"(e.control)); e.available=1;
#endif
    return e;
}
static void set_control(uint64_t u)
{
#if HAVE_MXCSR
    _mm_setcsr((unsigned)u);
#elif HAVE_FPCR
    __asm__ volatile("msr fpcr, %0"::"r"(u));
    __asm__ volatile("isb");
#else
    (void)u;
#endif
}
static void emit_environment(const char* name,int thread,Environment e)
{
    prefix("environment",name);
    printf(",\"thread\":%d,\"scope\":\"synthetic_os_thread_not_engine_pool\",\"round\":%d,\"exceptions\":%d,\"register_available\":%s,\"control\":\"%016" PRIx64 "\"",thread,e.round,e.exceptions,e.available?"true":"false",e.control);
#if HAVE_MXCSR
    printf(",\"register\":\"MXCSR\",\"ftz\":%s,\"daz\":%s",(e.control&(1u<<15))?"true":"false",(e.control&(1u<<6))?"true":"false");
#elif HAVE_FPCR
    printf(",\"register\":\"FPCR\",\"fz\":%s,\"fz16\":%s",(e.control&(UINT64_C(1)<<24))?"true":"false",(e.control&(UINT64_C(1)<<19))?"true":"false");
#else
    printf(",\"register\":\"unsupported\"");
#endif
    printf(",\"subnormal_input_mul2_bits\":\"%08" PRIx32 "\",\"normal_input_mulhalf_bits\":\"%08" PRIx32 "\"}\n",e.subInput,e.subOutput);
}
static NOINLINE float separate(float a,float b,float c) { volatile float product=a*b; volatile float sum=product+c; return sum; }
static NOINLINE float expression(float a,float b,float c) { return a*b+c; }
static NOINLINE float fused(float a,float b,float c) { return fmaf(a,b,c); }
static NOINLINE Environment sample_environment(void)
{
    Environment e=environment();
    volatile float tiny=runtime_float(1),normal=runtime_float(0x00800000u),two=2.0f,half=0.5f;
    e.subInput=bits(tiny*two); e.subOutput=bits(normal*half); return e;
}
#if defined(_WIN32)
static DWORD WINAPI thread_probe(LPVOID p) { *(Environment*)p=sample_environment(); return 0; }
#else
static void* thread_probe(void* p) { *(Environment*)p=sample_environment(); return NULL; }
#endif
static void thread_environments(void)
{
    Environment e[4]; int ok[4]={0};
#if defined(_WIN32)
    HANDLE threads[4]={0};
    for(int i=0;i<workers;++i) { threads[i]=CreateThread(NULL,0,thread_probe,e+i,0,NULL); ok[i]=threads[i]!=NULL; }
    for(int i=0;i<workers;++i) if(ok[i]) { WaitForSingleObject(threads[i],INFINITE); CloseHandle(threads[i]); }
#else
    pthread_t threads[4];
    for(int i=0;i<workers;++i) ok[i]=pthread_create(threads+i,NULL,thread_probe,e+i)==0;
    for(int i=0;i<workers;++i) if(ok[i]) pthread_join(threads[i],NULL);
#endif
    for(int i=0;i<workers;++i) { if(ok[i]) emit_environment("thread_inherit",i,e[i]); else { ++failures; prefix("result","thread_inherit"); printf(",\"thread\":%d,\"status\":\"FAIL\",\"reason\":\"thread_creation\"}\n",i); } }
}
static void zeros(void)
{
    for(int i=0;i<4;++i) {
        float a=runtime_float((i&1)?0x80000000u:0u),b=runtime_float((i&2)?0x80000000u:0u);
        float out[16]={a,b,b3MinFloat(a,b),b3MaxFloat(a,b)};
        b3FloatW4 aw=b3SplatW4(a),bw=b3SplatW4(b);
        b3StoreW4(out+4,b3MinW4(aw,bw)); b3StoreW4(out+8,b3MaxW4(aw,bw)); b3StoreW4(out+12,b3SymClampW4(aw,bw));
        observation("signed_zero_scalar_and_upstream_wide4",i,out,16);
#if defined(B3_SIMD_AVX2)
        float wide[24]; b3FloatW8 av=b3SplatW8(a),bv=b3SplatW8(b);
        b3StoreW8(wide,b3MinW8(av,bv)); b3StoreW8(wide+8,b3MaxW8(av,bv)); b3StoreW8(wide+16,b3SymClampW8(av,bv));
        observation("signed_zero_upstream_wide8",i,wide,24);
#endif
    }
    for(int i=0;i<8;++i) {
        float a=runtime_float((i&1)?0x80000000u:0u),lo=runtime_float((i&2)?0x80000000u:0u),hi=runtime_float((i&4)?0x80000000u:0u);
        float out[]={a,lo,hi,b3ClampFloat(a,lo,hi)}; observation("signed_zero_scalar_clamp",i,out,4);
    }
}
static void arithmetic(void)
{
    // Binary32 operands produce exact fused -2^-46 and separately rounded +0
    // in round-to-nearest gradual arithmetic. fmaf may use libm, not hardware FMA.
    float a=runtime_float(0x3f800001u),b=runtime_float(0x3f7ffffeu),c=runtime_float(0xbf800000u);
    float out[11]={a,b,c,separate(a,b,c),fused(a,b,c),expression(a,b,c)};
    b3StoreW4(out+6,b3MulAddW4(b3SplatW4(c),b3SplatW4(a),b3SplatW4(b)));
    out[10]=b3MulAdd((b3Vec3){c,c,c},a,(b3Vec3){b,b,b}).x;
    observation("fma_runtime_witness",0,out,11);
    int witness=bits(out[3])!=bits(out[4]); if(!witness) ++failures;
    prefix("result","fma_runtime_witness"); printf(",\"status\":\"%s\",\"fused_differs_from_volatile_separate\":%s,\"expression_matches_fused\":%s,\"hardware_fma_instruction\":\"unverified_inspect_disassembly\"}\n",witness?"PASS":"FAIL",witness?"true":"false",bits(out[4])==bits(out[5])?"true":"false");
    Environment before=environment(); emit_environment("main_inherit",-1,sample_environment());
    if(before.available) {
        uint64_t mask=0;
#if HAVE_MXCSR
        mask=(1u<<15)|(1u<<6);
#elif HAVE_FPCR
        mask=UINT64_C(1)<<24;
#endif
        for(int mode=0;mode<2;++mode) {
            set_control(mode?(before.control|mask):(before.control&~mask));
            Environment e=sample_environment(); emit_environment(mode?"controlled_flush":"controlled_gradual",-1,e);
            float results[]={value(e.subInput),value(e.subOutput)};
            observation(mode?"subnormal_flush_contract":"subnormal_gradual_contract",0,results,2);
        }
        set_control(before.control);
        Environment restored=environment(); int ok=restored.control==before.control;
        if(!ok) ++failures;
        prefix("result","controlled_environment_restore"); printf(",\"status\":\"%s\",\"expected_control\":\"%016" PRIx64 "\",\"actual_control\":\"%016" PRIx64 "\",\"scope\":\"MXCSR_or_FPCR_control_only_not_all_exception_status\"}\n",ok?"PASS":"FAIL",before.control,restored.control);
    }
}
static b3BodyId sphere_body(b3WorldId w,float x,float vx,int dynamic)
{
    b3BodyDef bd=b3DefaultBodyDef(); bd.type=dynamic?b3_dynamicBody:b3_staticBody;
    bd.position=(b3Pos){x,0,0}; bd.linearVelocity=(b3Vec3){vx,0,0}; bd.linearDamping=0; bd.angularDamping=0;
    b3BodyId body=b3CreateBody(w,&bd); b3ShapeDef sd=b3DefaultShapeDef(); sd.density=1;
    sd.baseMaterial.friction=0; sd.baseMaterial.restitution=1;
    b3Sphere s={{0,0,0},0.5f}; b3CreateSphereShape(body,&sd,&s); return body;
}
static void threshold_frontiers(void)
{
    b3WorldDef defaults=b3DefaultWorldDef(); b3BodyDef bodyDefaults=b3DefaultBodyDef();
    float thresholds[]={B3_LINEAR_SLOP,B3_SPECULATIVE_DISTANCE,B3_TIME_TO_SLEEP,bodyDefaults.sleepThreshold,defaults.restitutionThreshold};
    const char* labels[]={"linear_slop","speculative_distance","time_to_sleep","sleep_speed","restitution_speed"};
    for(int t=0;t<5;++t) {
        float x=thresholds[t],out[]={nextafterf(x,-INFINITY),x,nextafterf(x,INFINITY)};
        observation(labels[t],0,out,3);
        prefix("criteria",labels[t]); printf(",\"source\":\"%s\",\"meaning\":\"adjacent_input_bits_not_claimed_solver_branch_activation\"}\n",t<3?"include/box3d/constants.h":"src/types.c");
    }
    // Exact public sphere narrowphase threshold is totalRadius^2, not speculative distance.
    for(int side=-1;side<=1;++side) {
        float x=side<0?nextafterf(1.0f,0.0f):side>0?nextafterf(1.0f,INFINITY):1.0f;
        b3Sphere s={{0,0,0},0.5f}; b3LocalManifoldPoint point={0}; b3LocalManifold manifold={0}; manifold.points=&point;
        b3Transform xf=b3Transform_identity; xf.p.x=x; b3CollideSpheres(&manifold,1,&s,&s,xf);
        float out[]={x,(float)manifold.pointCount,point.separation,manifold.normal.x,manifold.normal.y,manifold.normal.z};
        observation("sphere_touch_ulp",side+1,out,6);
        int expected=side<=0; int ok=manifold.pointCount==expected; if(!ok) ++failures;
        prefix("result","sphere_touch_ulp"); printf(",\"index\":%d,\"status\":\"%s\",\"expected_point_count\":%d,\"source\":\"src/convex_manifold.c:b3CollideSpheres distanceSq>totalRadius*totalRadius\"}\n",side+1,ok?"PASS":"FAIL",expected);
    }
    // Hull/sphere uses the real speculative-distance rejection branch. Adjacent
    // centers may lose low bits when distance is computed: expose, do not hide.
    for(int side=-1;side<=1;++side) {
        float boundary=1.0f+B3_SPECULATIVE_DISTANCE;
        float center=side<0?nextafterf(boundary,0):side>0?nextafterf(boundary,INFINITY):boundary;
        b3BoxHull hull=b3MakeBoxHull(0.5f,0.5f,0.5f); b3Sphere sp={{0,0,0},0.5f};
        b3LocalManifoldPoint point={0}; b3LocalManifold manifold={0}; manifold.points=&point;
        b3SimplexCache cache={0}; b3Transform xf=b3Transform_identity; xf.p.x=center;
        b3CollideHullAndSphere(&manifold,1,&hull.base,&sp,xf,&cache);
        float out[]={center,boundary,B3_SPECULATIVE_DISTANCE,(float)manifold.pointCount,point.separation,manifold.normal.x};
        observation("hull_sphere_speculative_ulp",side+1,out,6);
        prefix("result","hull_sphere_speculative_ulp"); printf(",\"index\":%d,\"status\":\"OBSERVED\",\"source\":\"src/convex_manifold.c:b3CollideHullAndSphere distanceOutput.distance>radius+B3_SPECULATIVE_DISTANCE\"}\n",side+1);
    }
    for(int side=-1;side<=1;++side) {
        float speed=side<0?nextafterf(defaults.restitutionThreshold,0):side>0?nextafterf(defaults.restitutionThreshold,INFINITY):defaults.restitutionThreshold;
        b3WorldDef wd=defaults; wd.gravity=(b3Vec3){0,0,0}; wd.enableSleep=false; wd.enableContinuous=false; wd.workerCount=(uint32_t)workers;
        b3WorldId w=b3CreateWorld(&wd); sphere_body(w,0,0,0); b3BodyId b=sphere_body(w,1.0f,-speed,1);
        b3World_Step(w,1.0f/60,4); b3Vec3 v=b3Body_GetLinearVelocity(b); b3ContactData contacts[4]; int n=b3Body_GetContactData(b,contacts,4);
        float impulse=0,normalVelocity=0;
        for(int j=0;j<n;++j) for(int m=0;m<contacts[j].manifoldCount;++m) for(int p=0;p<contacts[j].manifolds[m].pointCount;++p) {
            impulse+=contacts[j].manifolds[m].points[p].totalNormalImpulse;
            normalVelocity=contacts[j].manifolds[m].points[p].normalVelocity;
        }
        float out[]={speed,v.x,v.y,v.z,impulse,normalVelocity}; observation("restitution_speed_ulp_world",side+1,out,6);
        prefix("result","restitution_speed_ulp_world"); printf(",\"index\":%d,\"contacts\":%d,\"actual_workers\":%d,\"simd_actual\":%d,\"status\":\"OBSERVED\",\"source\":\"src/contact_solver.c armed relativeVelocity<-threshold and positive compression\"}\n",side+1,n,b3World_GetWorkerCount(w),b3GetWorldFromId(w)->simdWidth);
        b3DestroyWorld(w);
    }
    for(int side=-1;side<=1;++side) {
        float speed=side<0?nextafterf(bodyDefaults.sleepThreshold,0):side>0?nextafterf(bodyDefaults.sleepThreshold,INFINITY):bodyDefaults.sleepThreshold;
        b3WorldDef wd=defaults; wd.gravity=(b3Vec3){0,0,0}; wd.workerCount=(uint32_t)workers; wd.enableContinuous=false;
        b3WorldId w=b3CreateWorld(&wd); b3BodyId b=sphere_body(w,0,speed,1); int firstSleep=-1;
        for(int i=1;i<=45;++i) { b3World_Step(w,1.0f/60,1); if(firstSleep<0&&!b3Body_IsAwake(b)) firstSleep=i; }
        b3Vec3 v=b3Body_GetLinearVelocity(b); b3Pos p=b3Body_GetPosition(b);
        float out[]={speed,v.x,p.x,(float)firstSleep,(float)b3Body_IsAwake(b)}; observation("sleep_speed_ulp_world",side+1,out,5);
        prefix("result","sleep_speed_ulp_world"); printf(",\"index\":%d,\"first_sleep_frame\":%d,\"actual_workers\":%d,\"simd_actual\":%d,\"status\":\"OBSERVED\",\"source\":\"src/solver.c sleepVelocity>body->sleepThreshold; position and rotation corrections also contribute\"}\n",side+1,firstSleep,b3World_GetWorkerCount(w),b3GetWorldFromId(w)->simdWidth); b3DestroyWorld(w);
    }
    // Time boundary is exercised with exactly representable 0.5 seconds and neighbors.
    for(int side=-1;side<=1;++side) {
        float dt=side<0?nextafterf(B3_TIME_TO_SLEEP,0):side>0?nextafterf(B3_TIME_TO_SLEEP,INFINITY):B3_TIME_TO_SLEEP;
        b3WorldDef wd=defaults; wd.gravity=(b3Vec3){0,0,0}; wd.workerCount=(uint32_t)workers;
        b3WorldId w=b3CreateWorld(&wd); b3BodyId b=sphere_body(w,0,0,1); b3World_Step(w,dt,1);
        float out[]={dt,(float)b3Body_IsAwake(b)}; observation("sleep_time_ulp_world",side+1,out,2); b3DestroyWorld(w);
    }
}
int main(int argc,char** argv)
{
    const char* tracePath=NULL;
    for(int i=1;i<argc;++i) {
        if(!strcmp(argv[i],"--profile")&&i+1<argc&&safe_label(argv[i+1])) profile=argv[++i];
        else if(!strcmp(argv[i],"--workers")&&i+1<argc) { const char* n=argv[++i]; if(strcmp(n,"1")&&strcmp(n,"2")&&strcmp(n,"4")) return 64; workers=atoi(n); }
        else if((!strcmp(argv[i],"--case")||!strcmp(argv[i],"--mode"))&&i+1<argc) { int mode=!strcmp(argv[i],"--mode"); if(strcmp(argv[++i],mode?"probe":"all")) return 64; }
        else if(!strcmp(argv[i],"--trace")&&i+1<argc) tracePath=argv[++i];
        else { fputs("Usage: frontier-numeric [--profile LABEL] [--workers 1|2|4] [--case all] [--mode probe] [--trace FILE]\n",stderr); return 64; }
    }
    prefix("criteria","contract"); printf(",\"workers_requested\":%d,\"comparison\":\"exact_bits_within_same_compiler_fp_simd_contract_only\",\"cross_contract_equality_required\":false,\"zeros_normalized\":false,\"fma_witness_required\":true,\"finite_gate_via_bits\":true,\"output_limit_lines\":100,\"upstream\":\"5d83df83ab47172c2c745b44315e7538ead02397\"}\n",workers);
    prefix("environment","build");
#if defined(__clang__)
    printf(",\"compiler\":\"clang\",\"version\":\"%s\"",__clang_version__);
#elif defined(__GNUC__)
    printf(",\"compiler\":\"gcc\",\"version\":\"%s\"",__VERSION__);
#elif defined(_MSC_VER)
    printf(",\"compiler\":\"msvc\",\"version\":%d",_MSC_VER);
#else
    printf(",\"compiler\":\"unknown\"");
#endif
#if defined(B3_SIMD_NEON)
    printf(",\"wide4\":\"NEON\"");
#elif defined(B3_SIMD_SSE2)
    printf(",\"wide4\":\"SSE2\"");
#else
    printf(",\"wide4\":\"scalar\"");
#endif
#if defined(__FAST_MATH__)
    printf(",\"fast_math_macro\":true");
#else
    printf(",\"fast_math_macro\":false");
#endif
    puts(",\"complete_flags\":\"wrapper_build_manifest_required\"}");
    if(tracePath) { trace=fopen(tracePath,"wb"); if(!trace) { prefix("summary","all"); puts(",\"status\":\"FAIL\",\"reason\":\"trace_open\"}"); return 1; } }
    Environment initial=environment();
    thread_environments(); zeros(); arithmetic(); threshold_frontiers();
    set_control(initial.control);
    if(trace) { if(fclose(trace)!=0) traceFailed=1; trace=NULL; } if(traceFailed) ++failures;
    prefix("summary","all"); printf(",\"status\":\"%s\",\"failures\":%d,\"observations\":%d,\"digest\":\"%016" PRIx64 "\",\"digest_scope\":\"ordered_binary32_observations_excludes_environment_and_timing\"}\n",failures?"FAIL":"PASS",failures,observations,digest);
    return failures?1:0;
}
