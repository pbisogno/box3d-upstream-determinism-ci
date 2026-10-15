// SPDX-License-Identifier: MIT
// Isolated operator measurements and an EXPERIMENTAL zero-encoding policy.
// This does not claim emitted opcodes or validate the engine patch's physics.
#include "box3d/box3d.h"
#include "simd.h"
#include "sweeplab/canonical-zero.h"
#include <inttypes.h>
#include <fenv.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#if defined(__x86_64__) || defined(_M_X64)
#include <xmmintrin.h>
#endif
#if defined(_MSC_VER)
#define SW_NO_INLINE __declspec(noinline)
#else
#define SW_NO_INLINE __attribute__((noinline))
#endif
#ifdef BOX3D_DOUBLE_PRECISION
#error "This experiment measures binary32 only"
#endif
_Static_assert(sizeof(float)==4,"binary32 required");
typedef struct { uint32_t input,bound; const char* name; } Fixture;
static const Fixture fixtures[]={
    {0x00000000u,0x00000000u,"plus_zero_plus_zero"},
    {0x80000000u,0x00000000u,"minus_zero_plus_zero"},
    {0x00000000u,0x80000000u,"plus_zero_minus_zero"},
    {0x80000000u,0x80000000u,"minus_zero_minus_zero"},
    {0x00000000u,0x3f800000u,"plus_zero_unit_bound"},
    {0x80000000u,0x3f800000u,"minus_zero_unit_bound"},
    {0x00000001u,0x00000000u,"positive_min_subnormal_zero_bound"},
    {0x80000001u,0x00000000u,"negative_min_subnormal_zero_bound"},
    {0x00000001u,0x80000000u,"positive_min_subnormal_minus_zero_bound"},
    {0x80000001u,0x80000000u,"negative_min_subnormal_minus_zero_bound"},
    {0x00000001u,0x00000001u,"positive_min_subnormal_equal_bound"},
    {0x80000001u,0x00000001u,"negative_min_subnormal_equal_bound"},
    {0x007fffffu,0x00800000u,"positive_max_subnormal_normal_bound"},
    {0x807fffffu,0x00800000u,"negative_max_subnormal_normal_bound"},
    {0x33800000u,0x00000000u,"positive_small_normal_zero_bound"},
    {0xb3800000u,0x00000000u,"negative_small_normal_zero_bound"},
    {0x3e000000u,0x3d000000u,"positive_saturation_small_bound"},
    {0xbe000000u,0x3d000000u,"negative_saturation_small_bound"},
    {0x3d000000u,0x3d000000u,"positive_equal_bound"},
    {0xbd000000u,0x3d000000u,"negative_equal_bound"},
    {0x3c800000u,0x3d000000u,"positive_interior"},
    {0xbc800000u,0x3d000000u,"negative_interior"},
    {0x40000000u,0x3f800000u,"positive_unit_saturation"},
    {0xc0000000u,0x3f800000u,"negative_unit_saturation"},
    {0x3f800000u,0x7f7fffffu,"positive_large_bound"},
    {0xbf800000u,0x7f7fffffu,"negative_large_bound"},
    {0x7f7fffffu,0x7f7fffffu,"positive_max_finite_equal_bound"},
    {0xff7fffffu,0x7f7fffffu,"negative_max_finite_equal_bound"},
    {0x00800000u,0x007fffffu,"positive_normal_subnormal_bound"},
    {0x80800000u,0x007fffffu,"negative_normal_subnormal_bound"},
};
static const uint32_t mapper_patterns[]={
    0x00000000u,0x80000000u,0x00000001u,0x80000001u,
    0x007fffffu,0x807fffffu,0x00800000u,0x80800000u,
    0x3d000000u,0xbd000000u,0x7f800000u,0xff800000u,
    0x7fc12345u,0xffc54321u,0x7f812345u,0xff854321u,
};
enum { FIXTURES=30, MAPPER_PATTERNS=16 };
_Static_assert(sizeof(fixtures)/sizeof(fixtures[0])==FIXTURES,"frozen operator roster");
_Static_assert(sizeof(mapper_patterns)/sizeof(mapper_patterns[0])==MAPPER_PATTERNS,"frozen mapper roster");
static const char* profile="unspecified";
static int requested=4,width=0,results=0,failures=0,io_ok=1,numeric_failures=0;
static int nan_preserved=1,zero_control_seen=0,positive_control_seen=0;
static uint32_t zero_control_raw,positive_control_raw;
static FILE* trace;
static uint32_t bits(float value) { uint32_t u;memcpy(&u,&value,4);return u; }
static float from_bits(uint32_t u) { float f;memcpy(&f,&u,4);return f; }
static SW_NO_INLINE float runtime_float(uint32_t u)
{ volatile float v=from_bits(u);return v; }
static double numeric_value(uint32_t u)
{
    // Decode via integers into normal binary64 values. Converting a binary32
    // subnormal with a floating instruction could itself flush under DAZ and
    // silently weaken this independent numeric oracle.
    int exponent=(int)((u>>23)&255u);uint32_t fraction=u&0x007fffffu;
    double magnitude=exponent==0?ldexp((double)fraction,-149):ldexp((double)(fraction|0x00800000u),exponent-150);
    return (u&0x80000000u)?-magnitude:magnitude;
}
static const char* truth(int v) { return v?"true":"false"; }
static const char* backend(void)
{
#if defined(BOX3D_DISABLE_SIMD)
    return "scalar";
#elif defined(__aarch64__) || defined(_M_ARM64)
    return "neon";
#else
    return width==8?"avx2":"sse2";
#endif
}
static int safe_label(const char* s)
{
    if(!*s || strlen(s)>120)return 0;
    for(;*s;++s)if(!((*s>='a'&&*s<='z')||(*s>='A'&&*s<='Z')||(*s>='0'&&*s<='9')||strchr("_./:+-",*s)))return 0;
    return 1;
}
static SW_NO_INLINE int actual_wide_clamp(float input,float bound,float out[8])
{
    if(width==4){b3StoreW4(out,b3SymClampW4(b3SplatW4(input),b3SplatW4(bound)));return 1;}
#if defined(B3_SIMD_AVX2)
    if(width==8){b3StoreW8(out,b3SymClampW8(b3SplatW8(input),b3SplatW8(bound)));return 1;}
#endif
    return 0;
}
static SW_NO_INLINE float actual_scalar_clamp(float input,float bound)
{ return b3ClampFloat(input,-bound,bound); }
static int mapped_bits_valid(uint32_t raw,uint32_t output,int mode)
{ return output==(mode && (raw&0x7fffffffu)==0?0:raw); }
static void projection(FILE* f,const char* op,int case_id,int lane,int op_width,
    uint32_t input,uint32_t bound,uint32_t raw,uint32_t identity,uint32_t canonical)
{
    fprintf(f,"\"result_index\":%d,\"operator\":\"%s\",\"case_id\":%d,\"width\":%d,\"lane\":%d,\"input_bits\":\"%08" PRIx32 "\",\"bound_bits\":\"%08" PRIx32 "\",\"raw_bits\":\"%08" PRIx32 "\",\"identity_bits\":\"%08" PRIx32 "\",\"canonical_bits\":\"%08" PRIx32 "\"",
        results,op,case_id,op_width,lane,input,bound,raw,identity,canonical);
}
static void result(const char* op,int case_id,const char* name,int lane,int op_width,
    uint32_t input,uint32_t bound,uint32_t raw,int numeric_applicable)
{
    uint32_t identity=bits(sweeplabCanonicalZeroFloat(runtime_float(raw),0));
    uint32_t canonical=bits(sweeplabCanonicalZeroFloat(runtime_float(raw),1));
    int identity_ok=identity==raw;
    int canonical_ok=mapped_bits_valid(raw,canonical,1);
    int nonzero_ok=(raw&0x7fffffffu)==0 || (identity==raw && canonical==raw);
    int numeric_ok=1;
    if(numeric_applicable){
        double a=numeric_value(input),b=numeric_value(bound),expected=a < -b?-b:(a>b?b:a);
        numeric_ok=(raw&0x7f800000u)!=0x7f800000u && numeric_value(raw)==expected;
        if(!strcmp(op,"wide_sym_clamp") && lane==0 && case_id==0){zero_control_seen=1;zero_control_raw=raw;}
        if(!strcmp(op,"wide_sym_clamp") && lane==0 && case_id==18){positive_control_seen=1;positive_control_raw=raw;}
    }
    if((raw&0x7f800000u)==0x7f800000u && (raw&0x007fffffu)!=0 && !nonzero_ok)nan_preserved=0;
    int pass=identity_ok && canonical_ok && nonzero_ok && numeric_ok;
    if(!pass)++failures;if(!numeric_ok)++numeric_failures;
    printf("{\"schema\":\"box3d-sweeplab-v1\",\"driver\":\"operator\",\"type\":\"result\",\"profile\":\"%s\",\"backend_actual\":\"%s\",\"simd_actual\":%d,",profile,backend(),width);
    projection(stdout,op,case_id,lane,op_width,input,bound,raw,identity,canonical);
    printf(",\"case\":\"%s\",\"numeric_applicable\":%s,\"canonical_changed\":%s,\"execution_pass\":%s,\"status\":\"%s\",\"gates\":{\"numeric_contract\":%s,\"identity_bits_exact\":%s,\"canonical_zero_only\":%s,\"nonzero_bits_preserved\":%s}}\n",
        name,truth(numeric_applicable),truth(canonical!=raw),truth(pass),pass?"complete":"gate_failed",truth(numeric_ok),truth(identity_ok),truth(canonical_ok),truth(nonzero_ok));
    if(trace){fputc('{',trace);projection(trace,op,case_id,lane,op_width,input,bound,raw,identity,canonical);fputs("}\n",trace);if(ferror(trace))io_ok=0;}
    ++results;
}
static uint64_t fp_control(const char** name)
{
#if defined(__x86_64__) || defined(_M_X64)
    *name="MXCSR";return _mm_getcsr();
#elif defined(__aarch64__) && (defined(__clang__) || defined(__GNUC__))
    uint64_t u;__asm__ volatile("mrs %0, fpcr":"=r"(u));*name="FPCR";return u;
#else
    *name="unavailable";return 0;
#endif
}
static void print_fp_environment(const char* name,uint64_t control,int round)
{
    printf("{\"round\":%d,\"register\":\"%s\",\"raw_control\":\"%016" PRIx64 "\",\"ftz\":",round,name,control);
    if(!strcmp(name,"MXCSR"))printf("%s",truth((control&(UINT64_C(1)<<15))!=0));else fputs("null",stdout);
    fputs(",\"daz\":",stdout);
    if(!strcmp(name,"MXCSR"))printf("%s",truth((control&(UINT64_C(1)<<6))!=0));else fputs("null",stdout);
    fputs(",\"fz\":",stdout);
    if(!strcmp(name,"FPCR"))printf("%s",truth((control&(UINT64_C(1)<<24))!=0));else fputs("null",stdout);
    putchar('}');
}
int main(int argc,char** argv)
{
    const char* trace_path=NULL;
    for(int i=1;i<argc;i+=2){
        if(i+1>=argc)goto usage;
        if(!strcmp(argv[i],"--profile"))profile=argv[i+1];
        else if(!strcmp(argv[i],"--trace"))trace_path=argv[i+1];
        else if(!strcmp(argv[i],"--workers")){if(strcmp(argv[i+1],"1"))goto usage;}
        else if(!strcmp(argv[i],"--simd")){if(strcmp(argv[i+1],"0")&&strcmp(argv[i+1],"4")&&strcmp(argv[i+1],"8"))goto usage;requested=atoi(argv[i+1]);}
        else goto usage;
    }
    if(!safe_label(profile))goto usage;
#if defined(BOX3D_DISABLE_SIMD)
    if(requested!=0)goto usage;
#else
    if(requested==0)goto usage;
#endif
    b3SetSIMDWidth(requested==8?8:4);width=b3GetSIMDWidth();
    if(width!=(requested==8?8:4)){fputs("requested backend width is unavailable\n",stderr);return 5;}
#if !defined(B3_SIMD_AVX2)
    if(width==8){fputs("operator translation unit lacks AVX2 helpers\n",stderr);return 5;}
#endif
    if(trace_path){trace=fopen(trace_path,"wb");if(!trace){perror("trace");return 5;}}
    const char* register_name;uint64_t control=fp_control(&register_name);int initial_round=fegetround();
    int expected_results=FIXTURES*(width+1)+MAPPER_PATTERNS;
    printf("{\"schema\":\"box3d-sweeplab-v1\",\"driver\":\"operator\",\"type\":\"criteria\",\"profile\":\"%s\",\"experimental_policy\":true,\"fixtures\":30,\"mapper_patterns\":16,\"results_expected\":%d,\"numeric_scope\":\"finite_symmetric_clamp_with_gradual_inputs_signed_zeros_numerically_equal\",\"raw_bit_scope\":\"identity_all_encodings_canonical_only_zero_encodings\",\"nan_scope\":\"mapper_only_no_NaN_clamp_semantics_claim\",\"opcode_disassembly_verified\":false,\"fp_environment\":",profile,expected_results);
    print_fp_environment(register_name,control,initial_round);puts("}");
    for(int c=0;c<FIXTURES;++c){
        volatile uint32_t iu=fixtures[c].input,bu=fixtures[c].bound;
        float input=runtime_float(iu),bound=runtime_float(bu),outputs[8];
        if(!actual_wide_clamp(input,bound,outputs)){++failures;break;}
        for(int lane=0;lane<width;++lane)result("wide_sym_clamp",c,fixtures[c].name,lane,width,iu,bu,bits(outputs[lane]),1);
        result("scalar_clamp",c,fixtures[c].name,0,1,iu,bu,bits(actual_scalar_clamp(input,bound)),1);
    }
    for(int c=0;c<MAPPER_PATTERNS;++c){
        volatile uint32_t u=mapper_patterns[c];
        result("bit_mapper",c,"raw_encoding_roundtrip",0,1,u,0,u,0);
    }
    int positive=zero_control_seen && positive_control_seen && (zero_control_raw&0x7fffffffu)==0 && positive_control_raw==0x3d000000u;
    int nonzero_mutant=!mapped_bits_valid(0x3d000000u,0,1);
    int subnormal_mutant=!mapped_bits_valid(0x00000001u,0,1);
    int negative_zero=sweeplabCanonicalZeroBits(0x80000000u,1)==0 && sweeplabCanonicalZeroBits(0x80000000u,0)==0x80000000u;
    if(trace && fclose(trace))io_ok=0;trace=NULL;
    int pass=results==expected_results && failures==0 && io_ok && positive && nonzero_mutant && subnormal_mutant && negative_zero && nan_preserved;
    printf("{\"schema\":\"box3d-sweeplab-v1\",\"driver\":\"operator\",\"type\":\"summary\",\"profile\":\"%s\",\"status\":\"%s\",\"execution_pass\":%s,\"trace_io_ok\":%s,\"results\":%d,\"results_expected\":%d,\"gate_failures\":%d,\"numeric_failures\":%d,\"backend_actual\":\"%s\",\"simd_actual\":%d,\"controls\":{\"positive_numeric_response\":%s,\"nonzero_mutant_rejected\":%s,\"subnormal_mutant_rejected\":%s,\"negative_zero_maps_plus\":%s,\"nan_payload_preserved\":%s},\"fp_environment_before\":",
        profile,pass?"complete":"gate_failed",truth(pass),truth(io_ok),results,expected_results,failures,numeric_failures,backend(),width,truth(positive),truth(nonzero_mutant),truth(subnormal_mutant),truth(negative_zero),truth(nan_preserved));
    print_fp_environment(register_name,control,initial_round);fputs(",\"fp_environment_after\":",stdout);
    const char* final_register;uint64_t final_control=fp_control(&final_register);
    print_fp_environment(final_register,final_control,fegetround());puts("}");
    return pass?0:5;
usage:
    fputs("Usage: sweeplab-operator --profile LABEL --workers 1 --simd 0|4|8 [--trace FILE]\n",stderr);return 64;
}
