// SPDX-License-Identifier: MIT
// Standalone SHA-256, written for this synthetic laboratory from the standard
// 32-bit SHA-256 equations. No allocation, SDK, or external library dependency.
#ifndef SNAPSHOTLAB_SHA256_H
#define SNAPSHOTLAB_SHA256_H

#include <stddef.h>
#include <stdint.h>
#include <string.h>

typedef struct SKSHA256
{
    uint32_t h[8];
    uint64_t bytes;
    size_t used;
    uint8_t block[64];
} SKSHA256;

static uint32_t sk_sha_rotr(uint32_t x, unsigned n)
{
    return (x >> n) | (x << (32u - n));
}

static void sk_sha_transform(SKSHA256* c, const uint8_t block[64])
{
    static const uint32_t k[64] = {
        0x428a2f98u,0x71374491u,0xb5c0fbcfu,0xe9b5dba5u,0x3956c25bu,0x59f111f1u,0x923f82a4u,0xab1c5ed5u,
        0xd807aa98u,0x12835b01u,0x243185beu,0x550c7dc3u,0x72be5d74u,0x80deb1feu,0x9bdc06a7u,0xc19bf174u,
        0xe49b69c1u,0xefbe4786u,0x0fc19dc6u,0x240ca1ccu,0x2de92c6fu,0x4a7484aau,0x5cb0a9dcu,0x76f988dau,
        0x983e5152u,0xa831c66du,0xb00327c8u,0xbf597fc7u,0xc6e00bf3u,0xd5a79147u,0x06ca6351u,0x14292967u,
        0x27b70a85u,0x2e1b2138u,0x4d2c6dfcu,0x53380d13u,0x650a7354u,0x766a0abbu,0x81c2c92eu,0x92722c85u,
        0xa2bfe8a1u,0xa81a664bu,0xc24b8b70u,0xc76c51a3u,0xd192e819u,0xd6990624u,0xf40e3585u,0x106aa070u,
        0x19a4c116u,0x1e376c08u,0x2748774cu,0x34b0bcb5u,0x391c0cb3u,0x4ed8aa4au,0x5b9cca4fu,0x682e6ff3u,
        0x748f82eeu,0x78a5636fu,0x84c87814u,0x8cc70208u,0x90befffau,0xa4506cebu,0xbef9a3f7u,0xc67178f2u
    };
    uint32_t w[64];
    for (unsigned i = 0; i < 16; ++i)
    {
        const uint8_t* p = block + 4u * i;
        w[i] = ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3];
    }
    for (unsigned i = 16; i < 64; ++i)
    {
        uint32_t x = w[i-15], y = w[i-2];
        uint32_t s0 = sk_sha_rotr(x,7) ^ sk_sha_rotr(x,18) ^ (x >> 3);
        uint32_t s1 = sk_sha_rotr(y,17) ^ sk_sha_rotr(y,19) ^ (y >> 10);
        w[i] = w[i-16] + s0 + w[i-7] + s1;
    }
    uint32_t a=c->h[0], b=c->h[1], d=c->h[3], e=c->h[4];
    uint32_t f=c->h[5], g=c->h[6], h=c->h[7], cc=c->h[2];
    for (unsigned i = 0; i < 64; ++i)
    {
        uint32_t s1=sk_sha_rotr(e,6)^sk_sha_rotr(e,11)^sk_sha_rotr(e,25);
        uint32_t ch=(e&f)^(~e&g);
        uint32_t t1=h+s1+ch+k[i]+w[i];
        uint32_t s0=sk_sha_rotr(a,2)^sk_sha_rotr(a,13)^sk_sha_rotr(a,22);
        uint32_t maj=(a&b)^(a&cc)^(b&cc);
        uint32_t t2=s0+maj;
        h=g; g=f; f=e; e=d+t1; d=cc; cc=b; b=a; a=t1+t2;
    }
    c->h[0]+=a; c->h[1]+=b; c->h[2]+=cc; c->h[3]+=d;
    c->h[4]+=e; c->h[5]+=f; c->h[6]+=g; c->h[7]+=h;
}

static void sk_sha_init(SKSHA256* c)
{
    static const uint32_t initial[8] = {
        0x6a09e667u,0xbb67ae85u,0x3c6ef372u,0xa54ff53au,
        0x510e527fu,0x9b05688cu,0x1f83d9abu,0x5be0cd19u
    };
    memcpy(c->h,initial,sizeof(initial));
    c->bytes=0; c->used=0;
}

static void sk_sha_update(SKSHA256* c, const void* data, size_t size)
{
    const uint8_t* p=(const uint8_t*)data;
    c->bytes+=(uint64_t)size;
    while (size != 0)
    {
        size_t take=64-c->used;
        if (take>size) take=size;
        memcpy(c->block+c->used,p,take);
        c->used+=take; p+=take; size-=take;
        if (c->used==64) { sk_sha_transform(c,c->block); c->used=0; }
    }
}

static void sk_sha_final(SKSHA256* c, uint8_t out[32])
{
    uint64_t bits=c->bytes*UINT64_C(8);
    c->block[c->used++]=0x80;
    if (c->used>56)
    {
        memset(c->block+c->used,0,64-c->used);
        sk_sha_transform(c,c->block); c->used=0;
    }
    memset(c->block+c->used,0,56-c->used);
    for (unsigned i=0;i<8;++i) c->block[63-i]=(uint8_t)(bits>>(8u*i));
    sk_sha_transform(c,c->block);
    for (unsigned i=0;i<8;++i)
    {
        out[4*i]=(uint8_t)(c->h[i]>>24); out[4*i+1]=(uint8_t)(c->h[i]>>16);
        out[4*i+2]=(uint8_t)(c->h[i]>>8); out[4*i+3]=(uint8_t)c->h[i];
    }
}

static void sk_sha256(const void* data, size_t size, uint8_t out[32])
{
    SKSHA256 c; sk_sha_init(&c); sk_sha_update(&c,data,size); sk_sha_final(&c,out);
}

static void sk_sha256_hex(const uint8_t digest[32], char out[65])
{
    static const char digits[]="0123456789abcdef";
    for (unsigned i=0;i<32;++i) { out[2*i]=digits[digest[i]>>4]; out[2*i+1]=digits[digest[i]&15]; }
    out[64]=0;
}

// Independently computed pattern vectors from Python hashlib, plus standard
// empty/abc vectors. Boundary sizes exercise both SHA padding branches and
// streaming updates that split blocks. Native runners must execute this gate.
static int sk_sha256_selftest(void)
{
    static const size_t lengths[]={0,1,55,56,63,64,65,119,120,127,128,129,255,256,257};
    static const char* expected[]={
        "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855",
        "e7cf46a078fed4fafd0b5e3aff144802b853f8ae459a4f0c14add3314b7cc3a6",
        "2900465fcb533e05a158fd2b3be0e5e3b03740d83060aa3580e0d98a96bf2384",
        "31454ff48ef36af2f08fd511bdc37d9d5855ac23e992e5ff5445cb6b7674a674",
        "5f6401b96532c36de4e65beec0409b69b1d181864c8009b7a04f43e5d56350d1",
        "94eb5de4943613fd048dc93393ab06877405faa39c11f53e9386083339833e7e",
        "fc518669b6eb4b4dd91827ecacef86689c725bd5bab888fd3b26dbb196eec954",
        "b0dc41b1a384e2f1203f0351b38fbeaafceef577ce1191d5bfc25da39f721eae",
        "5df24dd802ac26132ce608dcb5f09841eef039ee0f152acf98d26d17fe4e88e6",
        "0fe729ff19257bd6fec853acc2ea355f6b34b58e6c0f684c3e188fcdfcd9baae",
        "0aedd4856f8eba0963627336ad5144a9a7dbe12498e6066f0165fc97d8ddee4c",
        "4f1757ae4bffbae86d775b831765b75af154d52f7deaa46dd378051a2d3ad57f",
        "3c835ac0bba7147eaa568a76183d465e72ac456df24b55e01d44dc87be05a971",
        "3ef33734daae0e353f132ff5f3241d8f86ba81f851c0b9685149f079c16eb45b",
        "08570fb3dfab53bcb3e8f4ca60b7331d9c603460a45d978226e18dd019468d07"
    };
    uint8_t pattern[257],digest[32]; char text[65];
    for (size_t i=0;i<sizeof(pattern);++i) pattern[i]=(uint8_t)(i*37u+11u);
    sk_sha256("abc",3,digest); sk_sha256_hex(digest,text);
    if (strcmp(text,"ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad")!=0) return 0;
    for (size_t i=0;i<sizeof(lengths)/sizeof(lengths[0]);++i)
    {
        sk_sha256(pattern,lengths[i],digest); sk_sha256_hex(digest,text);
        if (strcmp(text,expected[i])!=0) return 0;
        SKSHA256 c; sk_sha_init(&c);
        for (size_t j=0;j<lengths[i];++j) sk_sha_update(&c,pattern+j,1);
        sk_sha_final(&c,digest); sk_sha256_hex(digest,text);
        if (strcmp(text,expected[i])!=0) return 0;
    }
    return 1;
}

#endif
