// SPDX-License-Identifier: MIT
#pragma once
#include <stdint.h>
#include <string.h>

// EXPERIMENTAL policy at the twist-clamp output only. Integer classification
// deliberately avoids floating comparisons, which can treat subnormal input
// as zero under DAZ. Every nonzero encoding, including NaN payloads, is copied.
static inline uint32_t sweeplabCanonicalZeroBits(uint32_t bits,int mode)
{
    return mode && (bits&UINT32_C(0x7fffffff))==0 ? UINT32_C(0) : bits;
}
static inline float sweeplabCanonicalZeroFloat(float value,int mode)
{
    uint32_t bits;
    memcpy(&bits,&value,sizeof(bits));
    bits=sweeplabCanonicalZeroBits(bits,mode);
    memcpy(&value,&bits,sizeof(value));
    return value;
}
