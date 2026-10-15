#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Generate an EXPERIMENTAL twist-clamp-output policy in pinned copies.

Mode 0 is an identity clone. Mode 1 maps only output encodings +/-0 to +0.
This is an arithmetic policy experiment, not read-only instrumentation.
The original clamp expression is evaluated exactly once at each of two sites.
Original upstream files are never written.
"""
import argparse
import hashlib
import json
from pathlib import Path

EXPECTED = {
    'contact_solver.c': '200d1d6010fb9995c06cddf8e259493dc50bbd94d45fbb9b08d9aa7e2a889e93',
    'contact_solver_wide.inl': '227c779936267ac2d46ee4c7c534b6e0c4b38c5dce8c557fa6272eade1acbd41',
    'wide4.c': '471a5e51da35fc2013da08ff970c696e31a68fbaef64a127818a74cd00f64e85',
    'wide8.c': 'e13bfec993601448ac284d8a3e1d14d496a20b354161c14c036dd20716c47b2a',
}


def one(source, anchor, replacement):
    if source.count(anchor) != 1:
        raise ValueError(f'expected unique experimental patch anchor: {anchor!r}')
    return source.replace(anchor, replacement, 1)


def generate(upstream, output, mode):
    if mode not in (0, 1):
        raise ValueError('mode must be 0 or 1')
    sources = {}
    for name, expected in EXPECTED.items():
        data = (upstream / 'src' / name).read_bytes().replace(b'\r\n', b'\n')
        if hashlib.sha256(data).hexdigest() != expected:
            raise ValueError(f'pinned upstream hash mismatch: {name}')
        sources[name] = data.decode('utf-8')
    scalar_anchor = 'constraint->twistImpulse = b3ClampFloat( oldImpulse + deltaImpulse, -maxImpulse, maxImpulse );'
    scalar = one(sources['contact_solver.c'], '#include "contact_solver.h"',
                 '#include "contact_solver.h"\n#include "canonical-zero.h"')
    scalar = one(scalar, scalar_anchor,
        'constraint->twistImpulse = sweeplabCanonicalZeroFloat( '
        f'b3ClampFloat( oldImpulse + deltaImpulse, -maxImpulse, maxImpulse ), {mode} );')
    wide = one(sources['contact_solver_wide.inl'], '#include "simd_wide.h"',
        '#include "simd_wide.h"\n#include "canonical-zero.h"\n' + '''
// EXPERIMENTAL result policy; no operand or clamp evaluation is duplicated.
static inline b3FloatW B3_WIDE(sweeplabCanonicalTwistResult)( b3FloatW value )
{
''' + ('''    return value;\n''' if mode == 0 else '''    float lanes[B3_SIMD_WIDTH];
    b3StoreW( lanes, value );
    for ( int lane = 0; lane < B3_SIMD_WIDTH; ++lane )
    {
        uint32_t bits;
        memcpy( &bits, lanes + lane, sizeof(bits) );
        bits = sweeplabCanonicalZeroBits( bits, 1 );
        memcpy( lanes + lane, &bits, sizeof(bits) );
    }
    return b3LoadW( lanes );
''') + '}\n')
    wide_anchor = 'cm->twistImpulse = b3SymClampW( b3AddW( oldImpulse, deltaImpulse ), maxLambda );'
    wide = one(wide, wide_anchor,
        'cm->twistImpulse = B3_WIDE(sweeplabCanonicalTwistResult)( '
        'b3SymClampW( b3AddW( oldImpulse, deltaImpulse ), maxLambda ) );')
    sources['contact_solver.c'] = scalar
    sources['contact_solver_wide.inl'] = wide
    # Translation units are copied so their quoted inl includes resolve here.
    output.mkdir(parents=True, exist_ok=True)
    for name, source in sources.items():
        (output / name).write_text(source, encoding='utf-8', newline='\n')
    return {name: hashlib.sha256(source.encode()).hexdigest() for name, source in sources.items()}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('upstream', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--mode', type=int, choices=(0, 1), required=True)
    args = parser.parse_args()
    print(json.dumps({'schema': 'box3d-sweeplab-zero-patch-v1', 'experimental': True,
        'mode': args.mode, 'patch_sites': 2, 'original_clamp_evaluations_per_site': 1,
        'upstream_normalized_sha256': EXPECTED,
        'generated_sha256': generate(args.upstream, args.output, args.mode)}, sort_keys=True))
