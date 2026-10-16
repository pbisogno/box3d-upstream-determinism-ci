#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Generate a narrowly changed serializer copy, never modify upstream sources.

No simulation value is rounded or zero-normalized here. The geometry union in
the raw shape image is redundant: the upstream reader clears it, then reads the
separate geometry payload/registry. Padding bytes named below are not fields.
"""
import argparse
import hashlib
import json
from pathlib import Path

EXPECTED = '175bc643d64b4010b3608b46a412055a4ea154d8ad7f4494daa0d4e58bab7245'

def digest(b):
    return hashlib.sha256(b).hexdigest()

def generate(source, output):
    raw = source.read_bytes()
    original = raw.replace(b'\r\n', b'\n')
    if digest(original) != EXPECTED:
        raise SystemExit('snapshot source SHA mismatch')
    text = original.decode('utf-8')
    anchor = '\t\tb3SnapW_Bytes( buf, &shape, sizeof( b3Shape ) );'
    if text.count(anchor) != 1:
        raise SystemExit('shape raw-write anchor count mismatch')
    patch = '''
		// Snapshotlab wire experiment: only duplicated geometry and proven padding.
		_Static_assert( offsetof( b3Shape, capsule ) < sizeof( b3Shape ), "geometry union range" );
		_Static_assert( offsetof( b3Filter, categoryBits ) == 0 && offsetof( b3Filter, maskBits ) == 8 &&
		                offsetof( b3Filter, groupIndex ) == 16 && sizeof( b3Filter ) == 24, "filter layout" );
		memset( (uint8_t*)&shape + offsetof( b3Shape, capsule ), 0,
		        sizeof( b3Shape ) - offsetof( b3Shape, capsule ) );
		memset( (uint8_t*)&shape.filter + offsetof( b3Filter, groupIndex ) + sizeof( shape.filter.groupIndex ), 0,
		        sizeof( b3Filter ) - offsetof( b3Filter, groupIndex ) - sizeof( shape.filter.groupIndex ) );
'''
    output_text = text.replace(anchor, patch + anchor)
    # Exact reversal proves no unrelated arithmetic/source change in this copy.
    if output_text.replace(patch, '') != text:
        raise SystemExit('source reversal failed')
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_bytes(output_text.encode('utf-8'))
    return dict(schema='box3d-snapshotlab-source-v1', input_sha256=digest(raw),
                input_normalized_sha256=digest(original), output_sha256=digest(output.read_bytes()),
                reversal_equal=True, changes=['raw_shape_geometry_union_redundant_bytes', 'b3Filter_tail_padding_only'],
                floats_modified=False, loader_layout_gate_retained=True)

if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--source', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    print(json.dumps(generate(args.source, args.output), sort_keys=True))
