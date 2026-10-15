#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Generate an observational variant of four pinned upstream files.

Original sources remain untouched. Anchor and normalized-source hashes fail
closed if the upstream implementation changes. No arithmetic is replaced.
"""
import hashlib
import json
from pathlib import Path
import sys

EXPECTED = {
    'contact_solver.c': '200d1d6010fb9995c06cddf8e259493dc50bbd94d45fbb9b08d9aa7e2a889e93',
    'contact_solver_wide.inl': '227c779936267ac2d46ee4c7c534b6e0c4b38c5dce8c557fa6272eade1acbd41',
    'wide4.c': '471a5e51da35fc2013da08ff970c696e31a68fbaef64a127818a74cd00f64e85',
    'wide8.c': 'e13bfec993601448ac284d8a3e1d14d496a20b354161c14c036dd20716c47b2a',
}


def one(text, anchor, replacement):
    if text.count(anchor) != 1:
        raise ValueError(f'expected one instrumentation anchor: {anchor[:90]!r}')
    return text.replace(anchor, replacement, 1)


def wide_observe(stage, values, contacts='c->contacts', manifold='manifoldIndex'):
    arrays = '\n'.join(f'float fc_v{k}[B3_SIMD_WIDTH]; b3StoreW(fc_v{k}, {v});'
                       for k, v in enumerate(values))
    cells = ', '.join(f'fc_v{k}[fc_lane]' for k in range(len(values)))
    return '\n{\n' + arrays + f'''
        for (int fc_lane=0; fc_lane<B3_SIMD_WIDTH; ++fc_lane) {{
            const b3Contact* fc_contact = {contacts}[fc_lane];
            if (fc_contact != NULL && {manifold} < fc_contact->manifoldCount) {{
                float fc_values[] = {{{cells}}};
                contactlabZeroObserve({stage}, context->world, fc_contact, {manifold},
                    B3_SIMD_WIDTH, fc_lane, fc_values, {len(values)});
            }}
        }}
    }}\n'''


def generate(upstream, output):
    output.mkdir(parents=True, exist_ok=True)
    sources = {}
    for name, expected in EXPECTED.items():
        data = (upstream / 'src' / name).read_bytes().replace(b'\r\n', b'\n')
        if hashlib.sha256(data).hexdigest() != expected:
            raise ValueError(f'upstream source mismatch: {name}')
        sources[name] = data.decode('utf-8')

    wide = sources['contact_solver_wide.inl']
    wide = one(wide, '#include "body.h"', '#include "body.h"\n#include "zero-hook.h"')
    anchor = 'cm->twistImpulse = b3MulW( warmStartScale, twistImpulse );'
    wide = one(wide, anchor, anchor + wide_observe(1, ['twistImpulse', 'cm->twistImpulse'], 'contactLanes'))
    # Only the warm-start function receives stages 2/3.
    begin = wide.index('B3_FORCE_INLINE void b3WarmStartContactsW(')
    end = wide.index('// Solve the non-penetration constraints', begin)
    warm = wide[begin:end]
    anchor = 'b3Vec3W twistImpulse = b3MulSVW( cm->twistImpulse, normal );'
    warm = one(warm, anchor, anchor + wide_observe(2,
        ['cm->twistImpulse', 'twistImpulse.X', 'twistImpulse.Y', 'twistImpulse.Z']))
    anchor = 'b3ScatterBodies( states, c->indexA, &bA );'
    # Velocity application is per constraint; identify every owned manifold.
    after = 'for (int fc_m=0; fc_m<manifoldCount; ++fc_m)' + wide_observe(3,
        ['bA.w.X', 'bA.w.Y', 'bA.w.Z', 'bB.w.X', 'bB.w.Y', 'bB.w.Z'], manifold='fc_m')
    warm = one(warm, anchor, after + '\n' + anchor)
    wide = wide[:begin] + warm + wide[end:]
    anchor = 'm->twistImpulse = twistImpulse[lane];'
    wide = one(wide, anchor, anchor + '''
                { float fc_values[] = {m->twistImpulse};
                  contactlabZeroObserve(4, world, contact, manifoldIndex, B3_SIMD_WIDTH,
                    lane, fc_values, 1); }
''')

    scalar = sources['contact_solver.c']
    scalar = one(scalar, '#include "contact_solver.h"',
                 '#include "contact_solver.h"\n#include "zero-hook.h"')
    anchor = 'constraint->twistImpulse = warmStartScale * manifold->twistImpulse;'
    scalar = one(scalar, anchor, anchor + '''
                    { float fc_values[] = {manifold->twistImpulse, constraint->twistImpulse};
                      contactlabZeroObserve(1, world, contact, manifoldIndex, 1, 0, fc_values, 2); }
''')
    begin = scalar.index('void b3WarmStartContacts_Mesh(')
    end = scalar.index('// Merged normal and friction loops.', begin)
    warm = scalar[begin:end]
    anchor = 'b3Vec3 twistImpulse = b3MulSV( constraint->twistImpulse, normal );'
    warm = one(warm, anchor, anchor + '''
            { float fc_values[] = {constraint->twistImpulse, twistImpulse.x, twistImpulse.y, twistImpulse.z};
              contactlabZeroObserve(2, world, contactConstraint->contact, manifoldIndex, 1, 0, fc_values, 4); }
''')
    anchor = '\n\t}\n}\n\n'
    warm = one(warm, anchor, '''
        for (int fc_m=0; fc_m<manifoldCount; ++fc_m) {
            float fc_values[] = {stateA->angularVelocity.x, stateA->angularVelocity.y,
                stateA->angularVelocity.z, stateB->angularVelocity.x,
                stateB->angularVelocity.y, stateB->angularVelocity.z};
            contactlabZeroObserve(3, world, contactConstraint->contact, fc_m, 1, 0, fc_values, 6);
        }
\t}
}

''')
    scalar = scalar[:begin] + warm + scalar[end:]
    anchor = 'manifold->twistImpulse = constraint->twistImpulse;'
    scalar = one(scalar, anchor, anchor + '''
                { float fc_values[] = {manifold->twistImpulse};
                  contactlabZeroObserve(4, world, contact, manifoldIndex, 1, 0, fc_values, 1); }
''')
    # Original assignments stay byte-for-byte intact. Extra evaluations below
    # only expose operands and helper semantics; they never feed the solver.
    anchor = 'cm->twistImpulse = b3SymClampW( b3AddW( oldImpulse, deltaImpulse ), maxLambda );'
    wide = one(wide, anchor, wide_observe(7, ['oldImpulse', 'deltaImpulse', 'maxLambda']) + anchor + wide_observe(5, [
        'oldImpulse', 'deltaImpulse', 'maxLambda', 'cm->twistImpulse',
        'b3AddW( oldImpulse, deltaImpulse )', 'b3NegW( maxLambda )',
        'twistSpeed', 'cm->twistMass', 'totalTwistLimit', 'c->friction',
        'b3MaxW( b3NegW( maxLambda ), b3AddW( oldImpulse, deltaImpulse ) )',
        'b3MinW( b3MaxW( b3NegW( maxLambda ), b3AddW( oldImpulse, deltaImpulse ) ), maxLambda )']))
    begin = wide.index('B3_FORCE_INLINE void b3SolveContactsW(')
    end = wide.index('// Central twist friction.', begin)
    part = wide[begin:end]
    anchor = 'b3FloatW newImpulse = b3MaxW( b3SubW( cp->normalImpulses, negImpulse ), zeroW );'
    part = one(part, anchor, anchor + wide_observe(6, [
        'b3SubW( cp->normalImpulses, negImpulse )', 'zeroW', 'newImpulse',
        'cp->normalImpulses', 'negImpulse']))
    wide = wide[:begin] + part + wide[end:]
    anchor = 'constraint->twistImpulse = b3ClampFloat( oldImpulse + deltaImpulse, -maxImpulse, maxImpulse );'
    scalar = one(scalar, anchor, '''
                { float z_values[] = {oldImpulse, deltaImpulse, maxImpulse};
                  contactlabZeroObserve(7, world, contactConstraint->contact, j, 1, 0, z_values, 3); }
''' + anchor + '''
                { float z_values[] = {oldImpulse, deltaImpulse, maxImpulse,
                    constraint->twistImpulse, oldImpulse + deltaImpulse, -maxImpulse,
                    twistSpeed, constraint->twistMass, totalTwistLimit, friction,
                    b3MaxFloat(-maxImpulse, oldImpulse + deltaImpulse),
                    b3MinFloat(b3MaxFloat(-maxImpulse, oldImpulse + deltaImpulse), maxImpulse)};
                  contactlabZeroObserve(5, world, contactConstraint->contact, j, 1, 0, z_values, 12); }
''')
    begin = scalar.index('void b3SolveContacts_Mesh(')
    end = scalar.index('// Central twist friction', begin)
    part = scalar[begin:end]
    anchor = 'float newImpulse = b3MaxFloat( cp->normalImpulse + deltaImpulse, 0.0f );'
    part = one(part, anchor, anchor + '''
                { float z_values[] = {cp->normalImpulse + deltaImpulse, 0.0f,
                    newImpulse, cp->normalImpulse, deltaImpulse};
                  contactlabZeroObserve(6, world, contactConstraint->contact, j, 1, 0, z_values, 5); }
''')
    scalar = scalar[:begin] + part + scalar[end:]
    sources['contact_solver.c'] = scalar
    sources['contact_solver_wide.inl'] = wide
    for name, text in sources.items():
        (output / name).write_text(text, encoding='utf-8', newline='\n')
    return {name: hashlib.sha256(text.encode()).hexdigest() for name, text in sources.items()}


if __name__ == '__main__':
    if len(sys.argv) != 3:
        raise SystemExit('usage: instrument_zero.py UPSTREAM OUTPUT')
    print(json.dumps({'schema': 'box3d-contactlab-zero-instrumentation-v1',
        'upstream_normalized_sha256': EXPECTED,
        'generated_sha256': generate(Path(sys.argv[1]), Path(sys.argv[2]))}, sort_keys=True))
