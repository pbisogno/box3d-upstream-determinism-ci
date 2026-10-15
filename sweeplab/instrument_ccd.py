#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Generate observational copies of two pinned public upstream sources.

Original input bytes are never written. Hashes use LF-normalized source;
all anchors are unique and reversible. No original arithmetic is replaced. The predicate return/caller remain intact.
The repeated threshold product is diagnostic, not a captured machine intermediate.
"""
import hashlib
import json
from pathlib import Path
import sys
EXPECTED={
    'solver.c':'57e4eeb468b3b5ec3c9f68cca4524d67799d6c93defd564c46774221caa93614',
    'contact.c':'2876bf44e85d262b87662298fab6848c04918ad5162b13fba6a17884edbe37ce',
}
def one(text,anchor,suffix):
    if text.count(anchor)!=1:
        raise ValueError('expected one anchor: '+repr(anchor))
    return text.replace(anchor,anchor+suffix,1)
def generate(upstream,output):
    sources={}
    for name,expected in EXPECTED.items():
        data=(upstream/'src'/name).read_bytes().replace(b'\r\n',b'\n')
        if hashlib.sha256(data).hexdigest()!=expected: raise ValueError('upstream mismatch: '+name)
        sources[name]=data.decode('utf-8')
    solver=sources['solver.c']
    solver=one(solver,'#include "solver.h"','\n#include "sweep-hook.h"')
    solver=one(solver,'sim->flags |= b3_isFast;',
        '\n                sweeplabCcdObserve(1, world, sim->bodyId, maxMotion, (sim->flags & b3_isBullet) != 0);')
    solver=one(solver,'stepContext->bulletBodies[bulletIndex] = simIndex;',
        '\n                    sweeplabCcdObserve(2, world, sim->bodyId, maxMotion, 1);')
    solver=one(solver,'b3World* world = continuousContext->world;',
        '\n    sweeplabCcdObserve(3, world, shapeId, 0.0f, 0);')
    solver=one(solver,'b3TOIOutput output = b3ShapeTimeOfImpact( shape, fastShape, &sweepA, &continuousContext->sweep, continuousContext->fraction );',
        '\n    sweeplabCcdObserve(4, world, shapeId, output.fraction, output.state);')
    solver=one(solver,'continuousContext->fraction = output.fraction;',
        '\n            sweeplabCcdObserve(5, world, shapeId, output.fraction, output.state);')
    solver=one(solver,'fastBodySim->flags |= b3_hadTimeOfImpact;',
        '\n        sweeplabCcdObserve(6, world, fastBodySim->bodyId, context.fraction, isBullet);')
    anchor='return maxMotion > safetyFactor * minExtent;'
    block='''
    { float sl_values[]={maxMotion,minExtent,safetyFactor,safetyFactor*minExtent};
      sweeplabCcdPredicate(10,shape->id,-1,sl_values,4); }
'''
    if solver.count(anchor)!=1:raise ValueError('shape predicate anchor')
    solver=solver.replace(anchor,block+anchor,1)
    anchor='bool isShapeFast = b3IsShapeFast( fastShape, context.centroid1, context.centroid2, rotationChord, safetyFactor );'
    solver=one(solver,anchor,'\n            sweeplabCcdPredicate(11,fastShape->id,isShapeFast,NULL,0);')
    anchor='\t\tif ( isBullet == false )\n\t\t{\n\t\t\tbool isShapeFast = b3IsShapeFast'
    if solver.count(anchor)!=1:raise ValueError('bullet bypass anchor')
    solver=solver.replace(anchor,'\t\tif (isBullet) sweeplabCcdPredicate(12,fastShape->id,-1,NULL,0);\n'+anchor,1)
    def integration(stage):
        return '''
        { float sl_values[]={h,state->deltaPosition.x,state->deltaPosition.y,state->deltaPosition.z,
            state->deltaRotation.v.x,state->deltaRotation.v.y,state->deltaRotation.v.z,state->deltaRotation.s,
            v.x,v.y,v.z,w.x,w.y,w.z};
          sweeplabCcdIntegration(STAGE,context->world,context->sims[i].bodyId,sl_values,14); }
'''.replace('STAGE',str(stage))
    anchor='\t\tstate->deltaPosition = b3MulAdd( state->deltaPosition, h, v );'
    if solver.count(anchor)!=1:raise ValueError('integration before anchor')
    solver=solver.replace(anchor,integration(13)+anchor,1)
    anchor='state->deltaRotation = b3IntegrateRotation( state->deltaRotation, b3MulSV( h, w ) );'
    solver=one(solver,anchor,integration(14))
    anchor='\t\t\t}\n\t\t}\n\t\telse\n\t\t{\n\t\t\t// Body is safe to advance and is falling asleep'
    if solver.count(anchor)!=1:raise ValueError('body fast decision anchor')
    body_observation='''\t\t\t}
            { float sl_values[]={maxMotion,sim->minExtent,safetyFactor,safetyFactor*sim->minExtent,
                maxDeltaPosition,maxVelocity,timeStep};
              sweeplabCcdBodyDecision(world,sim->bodyId,(sim->flags & b3_isFast)!=0,sl_values,7); }
\t\t}
\t\telse
\t\t{
\t\t\t// Body is safe to advance and is falling asleep'''
    solver=solver.replace(anchor,body_observation,1)
    contact=sources['contact.c']
    contact=one(contact,'#include "contact.h"','\n#include "sweep-hook.h"')
    contact=one(contact,'contact->generation = generation + 1;',
        '\n    sweeplabCcdObserve(8, world, contactId, 0.0f, contact->generation);')
    anchor='\tif ( geomManifold.pointCount == 0 )'
    if contact.count(anchor)!=1:raise ValueError('narrowphase anchor')
    contact=contact.replace(anchor,'\tsweeplabCcdObserve(7, world, contact->contactId, 0.0f, geomManifold.pointCount);\n'+anchor,1)
    # Final return is scoped to the public source update routine, not its helpers.
    start=contact.index('// Update the contact manifold and touching status.')
    prefix,tail=contact[:start],contact[start:]
    anchor='\treturn touching;'
    if tail.count(anchor)!=1:raise ValueError('touching anchor')
    tail=tail.replace(anchor,'\tsweeplabCcdObserve(9, world, contact->contactId, 0.0f, touching);\n'+anchor,1)
    sources={'solver.c':solver,'contact.c':prefix+tail}
    output.mkdir(parents=True,exist_ok=True)
    for name,text in sources.items():(output/name).write_text(text,encoding='utf-8',newline='\n')
    return {name:hashlib.sha256(text.encode()).hexdigest() for name,text in sources.items()}
if __name__=='__main__':
    if len(sys.argv)!=3:raise SystemExit('usage: instrument_ccd.py UPSTREAM OUTPUT')
    print(json.dumps({'schema':'box3d-sweeplab-instrumentation-v1',
        'upstream_normalized_sha256':EXPECTED,
        'generated_sha256':generate(Path(sys.argv[1]),Path(sys.argv[2]))},sort_keys=True))
