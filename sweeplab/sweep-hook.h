// SPDX-License-Identifier: MIT
#ifndef SWEEPLAB_SWEEP_HOOK_H
#define SWEEPLAB_SWEEP_HOOK_H
// W1 diagnostic only. Stages1..9 copy already-calculated CCD/contact values.
void sweeplabCcdObserve(int stage,const void* world,int key,float value,int aux);
// Stage10: existing maxMotion,minExtent,safetyFactor plus a DIAGNOSTIC repeat
// of factor*extent. Original comparison/return remain untouched. Stage11 logs
// the ACTUAL existing caller boolean. Stage12 explicitly means NOT EVALUATED
// because bullet bypasses this shape predicate. Stages11/12 have zero floats.
void sweeplabCcdPredicate(int stage,int shapeId,int result,const float* values,int count);
// Stages13/14: immediately before/after ORIGINAL position integration. Values:
// h, accumulated LOCAL deltaPosition xyz, accumulated LOCAL deltaQuaternion
// xyzs, actual limited linearVelocity xyz, actual limited angularVelocity xyz.
// These are NOT world poses or hypothetical reconstructed substep positions.
void sweeplabCcdIntegration(int stage,const void* world,int bodyId,const float* values,int count);
// Stage15: actual BODY fast flag after unchanged original decision, with
// existing maxMotion,minExtent,safetyFactor, diagnostic repeated product,
// maxDeltaPosition,maxVelocity,timeStep. This is distinct from SHAPE predicate.
void sweeplabCcdBodyDecision(const void* world,int bodyId,int actualFast,const float* values,int count);
#endif
