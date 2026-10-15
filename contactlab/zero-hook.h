// SPDX-License-Identifier: MIT
#pragma once
struct b3World;
struct b3Contact;
// Read-only observations. Stage 1: cached/prepared; 2: warm impulse/product;
// 3: warm angular velocity; 4: stored; 5: twist clamp operands and result;
// 6: normal max operands and result; 7: live [old,delta,bound] BEFORE clamp.
// Stage 5's fifth value is a repeated evaluation of the original add,
// solely for observation. No solver value
// is replaced. Values: stage5 [old, delta, bound, result, old+delta, -bound,
// twistSpeed, twistMass, totalTwistLimit, friction, max(-bound,input), min(...,bound)].
// The last two are diagnostic re-evaluations of helper semantics, not a claim
// that the scalar clamp implementation executes those helpers internally.
void contactlabZeroObserve(int stage, const struct b3World* world,
    const struct b3Contact* contact, int manifold, int width, int lane,
    const float* values, int count);
