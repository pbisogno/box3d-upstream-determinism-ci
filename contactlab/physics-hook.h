// SPDX-License-Identifier: MIT
#ifndef CONTACTLAB_PHYSICS_HOOK_H
#define CONTACTLAB_PHYSICS_HOOK_H
// Observational only. Keys/values are copied after the original calculation.
// Stage 1 fast body; 2 bullet queued; 3 CCD candidate after same-shape rejection;
// 4 TOI evaluated; 5 accepted fraction; 6 applied fraction; 7 convex narrowphase;
// 8 broadphase contact allocated; 9 final touching update.
void contactlabPhysicsObserve(int stage,const void* world,int key,float value,int aux);
#endif
