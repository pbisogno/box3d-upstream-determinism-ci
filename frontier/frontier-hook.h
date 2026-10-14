// SPDX-License-Identifier: MIT
#pragma once
struct b3World;
struct b3Contact;
// 1: loaded/prepared twist; 2: twist/product used by warm start;
// 3: angular velocities after warm-start application; 4: stored twist.
// The callback is observational. It must not mutate the solver or its operands.
void frontierCacheObserve(int stage, const struct b3World* world,
    const struct b3Contact* contact, int manifold, int width, int lane,
    const float* values, int count);
