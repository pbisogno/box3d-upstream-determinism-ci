// SPDX-License-Identifier: MIT
// Generic sizeof/offsetof inspection of publicly available upstream headers.
// Internal structs describe this pinned format; they are not a supported external ABI.
// Source: erincatto/box3d at 5d83df83ab47172c2c745b44315e7538ead02397.
#include "body.h"
#include "contact.h"
#include "dynamic_tree.h"
#include "island.h"
#include "joint.h"
#include "shape.h"
#include "solver_set.h"
#include <stddef.h>
#include <stdio.h>

#define SIZE(T) printf("\"" #T "\":%zu,", sizeof(T))
#define FIELD(N) printf("{\"name\":\"" #N "\",\"offset\":%zu,\"size\":%zu},", offsetof(b3Shape, N), sizeof(((b3Shape*)0)->N))
int main(void)
{
    printf("{\"schema\":\"box3d-public-layout-v1\",\"offset_convention\":\"zero_based_bytes\",\"pointer_bytes\":%zu,\"sizes\":{", sizeof(void*));
    SIZE(b3Body); SIZE(b3BodySim); SIZE(b3BodyState); SIZE(b3Contact); SIZE(b3ContactSpec);
    SIZE(b3Joint); SIZE(b3JointSim); SIZE(b3Island); SIZE(b3IslandSim); SIZE(b3SolverSet);
    SIZE(b3TreeNode); SIZE(b3Vec3); SIZE(b3Pos); SIZE(b3AABB); SIZE(b3SurfaceMaterial);
    SIZE(b3Capsule); SIZE(b3Sphere); SIZE(b3Mesh);
    printf("\"b3Shape\":%zu},\"shape_fields\":[", sizeof(b3Shape));
    FIELD(id); FIELD(bodyId); FIELD(prevShapeId); FIELD(nextShapeId); FIELD(sensorIndex);
    FIELD(proxyKey); FIELD(type); FIELD(density); FIELD(explosionScale); FIELD(aabbMargin);
    FIELD(aabb); FIELD(localCentroid); FIELD(materialCount); FIELD(material); FIELD(materials);
    FIELD(filter); FIELD(userData); FIELD(userShape); FIELD(nameId); FIELD(generation); FIELD(flags);
    FIELD(capsule); FIELD(sphere); FIELD(hull); FIELD(mesh); FIELD(heightField);
    printf("{\"name\":\"compound\",\"offset\":%zu,\"size\":%zu}]}\n", offsetof(b3Shape, compound), sizeof(((b3Shape*)0)->compound));
    return 0;
}
