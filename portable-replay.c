// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Scene adapted from upstream test/test_recording.c::MidStreamContacts at
// https://github.com/erincatto/box3d/tree/5d83df83ab47172c2c745b44315e7538ead02397
// Only public APIs; embedded upstream hashes are not a full-state byte comparison.
#include "box3d/box3d.h"
#include "box3d/collision.h"
#include <stdio.h>
#include <string.h>

enum { frames = 128 };

static int capture(const char* path)
{
    b3WorldDef wd = b3DefaultWorldDef();
    b3WorldId world = b3CreateWorld(&wd);
    b3World_SetGravity(world, (b3Vec3){0, -10, 0});
    b3BodyDef bd = b3DefaultBodyDef();
    b3BodyId ground = b3CreateBody(world, &bd);
    b3ShapeDef sd = b3DefaultShapeDef();
    b3BoxHull floor = b3MakeBoxHull(50, 1, 50);
    b3CreateHullShape(ground, &sd, &floor.base);
    bd.type = b3_dynamicBody;
    bd.position = (b3Pos){0, 5, 0};
    b3BodyId body = b3CreateBody(world, &bd);
    b3BoxHull box = b3MakeBoxHull(0.5f, 0.5f, 0.5f);
    sd.density = 1;
    b3CreateHullShape(body, &sd, &box.base);
    for (int i = 0; i < 60; ++i) b3World_Step(world, 1.0f / 60.0f, 4);
    b3Counters seed = b3World_GetCounters(world);
    int seedAwake = b3World_GetAwakeBodyCount(world);
    b3ContactData touching;
    if (seed.contactCount < 1 || b3Body_GetContactData(body, &touching, 1) != 1 || touching.manifoldCount < 1) {
        b3DestroyWorld(world); puts("CAPTURE_GATE_FAILED reason=no_touching_manifold" ); return 5;
    }
    b3Recording* rec = b3CreateRecording(0);
    if (!rec) { b3DestroyWorld(world); return 5; }
    b3World_StartRecording(world, rec);
    int sleepFrame = seedAwake == 0 ? 0 : -1;
    for (int i = 1; i <= frames; ++i) {
        b3World_Step(world, 1.0f / 60.0f, 4);
        if (sleepFrame < 0 && b3World_GetAwakeBodyCount(world) == 0) sleepFrame = i;
    }
    b3Counters end = b3World_GetCounters(world);
    int awake = b3World_GetAwakeBodyCount(world);
    b3World_StopRecording(world);
    b3DestroyWorld(world);
    int bytes = b3Recording_GetSize(rec);
    int ok = b3ValidateReplay(b3Recording_GetData(rec), bytes, 1)
          && b3SaveRecordingToFile(rec, path);
    b3DestroyRecording(rec);
    if (!ok) { puts("CAPTURE_FAILED"); return 5; }
    printf("CAPTURE frames=%d bytes=%d seed_contacts=%d seed_awake=%d final_contacts=%d final_awake=%d sleep_frame=%d\n",
           frames, bytes, seed.contactCount, seedAwake, end.contactCount, awake, sleepFrame);
    return 0;
}

static int replay(const char* path)
{
    b3Recording* rec = b3LoadRecordingFromFile(path);
    if (!rec) { puts("LOAD_REJECTED reason=io_or_header_unknown"); return 2; }
    b3RecPlayer* player = b3CreatePlayer(b3Recording_GetData(rec), b3Recording_GetSize(rec), 1);
    if (!player) { b3DestroyRecording(rec); puts("PLAYER_REJECTED reason=header_or_snapshot_unknown"); return 3; }
    b3RecPlayerInfo info = b3RecPlayer_GetInfo(player);
    b3WorldId world = b3RecPlayer_GetWorldId(player);
    int seedContacts = b3World_GetCounters(world).contactCount;
    int seedAwake = b3World_GetAwakeBodyCount(world);
    printf("REPLAY_ACCEPTED frames=%d seed_contacts=%d seed_awake=%d\n", info.frameCount, seedContacts, seedAwake);
    int count = 0, sleepFrame = seedAwake == 0 ? 0 : -1;
    while (b3RecPlayer_StepFrame(player)) {
        ++count;
        if (sleepFrame < 0 && b3World_GetAwakeBodyCount(world) == 0) sleepFrame = count;
        if (b3RecPlayer_HasDiverged(player)) break;
    }
    int first = b3RecPlayer_GetDivergeFrame(player);
    int ok = !b3RecPlayer_HasDiverged(player) && b3RecPlayer_IsAtEnd(player)
          && info.frameCount == frames && count == frames
          && b3RecPlayer_GetFrame(player) == frames
          && info.timeStep == 1.0f / 60.0f && info.subStepCount == 4 && seedContacts > 0;
    int contacts = b3World_GetCounters(world).contactCount;
    int awake = b3World_GetAwakeBodyCount(world);
    if (ok) {
        b3RecPlayer_SeekFrame(player, frames / 2);
        world = b3RecPlayer_GetWorldId(player);
        ok = b3RecPlayer_GetFrame(player) == frames / 2 && !b3RecPlayer_HasDiverged(player);
        b3RecPlayer_SeekFrame(player, frames);
        world = b3RecPlayer_GetWorldId(player);
        ok = ok && b3RecPlayer_GetFrame(player) == frames && !b3RecPlayer_HasDiverged(player)
          && b3World_GetCounters(world).contactCount == contacts
          && b3World_GetAwakeBodyCount(world) == awake;
        first = b3RecPlayer_GetDivergeFrame(player);
    }
    int diverged = b3RecPlayer_HasDiverged(player);
    b3DestroyPlayer(player);
    if (ok) ok = b3ValidateReplay(b3Recording_GetData(rec), b3Recording_GetSize(rec), 1);
    b3DestroyRecording(rec);
    if (diverged) { printf("REPLAY_DIVERGED first_divergent_frame=%d\n", first); return 4; }
    if (!ok) { puts("REPLAY_GATE_FAILED"); return 5; }
    printf("REPLAY_PASS frames=%d first_divergent_frame=%d seek=64->128 final_contacts=%d final_awake=%d sleep_frame=%d\n",
           count, first, contacts, awake, sleepFrame);
    return 0;
}

int main(int argc, char** argv)
{
    if (argc == 3 && strcmp(argv[1], "capture") == 0) return capture(argv[2]);
    if (argc == 3 && strcmp(argv[1], "replay") == 0) return replay(argv[2]);
    fputs("Usage: portable-replay capture|replay FILE.b3rec\n", stderr);
    return 64;
}
