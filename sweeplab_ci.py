#!/usr/bin/env python3
"""Build and execute bounded public Box3D sweep-lab diagnostics on Actions."""
from __future__ import annotations

import argparse
import base64
import hashlib
import importlib.util
import json
import lzma
import os
import pathlib
import platform
import subprocess
import sys
import tempfile
import time
import zlib
from typing import Any

ROOT = pathlib.Path(__file__).resolve().parent
LAB = ROOT / "sweeplab"
CI_PATH = ROOT / "ci.py"
PIN = "5d83df83ab47172c2c745b44315e7538ead02397"
CI_SHA256 = "a3ade53fc8bcb6a98d0b1816c2dfcb2c999155c8b54606a68446dd4270f9aa3f"
COMPILER = {
    "windows-msvc-x64": "msvc", "linux-gcc-x64": "gcc",
    "linux-clang-arm64": "clang", "macos-clang-arm64": "clang",
    "macos-clang-x64": "clang",
}
HOST = {
    "windows-msvc-x64": ("Windows", "x86_64"), "linux-gcc-x64": ("Linux", "x86_64"),
    "linux-clang-arm64": ("Linux", "arm64"), "macos-clang-arm64": ("Darwin", "arm64"),
    "macos-clang-x64": ("Darwin", "x86_64"),
}
CCD_GATES = {
    "finite", "configuration", "trace_io", "hook_integrity", "predicate_pairing",
    "integration_complete", "coordinate_neighbors_distinct", "matched_initial_state",
}
CCD_STAGE_FIELDS = {10: 4, 11: 0, 12: 0, 13: 14, 14: 14, 15: 7}
CCD_HOOK_CHUNK_EVENTS = 32
CCD_HOOK_MAX_EVENTS_PER_CASE = 1024
CCD_HOOK_MAX_PARTS_PER_CASE = 32
CCD_HOOK_MAX_EVENTS_PER_PROFILE = 39168
OPERATOR_GATES = {"numeric_contract", "identity_bits_exact", "canonical_zero_only", "nonzero_bits_preserved"}
OPERATOR_CONTROLS = {
    "positive_numeric_response", "nonzero_mutant_rejected", "subnormal_mutant_rejected",
    "negative_zero_maps_plus", "nan_payload_preserved",
}
FP_ENV_FIELDS = {"round", "register", "raw_control", "ftz", "daz", "fz"}
ZERO_GATES = {
    "coverage", "finite", "locality", "prefix_exact", "reference_sham_exact",
    "backend", "positive_response", "save_load_exact", "instrumentation_contract",
}
OPERATOR_TRACE_FIELDS = (
    "result_index", "operator", "case_id", "width", "lane", "input_bits", "bound_bits",
    "raw_bits", "identity_bits", "canonical_bits",
)
MAX_LINE = 20_000
CHUNK_CHARS = 19_000
MAX_TRACE_RAW = 64 * 1024 * 1024
MAX_PROFILE_COMPRESSED = 8 * 1024 * 1024
MAX_PROCESS_OUTPUT = 32 * 1024 * 1024
MAX_BUILD_OUTPUT = 16 * 1024 * 1024
RUN_SCHEMA = "box3d-ci-sweeplab-run-v1"
CONFIG_SCHEMA = "box3d-ci-sweeplab-configured-v1"
DRIVER_SCHEMA = "box3d-ci-sweeplab-driver-v1"
TRACE_SCHEMA = "box3d-ci-sweeplab-trace-v1"
CHUNK_SCHEMA = "box3d-ci-sweeplab-trace-chunk-v1"


class SweeplabError(RuntimeError):
    pass


def sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def emit(row: dict[str, Any]) -> None:
    payload = json.dumps(row, sort_keys=True, separators=(",", ":"), ensure_ascii=True).encode("ascii") + b"\n"
    if len(payload) > MAX_LINE:
        raise SweeplabError("wrapper JSON record exceeds 20,000 bytes")
    sys.stdout.buffer.write(payload)
    sys.stdout.buffer.flush()


def load_ci():
    source = CI_PATH.read_bytes()
    if sha256(source) != CI_SHA256:
        raise SweeplabError("existing ci.py source hash changed")
    spec = importlib.util.spec_from_file_location("box3d_sweeplab_ci_helper", CI_PATH)
    if spec is None or spec.loader is None:
        raise SweeplabError("could not load existing public ci.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    if getattr(module, "PINNED_REVISION", None) != PIN:
        raise SweeplabError("ci.py is not pinned to the required upstream revision")
    return module


def runner_temp() -> pathlib.Path:
    if os.environ.get("GITHUB_ACTIONS", "").casefold() != "true":
        raise SweeplabError("native sweep-lab runs are allowed only on ephemeral GitHub Actions runners")
    raw = os.environ.get("RUNNER_TEMP")
    if not raw:
        raise SweeplabError("RUNNER_TEMP is required")
    root = pathlib.Path(raw).resolve(strict=True)
    if not root.is_dir():
        raise SweeplabError("RUNNER_TEMP is not a directory")
    return root


def contained(root: pathlib.Path, path: pathlib.Path) -> pathlib.Path:
    root = root.resolve(strict=True)
    result = path.resolve(strict=False)
    try:
        result.relative_to(root)
    except ValueError as exc:
        raise SweeplabError("temporary path escaped RUNNER_TEMP") from exc
    return result


def validate_args(args: argparse.Namespace, ci: Any) -> pathlib.Path:
    temp = runner_temp()
    if args.origin not in COMPILER or COMPILER[args.origin] != args.compiler:
        raise SweeplabError("origin and compiler do not match a supported profile")
    expected_system, expected_arch = HOST[args.origin]
    if platform.system() != expected_system or ci.normalized_arch(platform.machine()) != expected_arch:
        raise SweeplabError("origin does not match this runner OS and architecture")
    if args.config != "RelWithDebInfo":
        raise SweeplabError("this sweep-lab campaign uses RelWithDebInfo only")
    if args.simd not in (0, 4, 8) or args.workers != 1:
        raise SweeplabError("this campaign supports SIMD 0/4/8 and a single worker only")
    if not 1 <= args.parallel <= 8 or not 1 <= args.timeout <= 1800:
        raise SweeplabError("parallel must be 1..8 and timeout 1..1800 seconds")
    ci.validate_compiler_selection(args.compiler)
    if args.simd == 8:
        if expected_arch != "x86_64":
            raise SweeplabError("SIMD8 profiles are supported only on x86_64 runners")
        ci.require_avx2_support()
    ci.verified_source(ROOT / "upstream")
    return temp


def validate_compiler_ids(origin: str, ids: dict[str, str]) -> None:
    values = [str(ids.get(key, "")).strip().casefold() for key in ("c", "cxx")]
    if any(not value for value in values):
        raise SweeplabError("CMake did not identify both C and CXX compilers")
    if origin == "windows-msvc-x64":
        valid = all(value.startswith("msvc") for value in values)
    elif origin == "linux-gcc-x64":
        valid = all(value.startswith("gnu") for value in values)
    elif origin == "linux-clang-arm64":
        valid = all(value.startswith("clang") for value in values)
    else:
        valid = all(value.startswith(("appleclang", "clang")) for value in values)
    if not valid:
        raise SweeplabError("CMake compiler IDs do not match the selected profile")


def bounded(argv: list[str], cwd: pathlib.Path, stdout_path: pathlib.Path,
            stderr_path: pathlib.Path, timeout: int, limit: int) -> dict[str, Any]:
    started = time.monotonic()
    reason = None
    with stdout_path.open("wb") as out, stderr_path.open("wb") as err:
        try:
            proc = subprocess.Popen(argv, cwd=str(cwd), stdin=subprocess.DEVNULL,
                                    stdout=out, stderr=err, shell=False)
        except OSError as exc:
            raise SweeplabError(f"could not start {pathlib.Path(argv[0]).name}: {exc}") from exc
        deadline = started + timeout
        while proc.poll() is None:
            if time.monotonic() >= deadline:
                reason = "timeout"
                proc.kill()
                break
            if stdout_path.stat().st_size > limit or stderr_path.stat().st_size > limit:
                reason = "output_limit"
                proc.kill()
                break
            time.sleep(0.1)
        try:
            code = proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            proc.kill()
            code = proc.wait(timeout=10)
    out_size, err_size = stdout_path.stat().st_size, stderr_path.stat().st_size
    overflow = reason == "output_limit" or out_size > limit or err_size > limit
    out_data = stdout_path.read_bytes() if out_size <= limit else b""
    err_data = stderr_path.read_bytes() if err_size <= limit else b""
    return {
        "returncode": code, "timed_out": reason == "timeout", "overflow": overflow,
        "elapsed_seconds": round(time.monotonic() - started, 3),
        "stdout": out_data, "stderr": err_data, "stdout_bytes": out_size, "stderr_bytes": err_size,
        "stdout_sha256": sha256(out_data) if out_size <= limit else None,
        "stderr_sha256": sha256(err_data) if err_size <= limit else None,
    }


def phase(name: str, result: dict[str, Any]) -> None:
    emit({"schema": "box3d-ci-sweeplab-phase-v1", "phase": name,
          "returncode": result["returncode"], "timed_out": result["timed_out"],
          "output_limit": result["overflow"], "elapsed_seconds": result["elapsed_seconds"],
          "stdout_bytes": result["stdout_bytes"], "stdout_sha256": result["stdout_sha256"],
          "stderr_bytes": result["stderr_bytes"], "stderr_sha256": result["stderr_sha256"]})
    if result["returncode"] or result["timed_out"] or result["overflow"]:
        if result["stdout"]:
            sys.stdout.buffer.write(result["stdout"][-8_000:])
        if result["stderr"]:
            sys.stderr.buffer.write(result["stderr"])
        sys.stdout.flush()
        sys.stderr.flush()


def parse_jsonl(raw: bytes) -> tuple[list[dict[str, Any]], list[str]]:
    errors: list[str] = []
    try:
        text = raw.decode("utf-8", errors="strict")
    except UnicodeDecodeError as exc:
        return [], [f"stdout is not UTF-8: {exc}"]
    records: list[dict[str, Any]] = []
    for index, line in enumerate(text.splitlines(), 1):
        if not line:
            errors.append(f"blank JSONL line {index}")
            continue
        if len(line.encode("utf-8")) > MAX_LINE:
            errors.append(f"JSONL line {index} exceeds {MAX_LINE} bytes")
            continue
        try:
            row = json.loads(line, object_pairs_hook=_unique_object)
        except (json.JSONDecodeError, ValueError) as exc:
            errors.append(f"invalid JSONL line {index}: {exc}")
            continue
        if not isinstance(row, dict):
            errors.append(f"JSONL line {index} is not an object")
            continue
        records.append(row)
    return records, errors


def _unique_object(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"duplicate JSON key: {key}")
        result[key] = value
    return result


def _common_native(records: list[dict[str, Any]], schema: str, driver: str,
                   profile: str, instrumented: bool | None) -> list[str]:
    errors: list[str] = []
    if not records:
        return ["driver emitted no JSONL records"]
    if any(row.get("schema") != schema for row in records):
        errors.append("native schema mismatch")
    if any(row.get("driver") != driver or row.get("profile") != profile for row in records):
        errors.append("native driver/profile attribution mismatch")
    if instrumented is not None and any(row.get("instrumented") is not instrumented for row in records):
        errors.append("native instrumentation identity mismatch")
    if records[-1].get("type") != "summary" or sum(row.get("type") == "summary" for row in records) != 1:
        errors.append("native summary must occur exactly once as the final row")
    return errors


def parse_ccd(raw: bytes, profile: str, instrumented: bool, simd: int,
              backend: str) -> tuple[list[dict[str, Any]], list[str], str]:
    records, errors = parse_jsonl(raw)
    errors.extend(_common_native(records, "box3d-sweeplab-v1", "ccd", profile, instrumented))
    results = [row for row in records if row.get("type") == "result"]
    criteria = [row for row in records if row.get("type") == "criteria"]
    summaries = [row for row in records if row.get("type") == "summary"]
    chunks = [row for row in records if row.get("type") == "hooks"]
    if len(results) != 864 or len(criteria) != 1 or len(summaries) != 1:
        errors.append("CCD result/criteria/summary roster must be 864/1/1")
    if len(criteria) == 1:
        criterion = criteria[0]
        if not records or records[0] is not criterion:
            errors.append("CCD criteria record must precede all cases")
        if criterion.get("results_expected") != 864:
            errors.append("CCD criteria result count differs from the frozen roster")
        if criterion.get("detailed_hook_upper_bound") != CCD_HOOK_MAX_EVENTS_PER_PROFILE:
            errors.append("CCD criteria detailed-hook bound differs from the frozen contract")
        if criterion.get("trace_states_expected") != 46080:
            errors.append("CCD criteria trace-state count differs from the fixed roster")
    if any(row.get("type") not in {"criteria", "result", "hooks", "summary"} for row in records):
        errors.append("CCD output contains an unknown record type")
    if [row.get("case_id") for row in results] != list(range(864)):
        errors.append("CCD case IDs must be ordered and complete from 0 through 863")
    for row in results:
        if row.get("status") not in {"PASS", "FAIL"}:
            errors.append("CCD result has an invalid quality status")
        actual_workers = row.get("actual_workers", row.get("workers_actual"))
        if row.get("workers_requested") != 1 or actual_workers != 1:
            errors.append("CCD result worker count differs from W1")
        if row.get("simd_actual") != (8 if simd == 8 else 4) or row.get("backend_actual") != backend:
            errors.append("CCD observed SIMD width/backend differs from the requested profile")
        gates = row.get("execution_gates")
        if not isinstance(gates, dict) or set(gates) != CCD_GATES or any(value is not True for value in gates.values()):
            errors.append(f"CCD case {row.get('case_id')} failed or omitted an execution gate")
        total, parts = row.get("hooks_count"), row.get("hook_parts")
        outside = row.get("hooks_outside_window_excluded")
        if (type(total) is not int or not 0 <= total <= CCD_HOOK_MAX_EVENTS_PER_CASE or
                type(parts) is not int or not 0 <= parts <= CCD_HOOK_MAX_PARTS_PER_CASE or
                type(outside) is not int or outside < 0 or row.get("hook_overflow") is not False):
            errors.append(f"CCD case {row.get('case_id')} has invalid bounded-hook metadata")
        if (type(row.get("frames")) is not int or row.get("frames") < 1 or
                type(row.get("target_dynamic")) is not bool or
                type(row.get("ccd_enabled")) is not bool):
            errors.append(f"CCD case {row.get('case_id')} has invalid frame/target/CCD metadata")
    if len(summaries) == 1:
        summary = summaries[0]
        if (summary.get("execution_pass") is not True or summary.get("trace_io_ok") is not True or
                summary.get("activation_controls_pass") is not True or summary.get("results") != 864 or
                type(summary.get("execution_failures")) is not int or summary.get("execution_failures") != 0):
            errors.append("CCD summary execution, trace, activation, or count gate failed")
        if summary.get("status") not in {"PASS", "FAIL"}:
            errors.append("CCD summary quality status is invalid")
        passes = sum(row.get("status") == "PASS" for row in results)
        failures = sum(row.get("status") == "FAIL" for row in results)
        if summary.get("quality_passes") != passes or summary.get("quality_failures") != failures or passes + failures != 864:
            errors.append("CCD quality counts do not match the per-case results")
        if summary.get("status") != ("PASS" if failures == 0 else "FAIL"):
            errors.append("CCD summary quality status does not match its results")
        retained = sum(row.get("hooks_count", 0) for row in results
                       if type(row.get("hooks_count")) is int)
        excluded = sum(row.get("hooks_outside_window_excluded", 0) for row in results
                       if type(row.get("hooks_outside_window_excluded")) is int)
        if (summary.get("hooks_retained") != retained or
                summary.get("hooks_outside_window_excluded") != excluded or
                retained > CCD_HOOK_MAX_EVENTS_PER_PROFILE):
            errors.append("CCD summary hook totals do not reconcile with bounded per-case records")
        if summary.get("trace_states") != 46080:
            errors.append("CCD summary trace state count differs from the fixed 46,080-state roster")
    errors.extend(validate_ccd_hooks(records, results, chunks, profile, instrumented))
    quality = "pass" if results and all(row.get("status") == "PASS" for row in results) else (
        "fail" if results and all(row.get("status") in {"PASS", "FAIL"} for row in results) else "unknown")
    return records, errors, quality


def validate_ccd_hooks(records: list[dict[str, Any]], results: list[dict[str, Any]],
                        chunks: list[dict[str, Any]], profile: str,
                        instrumented: bool) -> list[str]:
    errors: list[str] = []
    positions = {row.get("case_id"): i for i, row in enumerate(records) if row.get("type") == "result"}
    later_positions: dict[int, int] = {}
    result_ids = [row.get("case_id") for row in results]
    for index, case_id in enumerate(result_ids):
        if type(case_id) is int and index + 1 < len(result_ids) and type(result_ids[index + 1]) is int:
            later_positions[case_id] = positions.get(result_ids[index + 1], len(records))
    by_case: dict[int, list[tuple[int, dict[str, Any]]]] = {}
    for position, row in enumerate(records):
        if row.get("type") != "hooks":
            continue
        case_id = row.get("case_id")
        if type(case_id) is not int or not 0 <= case_id < 864:
            errors.append("CCD hook chunk has an invalid case_id")
            continue
        by_case.setdefault(case_id, []).append((position, row))
    for result in results:
        case_id = result.get("case_id")
        if type(case_id) is not int or not 0 <= case_id < 864:
            continue
        total, parts = result.get("hooks_count"), result.get("hook_parts")
        if type(total) is not int or type(parts) is not int:
            continue
        expected_parts = (total + CCD_HOOK_CHUNK_EVENTS - 1) // CCD_HOOK_CHUNK_EVENTS
        case_chunks = by_case.get(case_id, [])
        if parts != expected_parts or len(case_chunks) != parts or len(case_chunks) > CCD_HOOK_MAX_PARTS_PER_CASE:
            errors.append(f"CCD case {case_id} hook chunk count is incomplete or inconsistent")
            continue
        if [row.get("part") for _, row in case_chunks] != list(range(parts)):
            errors.append(f"CCD case {case_id} hook parts are missing, duplicated, or reordered")
        event_offset = 0
        events_flat: list[Any] = []
        for part_index, (position, chunk) in enumerate(case_chunks):
            expected_count = min(CCD_HOOK_CHUNK_EVENTS, max(0, total - event_offset))
            events = chunk.get("hooks")
            if (chunk.get("schema") != "box3d-sweeplab-v1" or chunk.get("type") != "hooks" or
                    chunk.get("driver") != "ccd" or chunk.get("profile") != profile or
                    chunk.get("instrumented") is not instrumented or chunk.get("case_id") != case_id or
                    type(chunk.get("part")) is not int or chunk.get("part") != part_index or
                    type(chunk.get("parts")) is not int or chunk.get("parts") != parts or
                    type(chunk.get("offset")) is not int or chunk.get("offset") != event_offset or
                    type(chunk.get("count")) is not int or chunk.get("count") != expected_count or
                    type(chunk.get("total_count")) is not int or chunk.get("total_count") != total or
                    not isinstance(events, list) or len(events) != expected_count):
                errors.append(f"CCD case {case_id} hook chunk metadata/count is invalid")
                events_flat.extend(events if isinstance(events, list) else [])
            else:
                events_flat.extend(events)
            if position <= positions.get(case_id, -1):
                errors.append(f"CCD case {case_id} hook chunk precedes its result")
            if position >= later_positions.get(case_id, len(records)):
                errors.append(f"CCD case {case_id} hook chunks occur after the next result")
            if part_index and position <= case_chunks[part_index - 1][0]:
                errors.append(f"CCD case {case_id} hook chunks are out of order")
            event_offset += expected_count
        if event_offset != total:
            errors.append(f"CCD case {case_id} hook count does not reconcile")
        target_dynamic = result.get("target_dynamic") is True
        frames = result.get("frames") if type(result.get("frames")) is int else 0
        stage_rows = result.get("activation", {}).get("stages") if isinstance(result.get("activation"), dict) else None
        if (not isinstance(stage_rows, list) or len(stage_rows) != 15 or
                [stage.get("stage") if isinstance(stage, dict) else None for stage in stage_rows] != list(range(1, 16))):
            errors.append(f"CCD case {case_id} activation aggregate must enumerate stages 1..15")
            stage_counts: dict[int, int] = {}
        else:
            stage_counts = {}
            for stage_row in stage_rows:
                stage = stage_row.get("stage")
                count = stage_row.get("count")
                if (type(count) is not int or count < 0 or
                        type(stage_row.get("first_step")) is not int or
                        type(stage_row.get("last_step")) is not int):
                    errors.append(f"CCD case {case_id} has malformed stage {stage} aggregate")
                    continue
                first_step, last_step = stage_row.get("first_step"), stage_row.get("last_step")
                for field in ("min_value_bits", "max_value_bits"):
                    value = stage_row.get(field)
                    if not isinstance(value, str) or len(value) != 8 or any(
                            ch not in "0123456789abcdefABCDEF" for ch in value):
                        errors.append(f"CCD case {case_id} stage {stage} has malformed {field}")
                if count == 0 and (first_step != -1 or last_step != -1):
                    errors.append(f"CCD case {case_id} empty stage {stage} has nonempty step bounds")
                if count > 0 and not (1 <= first_step <= last_step <= frames):
                    errors.append(f"CCD case {case_id} stage {stage} has invalid step bounds")
                if type(stage_row.get("min_aux")) is not int or type(stage_row.get("max_aux")) is not int:
                    errors.append(f"CCD case {case_id} stage {stage} has malformed auxiliary bounds")
                stage_counts[stage] = count
        if stage_counts:
            stage15_total = stage_counts.get(15, -1)
            if not instrumented:
                if stage15_total != 0:
                    errors.append(f"pristine CCD case {case_id} stage15 aggregate is not empty")
            elif (frames <= 0 or stage15_total <= 0 or stage15_total % frames != 0 or
                  not 1 <= stage15_total // frames <= 2):
                errors.append(f"CCD case {case_id} stage15 aggregate is not a stable per-frame body count")
            if not result.get("ccd_enabled"):
                for stage in (1, 2, 3, 4, 5, 6, 10, 11, 12):
                    if stage_counts.get(stage) != 0:
                        errors.append(f"CCD-off case {case_id} unexpectedly observed stage {stage}")
            activation = result.get("activation", {})
            expected_activation = "unobserved" if not instrumented else (
                "active" if stage_counts.get(6, 0) else "inactive")
            if activation.get("status") != expected_activation:
                errors.append(f"CCD case {case_id} activation status conflicts with aggregate observations")
            expected_control = result.get("ccd_enabled") and (
                not target_dynamic or result.get("bullet_enabled") is True)
            if (type(activation.get("expected_control")) is not bool or
                    activation.get("expected_control") != expected_control):
                errors.append(f"CCD case {case_id} activation control expectation is inconsistent")
        if not instrumented and (total != 0 or parts != 0 or case_chunks):
            errors.append(f"pristine CCD case {case_id} emitted hook observations")
        if instrumented:
            keys_by_stage: dict[int, set[tuple[int, int, int]]] = {stage: set() for stage in CCD_STAGE_FIELDS}
            for event in events_flat:
                if not isinstance(event, dict) or set(event) != {"frame", "stage", "key", "aux", "ordinal", "bits"}:
                    errors.append(f"CCD case {case_id} contains a malformed hook event")
                    continue
                frame, stage, key, aux, ordinal, bits = (
                    event.get("frame"), event.get("stage"), event.get("key"),
                    event.get("aux"), event.get("ordinal"), event.get("bits"))
                if (any(type(value) is not int for value in (frame, stage, key, aux, ordinal)) or
                        type(bits) is not list or stage not in CCD_STAGE_FIELDS or
                        not 1 <= frame <= 8 or key not in (0, 1) or ordinal < 1 or
                        len(bits) != CCD_STAGE_FIELDS.get(stage, -1) or
                        any(not isinstance(value, str) or len(value) != 8 or
                            any(ch not in "0123456789abcdefABCDEF" for ch in value) for value in bits)):
                    errors.append(f"CCD case {case_id} hook event violates its typed field contract")
                    continue
                if stage == 11 and aux not in (0, 1):
                    errors.append(f"CCD case {case_id} actual-return hook has invalid boolean auxiliary")
                if stage == 12 and aux != -1:
                    errors.append(f"CCD case {case_id} NOT_EVALUATED hook lacks sentinel -1")
                if stage == 14 and aux not in (0, 1):
                    errors.append(f"CCD case {case_id} post-integration hook has invalid boolean auxiliary")
                if stage == 15 and (aux not in (0, 1) or ordinal != 1):
                    errors.append(f"CCD case {case_id} body-fast hook has invalid boolean or ordinal")
                if stage == 15 and not result.get("ccd_enabled") and aux != 0:
                    errors.append(f"CCD-off case {case_id} body-fast hook reported an active decision")
                if stage == 15 and not 1 <= frame <= 8:
                    errors.append(f"CCD case {case_id} stage15 event escaped the first-eight-frame window")
                keys_by_stage[stage].add((frame, key, ordinal))
            for stage in CCD_STAGE_FIELDS:
                event_count = sum(1 for event in events_flat
                                  if isinstance(event, dict) and event.get("stage") == stage)
                if len(keys_by_stage[stage]) != event_count:
                    errors.append(f"CCD case {case_id} stage {stage} contains duplicate frame/body ordinals")
            for left, right in ((10, 11), (13, 14)):
                if keys_by_stage[left] != keys_by_stage[right]:
                    errors.append(f"CCD case {case_id} stages {left}/{right} are not one-to-one paired")
            if instrumented:
                body_calls = (stage_counts.get(15, 0) // frames
                              if frames > 0 and stage_counts.get(15, 0) % frames == 0 else 0)
                detail_frames = min(result.get("frames", 0), 8)
                stage15_events = [event for event in events_flat
                                  if isinstance(event, dict) and event.get("stage") == 15]
                if len(stage15_events) != detail_frames * body_calls:
                    errors.append(f"CCD case {case_id} stage15 detail count differs from its frame window")
                detail_counts: dict[int, int] = {}
                for event in stage15_events:
                    frame = event.get("frame")
                    if type(frame) is int:
                        detail_counts[frame] = detail_counts.get(frame, 0) + 1
                if any(detail_counts.get(frame) != body_calls for frame in range(1, detail_frames + 1)):
                    errors.append(f"CCD case {case_id} stage15 calls are missing within the detail window")
                keys_by_frame = {
                    frame: {event.get("key") for event in stage15_events if event.get("frame") == frame}
                    for frame in range(1, detail_frames + 1)
                }
                if keys_by_frame and any(value != next(iter(keys_by_frame.values()))
                                         for value in list(keys_by_frame.values())[1:]):
                    errors.append(f"CCD case {case_id} stage15 body IDs changed within the detail window")
    if not instrumented and chunks:
        errors.append("pristine CCD output contains hook chunks")
    if set(by_case) - set(positions):
        errors.append("CCD hook chunk has no matching result")
    return errors


def parse_operator(raw: bytes, profile: str, simd: int,
                   backend: str) -> tuple[list[dict[str, Any]], list[str], str]:
    records, errors = parse_jsonl(raw)
    errors.extend(_common_native(records, "box3d-sweeplab-v1", "operator", profile, None))
    results = [row for row in records if row.get("type") == "result"]
    criteria = [row for row in records if row.get("type") == "criteria"]
    summaries = [row for row in records if row.get("type") == "summary"]
    expected_count = 286 if simd == 8 else 166
    if len(results) != expected_count or len(criteria) != 1 or len(summaries) != 1:
        errors.append(f"operator roster must be {expected_count} results plus one criteria and summary")
    if any(row.get("type") not in {"criteria", "result", "summary"} for row in records):
        errors.append("operator emitted an unknown record type")
    if len(criteria) == 1 and not valid_fp_environment(criteria[0].get("fp_environment")):
        errors.append("operator criteria lacks typed floating-point environment metadata")
    if len(criteria) == 1 and criteria[0].get("results_expected") != expected_count:
        errors.append("operator criteria expected-result count differs from requested SIMD width")
    if [row.get("result_index") for row in results] != list(range(expected_count)):
        errors.append("operator result indexes are incomplete, duplicated, or reordered")
    bit_fields = ("input_bits", "bound_bits", "raw_bits", "identity_bits", "canonical_bits")
    for row in results:
        if row.get("status") != "complete" or row.get("execution_pass") is not True:
            errors.append(f"operator result {row.get('result_index')} did not complete")
        gates = row.get("gates")
        if not isinstance(gates, dict) or set(gates) != OPERATOR_GATES or any(value is not True for value in gates.values()):
            errors.append(f"operator result {row.get('result_index')} failed or omitted an execution gate")
        if row.get("backend_actual") != backend or row.get("simd_actual") != (8 if simd == 8 else 4):
            errors.append("operator observed backend or SIMD width differs from requested profile")
        if (not isinstance(row.get("operator"), str) or type(row.get("case_id")) is not int or
                type(row.get("width")) is not int or row.get("width") not in {1, (8 if simd == 8 else 4)} or
                type(row.get("lane")) is not int or not 0 <= row.get("lane") < row.get("width")):
            errors.append("operator row has invalid case, lane, or width metadata")
        if type(row.get("numeric_applicable")) is not bool or type(row.get("canonical_changed")) is not bool:
            errors.append("operator row omitted numeric/canonical effect applicability")
        for field in bit_fields:
            value = row.get(field)
            if not isinstance(value, str) or len(value) != 8 or any(ch not in "0123456789abcdefABCDEF" for ch in value):
                errors.append(f"operator row has malformed {field}")
    if len(summaries) == 1:
        summary = summaries[0]
        if (summary.get("status") != "complete" or summary.get("execution_pass") is not True or
                summary.get("trace_io_ok") is not True or
                summary.get("results") != expected_count or summary.get("results_expected") != expected_count):
            errors.append("operator summary execution/trace/count gate failed")
        if (summary.get("backend_actual") != backend or
                summary.get("simd_actual") != (8 if simd == 8 else 4)):
            errors.append("operator summary backend/SIMD attribution differs from the requested profile")
        controls = summary.get("controls")
        if not isinstance(controls, dict) or set(controls) != OPERATOR_CONTROLS or any(value is not True for value in controls.values()):
            errors.append("operator control suite is incomplete or failed")
        for field in ("fp_environment_before", "fp_environment_after"):
            if not valid_fp_environment(summary.get(field)):
                errors.append(f"operator summary lacks typed {field} metadata")
    return records, errors, "pass" if not errors else "unknown"


def valid_fp_environment(value: Any) -> bool:
    if not isinstance(value, dict) or set(value) != FP_ENV_FIELDS:
        return False
    if type(value.get("round")) is not int:
        return False
    register = value.get("register")
    if register not in {"MXCSR", "FPCR", "unavailable"}:
        return False
    raw_control = value.get("raw_control")
    if (not isinstance(raw_control, str) or len(raw_control) != 16 or
            any(ch not in "0123456789abcdefABCDEF" for ch in raw_control)):
        return False
    flags = (value.get("ftz"), value.get("daz"), value.get("fz"))
    if any(item is not None and type(item) is not bool for item in flags):
        return False
    if register == "MXCSR":
        return type(value.get("ftz")) is bool and type(value.get("daz")) is bool and value.get("fz") is None
    if register == "FPCR":
        return value.get("ftz") is None and value.get("daz") is None and type(value.get("fz")) is bool
    return all(item is None for item in flags)


def validate_operator_trace(path: pathlib.Path, results: list[dict[str, Any]]) -> list[str]:
    errors: list[str] = []
    try:
        raw = path.read_bytes()
        text = raw.decode("utf-8", errors="strict")
    except (OSError, UnicodeDecodeError) as exc:
        return [f"operator projection trace is unreadable: {exc}"]
    if raw and (not raw.endswith(b"\n") or b"\r" in raw):
        errors.append("operator projection trace must be complete LF JSONL")
    lines = text.splitlines()
    if len(lines) != len(results):
        errors.append("operator projection row count differs from native results")
    for index, line in enumerate(lines):
        if len(line.encode("utf-8")) > MAX_LINE:
            errors.append(f"operator projection row {index} exceeds line limit")
            continue
        try:
            trace_row = json.loads(line, object_pairs_hook=_unique_object)
        except (json.JSONDecodeError, ValueError):
            errors.append(f"operator projection row {index} is invalid JSON")
            continue
        if not isinstance(trace_row, dict) or set(trace_row) != set(OPERATOR_TRACE_FIELDS):
            errors.append(f"operator projection row {index} has an unexpected shape")
            continue
        if index < len(results) and any(trace_row.get(key) != results[index].get(key) for key in OPERATOR_TRACE_FIELDS):
            errors.append(f"operator projection row {index} does not match native result")
    return errors


def parse_zero(raw: bytes, profile: str, simd: int,
               backend: str) -> tuple[list[dict[str, Any]], list[str]]:
    records, errors = parse_jsonl(raw)
    errors.extend(_common_native(records, "box3d-contactlab-v1", "zero", profile, False))
    results = [row for row in records if row.get("type") == "result"]
    summaries = [row for row in records if row.get("type") == "summary"]
    if len(results) != 4 or len(summaries) != 1 or any(
            row.get("type") not in {"result", "summary"} for row in records):
        errors.append("zero twin must emit exactly four results and one summary")
    if [row.get("trial") for row in results] != [1, 2, 3, 4]:
        errors.append("zero twin trial roster is incomplete or out of order")
    for row in results:
        if row.get("status") != "complete" or row.get("execution_pass") is not True:
            errors.append(f"zero twin trial {row.get('trial')} did not complete")
        gates = row.get("gates")
        if not isinstance(gates, dict) or set(gates) != ZERO_GATES or any(value is not True for value in gates.values()):
            errors.append(f"zero twin trial {row.get('trial')} has a failed execution gate")
        if (row.get("workers_requested") != 1 or row.get("workers_actual") != 1 or
                row.get("simd_requested") != simd or
                row.get("simd_actual") != (8 if simd == 8 else 4) or row.get("backend_actual") != backend):
            errors.append("zero twin observed worker/backend profile mismatch")
        if (row.get("observation_coverage") is not False or
                type(row.get("hooks_count")) is not int or row.get("hooks_count") != 0 or
                type(row.get("hook_parts")) is not int or row.get("hook_parts") != 0 or
                row.get("hook_overflow") is not False):
            errors.append("pristine zero clone reported hook coverage or invalid hook metadata")
    if len(summaries) == 1:
        summary = summaries[0]
        if (summary.get("execution_pass") is not True or summary.get("trace_io_ok") is not True or
                summary.get("trials") != 4 or summary.get("status") != "complete" or
                summary.get("quality_status") != "not_applicable" or
                summary.get("instrumented") is not False or
                summary.get("backend_actual") != backend or
                summary.get("simd_actual") != (8 if simd == 8 else 4) or
                summary.get("simd_requested") != simd):
            errors.append("zero twin summary execution/trace gate failed")
    return records, errors


def validate_ccd_trace(path: pathlib.Path, expected_states: Any) -> list[str]:
    errors: list[str] = []
    try:
        raw = path.read_bytes()
        text = raw.decode("utf-8", errors="strict")
    except (OSError, UnicodeDecodeError) as exc:
        return [f"CCD projection trace is unreadable: {exc}"]
    if raw and (not raw.endswith(b"\n") or b"\r" in raw):
        errors.append("CCD projection trace must be complete LF JSONL")
    count = 0
    last = (-1, -1)
    cases: set[int] = set()
    current_case = -1
    last_frame_by_case: dict[int, int] = {}
    for line_no, line in enumerate(text.splitlines(), 1):
        if len(line.encode("utf-8")) > MAX_LINE:
            errors.append(f"CCD projection row {line_no} exceeds line limit")
            continue
        try:
            row = json.loads(line, object_pairs_hook=_unique_object)
        except (json.JSONDecodeError, ValueError):
            errors.append(f"CCD projection row {line_no} is invalid JSON")
            continue
        if not isinstance(row, dict):
            errors.append(f"CCD projection row {line_no} is not an object")
            continue
        case_id, frame = row.get("case_id"), row.get("frame")
        if type(case_id) is not int or not 0 <= case_id < 864 or type(frame) is not int or frame < 0:
            errors.append(f"CCD projection row {line_no} has invalid case/frame")
            continue
        if (case_id, frame) <= last or (case_id != current_case and case_id in cases):
            errors.append("CCD projection case/frame order is not strictly increasing and contiguous")
        expected_frame = last_frame_by_case.get(case_id, -1) + 1
        if frame != expected_frame:
            errors.append(f"CCD projection case {case_id} frame sequence has a gap or nonzero start")
        last_frame_by_case[case_id] = frame
        last, current_case = (case_id, frame), case_id
        cases.add(case_id)
        bodies, geometry, contacts = row.get("bodies"), row.get("geometry_bits"), row.get("contacts")
        if (not isinstance(bodies, list) or len(bodies) != 2 or
                any(not isinstance(body, list) or len(body) != 13 or
                    any(not isinstance(v, str) or len(v) != 8 or
                        any(ch not in "0123456789abcdefABCDEF" for ch in v) for v in body)
                    for body in bodies)):
            errors.append(f"CCD body projection is malformed at row {line_no}")
        if (not isinstance(geometry, list) or len(geometry) != 2 or
                any(not isinstance(v, str) or len(v) != 16 or
                    any(ch not in "0123456789abcdefABCDEF" for ch in v) for v in geometry)):
            errors.append(f"CCD geometry projection is malformed at row {line_no}")
        if not isinstance(contacts, list):
            errors.append(f"CCD contacts projection is malformed at row {line_no}")
        count += 1
    if type(expected_states) is not int or expected_states != count:
        errors.append(f"CCD trace state count {count} does not match native summary {expected_states!r}")
    if cases != set(range(864)):
        errors.append("CCD projection trace does not contain all 864 cases")
    return errors


def expected_backend(origin: str, simd: int) -> str:
    if simd == 0:
        return "scalar"
    if simd == 8:
        return "avx2"
    return "sse2" if origin in ("windows-msvc-x64", "linux-gcc-x64", "macos-clang-x64") else "neon"


def trace_to_logs(path: pathlib.Path, *, temp: pathlib.Path, profile: str,
                  origin: str, driver: str, used: int) -> tuple[int, int, int, str | None, str | None]:
    try:
        resolved = path.resolve(strict=True)
        resolved.relative_to(temp.resolve(strict=True))
    except (OSError, ValueError):
        return used, 0, 0, None, "trace missing or outside RUNNER_TEMP"
    if path.is_symlink() or not resolved.is_file():
        return used, 0, 0, None, "trace is not a regular file"
    size = resolved.stat().st_size
    if not 1 <= size <= MAX_TRACE_RAW:
        return used, size, 0, None, "trace raw size outside 1..64 MiB"
    raw = resolved.read_bytes()
    if driver.startswith("zero-") and not raw.startswith(b"ZCL1"):
        return used, size, 0, sha256(raw), "zero twin trace lacks ZCL1 header"
    zbytes = zlib.compress(raw, level=9)
    xbytes = lzma.compress(raw, format=lzma.FORMAT_XZ, preset=6)
    compressed, encoding = (xbytes, "xz+base64") if len(xbytes) < len(zbytes) else (zbytes, "zlib+base64")
    if used + len(compressed) > MAX_PROFILE_COMPRESSED:
        return used, size, len(compressed), sha256(raw), "profile traces exceed 8 MiB compressed"
    encoded = base64.b64encode(compressed).decode("ascii")
    chunks = [encoded[i:i + CHUNK_CHARS] for i in range(0, len(encoded), CHUNK_CHARS)]
    emit({"schema": TRACE_SCHEMA, "profile": profile, "origin": origin, "driver": driver,
          "bytes": len(raw), "sha256": sha256(raw), "compressed_bytes": len(compressed),
          "compressed_sha256": sha256(compressed), "encoding": encoding,
          "chunk_chars": CHUNK_CHARS, "chunk_count": len(chunks)})
    for index, payload in enumerate(chunks):
        emit({"schema": CHUNK_SCHEMA, "profile": profile, "origin": origin,
              "driver": driver, "index": index, "payload": payload})
    return used + len(compressed), size, len(compressed), sha256(raw), None


def source_hashes() -> dict[str, dict[str, Any]]:
    fixed = [
        ROOT / "sweeplab_ci.py", CI_PATH, ROOT / "contactlab-zero.c", ROOT / "stress-replay.c",
        ROOT / "contactlab" / "zero-hook.h",
        LAB / "CMakeLists.txt", LAB / "sweep-hook.h", LAB / "canonical-zero.h",
        LAB / "instrument_ccd.py", LAB / "instrument_zero.py",
        ROOT / "sweeplab-ccd.c", ROOT / "sweeplab-zero.c",
        ROOT / ".github" / "workflows" / "sweeplab.yml",
    ]
    missing = [path.relative_to(ROOT).as_posix() for path in fixed if not path.is_file()]
    if missing:
        raise SweeplabError("required sweep-lab source inputs are missing: " + ", ".join(missing))
    paths = sorted(set(path.resolve(strict=True) for path in fixed), key=lambda path: path.as_posix())
    result: dict[str, dict[str, Any]] = {}
    for path in paths:
        data = path.read_bytes()
        lf_data = data.replace(b"\r\n", b"\n")
        result[path.relative_to(ROOT).as_posix()] = {
            "bytes": len(data), "raw_sha256": sha256(data), "lf_sha256": sha256(lf_data),
            "crlf_lines": data.count(b"\r\n"),
        }
    return result


def exe_path(build: pathlib.Path, target: str) -> pathlib.Path:
    suffix = ".exe" if os.name == "nt" else ""
    matches = list((build / "bin").rglob(target + suffix))
    if len(matches) != 1 or not matches[0].is_file():
        raise SweeplabError(f"expected exactly one executable for {target}")
    return matches[0].resolve(strict=True)


def target_roster(simd: int, zero_variant: bool) -> list[dict[str, Any]]:
    operator_results = 286 if simd == 8 else 166
    roster = [
        {"driver": "ccd", "target": "sweeplab-ccd", "kind": "ccd", "instrumented": True},
        {"driver": "ccd-pristine", "target": "sweeplab-ccd-pristine", "kind": "ccd", "instrumented": False},
        {"driver": "operator", "target": "sweeplab-operator", "kind": "operator",
         "instrumented": None, "results": operator_results},
    ]
    if zero_variant:
        roster.extend([
            {"driver": "zero-reference", "target": "sweeplab-zero-reference", "kind": "zero", "instrumented": False},
            {"driver": "zero-identity", "target": "sweeplab-zero-identity", "kind": "zero", "instrumented": False},
            {"driver": "zero-canonical", "target": "sweeplab-zero-canonical", "kind": "zero", "instrumented": False},
        ])
    return roster


def run_one(item: dict[str, Any], exe: pathlib.Path, scratch: pathlib.Path, temp: pathlib.Path,
            profile: str, origin: str, simd: int, timeout: int,
            compressed_used: int) -> tuple[dict[str, Any], int, list[dict[str, Any]]]:
    driver = item["driver"]
    native_profile = profile + ("/" + driver if item["kind"] == "zero" else "")
    native_driver = "zero" if item["kind"] == "zero" else item["kind"]
    trace = scratch / (driver + ".trace")
    argv = [str(exe), "--profile", native_profile, "--workers", "1",
            "--simd", str(simd), "--trace", str(trace)]
    logdir = scratch / "logs"
    logdir.mkdir(exist_ok=True)
    result = bounded(argv, ROOT, logdir / (driver + ".stdout"),
                     logdir / (driver + ".stderr"), timeout, MAX_PROCESS_OUTPUT)
    raw_stdout = result["stdout"]
    if raw_stdout:
        sys.stdout.buffer.write(raw_stdout)
        if not raw_stdout.endswith(b"\n"):
            sys.stdout.buffer.write(b"\n")
        sys.stdout.buffer.flush()
    backend = expected_backend(origin, simd)
    quality = "not_applicable"
    if item["kind"] == "ccd":
        records, errors, quality = parse_ccd(raw_stdout, native_profile, item["instrumented"], simd, backend)
    elif item["kind"] == "operator":
        records, errors, quality = parse_operator(raw_stdout, native_profile, simd, backend)
    else:
        records, errors = parse_zero(raw_stdout, native_profile, simd, backend)
    trace_error = None
    trace_raw = trace_compressed = 0
    trace_hash = None
    if result["timed_out"] or result["overflow"]:
        trace_error = "driver exceeded time or output limit"
    elif item["kind"] == "operator":
        result_rows = [row for row in records if row.get("type") == "result"]
        errors.extend(validate_operator_trace(trace, result_rows))
        if errors:
            trace_error = "operator projection trace validation failed"
        else:
            compressed_used, trace_raw, trace_compressed, trace_hash, trace_error = trace_to_logs(
                trace, temp=temp, profile=profile, origin=origin, driver=driver, used=compressed_used)
    else:
        if item["kind"] == "ccd":
            summaries = [row for row in records if row.get("type") == "summary"]
            trace_expected = summaries[0].get("trace_states") if len(summaries) == 1 else None
            errors.extend(validate_ccd_trace(trace, trace_expected))
        compressed_used, trace_raw, trace_compressed, trace_hash, trace_error = trace_to_logs(
            trace, temp=temp, profile=profile, origin=origin, driver=driver, used=compressed_used)
    statuses = [str(row.get("status", "")).upper() for row in records if row.get("type") == "result"]
    summary_rows = [row for row in records if row.get("type") == "summary"]
    summary = summary_rows[0] if len(summary_rows) == 1 else {}
    quality_fail = item["kind"] == "ccd" and quality == "fail" and not errors
    exit_ok = result["returncode"] == 0 or (quality_fail and result["returncode"] == 1)
    execution_ok = (exit_ok and not result["timed_out"] and not result["overflow"]
                    and not errors and trace_error is None)
    row = {
        "schema": DRIVER_SCHEMA, "profile": profile, "driver": driver, "target": item["target"],
        "native_driver": native_driver, "native_profile": native_profile,
        "returncode": result["returncode"], "timed_out": result["timed_out"],
        "output_limit": result["overflow"], "elapsed_seconds": result["elapsed_seconds"],
        "stdout_bytes": result["stdout_bytes"], "stdout_sha256": result["stdout_sha256"],
        "stderr_bytes": result["stderr_bytes"], "stderr_sha256": result["stderr_sha256"],
        "jsonl_records": len(records), "reported_statuses": statuses, "jsonl_errors": errors,
        "quality_gate": quality, "execution_gate": execution_ok,
        "trace_bytes": trace_raw, "trace_compressed_bytes": trace_compressed,
        "trace_sha256": trace_hash, "trace_error": trace_error,
        "native_execution_pass": summary.get("execution_pass"),
        "native_trace_io_ok": summary.get("trace_io_ok"),
        "cross_profile_comparison": "deferred_to_private_audit",
    }
    emit(row)
    if result["stderr"]:
        sys.stderr.buffer.write(result["stderr"])
        if not result["stderr"].endswith(b"\n"):
            sys.stderr.buffer.write(b"\n")
        sys.stderr.buffer.flush()
    return row, compressed_used, records


def run(args: argparse.Namespace) -> int:
    ci = load_ci()
    temp = validate_args(args, ci)
    profile = f"{args.origin}/{args.config}/simd{args.simd}/w{args.workers}"
    hashes = source_hashes()
    flags = ci.build_flags(args.config, 4)
    flags = [flag for flag in flags if not flag.startswith("-DBOX3D_DISABLE_SIMD=")]
    flags.append("-DBOX3D_DISABLE_SIMD=" + ("ON" if args.simd == 0 else "OFF"))
    flags = [flag for flag in flags if not flag.startswith("-DBOX3D_AVX2=")]
    flags.append("-DBOX3D_AVX2=" + ("ON" if args.simd == 8 else "OFF"))
    roster = target_roster(args.simd, args.zero_variant)
    with tempfile.TemporaryDirectory(prefix="box3d-sweeplab-", dir=str(temp)) as raw_tmp:
        scratch = pathlib.Path(raw_tmp).resolve(strict=True)
        contained(temp, scratch)
        build = scratch / "build"
        configure_args = ["cmake", "-S", str(LAB), "-B", str(build)] + flags
        configure_args += ci.cmake_compiler_args(args.compiler, None)
        configure_args += ["-DPython3_EXECUTABLE=" + sys.executable]
        configured = bounded(configure_args, ROOT, scratch / "configure.stdout",
                             scratch / "configure.stderr", args.timeout, MAX_BUILD_OUTPUT)
        phase("configure", configured)
        if configured["returncode"] or configured["timed_out"] or configured["overflow"]:
            emit({"schema": RUN_SCHEMA, "profile": profile, "status": "execution_failed",
                  "phase": "configure", "source_hashes": hashes})
            return 1
        compiler_ids = ci.cmake_ids(configured["stdout"].decode("utf-8", errors="replace"))
        validate_compiler_ids(args.origin, compiler_ids)
        cache_text = (build / "CMakeCache.txt").read_text(encoding="utf-8", errors="replace")
        expected_disable = "ON" if args.simd == 0 else "OFF"
        expected_avx2 = "ON" if args.simd == 8 else "OFF"
        cache_disable = next((line.split("=", 1)[1] for line in cache_text.splitlines()
                              if line.startswith("BOX3D_DISABLE_SIMD:BOOL=")), None)
        cache_avx2 = next((line.split("=", 1)[1] for line in cache_text.splitlines()
                           if line.startswith("BOX3D_AVX2:BOOL=")), None)
        cache_ok = cache_disable == expected_disable and cache_avx2 == expected_avx2
        emit({"schema": CONFIG_SCHEMA, "profile": profile, "origin": args.origin,
              "compiler": args.compiler, "config": args.config, "simd": args.simd, "workers": 1,
              "compiler_ids": compiler_ids, "box3d_disable_simd": cache_disable,
              "box3d_avx2": cache_avx2, "backend_cache_gate": cache_ok,
              "ci_helper_sha256": CI_SHA256, "upstream_pin": PIN,
              "zero_variant": args.zero_variant, "source_hashes": hashes})
        if not cache_ok:
            emit({"schema": RUN_SCHEMA, "profile": profile, "status": "execution_failed",
                  "phase": "backend_cache_verification", "source_hashes": hashes})
            return 1
        driver_rows: list[dict[str, Any]] = []
        records_by_driver: dict[str, list[dict[str, Any]]] = {}
        build_gates: dict[str, bool] = {}
        compressed = 0
        for item in roster:
            driver, target = item["driver"], item["target"]
            built = bounded(["cmake", "--build", str(build), "--config", args.config,
                             "--parallel", str(args.parallel), "--target", target],
                            ROOT, scratch / ("build-" + driver + ".stdout"),
                            scratch / ("build-" + driver + ".stderr"), args.timeout, MAX_BUILD_OUTPUT)
            phase("build-" + driver, built)
            build_ok = built["returncode"] == 0 and not built["timed_out"] and not built["overflow"]
            build_gates[driver] = build_ok
            if not build_ok:
                row = {"schema": DRIVER_SCHEMA, "profile": profile, "driver": driver,
                       "target": target, "build_gate": False, "execution_gate": False,
                       "quality_gate": "not_run", "cross_profile_comparison": "deferred_to_private_audit"}
                emit(row)
                driver_rows.append(row)
                records_by_driver[driver] = []
                continue
            try:
                executable = exe_path(build, target)
                row, compressed, records = run_one(item, executable, scratch, temp, profile,
                                                   args.origin, args.simd, args.timeout, compressed)
                row["build_gate"] = True
                records_by_driver[driver] = records
            except (SweeplabError, OSError, subprocess.SubprocessError, ValueError) as exc:
                row = {"schema": DRIVER_SCHEMA, "profile": profile, "driver": driver,
                       "target": target, "build_gate": True, "execution_gate": False,
                       "quality_gate": "unknown", "error": str(exc),
                       "cross_profile_comparison": "deferred_to_private_audit"}
                emit(row)
                records_by_driver[driver] = []
            driver_rows.append(row)
        parity = []
        for left, right, required in (
                ("ccd", "ccd-pristine", True),
                ("zero-reference", "zero-identity", args.zero_variant)):
            if not required:
                continue
            left_row = next((row for row in driver_rows if row["driver"] == left), {})
            right_row = next((row for row in driver_rows if row["driver"] == right), {})
            equal = (isinstance(left_row.get("trace_sha256"), str) and
                     left_row.get("trace_sha256") == right_row.get("trace_sha256") and
                     left_row.get("trace_bytes") == right_row.get("trace_bytes"))
            parity.append({"schema": "box3d-ci-sweeplab-parity-v1", "profile": profile,
                           "left": left, "right": right, "status": "pass" if equal else "failed",
                           "byte_exact_trace_equal": equal,
                           "left_sha256": left_row.get("trace_sha256"),
                           "right_sha256": right_row.get("trace_sha256")})
            emit(parity[-1])
        parity_ok = all(row["byte_exact_trace_equal"] for row in parity)
        execution_ok = (len(driver_rows) == len(roster) and all(build_gates.values()) and parity_ok
                        and all(row.get("execution_gate") is True for row in driver_rows))
        quality_failures = [row["driver"] for row in driver_rows if row.get("quality_gate") == "fail"]
        quality_unknown = [row["driver"] for row in driver_rows if row.get("quality_gate") == "unknown"]
        quality_ok = not quality_failures and not quality_unknown
        passed = execution_ok and quality_ok
        status = "pass" if passed else (
            "quality_failed" if execution_ok and quality_failures else "execution_failed")
        emit({"schema": RUN_SCHEMA, "profile": profile, "status": status,
              "execution_gate": execution_ok, "quality_gate": quality_ok,
              "build_gates": build_gates, "driver_gates": {
                  row["driver"]: row.get("execution_gate") is True for row in driver_rows},
              "pristine_parity_gate": parity_ok, "quality_failures": quality_failures,
              "quality_unknown": quality_unknown, "zero_variant": args.zero_variant,
              "trace_compressed_bytes": compressed,
              "trace_compressed_limit_bytes": MAX_PROFILE_COMPRESSED,
              "source_hashes": hashes, "cross_profile_comparison": "deferred_to_private_audit"})
        return 0 if passed else 1


def main() -> int:
    parser = argparse.ArgumentParser(description="Run bounded public Box3D sweep-lab profiles on Actions.")
    command = parser.add_subparsers(dest="command", required=True)
    run_parser = command.add_parser("run", help="build and run CCD/operator diagnostics")
    run_parser.add_argument("--origin", choices=tuple(COMPILER), required=True)
    run_parser.add_argument("--compiler", choices=("gcc", "clang", "msvc"), required=True)
    run_parser.add_argument("--config", choices=("RelWithDebInfo",), default="RelWithDebInfo")
    run_parser.add_argument("--simd", type=int, choices=(0, 4, 8), required=True)
    run_parser.add_argument("--workers", type=int, choices=(1,), default=1)
    run_parser.add_argument("--parallel", type=int, default=2)
    run_parser.add_argument("--timeout", type=int, default=900)
    run_parser.add_argument("--zero-variant", action="store_true")
    args = parser.parse_args()
    try:
        return run(args)
    except (SweeplabError, OSError, subprocess.SubprocessError, ValueError) as exc:
        emit({"schema": RUN_SCHEMA, "status": "execution_failed", "error": str(exc),
              "cross_profile_comparison": "deferred_to_private_audit"})
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
