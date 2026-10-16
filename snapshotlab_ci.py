#!/usr/bin/env python3
"""Bounded snapshot transport and cost harness for the public Box3D pin."""
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
import re
import subprocess
import sys
import tempfile
import time
import zipfile
import zlib
from typing import Any

ROOT = pathlib.Path(__file__).resolve().parent
LAB = ROOT / "snapshotlab"
CI_PATH = ROOT / "ci.py"
PIN = "5d83df83ab47172c2c745b44315e7538ead02397"
CI_SHA256 = "a3ade53fc8bcb6a98d0b1816c2dfcb2c999155c8b54606a68446dd4270f9aa3f"
ORIGINS = {
    "linux-gcc-x64": {"compiler": "gcc", "system": "Linux", "arch": "x86_64"},
    "linux-clang-arm64": {"compiler": "clang", "system": "Linux", "arch": "arm64"},
    "windows-msvc-x64": {"compiler": "msvc", "system": "Windows", "arch": "x86_64"},
    "macos-clang-arm64": {"compiler": "clang", "system": "Darwin", "arch": "arm64"},
}
PRODUCER_ORIGINS = ("linux-gcc-x64", "linux-clang-arm64", "windows-msvc-x64", "macos-clang-arm64")
SMOKE_ORIGINS = ("linux-gcc-x64", "linux-clang-arm64")
NATIVE_SCHEMA = "box3d-snapshotlab-v1"
RUN_SCHEMA = "box3d-ci-snapshotlab-run-v1"
PHASE_SCHEMA = "box3d-ci-snapshotlab-phase-v1"
BUNDLE_SCHEMA = "box3d-ci-snapshotlab-bundle-v1"
BUNDLE_CHUNK_SCHEMA = "box3d-ci-snapshotlab-bundle-chunk-v1"
TRACE_SCHEMA = "box3d-ci-snapshotlab-trace-v1"
TRACE_CHUNK_SCHEMA = "box3d-ci-snapshotlab-trace-chunk-v1"
MAX_JSON_LINE = 20_000
MAX_NATIVE_OUTPUT = 32 * 1024 * 1024
MAX_BUILD_OUTPUT = 16 * 1024 * 1024
MAX_TRACE_RAW = 64 * 1024 * 1024
MAX_TRACE_COMPRESSED = 8 * 1024 * 1024
MAX_TRACE_COMPRESSED_PER_CONSUMER = 32 * 1024 * 1024
MAX_TRACE_COMPRESSED_PER_PRODUCER = 32 * 1024 * 1024
MAX_BUNDLE_RAW = 256 * 1024 * 1024
MAX_BUNDLE_COMPRESSED = 256 * 1024
MAX_BUNDLE_BASE64 = 350_000
CHUNK_CHARS = 18_000
OUTPUT_CHUNKS = 20
MAX_OUTPUT_ASCII = 400_000
MAX_ARCHIVE_FILES = 16
MAX_BODIES = (64, 256)
CHECKPOINTS = (60, 120)
POLICIES = ("stock", "wire")
HORIZON = 256


class SnapshotLabError(RuntimeError):
    pass


def sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def read_bounded_file(path: pathlib.Path, maximum: int) -> bytes:
    size = path.stat().st_size
    if size < 0 or size > maximum:
        raise SnapshotLabError(f"file exceeds its configured byte bound: {path.name}")
    with path.open("rb") as handle:
        data = handle.read(maximum + 1)
    if len(data) != size or len(data) > maximum:
        raise SnapshotLabError(f"file changed while it was being read: {path.name}")
    return data


def math_ceil_div(value: int, divisor: int) -> int:
    return (value + divisor - 1) // divisor


def emit(row: dict[str, Any]) -> None:
    payload = json.dumps(row, sort_keys=True, separators=(",", ":"), ensure_ascii=True).encode("ascii") + b"\n"
    if len(payload) > MAX_JSON_LINE:
        raise SnapshotLabError("wrapper JSON record exceeds the line limit")
    sys.stdout.buffer.write(payload)
    sys.stdout.buffer.flush()


def _unique_object(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"duplicate JSON key: {key}")
        result[key] = value
    return result


def parse_jsonl(raw: bytes) -> tuple[list[dict[str, Any]], list[str]]:
    errors: list[str] = []
    try:
        text = raw.decode("utf-8", errors="strict")
    except UnicodeDecodeError as exc:
        return [], [f"native stdout is not UTF-8: {exc}"]
    if raw and not raw.endswith(b"\n"):
        errors.append("native JSONL is missing its final newline")
    records: list[dict[str, Any]] = []
    for line_no, line in enumerate(text.splitlines(), 1):
        encoded = line.encode("utf-8")
        if not line:
            errors.append(f"blank native JSONL line {line_no}")
            continue
        if len(encoded) > MAX_JSON_LINE:
            errors.append(f"native JSONL line {line_no} exceeds the line limit")
            continue
        try:
            row = json.loads(line, object_pairs_hook=_unique_object)
        except (json.JSONDecodeError, ValueError) as exc:
            errors.append(f"invalid native JSONL line {line_no}: {exc}")
            continue
        if not isinstance(row, dict):
            errors.append(f"native JSONL line {line_no} is not an object")
            continue
        records.append(row)
    return records, errors


def runner_temp() -> pathlib.Path:
    if os.environ.get("GITHUB_ACTIONS", "").casefold() != "true":
        raise SnapshotLabError("native snapshotlab runs are allowed only on GitHub-hosted runners")
    value = os.environ.get("RUNNER_TEMP")
    if not value:
        raise SnapshotLabError("RUNNER_TEMP is required")
    path = pathlib.Path(value).resolve(strict=True)
    if not path.is_dir():
        raise SnapshotLabError("RUNNER_TEMP is not a directory")
    return path


def contained(root: pathlib.Path, path: pathlib.Path, *, strict: bool = False) -> pathlib.Path:
    root = root.resolve(strict=True)
    resolved = path.resolve(strict=strict)
    try:
        resolved.relative_to(root)
    except ValueError as exc:
        raise SnapshotLabError("temporary path escaped RUNNER_TEMP") from exc
    return resolved


def load_ci():
    raw = CI_PATH.read_bytes()
    if sha256(raw) != CI_SHA256:
        raise SnapshotLabError("frozen public ci.py hash changed")
    spec = importlib.util.spec_from_file_location("box3d_snapshotlab_ci_helper", CI_PATH)
    if spec is None or spec.loader is None:
        raise SnapshotLabError("cannot load frozen ci.py helper")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    if getattr(module, "PINNED_REVISION", None) != PIN:
        raise SnapshotLabError("ci.py pin does not match the snapshotlab upstream pin")
    return module


def validate_profile(profile: str, compiler: str, ci: Any) -> pathlib.Path:
    temp = runner_temp()
    expected = ORIGINS.get(profile)
    if expected is None or expected["compiler"] != compiler:
        raise SnapshotLabError("profile and compiler do not match a supported runner")
    if platform.system() != expected["system"] or ci.normalized_arch(platform.machine()) != expected["arch"]:
        raise SnapshotLabError("profile does not match this runner OS and architecture")
    ci.validate_compiler_selection(compiler)
    ci.verified_source(ROOT / "upstream")
    return temp


def validate_compiler_ids(profile: str, ids: dict[str, str]) -> None:
    values = [str(ids.get(k, "")).strip().casefold() for k in ("c", "cxx")]
    if not all(values):
        raise SnapshotLabError("CMake did not report both C and CXX compilers")
    if profile == "linux-gcc-x64":
        okay = all(value.startswith("gnu") for value in values)
    elif profile == "windows-msvc-x64":
        okay = all(value.startswith("msvc") for value in values)
    elif profile == "linux-clang-arm64":
        okay = all(value.startswith("clang") for value in values)
    else:
        okay = all(value.startswith(("appleclang", "clang")) for value in values)
    if not okay:
        raise SnapshotLabError("CMake compiler IDs do not match the selected profile")


def source_hashes() -> dict[str, dict[str, Any]]:
    fixed = [ROOT / "snapshotlab_ci.py", CI_PATH,
             ROOT / ".github" / "workflows" / "snapshotlab.yml",
             ROOT / "snapshotlab-transport.c", ROOT / "snapshotlab-cost.c"]
    if not LAB.is_dir():
        raise SnapshotLabError("snapshotlab source directory is missing")
    files = sorted((p for p in LAB.rglob("*") if p.is_file()), key=lambda p: p.as_posix())
    if len(files) > 100:
        raise SnapshotLabError("snapshotlab source file count exceeds the audit limit")
    all_files = fixed + files
    missing = [p.relative_to(ROOT).as_posix() for p in fixed if not p.is_file()]
    if missing:
        raise SnapshotLabError("required source files are missing: " + ", ".join(missing))
    total = sum(p.stat().st_size for p in all_files)
    if total > 16 * 1024 * 1024:
        raise SnapshotLabError("snapshotlab source set exceeds the hash audit limit")
    result: dict[str, dict[str, Any]] = {}
    for path in all_files:
        if path.is_symlink():
            raise SnapshotLabError("symbolic links are not accepted in the audited source set")
        raw = path.read_bytes()
        relative = path.relative_to(ROOT).as_posix()
        result[relative] = {
            "bytes": len(raw), "raw_sha256": sha256(raw),
            "lf_sha256": sha256(raw.replace(b"\r\n", b"\n")),
        }
    return result


def bounded(argv: list[str], cwd: pathlib.Path, stdout_path: pathlib.Path,
            stderr_path: pathlib.Path, timeout: int, limit: int) -> dict[str, Any]:
    started = time.monotonic()
    reason: str | None = None
    with stdout_path.open("wb") as out, stderr_path.open("wb") as err:
        try:
            process = subprocess.Popen(argv, cwd=str(cwd), stdin=subprocess.DEVNULL,
                                       stdout=out, stderr=err, shell=False)
        except OSError as exc:
            raise SnapshotLabError(f"could not start {pathlib.Path(argv[0]).name}: {exc}") from exc
        deadline = started + timeout
        while process.poll() is None:
            if time.monotonic() >= deadline:
                reason = "timeout"
                process.kill()
                break
            if stdout_path.stat().st_size > limit or stderr_path.stat().st_size > limit:
                reason = "output_limit"
                process.kill()
                break
            time.sleep(0.1)
        try:
            code = process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            process.kill()
            code = process.wait(timeout=10)
    out_size, err_size = stdout_path.stat().st_size, stderr_path.stat().st_size
    overflow = reason == "output_limit" or out_size > limit or err_size > limit
    out = stdout_path.read_bytes() if out_size <= limit else b""
    err = stderr_path.read_bytes() if err_size <= limit else b""
    return {
        "returncode": code, "timed_out": reason == "timeout", "overflow": overflow,
        "elapsed_seconds": round(time.monotonic() - started, 3),
        "stdout": out, "stderr": err, "stdout_bytes": out_size, "stderr_bytes": err_size,
        "stdout_sha256": sha256(out) if out_size <= limit else None,
        "stderr_sha256": sha256(err) if err_size <= limit else None,
    }


def emit_phase(name: str, result: dict[str, Any]) -> None:
    emit({"schema": PHASE_SCHEMA, "phase": name, "returncode": result["returncode"],
          "timed_out": result["timed_out"], "output_limit": result["overflow"],
          "elapsed_seconds": result["elapsed_seconds"], "stdout_bytes": result["stdout_bytes"],
          "stdout_sha256": result["stdout_sha256"], "stderr_bytes": result["stderr_bytes"],
          "stderr_sha256": result["stderr_sha256"]})
    if result["stderr"]:
        sys.stderr.buffer.write(result["stderr"])
        if not result["stderr"].endswith(b"\n"):
            sys.stderr.buffer.write(b"\n")
        sys.stderr.buffer.flush()


def forward_native_records(raw: bytes, records: list[dict[str, Any]], *,
                           profile: str, operation: str) -> None:
    if len(records) == 0:
        return
    for index, row in enumerate(records):
        emit({"schema": "box3d-ci-snapshotlab-native-record-v1", "profile": profile,
              "operation": operation, "index": index, "record": row})


def public_cost_record(row: dict[str, Any]) -> dict[str, Any]:
    """Keep measurements public while excluding any accidental budget metadata."""
    def clean(value: Any) -> Any:
        if isinstance(value, dict):
            return {key: clean(item) for key, item in value.items()
                    if "budget" not in str(key).casefold()}
        if isinstance(value, list):
            return [clean(item) for item in value]
        return value
    return clean(row)


def validate_native_records(records: list[dict[str, Any]], *, driver: str,
                            profile: str, operation: str) -> tuple[dict[str, Any] | None, list[str]]:
    errors: list[str] = []
    summaries = []
    for index, row in enumerate(records):
        if row.get("schema") != NATIVE_SCHEMA:
            errors.append(f"native record {index} has an unexpected schema")
        if row.get("driver") != driver:
            errors.append(f"native record {index} has an unexpected driver")
        if row.get("profile") != profile:
            errors.append(f"native record {index} has an unexpected profile")
        if row.get("type") == "summary":
            summaries.append(row)
    if driver == "transport":
        if len(records) != 1 or records[0].get("type") != "result":
            errors.append("transport must emit exactly one result record")
        row = records[0] if len(records) == 1 else None
        if row is not None:
            if row.get("mode") != operation:
                errors.append("transport result mode does not match the requested operation")
            if row.get("status") not in ("PASS", "FAIL"):
                errors.append("transport result status is invalid")
        return row, errors
    if driver != "cost":
        errors.append("unsupported native driver")
        return None, errors
    if len(summaries) != 1 or not records or records[-1].get("type") != "summary":
        errors.append("cost output must end with exactly one summary record")
    summary = summaries[0] if len(summaries) == 1 else None
    if sum(row.get("type") == "criteria" for row in records) != 1:
        errors.append("cost output must contain exactly one criteria record")
    if sum(row.get("type") == "result" for row in records) != 78:
        errors.append("cost output must contain exactly 78 measurement records")
    if sum(row.get("type") == "workload" for row in records) != 6:
        errors.append("cost output must contain exactly six workload records")
    allowed = {"criteria", "result", "workload", "summary"}
    if any(row.get("type") not in allowed for row in records):
        errors.append("cost output contains an unexpected record type")
    # Cost thresholds are reviewed privately. Fail closed if a native revision
    # reintroduces a public threshold or pass/fail field into emitted records.
    def has_budget_key(value: Any) -> bool:
        if isinstance(value, dict):
            return any("budget" in str(k).casefold() or has_budget_key(v) for k, v in value.items())
        if isinstance(value, list):
            return any(has_budget_key(v) for v in value)
        return False
    if any(has_budget_key(row) for row in records):
        errors.append("cost output contains a private budget field")
    if summary is not None:
        if summary.get("results") != 78 or summary.get("results_expected") != 78:
            errors.append("cost summary result counts are inconsistent")
        if summary.get("status") not in ("complete", "gate_failed"):
            errors.append("cost summary status is invalid")
    return summary, errors


def validate_seed(data: bytes, *, policy: str, bodies: int, checkpoint: int) -> list[str]:
    errors: list[str] = []
    if not 96 <= len(data) <= 64 * 1024 * 1024:
        return ["seed byte length is outside the format limit"]
    if data[:8] != b"SKSEED1\x00":
        errors.append("seed magic does not match SKSEED1")
    u32 = lambda off: int.from_bytes(data[off:off + 4], "little")
    u64 = lambda off: int.from_bytes(data[off:off + 8], "little")
    expected_policy = {"stock": 0, "wire": 1}.get(policy)
    if expected_policy is None or u32(8) != 1 or u32(12) != expected_policy:
        errors.append("seed format version or policy does not match the case")
    if u32(16) != bodies or u32(20) != checkpoint:
        errors.append("seed body count or checkpoint does not match the case")
    snapshot_bytes, registry_bytes = u64(24), u64(32)
    if snapshot_bytes < 16 or snapshot_bytes > 64 * 1024 * 1024 or registry_bytes > 64 * 1024 * 1024:
        errors.append("seed snapshot or registry length is outside the format limit")
    if 96 + snapshot_bytes + registry_bytes != len(data):
        errors.append("seed component lengths do not match the complete file")
    if u32(40) != 0x3F800000 or u32(44) != 0:
        errors.append("seed length scale or reserved field is invalid")
    if data[80:96] != bytes(16):
        errors.append("seed reserved bytes are not zero")
    envelope_digest = hashlib.sha256(data[:48] + data[80:]).digest()
    if envelope_digest != data[48:80]:
        errors.append("seed envelope SHA-256 does not match its header")
    return errors


def validate_reference(data: bytes, *, seed: bytes, policy: str,
                       bodies: int, checkpoint: int, horizon: int) -> list[str]:
    errors: list[str] = []
    if len(data) < 64:
        return ["reference is shorter than its fixed header"]
    if data[:8] != b"SKREF1\x00\x00":
        errors.append("reference magic does not match SKREF1")
    u32 = lambda off: int.from_bytes(data[off:off + 4], "little")
    expected_policy = {"stock": 0, "wire": 1}.get(policy)
    if (u32(8), u32(12), u32(16), u32(20), u32(24)) != (1, expected_policy, bodies, checkpoint, horizon):
        errors.append("reference version/policy/body/checkpoint/horizon fields do not match the case")
    count = u32(28)
    baseline = horizon - checkpoint + 1
    expected_count = baseline + 4 + 12
    if count != expected_count or len(data) != 64 + count * 160:
        errors.append("reference row count or file length is invalid")
    if data[32:64] != hashlib.sha256(seed).digest():
        errors.append("reference is bound to a different seed")
    if len(data) == 64 + count * 160 and count == expected_count:
        expected_rows = [(0, tick) for tick in range(checkpoint, horizon + 1)]
        expected_rows += [(4, tick) for tick in range(horizon - 3, horizon + 1)]
        expected_rows += [(12, tick) for tick in range(horizon - 11, horizon + 1)]
        for index, (branch, tick) in enumerate(expected_rows):
            offset = 64 + index * 160
            if int.from_bytes(data[offset:offset + 4], "little") != branch or int.from_bytes(data[offset + 4:offset + 8], "little") != tick:
                errors.append(f"reference row {index} has an unexpected branch or tick")
                break
            if data[offset + 156:offset + 160] != bytes(4):
                errors.append(f"reference row {index} has nonzero reserved data")
                break
    return errors


def validate_trace(data: bytes, *, reference: bytes, policy: str,
                   checkpoint: int, horizon: int) -> list[str]:
    """Validate the native trace framing and bind every observation to REF."""
    errors: list[str] = []
    if not 16 <= len(data) <= MAX_TRACE_RAW:
        return ["trace length is outside the format bound"]
    expected_policy = {"stock": 0, "wire": 1}.get(policy)
    if data[:8] != b"SKTRACE1" or int.from_bytes(data[8:12], "little") != 1 or int.from_bytes(data[12:16], "little") != expected_policy:
        return ["trace header does not match the SKTRACE1 version or policy"]
    expected_rows = [(0, tick) for tick in range(checkpoint, horizon + 1)]
    expected_rows += [(4, tick) for tick in range(horizon - 3, horizon + 1)]
    expected_rows += [(12, tick) for tick in range(horizon - 11, horizon + 1)]
    if len(reference) != 64 + len(expected_rows) * 160:
        return ["reference length is inconsistent with trace roster"]
    offset = 16
    for index, (branch, tick) in enumerate(expected_rows):
        if offset + 16 > len(data):
            errors.append(f"trace ended before observation {index}")
            break
        actual_branch = int.from_bytes(data[offset:offset + 4], "little")
        actual_tick = int.from_bytes(data[offset + 4:offset + 8], "little")
        payload_bytes = int.from_bytes(data[offset + 8:offset + 12], "little")
        if data[offset + 12:offset + 16] != bytes(4):
            errors.append(f"trace observation {index} has nonzero reserved data")
            break
        end = offset + 16 + payload_bytes
        if actual_branch != branch or actual_tick != tick:
            errors.append(f"trace observation {index} has an unexpected branch/tick")
            break
        if payload_bytes < 16 or end > len(data):
            errors.append(f"trace observation {index} has an invalid payload length")
            break
        ref_hash = reference[64 + index * 160 + 40:64 + index * 160 + 72]
        if hashlib.sha256(data[offset + 16:end]).digest() != ref_hash:
            errors.append(f"trace observation {index} does not match its typed reference hash")
            break
        offset = end
    if offset != len(data):
        errors.append("trace has trailing bytes or an unexpected observation count")
    return errors


def validate_transport_result(row: dict[str, Any] | None, *, operation: str,
                              profile: str, policy: str, bodies: int,
                              checkpoint: int, returncode: int) -> list[str]:
    errors: list[str] = []
    if row is None:
        return ["transport result record is missing"]
    expected = {"mode": operation, "profile": profile, "policy": policy,
                "bodies": bodies, "checkpoint": checkpoint, "horizon": HORIZON,
                "actual_workers": 1, "actual_simd": 4,
                "live_steps": HORIZON - checkpoint, "imported_recorded_frames": 0}
    for key, value in expected.items():
        if row.get(key) != value or type(row.get(key)) is not type(value):
            errors.append(f"transport result field {key} does not match the requested case")
    gates = row.get("execution_gates")
    expected_gates = {"io_and_native_calls", "sha_selftest", "configuration", "trace_io",
                      "coverage", "rollback_executed", "positive_body_and_checksum_control", "cleanup"}
    if not isinstance(gates, dict) or set(gates) != expected_gates or any(type(v) is not bool for v in gates.values()):
        errors.append("transport execution gate object is malformed")
    if type(row.get("execution_pass")) is not bool:
        errors.append("transport execution_pass is not Boolean")
    if type(row.get("corrupt_controls_rejected")) is not int or row.get("corrupt_controls_rejected") != (0 if operation == "produce" else 9):
        errors.append("transport corruption-control count is unexpected")
    for key in ("seed_roundtrip_equal", "baseline_state_equal", "baseline_observation_equal", "baseline_body_equal"):
        if type(row.get(key)) is not bool:
            errors.append(f"transport field {key} is not Boolean")
    if operation == "produce":
        if row.get("rollback_cases") != []:
            errors.append("producer unexpectedly reported consumer rollback cases")
    else:
        cases = row.get("rollback_cases")
        if not isinstance(cases, list) or len(cases) != 2:
            errors.append("consumer must report exactly two rollback cases")
        else:
            for expected_ticks, case in zip((4, 12), cases):
                if not isinstance(case, dict) or case.get("ticks") != expected_ticks:
                    errors.append("consumer rollback horizon is invalid")
                    continue
                if case.get("corrected_steps") != expected_ticks or case.get("sham_steps") != expected_ticks:
                    errors.append("consumer rollback step count is incomplete")
                for key in ("state_equal", "observation_equal", "body_equal", "sham_state_equal",
                            "sham_observation_equal", "positive_body", "positive_checksum"):
                    if type(case.get(key)) is not bool:
                        errors.append(f"consumer rollback field {key} is not Boolean")
    execution = row.get("execution_pass") is True and isinstance(gates, dict) and set(gates) == expected_gates and all(v is True for v in gates.values())
    result_status = row.get("status")
    if result_status not in ("PASS", "FAIL"):
        errors.append("transport experiment status is invalid")
    expected_rc = 0 if result_status == "PASS" and execution else 1 if result_status == "FAIL" and execution else 5
    if returncode != expected_rc:
        errors.append("transport process exit status disagrees with its execution/experiment fields")
    return errors


def validate_cost_records(records: list[dict[str, Any]], summary: dict[str, Any] | None,
                          trace_path: pathlib.Path) -> list[str]:
    errors: list[str] = []
    if not records or records[0].get("type") != "criteria" or summary is None:
        return ["cost criteria or summary record is missing"]
    criteria = records[0]
    if criteria.get("results_expected") != 78 or criteria.get("workloads_expected") != 6 or criteria.get("sample_count") != 31:
        errors.append("cost criteria roster does not match the declared workload")
    if criteria.get("policy") != "wire" or criteria.get("managed_runtime_cost_included") is not False:
        errors.append("cost criteria do not match the native wire-only workload")
    metric_order = [("step_persistent", 1), ("capture_complete", 0), ("snapshot_copy", 0),
                    ("snapshot_hash", 0), ("restore_fresh", 0)]
    metric_order += [(name, horizon) for name in ("resim_persistent", "rollback_b") for horizon in (1, 4, 8, 12)]
    results = [row for row in records if row.get("type") == "result"]
    workloads = [row for row in records if row.get("type") == "workload"]
    expected_roster = [(bodies, window, metric, horizon)
                       for bodies in (64, 256, 1024) for window in (12, 120)
                       for metric, horizon in metric_order]
    if len(results) != len(expected_roster) or len(workloads) != 6:
        errors.append("cost record roster is incomplete")
    for index, (row, expected) in enumerate(zip(results, expected_roster)):
        bodies, window, metric, horizon = expected
        if (row.get("result_index"), row.get("bodies"), row.get("window"), row.get("metric"), row.get("horizon")) != (index, bodies, window, metric, horizon):
            errors.append(f"cost result {index} is out of order or has unexpected dimensions")
            break
        samples = row.get("samples_ns")
        if not isinstance(samples, list) or len(samples) != 31 or any(type(v) is not int or v < 0 for v in samples):
            errors.append(f"cost result {index} has malformed raw timing samples")
            continue
        ordered = sorted(samples)
        if row.get("median_ns") != ordered[15] or row.get("p95_ns") != ordered[29]:
            errors.append(f"cost result {index} percentile fields do not match the raw samples")
        if row.get("execution_pass") is not True or row.get("status") != "complete":
            errors.append(f"cost result {index} failed its execution-validity gate")
        if row.get("workers_actual") != 1 or row.get("simd_actual") != 4:
            errors.append(f"cost result {index} has an unexpected runtime configuration")
        if row.get("policy") != "wire" or row.get("backend_actual") not in ("scalar", "sse2", "neon", "avx2"):
            errors.append(f"cost result {index} has an unexpected policy or backend")
    workload_roster = {(64, 12), (64, 120), (256, 12), (256, 120), (1024, 12), (1024, 120)}
    for row in workloads:
        key = (row.get("bodies"), row.get("window"))
        if key not in workload_roster or row.get("execution_pass") is not True or row.get("coverage_pass") is not True:
            errors.append("cost workload row failed or has unexpected dimensions")
            break
        workload_roster.remove(key)
        if row.get("policy") != "wire":
            errors.append("cost workload row is not the wire policy")
    if workload_roster:
        errors.append("cost workload dimensions are missing or duplicated")
    try:
        trace_records, trace_errors = parse_jsonl(read_bounded_file(trace_path, 4 * 1024 * 1024))
    except (OSError, SnapshotLabError):
        trace_records, trace_errors = [], ["cost trace file is missing"]
    errors.extend(trace_errors)
    if len(trace_records) != 78 * 31:
        errors.append("cost trace must contain all 2,418 per-sample rows")
    result_map = {(r.get("bodies"), r.get("window"), r.get("metric"), r.get("horizon")): r for r in results}
    seen_samples: set[tuple[int, int, str, int, int]] = set()
    for sample in trace_records:
        key = (sample.get("bodies"), sample.get("window"), sample.get("metric"), sample.get("horizon"))
        result = result_map.get(key)
        round_index = sample.get("round")
        raw_fields = (("ns", "samples_ns"),
                      ("engine_aligned_calls", "engine_aligned_alloc_calls"),
                      ("host_calls", "host_alloc_calls"),
                      ("known_backing_peak_bytes", "combined_known_backing_peak_bytes"))
        trace_matches = result is not None and type(round_index) is int and 0 <= round_index < 31
        if trace_matches:
            for trace_field, result_field in raw_fields:
                values = result.get(result_field)
                value = sample.get(trace_field)
                if (not isinstance(values, list) or len(values) != 31 or type(value) is not int or
                        value < 0 or type(values[round_index]) is not int or value != values[round_index]):
                    trace_matches = False
                    break
        if (sample.get("schema") != "box3d-snapshotlab-cost-trace-v1" or sample.get("type") != "sample" or
                not trace_matches):
            errors.append("cost trace sample does not match a raw measurement row")
            break
        sample_key = (*key, round_index)
        if sample_key in seen_samples:
            errors.append("cost trace contains a duplicate sample")
            break
        seen_samples.add(sample_key)
    if summary.get("execution_pass") is not True or summary.get("coverage_pass") is not True or summary.get("cleanup_pass") is not True or summary.get("clock_valid") is not True or summary.get("trace_io_ok") is not True:
        errors.append("cost summary reports an invalid execution, coverage, timer, cleanup, or trace gate")
    if summary.get("results") != 78 or summary.get("results_expected") != 78 or summary.get("policy") != "wire":
        errors.append("cost summary result count is inconsistent")
    return errors


def compress_smallest(raw: bytes) -> tuple[str, bytes]:
    candidates = [("zlib+base64", zlib.compress(raw, 9))]
    candidates.append(("xz+base64", lzma.compress(raw, format=lzma.FORMAT_XZ, preset=6)))
    return min(candidates, key=lambda item: len(item[1]))


def decompress_bounded(encoded: bytes, encoding: str, expected: int) -> bytes:
    if expected < 0 or expected > MAX_BUNDLE_RAW:
        raise SnapshotLabError("bundle decompressed length exceeds the configured bound")
    if encoding == "zlib+base64":
        decoder = zlib.decompressobj()
        raw = decoder.decompress(encoded, expected + 1)
        if decoder.unconsumed_tail or len(raw) > expected:
            raise SnapshotLabError("zlib bundle exceeds the declared size")
        raw += decoder.flush()
        if not decoder.eof or decoder.unused_data or len(raw) != expected:
            raise SnapshotLabError("zlib bundle is truncated, trailing, or wrong-sized")
        return raw
    if encoding == "xz+base64":
        decoder = lzma.LZMADecompressor(format=lzma.FORMAT_XZ, memlimit=64 * 1024 * 1024)
        raw = decoder.decompress(encoded, max_length=expected + 1)
        if not decoder.eof or decoder.unused_data or len(raw) != expected:
            raise SnapshotLabError("XZ bundle is truncated, trailing, or wrong-sized")
        return raw
    raise SnapshotLabError("unsupported bundle compression encoding")


def zip_bundle(entries: dict[str, bytes], manifest: dict[str, Any]) -> bytes:
    if len(entries) + 1 > MAX_ARCHIVE_FILES:
        raise SnapshotLabError("bundle file count exceeds the archive limit")
    raw_manifest = json.dumps(manifest, sort_keys=True, separators=(",", ":"), ensure_ascii=True).encode("ascii")
    import io
    output = io.BytesIO()
    with zipfile.ZipFile(output, "w", compression=zipfile.ZIP_STORED, allowZip64=False) as archive:
        for name, data in sorted({**entries, "manifest.json": raw_manifest}.items()):
            path = pathlib.PurePosixPath(name)
            if path.is_absolute() or ".." in path.parts or len(path.parts) != 1:
                raise SnapshotLabError("bundle entry path is not a safe basename")
            info = zipfile.ZipInfo(name, date_time=(1980, 1, 1, 0, 0, 0))
            info.compress_type = zipfile.ZIP_STORED
            info.external_attr = 0o100644 << 16
            archive.writestr(info, data)
    raw = output.getvalue()
    if len(raw) > MAX_BUNDLE_RAW:
        raise SnapshotLabError("uncompressed bundle exceeds the archive size limit")
    return raw


def safe_extract_bundle(raw: bytes, *, expected_profile: str, expected_bodies: int,
                        destination: pathlib.Path, temp: pathlib.Path) -> dict[str, Any]:
    import io
    destination = contained(temp, destination)
    with zipfile.ZipFile(io.BytesIO(raw), "r") as archive:
        infos = archive.infolist()
        if not 1 <= len(infos) <= MAX_ARCHIVE_FILES:
            raise SnapshotLabError("bundle entry count is invalid")
        names = [info.filename for info in infos]
        if len(names) != len(set(names)):
            raise SnapshotLabError("bundle contains duplicate filenames")
        total = 0
        for info in infos:
            path = pathlib.PurePosixPath(info.filename)
            mode = (info.external_attr >> 16) & 0xFFFF
            file_type = mode & 0o170000
            if (path.is_absolute() or ".." in path.parts or len(path.parts) != 1 or info.is_dir() or
                    (file_type not in (0, 0o100000))):
                raise SnapshotLabError("bundle contains an unsafe path or link")
            total += info.file_size
            if info.file_size > MAX_TRACE_RAW:
                raise SnapshotLabError("bundle member exceeds the per-file limit")
        if total > MAX_BUNDLE_RAW or "manifest.json" not in names:
            raise SnapshotLabError("bundle is over the size limit or has no manifest")
        manifest_info = archive.getinfo("manifest.json")
        if manifest_info.file_size > 1024 * 1024:
            raise SnapshotLabError("bundle manifest exceeds its size limit")
        manifest_raw = archive.read("manifest.json")
        try:
            manifest = json.loads(manifest_raw.decode("ascii"), object_pairs_hook=_unique_object)
        except (UnicodeDecodeError, json.JSONDecodeError, ValueError) as exc:
            raise SnapshotLabError(f"bundle manifest is invalid: {exc}") from exc
        if not isinstance(manifest, dict) or manifest.get("schema") != BUNDLE_SCHEMA:
            raise SnapshotLabError("bundle manifest schema is invalid")
        if manifest.get("profile") != expected_profile or manifest.get("bodies") != expected_bodies:
            raise SnapshotLabError("bundle manifest profile or body count is wrong")
        declared = manifest.get("files")
        if not isinstance(declared, list) or len(declared) != len(names) - 1:
            raise SnapshotLabError("bundle file manifest does not cover every archive member")
        expected_names: set[str] = set()
        payloads: dict[str, bytes] = {}
        for entry in declared:
            if not isinstance(entry, dict):
                raise SnapshotLabError("bundle file entry is not an object")
            name, size, digest = entry.get("name"), entry.get("bytes"), entry.get("sha256")
            if not isinstance(name, str) or name == "manifest.json" or type(size) is not int or not isinstance(digest, str):
                raise SnapshotLabError("bundle file metadata is malformed")
            if name in expected_names or name not in names:
                raise SnapshotLabError("bundle has an unexpected or duplicate file")
            info = archive.getinfo(name)
            if info.file_size > MAX_TRACE_RAW or info.file_size != size:
                raise SnapshotLabError(f"bundle member declared size is outside bounds: {name}")
            chunks: list[bytes] = []
            actual = 0
            with archive.open(info, "r") as source:
                while True:
                    piece = source.read(min(64 * 1024, size - actual + 1))
                    if not piece:
                        break
                    actual += len(piece)
                    if actual > size or actual > MAX_TRACE_RAW:
                        raise SnapshotLabError(f"bundle member expands past its declared bound: {name}")
                    chunks.append(piece)
            data = b"".join(chunks)
            if len(data) != size or sha256(data) != digest:
                raise SnapshotLabError(f"bundle member integrity mismatch: {name}")
            expected_names.add(name)
            payloads[name] = data
        if expected_names != set(names) - {"manifest.json"}:
            raise SnapshotLabError("bundle includes files absent from its manifest")
        destination.mkdir(parents=True, exist_ok=True)
        if any(destination.iterdir()):
            raise SnapshotLabError("receive destination is not empty")
        for name in sorted(expected_names):
            target = contained(destination, destination / name)
            target.write_bytes(payloads[name])
        manifest_path = contained(destination, destination / "manifest.json")
        with manifest_path.open("xb") as handle:
            handle.write(manifest_raw)
    return manifest


def write_bundle_outputs(meta: dict[str, Any], encoded: str, profile: str, bodies: int) -> None:
    if len(encoded) > MAX_BUNDLE_BASE64:
        raise SnapshotLabError("bundle base64 exceeds the 350,000-character transfer limit")
    pieces = [encoded[i:i + CHUNK_CHARS] for i in range(0, len(encoded), CHUNK_CHARS)]
    if not pieces or len(pieces) > OUTPUT_CHUNKS:
        raise SnapshotLabError("bundle requires more than twenty job-output chunks")
    manifest_text = json.dumps(meta, sort_keys=True, separators=(",", ":"), ensure_ascii=True).encode("ascii")
    output = os.environ.get("GITHUB_OUTPUT")
    if not output:
        raise SnapshotLabError("GITHUB_OUTPUT is required for producer bundle transfer")
    lines = ["bundle_meta=" + base64.b64encode(manifest_text).decode("ascii"),
             "bundle_count=" + str(len(pieces))]
    lines.extend(f"bundle_{index:02d}=" + (pieces[index] if index < len(pieces) else ".")
                 for index in range(OUTPUT_CHUNKS))
    encoded_lines = ("\n".join(lines) + "\n").encode("ascii")
    existing_bytes = pathlib.Path(output).stat().st_size if pathlib.Path(output).exists() else 0
    if existing_bytes + len(encoded_lines) > MAX_OUTPUT_ASCII:
        raise SnapshotLabError("GITHUB_OUTPUT payload exceeds the aggregate ASCII limit")
    with open(output, "ab") as handle:
        handle.write(encoded_lines)
    emit({"schema": BUNDLE_SCHEMA, "profile": profile, "bodies": bodies,
          "encoding": meta["encoding"], "compressed_bytes": meta["compressed_bytes"],
          "compressed_sha256": meta["compressed_sha256"], "archive_bytes": meta["archive_bytes"],
          "archive_sha256": meta["archive_sha256"], "base64_chars": len(encoded),
          "output_ascii_bytes": len(encoded_lines), "chunk_chars": CHUNK_CHARS,
          "chunk_count": len(pieces)})
    for index, piece in enumerate(pieces):
        emit({"schema": BUNDLE_CHUNK_SCHEMA, "profile": profile, "bodies": bodies,
              "index": index, "count": len(pieces), "payload": piece})


def emit_trace(path: pathlib.Path, *, profile: str, label: str,
               compressed_used: int) -> int:
    raw = read_bounded_file(path, MAX_TRACE_RAW)
    if not raw or len(raw) > MAX_TRACE_RAW:
        raise SnapshotLabError("trace is empty or over the per-file size limit")
    encoding, packed = compress_smallest(raw)
    if len(packed) > MAX_TRACE_COMPRESSED or compressed_used + len(packed) > MAX_TRACE_COMPRESSED_PER_CONSUMER:
        raise SnapshotLabError("compressed trace exceeds the bounded log-transfer budget")
    encoded = base64.b64encode(packed).decode("ascii")
    if len(encoded) > MAX_TRACE_COMPRESSED * 2:
        raise SnapshotLabError("encoded trace exceeds its transfer bound")
    chunks = [encoded[i:i + CHUNK_CHARS] for i in range(0, len(encoded), CHUNK_CHARS)]
    emit({"schema": TRACE_SCHEMA, "profile": profile, "case": label, "bytes": len(raw),
          "sha256": sha256(raw), "compressed_bytes": len(packed), "compressed_sha256": sha256(packed),
          "encoding": encoding, "chunk_chars": CHUNK_CHARS, "chunk_count": len(chunks)})
    for index, part in enumerate(chunks):
        emit({"schema": TRACE_CHUNK_SCHEMA, "profile": profile, "case": label,
              "index": index, "count": len(chunks), "payload": part})
    return compressed_used + len(packed)


def configure(build: pathlib.Path, profile: str, compiler: str, config: str,
              ci: Any, scratch: pathlib.Path, timeout: int) -> tuple[dict[str, str], bool]:
    flags = ci.build_flags(config, 4)
    replacements = {
        "-DBOX3D_DISABLE_SIMD=": "-DBOX3D_DISABLE_SIMD=OFF",
        "-DBOX3D_AVX2=": "-DBOX3D_AVX2=OFF",
        "-DBOX3D_DOUBLE_PRECISION=": "-DBOX3D_DOUBLE_PRECISION=OFF",
    }
    flags = [flag for flag in flags if not any(flag.startswith(key) for key in replacements)]
    flags.extend(replacements.values())
    argv = ["cmake", "-S", str(LAB), "-B", str(build), *flags]
    argv.extend(ci.cmake_compiler_args(compiler, None))
    argv.append("-DPython3_EXECUTABLE=" + sys.executable)
    result = bounded(argv, ROOT, scratch / "configure.stdout", scratch / "configure.stderr",
                     timeout, MAX_BUILD_OUTPUT)
    emit_phase("configure", result)
    if result["returncode"] or result["timed_out"] or result["overflow"]:
        return {}, False
    text = result["stdout"].decode("utf-8", errors="replace")
    ids = ci.cmake_ids(text)
    validate_compiler_ids(profile, ids)
    cache = (build / "CMakeCache.txt").read_text(encoding="utf-8", errors="strict")
    expected = {"BOX3D_DISABLE_SIMD": "OFF", "BOX3D_AVX2": "OFF", "BOX3D_DOUBLE_PRECISION": "OFF"}
    actual = {}
    for key in expected:
        match = re.search(rf"^{re.escape(key)}:BOOL=(.*)$", cache, re.MULTILINE)
        actual[key] = match.group(1).strip() if match else None
    okay = actual == expected
    emit({"schema": "box3d-ci-snapshotlab-configured-v1", "profile": profile,
          "compiler_ids": ids, "cache_options": actual, "backend_config_gate": okay,
          "config": config, "workers": 1, "simd_requested": 4, "upstream_pin": PIN})
    return ids, okay


def build_target(build: pathlib.Path, target: str, scratch: pathlib.Path,
                 config: str, parallel: int, timeout: int) -> bool:
    result = bounded(["cmake", "--build", str(build), "--config", config,
                      "--parallel", str(parallel), "--target", target], ROOT,
                     scratch / ("build-" + target + ".stdout"),
                     scratch / ("build-" + target + ".stderr"), timeout, MAX_BUILD_OUTPUT)
    emit_phase("build-" + target, result)
    return result["returncode"] == 0 and not result["timed_out"] and not result["overflow"]


def executable(build: pathlib.Path, target: str) -> pathlib.Path:
    suffix = ".exe" if os.name == "nt" else ""
    matches = list((build / "bin").rglob(target + suffix))
    if len(matches) != 1 or not matches[0].is_file():
        raise SnapshotLabError(f"expected one executable for target {target}")
    return matches[0].resolve(strict=True)


def invoke_native(program: pathlib.Path, operation: str, profile: str,
                  bodies: int, checkpoint: int, seed_path: pathlib.Path,
                  reference_path: pathlib.Path, trace_path: pathlib.Path,
                  scratch: pathlib.Path, timeout: int) -> tuple[list[dict[str, Any]], dict[str, Any] | None,
                                                                 list[str], dict[str, Any]]:
    argv = [str(program), operation, "--profile", profile, "--bodies", str(bodies),
            "--checkpoint", str(checkpoint), "--horizon", str(HORIZON),
            "--seed", str(seed_path), "--reference", str(reference_path), "--trace", str(trace_path)]
    result = bounded(argv, ROOT, scratch / ("native-" + operation + ".stdout"),
                     scratch / ("native-" + operation + ".stderr"), timeout, MAX_NATIVE_OUTPUT)
    emit_phase("native-" + operation, result)
    records, errors = parse_jsonl(result["stdout"])
    summary, schema_errors = validate_native_records(records, driver="transport", profile=profile,
                                                     operation=operation)
    errors.extend(schema_errors)
    if result["timed_out"] or result["overflow"]:
        errors.append("native command timed out or exceeded the output bound")
    forward_native_records(result["stdout"], records, profile=profile, operation=operation)
    if result["stderr"]:
        sys.stderr.buffer.write(result["stderr"])
        if not result["stderr"].endswith(b"\n"):
            sys.stderr.buffer.write(b"\n")
        sys.stderr.buffer.flush()
    return records, summary, errors, result


def parse_native_execution(record: dict[str, Any] | None, driver: str) -> bool:
    if not isinstance(record, dict) or record.get("execution_pass") is not True:
        return False
    if driver == "transport":
        gates = record.get("execution_gates")
        required = {"io_and_native_calls", "sha_selftest", "configuration", "trace_io",
                    "coverage", "rollback_executed", "positive_body_and_checksum_control", "cleanup"}
    else:
        required = {"coverage_pass": True, "cleanup_pass": True, "clock_valid": True,
                    "trace_io_ok": True}
        return all(record.get(key) is value for key, value in required.items())
    return isinstance(gates, dict) and set(gates) == required and all(type(v) is bool and v for v in gates.values())


def fail_record(command: str, profile: str | None, error: str,
                source_hashes_value: dict[str, Any] | None = None) -> None:
    emit({"schema": RUN_SCHEMA, "command": command, "profile": profile,
          "status": "execution_failed", "error": error,
          "source_hashes": source_hashes_value or {}})


def run_produce(args: argparse.Namespace) -> int:
    ci = load_ci()
    temp = validate_profile(args.profile, args.compiler, ci)
    if args.bodies not in MAX_BODIES or args.config != "RelWithDebInfo" or args.simd != 4 or args.workers != 1:
        raise SnapshotLabError("producer requires bodies 64/256, RelWithDebInfo, SIMD4 and W1")
    if not 1 <= args.parallel <= 8 or not 1 <= args.timeout <= 1800:
        raise SnapshotLabError("parallel or timeout is outside the bounded range")
    before = source_hashes()
    profile = f"{args.profile}/RelWithDebInfo/simd4/w1/n{args.bodies}"
    ok = True
    entries: dict[str, bytes] = {}
    case_results: list[dict[str, Any]] = []
    trace_compressed_bytes = 0
    with tempfile.TemporaryDirectory(prefix="box3d-snapshotlab-produce-", dir=str(temp)) as raw_scratch:
        scratch = pathlib.Path(raw_scratch).resolve(strict=True)
        contained(temp, scratch)
        build = scratch / "build"
        _, config_ok = configure(build, args.profile, args.compiler, args.config, ci, scratch, args.timeout)
        ok = config_ok
        programs: dict[str, pathlib.Path] = {}
        for policy, target in (("stock", "snapshotlab-transport-stock"),
                               ("wire", "snapshotlab-transport-wire")):
            built = config_ok and build_target(build, target, scratch, args.config, args.parallel, args.timeout)
            if built:
                try:
                    programs[policy] = executable(build, target)
                except SnapshotLabError as exc:
                    emit({"schema": PHASE_SCHEMA, "phase": "resolve-" + target,
                          "execution_gate": False, "error": str(exc)})
                    built = False
            ok = ok and built
        for policy in POLICIES:
            for checkpoint in CHECKPOINTS:
                case = f"{policy}-cp{checkpoint}"
                seed_path = scratch / (case + ".seed")
                reference_path = scratch / (case + ".ref")
                trace_path = scratch / (case + ".trace")
                if policy not in programs:
                    case_results.append({"case": case, "execution_gate": False, "error": "target unavailable"})
                    continue
                native_profile = f"{profile}/{case}"
                records, summary, errors, native = invoke_native(
                    programs[policy], "produce", native_profile, args.bodies, checkpoint,
                    seed_path, reference_path, trace_path, scratch, args.timeout)
                errors.extend(validate_transport_result(summary, operation="produce", profile=native_profile,
                                                        policy=policy, bodies=args.bodies,
                                                        checkpoint=checkpoint, returncode=native["returncode"]))
                files_ok = seed_path.is_file() and reference_path.is_file() and trace_path.is_file()
                if not files_ok:
                    errors.append("native producer did not create seed, reference, and trace files")
                file_rows = []
                if files_ok:
                    seed = read_bounded_file(seed_path, 64 * 1024 * 1024)
                    reference = read_bounded_file(reference_path, 64 * 1024 * 1024)
                    trace = read_bounded_file(trace_path, MAX_TRACE_RAW)
                    errors.extend(validate_seed(seed, policy=policy, bodies=args.bodies, checkpoint=checkpoint))
                    errors.extend(validate_reference(reference, seed=seed, policy=policy, bodies=args.bodies,
                                                     checkpoint=checkpoint, horizon=HORIZON))
                    errors.extend(validate_trace(trace, reference=reference, policy=policy,
                                                 checkpoint=checkpoint, horizon=HORIZON))
                    if (summary is not None and
                            (summary.get("seed_bytes") != len(seed) or
                             summary.get("seed_sha256") != sha256(seed) or
                             summary.get("trace_bytes") != len(trace))):
                        errors.append("native producer byte counts or seed digest do not match files")
                    if summary is not None:
                        snapshot_bytes = summary.get("seed_snapshot_bytes")
                        registry_bytes = summary.get("seed_registry_bytes")
                        if (type(snapshot_bytes) is not int or type(registry_bytes) is not int or
                                snapshot_bytes + registry_bytes + 96 != len(seed)):
                            errors.append("native producer seed component lengths do not match the file")
                    try:
                        trace_compressed_bytes = emit_trace(trace_path, profile=native_profile,
                                                            label=case, compressed_used=trace_compressed_bytes)
                    except (OSError, SnapshotLabError) as exc:
                        errors.append(f"producer trace transfer failed: {exc}")
                    for suffix, data in (("seed", seed), ("reference", reference)):
                        name = f"{case}.{suffix}"
                        entries[name] = data
                        file_rows.append({"name": name, "bytes": len(data), "sha256": sha256(data)})
                execution_gate = not errors and parse_native_execution(summary, "transport")
                ok = ok and execution_gate
                case_results.append({"case": case, "policy": policy, "checkpoint": checkpoint,
                                     "native_returncode": native["returncode"],
                                     "native_stdout_sha256": native["stdout_sha256"],
                                     "native_record_count": len(records),
                                     "native_execution_gate": parse_native_execution(summary, "transport"),
                                     "files_gate": files_ok and not errors,
                                     "execution_gate": execution_gate,
                                     "experiment_status": summary.get("status") if summary else None,
                                     "exact_state_equal": summary.get("exact_state_equal") if summary else None,
                                     "projection_equal": summary.get("projection_equal") if summary else None,
                                     "file_rows": file_rows,
                                     "trace_bytes": len(trace) if files_ok else None,
                                     "trace_sha256": sha256(trace) if files_ok else None,
                                     "errors": errors})
        after = source_hashes()
        if before != after:
            ok = False
            emit({"schema": PHASE_SCHEMA, "phase": "source-hash-stability",
                  "execution_gate": False, "error": "source hashes changed during producer execution"})
        if ok and len(entries) == 8:
            manifest = {"schema": BUNDLE_SCHEMA, "profile": args.profile, "bodies": args.bodies,
                        "upstream_pin": PIN, "native_profile": profile,
                        "cases": [row["case"] for row in case_results],
                        "files": [{"name": name, "bytes": len(data), "sha256": sha256(data)}
                                  for name, data in sorted(entries.items())]}
            archive = zip_bundle(entries, manifest)
            encoding, packed = compress_smallest(archive)
            encoded = base64.b64encode(packed).decode("ascii")
            if len(packed) > MAX_BUNDLE_COMPRESSED:
                raise SnapshotLabError("producer bundle exceeds the 256-KiB compressed transfer bound")
            if len(encoded) > MAX_BUNDLE_BASE64:
                raise SnapshotLabError("producer bundle exceeds the 350,000-character base64 bound")
            meta = {"schema": BUNDLE_SCHEMA, "profile": args.profile, "bodies": args.bodies,
                    "encoding": encoding, "archive_bytes": len(archive), "archive_sha256": sha256(archive),
                    "compressed_bytes": len(packed), "compressed_sha256": sha256(packed),
                    "base64_chars": len(encoded), "chunk_chars": CHUNK_CHARS,
                    "chunk_count": (len(encoded) + CHUNK_CHARS - 1) // CHUNK_CHARS}
            write_bundle_outputs(meta, encoded, args.profile, args.bodies)
        emit({"schema": RUN_SCHEMA, "command": "produce", "profile": args.profile,
              "bodies": args.bodies, "status": "complete" if ok else "execution_failed",
              "execution_gate": ok, "case_results": case_results,
              "bundle_created": ok and len(entries) == 8,
              "trace_compressed_bytes": trace_compressed_bytes,
              "trace_compressed_limit_bytes": MAX_TRACE_COMPRESSED_PER_PRODUCER,
              "source_hashes": before, "upstream_pin": PIN})
    return 0 if ok else 1


def run_receive(args: argparse.Namespace) -> int:
    temp = runner_temp()
    if args.source_profile not in PRODUCER_ORIGINS or args.bodies not in MAX_BODIES:
        raise SnapshotLabError("unsupported producer profile or body count")
    meta_encoded = os.environ.get("SNAPSHOTLAB_BUNDLE_META", "")
    try:
        meta_raw = base64.b64decode(meta_encoded, validate=True)
        meta = json.loads(meta_raw.decode("ascii"), object_pairs_hook=_unique_object)
    except (ValueError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise SnapshotLabError(f"bundle metadata is invalid: {exc}") from exc
    if not isinstance(meta, dict) or meta.get("schema") != BUNDLE_SCHEMA:
        raise SnapshotLabError("bundle metadata schema is invalid")
    if meta.get("profile") != args.source_profile or meta.get("bodies") != args.bodies:
        raise SnapshotLabError("bundle metadata does not match its declared producer")
    count = meta.get("chunk_count")
    encoded_length = meta.get("base64_chars")
    if type(count) is not int or not 1 <= count <= OUTPUT_CHUNKS or type(encoded_length) is not int or not 1 <= encoded_length <= MAX_BUNDLE_BASE64:
        raise SnapshotLabError("bundle chunk count or encoded size is outside bounds")
    if meta.get("chunk_chars") != CHUNK_CHARS or math_ceil_div(encoded_length, CHUNK_CHARS) != count:
        raise SnapshotLabError("bundle chunk framing metadata is inconsistent")
    for key, maximum in (("archive_bytes", MAX_BUNDLE_RAW), ("compressed_bytes", MAX_BUNDLE_COMPRESSED)):
        value = meta.get(key)
        if type(value) is not int or not 0 < value <= maximum:
            raise SnapshotLabError(f"bundle {key} is outside its configured bound")
    for key in ("archive_sha256", "compressed_sha256"):
        if not isinstance(meta.get(key), str) or not re.fullmatch(r"[0-9a-f]{64}", meta[key]):
            raise SnapshotLabError(f"bundle {key} is malformed")
    try:
        output_count = int(os.environ.get("SNAPSHOTLAB_BUNDLE_COUNT", ""))
    except ValueError as exc:
        raise SnapshotLabError("bundle output chunk count is missing or malformed") from exc
    if output_count != count:
        raise SnapshotLabError("bundle output chunk count disagrees with metadata")
    pieces: list[str] = []
    for index in range(OUTPUT_CHUNKS):
        piece = os.environ.get(f"SNAPSHOTLAB_BUNDLE_{index:02d}", "")
        if index < count:
            if not piece or piece == "." or len(piece) > CHUNK_CHARS:
                raise SnapshotLabError(f"bundle chunk {index} is missing or overlong")
            if index + 1 < count and len(piece) != CHUNK_CHARS:
                raise SnapshotLabError(f"non-final bundle chunk {index} has an unexpected length")
            pieces.append(piece)
        elif piece not in ("", "."):
            raise SnapshotLabError(f"unexpected nonempty bundle chunk {index}")
    encoded = "".join(pieces)
    if len(encoded) != encoded_length:
        raise SnapshotLabError("concatenated bundle length does not match metadata")
    try:
        packed = base64.b64decode(encoded, validate=True)
    except ValueError as exc:
        raise SnapshotLabError("bundle chunks are not valid base64") from exc
    if len(packed) != meta.get("compressed_bytes") or sha256(packed) != meta.get("compressed_sha256"):
        raise SnapshotLabError("compressed bundle size or SHA-256 mismatch")
    archive = decompress_bounded(packed, meta.get("encoding"), meta.get("archive_bytes"))
    if sha256(archive) != meta.get("archive_sha256"):
        raise SnapshotLabError("bundle archive SHA-256 mismatch")
    output_root = contained(temp, temp / "snapshotlab-received")
    destination = output_root / str(args.bodies) / args.source_profile
    manifest = safe_extract_bundle(archive, expected_profile=args.source_profile,
                                  expected_bodies=args.bodies, destination=destination, temp=temp)
    emit({"schema": "box3d-ci-snapshotlab-received-v1", "source_profile": args.source_profile,
          "bodies": args.bodies, "archive_bytes": len(archive), "archive_sha256": sha256(archive),
          "file_count": len(manifest["files"]), "destination": destination.as_posix(),
          "integrity_gate": True})
    return 0


def bundle_cases(directory: pathlib.Path, source_profile: str, bodies: int,
                 temp: pathlib.Path) -> tuple[dict[str, Any], dict[str, dict[str, pathlib.Path]]]:
    directory = contained(temp, directory, strict=True)
    manifest_path = directory / "manifest.json"
    if not manifest_path.is_file():
        raise SnapshotLabError("received bundle manifest is missing")
    try:
        manifest = json.loads(manifest_path.read_text(encoding="ascii"), object_pairs_hook=_unique_object)
    except (OSError, UnicodeDecodeError, json.JSONDecodeError, ValueError) as exc:
        raise SnapshotLabError(f"received bundle manifest cannot be read: {exc}") from exc
    if not isinstance(manifest, dict) or manifest.get("schema") != BUNDLE_SCHEMA:
        raise SnapshotLabError("received bundle manifest schema is invalid")
    if manifest.get("profile") != source_profile or manifest.get("bodies") != bodies or manifest.get("upstream_pin") != PIN:
        raise SnapshotLabError("received bundle manifest provenance mismatch")
    result: dict[str, dict[str, pathlib.Path]] = {}
    files = manifest.get("files")
    if not isinstance(files, list) or len(files) != 8:
        raise SnapshotLabError("producer bundle must contain exactly eight seed/reference files")
    for entry in files:
        if not isinstance(entry, dict):
            raise SnapshotLabError("received file manifest entry is invalid")
        name = entry.get("name")
        if not isinstance(name, str) or not re.fullmatch(r"(stock|wire)-cp(60|120)\.(seed|reference)", name):
            raise SnapshotLabError("received file has an unexpected name")
        path = contained(directory, directory / name, strict=True)
        data = read_bounded_file(path, MAX_TRACE_RAW)
        if len(data) != entry.get("bytes") or sha256(data) != entry.get("sha256"):
            raise SnapshotLabError("received bundle file hash mismatch")
        case = name.rsplit(".", 1)[0]
        kind = name.rsplit(".", 1)[1]
        result.setdefault(case, {})[kind] = path
    expected_cases = {f"{policy}-cp{checkpoint}" for policy in POLICIES for checkpoint in CHECKPOINTS}
    if set(result) != expected_cases or any(set(row) != {"seed", "reference"} for row in result.values()):
        raise SnapshotLabError("received bundle case roster is incomplete")
    return manifest, result


def receive_root(temp: pathlib.Path, bodies: int) -> pathlib.Path:
    return contained(temp, temp / "snapshotlab-received" / str(bodies))


def run_consume(args: argparse.Namespace) -> int:
    ci = load_ci()
    temp = validate_profile(args.profile, args.compiler, ci)
    if args.bodies not in MAX_BODIES or args.config != "RelWithDebInfo" or args.simd != 4 or args.workers != 1:
        raise SnapshotLabError("consumer requires bodies 64/256, RelWithDebInfo, SIMD4 and W1")
    if args.mode not in ("smoke", "full") or not 1 <= args.parallel <= 8 or not 1 <= args.timeout <= 1800:
        raise SnapshotLabError("consumer mode or bounded build options are invalid")
    expected_sources = SMOKE_ORIGINS if args.mode == "smoke" else PRODUCER_ORIGINS
    received: dict[str, dict[str, dict[str, pathlib.Path]]] = {}
    receive_errors: list[str] = []
    for source in expected_sources:
        directory = receive_root(temp, args.bodies) / source
        try:
            _, cases = bundle_cases(directory, source, args.bodies, temp)
            received[source] = cases
        except (SnapshotLabError, OSError) as exc:
            receive_errors.append(f"{source}: {exc}")
    before = source_hashes()
    profile = f"{args.profile}/RelWithDebInfo/simd4/w1/n{args.bodies}"
    run_rows: list[dict[str, Any]] = []
    total_trace_compressed = 0
    ok = not receive_errors
    with tempfile.TemporaryDirectory(prefix="box3d-snapshotlab-consume-", dir=str(temp)) as raw_scratch:
        scratch = pathlib.Path(raw_scratch).resolve(strict=True)
        contained(temp, scratch)
        build = scratch / "build"
        _, config_ok = configure(build, args.profile, args.compiler, args.config, ci, scratch, args.timeout)
        ok = ok and config_ok
        programs: dict[str, pathlib.Path] = {}
        for policy, target in (("stock", "snapshotlab-transport-stock"),
                               ("wire", "snapshotlab-transport-wire")):
            built = config_ok and build_target(build, target, scratch, args.config, args.parallel, args.timeout)
            if built:
                try:
                    programs[policy] = executable(build, target)
                except SnapshotLabError as exc:
                    run_rows.append({"target": target, "execution_gate": False, "error": str(exc)})
                    built = False
            ok = ok and built
        # Keep executing the available source/case pairs even when another bundle is missing.
        for source in expected_sources:
            if source not in received:
                run_rows.append({"source_profile": source, "execution_gate": False,
                                 "error": "producer bundle unavailable"})
                continue
            for case in sorted(received[source]):
                policy, checkpoint_text = case.split("-cp", 1)
                checkpoint = int(checkpoint_text)
                case_files = received[source][case]
                seed = read_bounded_file(case_files["seed"], 64 * 1024 * 1024)
                reference = read_bounded_file(case_files["reference"], 64 * 1024 * 1024)
                input_errors = validate_seed(seed, policy=policy, bodies=args.bodies, checkpoint=checkpoint)
                input_errors.extend(validate_reference(reference, seed=seed, policy=policy, bodies=args.bodies,
                                                      checkpoint=checkpoint, horizon=HORIZON))
                if input_errors:
                    run_rows.append({"source_profile": source, "case": case,
                                     "execution_gate": False, "errors": input_errors})
                    ok = False
                    continue
                if policy not in programs:
                    run_rows.append({"source_profile": source, "case": case,
                                     "execution_gate": False, "error": "target unavailable"})
                    ok = False
                    continue
                output_trace = scratch / f"{source}-{case}.consume.trace"
                native_profile = f"{profile}/from-{source}/{case}"
                records, summary, errors, native = invoke_native(
                    programs[policy], "consume", native_profile, args.bodies, checkpoint,
                    case_files["seed"], case_files["reference"], output_trace,
                    scratch, args.timeout)
                errors.extend(validate_transport_result(summary, operation="consume", profile=native_profile,
                                                        policy=policy, bodies=args.bodies,
                                                        checkpoint=checkpoint, returncode=native["returncode"]))
                trace_ok = output_trace.is_file()
                if not trace_ok:
                    errors.append("consumer did not produce its trace")
                trace_data = b""
                if trace_ok:
                    try:
                        trace_data = read_bounded_file(output_trace, MAX_TRACE_RAW)
                        errors.extend(validate_trace(trace_data, reference=reference,
                                                     policy=policy, checkpoint=checkpoint, horizon=HORIZON))
                        if summary is not None and summary.get("trace_bytes") != len(trace_data):
                            errors.append("consumer trace byte count does not match its native record")
                    except (OSError, SnapshotLabError) as exc:
                        errors.append(f"consumer trace is invalid: {exc}")
                if trace_ok and not errors:
                    try:
                        total_trace_compressed = emit_trace(output_trace, profile=native_profile,
                                                           label=f"{source}/{case}",
                                                           compressed_used=total_trace_compressed)
                    except (OSError, SnapshotLabError) as exc:
                        errors.append(f"trace transfer failed: {exc}")
                execution_gate = not errors and parse_native_execution(summary, "transport")
                ok = ok and execution_gate
                run_rows.append({"source_profile": source, "case": case,
                                 "native_returncode": native["returncode"],
                                 "native_stdout_sha256": native["stdout_sha256"],
                                 "native_record_count": len(records),
                                 "native_execution_gate": parse_native_execution(summary, "transport"),
                                 "execution_gate": execution_gate,
                                 "experiment_status": summary.get("status") if summary else None,
                                 "exact_state_equal": summary.get("exact_state_equal") if summary else None,
                                 "projection_equal": summary.get("projection_equal") if summary else None,
                                 "trace_bytes": output_trace.stat().st_size if trace_ok else None,
                                 "trace_sha256": sha256(trace_data) if trace_data else None,
                                 "errors": errors})
        after = source_hashes()
        if before != after:
            ok = False
            receive_errors.append("source hashes changed during consumer execution")
        emit({"schema": RUN_SCHEMA, "command": "consume", "profile": args.profile,
              "mode": args.mode, "bodies": args.bodies, "expected_sources": list(expected_sources),
              "received_sources": sorted(received), "receive_errors": receive_errors,
              "status": "complete" if ok else "execution_failed", "execution_gate": ok,
              "run_rows": run_rows, "trace_compressed_bytes": total_trace_compressed,
              "trace_compressed_limit_bytes": MAX_TRACE_COMPRESSED_PER_CONSUMER,
              "source_hashes": before, "upstream_pin": PIN})
    return 0 if ok else 1


def run_cost(args: argparse.Namespace) -> int:
    ci = load_ci()
    temp = validate_profile(args.profile, args.compiler, ci)
    if args.config != "RelWithDebInfo" or args.simd != 4 or args.workers != 1:
        raise SnapshotLabError("cost run requires RelWithDebInfo, SIMD4 and W1")
    if not 1 <= args.parallel <= 8 or not 1 <= args.timeout <= 1800:
        raise SnapshotLabError("parallel or timeout is outside the bounded range")
    before = source_hashes()
    profile = f"{args.profile}/RelWithDebInfo/simd4/w1"
    trace_path: pathlib.Path | None = None
    with tempfile.TemporaryDirectory(prefix="box3d-snapshotlab-cost-", dir=str(temp)) as raw_scratch:
        scratch = pathlib.Path(raw_scratch).resolve(strict=True)
        contained(temp, scratch)
        build = scratch / "build"
        _, config_ok = configure(build, args.profile, args.compiler, args.config, ci, scratch, args.timeout)
        built = config_ok and build_target(build, "snapshotlab-cost", scratch, args.config,
                                           args.parallel, args.timeout)
        errors: list[str] = []
        records: list[dict[str, Any]] = []
        native_result: dict[str, Any] | None = None
        summary: dict[str, Any] | None = None
        if built:
            try:
                program = executable(build, "snapshotlab-cost")
                trace_path = scratch / "native-cost.trace"
                argv = [str(program), "--profile", profile, "--workers", "1", "--simd", "4",
                        "--trace", str(trace_path)]
                native_result = bounded(argv, ROOT, scratch / "native-cost.stdout",
                                        scratch / "native-cost.stderr", args.timeout, MAX_NATIVE_OUTPUT)
                emit_phase("native-cost", native_result)
                records, errors = parse_jsonl(native_result["stdout"])
                summary, schema_errors = validate_native_records(records, driver="cost", profile=profile,
                                                                 operation="cost")
                errors.extend(schema_errors)
                if native_result["timed_out"] or native_result["overflow"]:
                    errors.append("native cost run timed out or exceeded output bound")
                errors.extend(validate_cost_records(records, summary, trace_path))
                forward_native_records(native_result["stdout"],
                                       [public_cost_record(row) for row in records],
                                       profile=profile, operation="cost")
                if trace_path.is_file() and not errors:
                    try:
                        emit_trace(trace_path, profile=profile, label="cost/raw-samples", compressed_used=0)
                    except (OSError, SnapshotLabError) as exc:
                        errors.append(f"cost trace transfer failed: {exc}")
                if native_result["stderr"]:
                    sys.stderr.buffer.write(native_result["stderr"])
                    if not native_result["stderr"].endswith(b"\n"):
                        sys.stderr.buffer.write(b"\n")
                    sys.stderr.buffer.flush()
            except (SnapshotLabError, OSError, subprocess.SubprocessError) as exc:
                errors.append(str(exc))
        else:
            errors.append("cost target was not built")
        after = source_hashes()
        if before != after:
            errors.append("source hashes changed during cost execution")
        execution_gate = not errors and parse_native_execution(summary, "cost")
        if native_result is not None and summary is not None:
            expected_rc = 0 if parse_native_execution(summary, "cost") else 1
            if native_result["returncode"] != expected_rc:
                execution_gate = False
                errors.append("cost process exit status disagrees with its execution-validity record")
        # Private budget decisions are deliberately absent from the public wrapper.
        emit({"schema": RUN_SCHEMA, "command": "cost", "profile": args.profile,
              "status": "complete" if execution_gate else "execution_failed",
              "execution_gate": execution_gate, "native_record_count": len(records),
              "native_stdout_sha256": native_result.get("stdout_sha256") if native_result else None,
              "experiment_status": summary.get("status") if summary else None,
              "trace_bytes": (trace_path.stat().st_size if trace_path is not None and trace_path.is_file() else None),
              "trace_sha256": (sha256(read_bounded_file(trace_path, 4 * 1024 * 1024)) if trace_path is not None and trace_path.is_file() else None),
              "errors": errors,
              "source_hashes": before, "upstream_pin": PIN})
    return 0 if execution_gate else 1


def main() -> int:
    parser = argparse.ArgumentParser(description="Run bounded public Box3D snapshot-lab measurements on GitHub Actions.")
    sub = parser.add_subparsers(dest="command", required=True)
    producer = sub.add_parser("produce", help="create a source-origin snapshot/reference bundle")
    consumer = sub.add_parser("consume", help="restore foreign source bundles and continue live steps")
    cost = sub.add_parser("cost", help="measure the fixed snapshot and rollback workload")
    receiver = sub.add_parser("receive", help="validate and unpack one bounded producer output bundle")
    for command in (producer, consumer, cost):
        command.add_argument("--profile", choices=tuple(ORIGINS), required=True)
        command.add_argument("--compiler", choices=("gcc", "clang", "msvc"), required=True)
        command.add_argument("--config", choices=("RelWithDebInfo",), default="RelWithDebInfo")
        command.add_argument("--simd", type=int, choices=(4,), default=4)
        command.add_argument("--workers", type=int, choices=(1,), default=1)
        command.add_argument("--parallel", type=int, choices=range(1, 9), default=2)
        command.add_argument("--timeout", type=int, default=900)
    for command in (producer, consumer):
        command.add_argument("--bodies", type=int, choices=MAX_BODIES, required=True)
    consumer.add_argument("--mode", choices=("smoke", "full"), required=True)
    receiver.add_argument("--source-profile", choices=PRODUCER_ORIGINS, required=True)
    receiver.add_argument("--bodies", type=int, choices=MAX_BODIES, required=True)
    args = parser.parse_args()
    try:
        if args.command == "produce":
            return run_produce(args)
        if args.command == "consume":
            return run_consume(args)
        if args.command == "cost":
            return run_cost(args)
        return run_receive(args)
    except (SnapshotLabError, OSError, subprocess.SubprocessError, ValueError, zipfile.BadZipFile) as exc:
        fail_record(args.command, getattr(args, "profile", None), str(exc))
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
