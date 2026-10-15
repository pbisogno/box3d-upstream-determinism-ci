#!/usr/bin/env python3
"""Build and run bounded public Box3D contact-cache diagnostics on Actions."""
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
LAB = ROOT / "contactlab"
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
TARGETS = (
    ("physics", "contactlab-physics", True, 408),
    ("physics-pristine", "contactlab-physics-pristine", False, 408),
    ("zero", "contactlab-zero", True, 4),
    ("zero-pristine", "contactlab-zero-pristine", False, 4),
)
RUN_SCHEMA = "box3d-ci-contactlab-run-v1"
CONFIG_SCHEMA = "box3d-ci-contactlab-configured-v1"
DRIVER_SCHEMA = "box3d-ci-contactlab-driver-v1"
TRACE_SCHEMA = "box3d-ci-contactlab-trace-v1"
CHUNK_SCHEMA = "box3d-ci-contactlab-trace-chunk-v1"
CHUNK_CHARS = 19_000
MAX_LINE = 20_000
MAX_TRACE_RAW = 64 * 1024 * 1024
MAX_PROFILE_COMPRESSED = 8 * 1024 * 1024
MAX_PROCESS_OUTPUT = 32 * 1024 * 1024
MAX_BUILD_OUTPUT = 16 * 1024 * 1024
QUALITY_STATUS = {"PASS", "FAIL"}
ZERO_HOOK_FIELD_COUNTS = {1: 2, 2: 4, 3: 6, 4: 1, 5: 12, 6: 5, 7: 3}
ZERO_HOOK_CHUNK_EVENTS = 32
ZERO_HOOK_MAX_EVENTS = 8_192
ZERO_HOOK_MAX_PARTS = 256


class ContactLabError(RuntimeError):
    pass


def sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def emit(row: dict[str, Any]) -> None:
    data = json.dumps(row, sort_keys=True, separators=(",", ":"), ensure_ascii=True).encode("ascii") + b"\n"
    if len(data) > MAX_LINE:
        raise ContactLabError("wrapper JSON record exceeds the 20,000-byte log line limit")
    sys.stdout.buffer.write(data)
    sys.stdout.buffer.flush()


def load_ci():
    source = CI_PATH.read_bytes()
    if sha256(source) != CI_SHA256:
        raise ContactLabError("existing ci.py source hash changed")
    spec = importlib.util.spec_from_file_location("box3d_contactlab_ci_helper", CI_PATH)
    if spec is None or spec.loader is None:
        raise ContactLabError("could not load existing public ci.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    if getattr(module, "PINNED_REVISION", None) != PIN:
        raise ContactLabError("ci.py is not pinned to the expected upstream revision")
    return module


def runner_temp() -> pathlib.Path:
    if os.environ.get("GITHUB_ACTIONS", "").casefold() != "true":
        raise ContactLabError("native contactlab runs are allowed only on ephemeral GitHub Actions runners")
    raw = os.environ.get("RUNNER_TEMP")
    if not raw:
        raise ContactLabError("RUNNER_TEMP is required")
    root = pathlib.Path(raw).resolve(strict=True)
    if not root.is_dir():
        raise ContactLabError("RUNNER_TEMP is not a directory")
    return root


def contained(root: pathlib.Path, path: pathlib.Path) -> pathlib.Path:
    root = root.resolve(strict=True)
    result = path.resolve(strict=False)
    try:
        result.relative_to(root)
    except ValueError as exc:
        raise ContactLabError("temporary path escaped RUNNER_TEMP") from exc
    return result


def validate_args(args: argparse.Namespace, ci: Any) -> pathlib.Path:
    temp = runner_temp()
    if args.origin not in COMPILER or COMPILER[args.origin] != args.compiler:
        raise ContactLabError("origin and compiler do not match a supported profile")
    expected_system, expected_arch = HOST[args.origin]
    if platform.system() != expected_system or ci.normalized_arch(platform.machine()) != expected_arch:
        raise ContactLabError("origin does not match this runner OS and architecture")
    if args.config not in ("Debug", "RelWithDebInfo"):
        raise ContactLabError("unsupported CMake configuration")
    if args.simd not in (0, 4, 8) or args.workers != 1:
        raise ContactLabError("this campaign supports SIMD 0/4/8 and a single worker only")
    if not 1 <= args.parallel <= 8 or not 1 <= args.timeout <= 1800:
        raise ContactLabError("parallel must be 1..8 and timeout 1..1800 seconds")
    ci.validate_compiler_selection(args.compiler)
    if args.simd == 8:
        ci.require_avx2_support()
    ci.verified_source(ROOT / "upstream")
    return temp


def validate_compiler_ids(origin: str, ids: dict[str, str]) -> None:
    values = [str(ids.get(key, "")).strip().casefold() for key in ("c", "cxx")]
    if any(not value for value in values):
        raise ContactLabError("CMake did not identify both C and CXX compilers")
    if origin == "windows-msvc-x64":
        valid = all(value.startswith("msvc") for value in values)
    elif origin == "linux-gcc-x64":
        valid = all(value.startswith("gnu") for value in values)
    elif origin == "linux-clang-arm64":
        valid = all(value.startswith("clang") for value in values)
    else:
        valid = all(value.startswith(("appleclang", "clang")) for value in values)
    if not valid:
        raise ContactLabError("CMake compiler IDs do not match the selected profile")


def bounded(argv: list[str], cwd: pathlib.Path, stdout_path: pathlib.Path,
            stderr_path: pathlib.Path, timeout: int, limit: int) -> dict[str, Any]:
    started = time.monotonic()
    reason = None
    with stdout_path.open("wb") as out, stderr_path.open("wb") as err:
        try:
            proc = subprocess.Popen(argv, cwd=str(cwd), stdin=subprocess.DEVNULL,
                                    stdout=out, stderr=err, shell=False)
        except OSError as exc:
            raise ContactLabError(f"could not start {pathlib.Path(argv[0]).name}: {exc}") from exc
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
        "stdout": out_data, "stderr": err_data, "stdout_bytes": out_size,
        "stderr_bytes": err_size,
        "stdout_sha256": sha256(out_data) if out_size <= limit else None,
        "stderr_sha256": sha256(err_data) if err_size <= limit else None,
    }


def phase(name: str, result: dict[str, Any]) -> None:
    emit({"schema": "box3d-ci-contactlab-phase-v1", "phase": name,
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


def parse_records(raw: bytes, profile: str, driver: str, instrumented: bool,
                  expected_results: int, expected_criteria: int,
                  expected_backend: str, expected_width: int) -> tuple[list[dict[str, Any]], list[str]]:
    errors: list[str] = []
    try:
        text = raw.decode("utf-8", errors="strict")
    except UnicodeDecodeError as exc:
        return [], [f"driver stdout is not UTF-8: {exc}"]
    lines = text.splitlines()
    if not lines:
        return [], ["driver emitted no JSONL records"]
    records: list[dict[str, Any]] = []
    for n, line in enumerate(lines, 1):
        if not line:
            errors.append(f"blank JSONL row at line {n}")
            continue
        if len(line.encode("utf-8")) > MAX_LINE:
            errors.append(f"native JSONL row {n} exceeds the 20,000-byte limit")
            continue
        try:
            row = json.loads(line)
        except json.JSONDecodeError as exc:
            errors.append(f"invalid native JSONL at line {n}: {exc}")
            continue
        if not isinstance(row, dict):
            errors.append(f"native JSONL row {n} is not an object")
            continue
        records.append(row)
    if any(row.get("schema") != "box3d-contactlab-v1" for row in records):
        errors.append("unexpected native JSON schema")
    if any(row.get("profile") != profile for row in records):
        errors.append("native profile attribution mismatch")
    if any(row.get("driver") != driver or row.get("instrumented") is not instrumented for row in records):
        errors.append("native driver or instrumented identity mismatch")
    types = [row.get("type") for row in records]
    results = [row for row in records if row.get("type") == "result"]
    criteria = [row for row in records if row.get("type") == "criteria"]
    summaries = [row for row in records if row.get("type") == "summary"]
    hook_rows = [row for row in records if row.get("type") == "hooks"]
    if len(results) != expected_results or len(criteria) != expected_criteria or len(summaries) != 1:
        errors.append("native record roster/count mismatch")
    if len(summaries) == 1 and (not records or records[-1].get("type") != "summary"):
        errors.append("native summary must be the final JSONL record")
    if any(kind not in {"criteria", "result", "summary", "hooks"} for kind in types):
        errors.append("native JSONL contains an unknown record type")
    if driver == "physics" and hook_rows:
        errors.append("physics driver unexpectedly emitted zero-hook rows")
    if driver == "zero":
        errors.extend(validate_zero_hook_rows(records, profile, instrumented))
    elif any(row.get("type") == "hooks" for row in records):
        errors.append("non-zero driver emitted hook chunks")
    if len(records) != expected_results + expected_criteria + 1 + len(hook_rows):
        errors.append("native record count does not match result/criteria/summary plus hook chunks")
    for row in results:
        status = str(row.get("status", "")).upper()
        if driver == "physics" and status not in QUALITY_STATUS:
            errors.append("physics result has an unknown quality status")
        if driver == "zero" and status not in {"PASS", "COMPLETE", "SUCCESS", "GATE_FAILED"}:
            errors.append("zero-cache result did not complete successfully")
        gates = row.get("execution_gates")
        if isinstance(gates, dict):
            if not gates or any(value is not True for value in gates.values()):
                errors.append("result execution_gates are empty or failed")
        elif driver == "physics":
            errors.append("physics result lacks execution_gates")
        if driver == "zero" and row.get("execution_pass") is not True:
            errors.append("result execution_pass is missing or false")
        actual_workers = row.get("actual_workers") if driver == "physics" else row.get("workers_actual")
        if row.get("workers_requested") != 1 or actual_workers != 1:
            errors.append("worker count did not match the approved W1 profile")
        if row.get("simd_actual") != expected_width:
            errors.append("observed SIMD width does not match the requested backend policy")
        if row.get("backend_actual") != expected_backend:
            errors.append("observed backend does not match the requested backend policy")
    summary = summaries[0] if summaries else {}
    if summary and (summary.get("execution_pass") is not True or summary.get("trace_io_ok") is not True):
        errors.append("summary execution or trace-I/O gate failed")
    if summary and driver == "physics" and instrumented and summary.get("activation_controls_pass") is not True:
        errors.append("summary did not confirm required hook activation controls")
    return records, errors


def validate_zero_hook_rows(records: list[dict[str, Any]], profile: str,
                            instrumented: bool) -> list[str]:
    """Check complete, ordered hook chunks without collapsing their source order."""
    errors: list[str] = []
    results = [row for row in records if row.get("type") == "result"]
    chunks = [row for row in records if row.get("type") == "hooks"]
    chunk_positions = [i for i, row in enumerate(records) if row.get("type") == "hooks"]
    result_positions = {row.get("trial"): i for i, row in enumerate(records)
                        if row.get("type") == "result"}
    result_trial_order = [row.get("trial") for row in records if row.get("type") == "result"]
    by_trial: dict[int, list[tuple[int, dict[str, Any]]]] = {}
    for position, row in zip(chunk_positions, chunks):
        trial = row.get("trial")
        if type(trial) is not int or not 1 <= trial <= 4:
            errors.append("zero hook chunk has an invalid trial number")
            continue
        by_trial.setdefault(trial, []).append((position, row))
    if (len(result_positions) != 4 or set(result_positions) != {1, 2, 3, 4} or
            result_trial_order != [1, 2, 3, 4]):
        errors.append("zero result trial roster is incomplete or duplicated")
    for result in results:
        trial = result.get("trial")
        if type(trial) is not int or not 1 <= trial <= 4:
            errors.append("zero result has an invalid trial number")
            continue
        total = result.get("hooks_count")
        parts = result.get("hook_parts")
        if type(total) is not int or total < 0 or type(parts) is not int or parts < 0:
            errors.append(f"zero result {trial} has invalid hook count metadata")
            continue
        if total > ZERO_HOOK_MAX_EVENTS or parts > ZERO_HOOK_MAX_PARTS:
            errors.append(f"zero result {trial} exceeds the bounded hook event/part limit")
            continue
        expected_parts = (total + ZERO_HOOK_CHUNK_EVENTS - 1) // ZERO_HOOK_CHUNK_EVENTS
        if parts != expected_parts:
            errors.append(f"zero result {trial} hook part count is inconsistent")
        trial_chunks = by_trial.get(trial, [])
        if len(trial_chunks) != parts:
            errors.append(f"zero result {trial} hook chunks are missing or duplicated")
        expected_part_indices = list(range(parts))
        got_part_indices = [row.get("part") for _, row in trial_chunks]
        if got_part_indices != expected_part_indices:
            errors.append(f"zero result {trial} hook chunk order/index is invalid")
        if parts == 0 and total != 0:
            errors.append(f"zero result {trial} declares hooks without chunks")
        event_offset = 0
        next_result_position = min((position for result_trial, position in result_positions.items()
                                    if result_trial > trial), default=len(records))
        for part_index, (position, chunk) in enumerate(trial_chunks):
            event_count = min(ZERO_HOOK_CHUNK_EVENTS, max(0, total - event_offset))
            events = chunk.get("hooks")
            if (type(chunk.get("part")) is not int or chunk.get("part") != part_index or
                    type(chunk.get("parts")) is not int or type(chunk.get("offset")) is not int or
                    type(chunk.get("count")) is not int or type(chunk.get("total_count")) is not int or
                    chunk.get("profile") != profile or chunk.get("driver") != "zero" or
                    chunk.get("instrumented") is not instrumented or
                    chunk.get("parts") != parts or chunk.get("offset") != event_offset or
                    chunk.get("count") != event_count or chunk.get("total_count") != total or
                    not isinstance(events, list) or len(events) != event_count):
                errors.append(f"zero result {trial} hook chunk metadata/count is invalid")
            if position <= result_positions.get(trial, -1):
                errors.append(f"zero result {trial} hook chunk precedes its result row")
            if position >= next_result_position:
                errors.append(f"zero result {trial} hook chunks occur after the next result row")
            if part_index and position <= trial_chunks[part_index - 1][0]:
                errors.append(f"zero result {trial} hook chunks are reordered")
            if isinstance(events, list):
                for event in events:
                    if not isinstance(event, dict):
                        errors.append(f"zero result {trial} contains a malformed hook event")
                        continue
                    stage = event.get("stage")
                    width, lane = event.get("width"), event.get("lane")
                    bits = event.get("bits")
                    integer_fields = ("tick", "arm", "stage", "contact_index", "generation",
                                      "manifold", "width", "lane")
                    if any(type(event.get(key)) is not int for key in integer_fields):
                        errors.append(f"zero result {trial} hook event has non-integer metadata")
                        continue
                    if (event["tick"] < 0 or event["tick"] > 1536 or event["arm"] not in range(4) or
                            stage not in ZERO_HOOK_FIELD_COUNTS or event["contact_index"] not in range(256) or
                            not 0 <= event["generation"] <= 0xffffffff or event["manifold"] not in range(4) or
                            width not in {1, 4, 8} or not 0 <= lane < width or
                            not isinstance(bits, list) or len(bits) != ZERO_HOOK_FIELD_COUNTS.get(stage) or
                            any(not isinstance(value, str) or len(value) != 8 or
                                any(ch not in "0123456789abcdefABCDEF" for ch in value) for value in bits)):
                        errors.append(f"zero result {trial} hook event violates the stage field contract")
            event_offset += event_count
        if event_offset != total:
            errors.append(f"zero result {trial} hook event total does not reconcile")
        if not instrumented and (total != 0 or parts != 0 or trial_chunks):
            errors.append(f"pristine zero result {trial} unexpectedly has hook observations")
    if set(by_trial) - set(result_positions):
        errors.append("zero hook chunk has no matching result trial")
    if not instrumented and chunks:
        errors.append("pristine zero driver emitted hook chunks")
    return errors


def trace_to_logs(path: pathlib.Path, *, temp: pathlib.Path, profile: str,
                  origin: str, driver: str, used: int) -> tuple[int, int, int, str | None]:
    try:
        resolved = path.resolve(strict=True)
        resolved.relative_to(temp.resolve(strict=True))
    except (OSError, ValueError):
        return used, 0, 0, "trace missing or outside RUNNER_TEMP"
    if path.is_symlink() or not resolved.is_file():
        return used, 0, 0, "trace is not a regular file"
    size = resolved.stat().st_size
    if not 1 <= size <= MAX_TRACE_RAW:
        return used, size, 0, "trace raw size outside 1..64 MiB"
    raw = resolved.read_bytes()
    zbytes = zlib.compress(raw, level=9)
    xbytes = lzma.compress(raw, format=lzma.FORMAT_XZ, preset=6)
    compressed, encoding = (xbytes, "xz+base64") if len(xbytes) < len(zbytes) else (zbytes, "zlib+base64")
    if used + len(compressed) > MAX_PROFILE_COMPRESSED:
        return used, size, len(compressed), "aggregate profile traces exceed 8 MiB compressed"
    encoded = base64.b64encode(compressed).decode("ascii")
    chunks = [encoded[i:i + CHUNK_CHARS] for i in range(0, len(encoded), CHUNK_CHARS)]
    emit({"schema": TRACE_SCHEMA, "profile": profile, "origin": origin, "driver": driver,
          "bytes": len(raw), "sha256": sha256(raw), "compressed_bytes": len(compressed),
          "compressed_sha256": sha256(compressed), "encoding": encoding,
          "chunk_chars": CHUNK_CHARS, "chunk_count": len(chunks)})
    for index, payload in enumerate(chunks):
        emit({"schema": CHUNK_SCHEMA, "profile": profile, "origin": origin, "driver": driver,
              "index": index, "payload": payload})
    return used + len(compressed), size, len(compressed), None


def input_hashes() -> dict[str, str]:
    paths = [CI_PATH, ROOT / "contactlab_ci.py", ROOT / "stress-replay.c",
             LAB / "CMakeLists.txt", LAB / "instrument_physics.py",
             LAB / "instrument_zero.py", LAB / "physics-hook.h", LAB / "zero-hook.h",
             ROOT / "contactlab-physics.c", ROOT / "contactlab-zero.c"]
    missing = [p.relative_to(ROOT).as_posix() for p in paths if not p.is_file()]
    if missing:
        raise ContactLabError("contactlab source inputs are missing: " + ", ".join(missing))
    return {p.relative_to(ROOT).as_posix(): sha256(p.read_bytes()) for p in paths}


def exe_path(build: pathlib.Path, target: str) -> pathlib.Path:
    suffix = ".exe" if os.name == "nt" else ""
    matches = list((build / "bin").rglob(target + suffix))
    if len(matches) != 1 or not matches[0].is_file():
        raise ContactLabError(f"expected exactly one executable for {target}")
    return matches[0].resolve(strict=True)


def run_driver(label: str, target: str, instrumented: bool, expected_results: int,
               exe: pathlib.Path, scratch: pathlib.Path, temp: pathlib.Path,
               profile: str, origin: str, workers: int, simd: int, timeout: int,
               compressed: int) -> tuple[dict[str, Any], int, list[dict[str, Any]]]:
    logdir = scratch / "logs"
    logdir.mkdir(exist_ok=True)
    trace = scratch / (label + ".trace")
    argv = [str(exe), "--profile", profile, "--workers", str(workers),
            "--simd", str(simd), "--trace", str(trace)]
    result = bounded(argv, ROOT, logdir / (label + ".stdout"),
                     logdir / (label + ".stderr"), timeout, MAX_PROCESS_OUTPUT)
    raw_stdout = result["stdout"]
    if raw_stdout:
        sys.stdout.buffer.write(raw_stdout)
        if not raw_stdout.endswith(b"\n"):
            sys.stdout.buffer.write(b"\n")
        sys.stdout.buffer.flush()
    driver = "physics" if label.startswith("physics") else "zero"
    expected_backend = ({0: "scalar", 4: ("sse2" if origin in ("windows-msvc-x64", "linux-gcc-x64", "macos-clang-x64") else "neon"),
                         8: "avx2"}[simd])
    expected_width = 8 if simd == 8 else 4
    records, errors = parse_records(raw_stdout, profile, driver, instrumented,
                                    expected_results, 1 if driver == "physics" else 0,
                                    expected_backend, expected_width)
    statuses = [str(row.get("status", "")).upper() for row in records if row.get("type") == "result"]
    summary_rows = [row for row in records if row.get("type") == "summary"]
    summary_native = summary_rows[0] if len(summary_rows) == 1 else {}
    quality_ok = driver == "zero" or all(status == "PASS" for status in statuses)
    legitimate_quality_failure = (driver == "physics" and bool(statuses)
                                  and all(status in QUALITY_STATUS for status in statuses) and not quality_ok)
    driver_exit_ok = result["returncode"] == 0 or (legitimate_quality_failure and result["returncode"] == 1)
    trace_error = None
    trace_bytes = trace_compressed = 0
    if result["timed_out"] or result["overflow"]:
        trace_error = "driver exceeded time or output limit"
    else:
        compressed, trace_bytes, trace_compressed, trace_error = trace_to_logs(
            trace, temp=temp, profile=profile, origin=origin, driver=label, used=compressed)
    execution_ok = (driver_exit_ok and not result["timed_out"] and not result["overflow"]
                    and not errors and trace_error is None)
    row = {
        "schema": DRIVER_SCHEMA, "profile": profile, "driver": label,
        "target": target, "returncode": result["returncode"],
        "timed_out": result["timed_out"], "output_limit": result["overflow"],
        "elapsed_seconds": result["elapsed_seconds"], "stdout_bytes": result["stdout_bytes"],
        "stdout_sha256": result["stdout_sha256"], "stderr_bytes": result["stderr_bytes"],
        "stderr_sha256": result["stderr_sha256"], "jsonl_records": len(records),
        "reported_statuses": statuses, "jsonl_errors": errors,
        "quality_gate": ("not_applicable" if driver == "zero" else
                         ("pass" if quality_ok else ("fail" if legitimate_quality_failure else "unknown"))),
        "execution_gate": execution_ok, "trace_bytes": trace_bytes,
        "trace_compressed_bytes": trace_compressed, "trace_error": trace_error,
        "native_execution_pass": summary_native.get("execution_pass"),
        "native_activation_controls_pass": summary_native.get("activation_controls_pass"),
        "trace_sha256": sha256(trace.read_bytes()) if trace.is_file() and not trace.is_symlink() else None,
        "cross_profile_comparison": "deferred_to_private_audit",
    }
    emit(row)
    if result["stderr"]:
        # Keep the original child stderr visible in the public job log, without rewriting it.
        sys.stderr.buffer.write(result["stderr"])
        if not result["stderr"].endswith(b"\n"):
            sys.stderr.buffer.write(b"\n")
        sys.stderr.buffer.flush()
    return row, compressed, records


def run(args: argparse.Namespace) -> int:
    ci = load_ci()
    temp = validate_args(args, ci)
    profile = f"{args.origin}/{args.config}/simd{args.simd}/w{args.workers}"
    hashes = input_hashes()
    cmake_flags = ci.build_flags(args.config, 4)
    cmake_flags = [flag for flag in cmake_flags if not flag.startswith("-DBOX3D_DISABLE_SIMD=")]
    cmake_flags.append("-DBOX3D_DISABLE_SIMD=" + ("ON" if args.simd == 0 else "OFF"))
    cmake_flags = [flag for flag in cmake_flags if not flag.startswith("-DBOX3D_AVX2=")]
    cmake_flags.append("-DBOX3D_AVX2=" + ("ON" if args.simd == 8 else "OFF"))
    expected_backend = {0: "scalar", 8: "avx2"}.get(
        args.simd, "sse2" if args.origin in ("windows-msvc-x64", "linux-gcc-x64", "macos-clang-x64") else "neon")
    with tempfile.TemporaryDirectory(prefix="box3d-contactlab-", dir=str(temp)) as raw_tmp:
        scratch = pathlib.Path(raw_tmp).resolve(strict=True)
        contained(temp, scratch)
        build = scratch / "build"
        configure_args = ["cmake", "-S", str(LAB), "-B", str(build)] + cmake_flags
        configure_args += ci.cmake_compiler_args(args.compiler, None)
        configure_args += ["-DPython3_EXECUTABLE=" + sys.executable]
        configured = bounded(configure_args, ROOT, scratch / "configure.stdout",
                             scratch / "configure.stderr", args.timeout, MAX_BUILD_OUTPUT)
        phase("configure", configured)
        if configured["returncode"] or configured["timed_out"] or configured["overflow"]:
            emit({"schema": RUN_SCHEMA, "profile": profile, "status": "execution_failed",
                  "phase": "configure", "source_hashes": hashes})
            return 1
        config_text = configured["stdout"].decode("utf-8", errors="replace")
        compiler_ids = ci.cmake_ids(config_text)
        validate_compiler_ids(args.origin, compiler_ids)
        cache = build / "CMakeCache.txt"
        cache_text = cache.read_text(encoding="utf-8", errors="replace")
        expected_disable = "ON" if args.simd == 0 else "OFF"
        expected_avx2 = "ON" if args.simd == 8 else "OFF"
        cache_disable = next((line.split("=", 1)[1] for line in cache_text.splitlines()
                              if line.startswith("BOX3D_DISABLE_SIMD:BOOL=")), None)
        cache_avx2 = next((line.split("=", 1)[1] for line in cache_text.splitlines()
                           if line.startswith("BOX3D_AVX2:BOOL=")), None)
        cache_ok = cache_disable == expected_disable and cache_avx2 == expected_avx2
        emit({"schema": CONFIG_SCHEMA, "profile": profile, "origin": args.origin,
              "compiler": args.compiler, "config": args.config, "requested_simd": args.simd,
              "expected_backend": expected_backend, "workers": args.workers,
              "compiler_ids": compiler_ids, "box3d_disable_simd": cache_disable,
              "box3d_avx2": cache_avx2, "backend_cache_gate": cache_ok,
              "ci_helper_sha256": CI_SHA256, "upstream_pin": PIN, "source_hashes": hashes})
        if not cache_ok:
            emit({"schema": RUN_SCHEMA, "profile": profile, "status": "execution_failed",
                  "phase": "backend_cache_verification", "source_hashes": hashes})
            return 1
        driver_rows: list[dict[str, Any]] = []
        by_label: dict[str, list[dict[str, Any]]] = {}
        build_gates: dict[str, bool] = {}
        compressed = 0
        for label, target, instrumented, expected_results in TARGETS:
            built = bounded(["cmake", "--build", str(build), "--config", args.config,
                             "--parallel", str(args.parallel), "--target", target],
                            ROOT, scratch / ("build-" + label + ".stdout"),
                            scratch / ("build-" + label + ".stderr"), args.timeout, MAX_BUILD_OUTPUT)
            phase("build-" + label, built)
            build_ok = built["returncode"] == 0 and not built["timed_out"] and not built["overflow"]
            build_gates[label] = build_ok
            if not build_ok:
                row = {"schema": DRIVER_SCHEMA, "profile": profile, "driver": label,
                       "target": target, "build_gate": False, "execution_gate": False,
                       "quality_gate": "not_run", "cross_profile_comparison": "deferred_to_private_audit"}
                emit(row)
                driver_rows.append(row)
                by_label[label] = []
                continue
            try:
                exe = exe_path(build, target)
                row, compressed, records = run_driver(
                    label, target, instrumented, expected_results, exe, scratch, temp,
                    profile, args.origin, args.workers, args.simd, args.timeout, compressed)
                row["build_gate"] = True
                by_label[label] = records
            except (ContactLabError, OSError, subprocess.SubprocessError, ValueError) as exc:
                row = {"schema": DRIVER_SCHEMA, "profile": profile, "driver": label,
                       "target": target, "build_gate": True, "execution_gate": False,
                       "quality_gate": "unknown", "error": str(exc),
                       "cross_profile_comparison": "deferred_to_private_audit"}
                emit(row)
                by_label[label] = []
            driver_rows.append(row)
        execution_ok = all(build_gates.values()) and len(driver_rows) == len(TARGETS) and all(
            row.get("execution_gate") is True for row in driver_rows)
        parity_rows = []
        for left, right in (("physics", "physics-pristine"), ("zero", "zero-pristine")):
            left_row = next((row for row in driver_rows if row.get("driver") == left), {})
            right_row = next((row for row in driver_rows if row.get("driver") == right), {})
            equal = (isinstance(left_row.get("trace_sha256"), str) and
                     left_row.get("trace_sha256") == right_row.get("trace_sha256"))
            parity_rows.append({"schema": "box3d-ci-contactlab-pristine-parity-v1",
                                "profile": profile, "instrumented_driver": left,
                                "pristine_driver": right, "status": "pass" if equal else "failed",
                                "byte_exact_trace_equal": equal,
                                "instrumented_trace_sha256": left_row.get("trace_sha256"),
                                "pristine_trace_sha256": right_row.get("trace_sha256")})
            emit(parity_rows[-1])
        parity_ok = all(row["byte_exact_trace_equal"] for row in parity_rows)
        execution_ok = execution_ok and parity_ok
        quality_failures = [row["driver"] for row in driver_rows if row.get("quality_gate") == "fail"]
        quality_unknown = [row["driver"] for row in driver_rows if row.get("quality_gate") == "unknown"]
        passed = execution_ok and not quality_failures and not quality_unknown
        emit({"schema": RUN_SCHEMA, "profile": profile,
              "status": "pass" if passed else ("quality_failed" if execution_ok and quality_failures else "execution_failed"),
              "execution_gate": execution_ok, "build_gates": build_gates,
              "pristine_parity_gate": parity_ok,
              "driver_gates": {row["driver"]: row.get("execution_gate") is True for row in driver_rows},
              "quality_failures": quality_failures, "quality_unknown": quality_unknown,
              "trace_compressed_bytes": compressed,
              "trace_compressed_limit_bytes": MAX_PROFILE_COMPRESSED,
              "cross_profile_comparison": "deferred_to_private_audit", "source_hashes": hashes})
        return 0 if passed else 1


def main() -> int:
    parser = argparse.ArgumentParser(description="Run bounded public Box3D contactlab profiles on Actions.")
    command = parser.add_subparsers(dest="command", required=True)
    run_parser = command.add_parser("run", help="build and execute all four contactlab targets")
    run_parser.add_argument("--origin", choices=tuple(COMPILER), required=True)
    run_parser.add_argument("--compiler", choices=("gcc", "clang", "msvc"), required=True)
    run_parser.add_argument("--config", choices=("Debug", "RelWithDebInfo"), required=True)
    run_parser.add_argument("--simd", type=int, choices=(0, 4, 8), required=True)
    run_parser.add_argument("--workers", type=int, choices=(1,), required=True)
    run_parser.add_argument("--parallel", type=int, default=2)
    run_parser.add_argument("--timeout", type=int, default=900)
    args = parser.parse_args()
    try:
        return run(args)
    except (ContactLabError, OSError, subprocess.SubprocessError, ValueError) as exc:
        emit({"schema": RUN_SCHEMA, "status": "execution_failed", "error": str(exc),
              "cross_profile_comparison": "deferred_to_private_audit"})
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
