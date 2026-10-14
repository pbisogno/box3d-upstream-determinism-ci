#!/usr/bin/env python3
"""Build and run bounded public Box3D frontier diagnostics on ephemeral Actions runners."""

from __future__ import annotations

import argparse
import base64
import hashlib
import importlib.util
import lzma
import json
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
FRONTIER = ROOT / "frontier"
CI_PATH = ROOT / "ci.py"
CI_SHA256 = "a3ade53fc8bcb6a98d0b1816c2dfcb2c999155c8b54606a68446dd4270f9aa3f"
PIN = "5d83df83ab47172c2c745b44315e7538ead02397"
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
DRIVERS = (
    ("cache", "frontier-cache", "cache"),
    ("cache-plain", "frontier-cache-plain", "cache"),
    ("rollback", "frontier-rollback", "rollback"),
    ("concurrency", "frontier-concurrency", "concurrency"),
    ("numeric", "frontier-numeric", "numeric"),
    ("quality", "frontier-quality", "quality"),
)
RUN_SCHEMA = "box3d-ci-frontier-run-v1"
TRACE_SCHEMA = "box3d-ci-frontier-trace-v1"
CHUNK_SCHEMA = "box3d-ci-frontier-trace-chunk-v1"
CHUNK_CHARS = 19_000
MAX_LINE = 20_000
MAX_TRACE_RAW = 64 * 1024 * 1024
MAX_PROFILE_COMPRESSED = 8 * 1024 * 1024
MAX_DRIVER_OUTPUT = 4 * 1024 * 1024
MAX_BUILD_OUTPUT = 16 * 1024 * 1024
NONFAIL_STATUSES = {"pass", "complete", "success", "observed", "exploratory"}


class FrontierError(RuntimeError):
    pass


def digest(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def emit(record: dict[str, Any]) -> None:
    line = json.dumps(record, sort_keys=True, separators=(",", ":"), ensure_ascii=True).encode("ascii") + b"\n"
    if len(line) > MAX_LINE:
        raise FrontierError("wrapper JSON record exceeds the 20,000-byte log-line limit")
    sys.stdout.buffer.write(line)
    sys.stdout.buffer.flush()


def load_ci():
    source = CI_PATH.read_bytes()
    if digest(source) != CI_SHA256:
        raise FrontierError("existing ci.py source hash changed")
    spec = importlib.util.spec_from_file_location("box3d_frontier_ci_helper", CI_PATH)
    if spec is None or spec.loader is None:
        raise FrontierError("could not load existing public ci.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    if getattr(module, "PINNED_REVISION", None) != PIN:
        raise FrontierError("ci.py does not use the expected upstream pin")
    return module


def runner_temp() -> pathlib.Path:
    if os.environ.get("GITHUB_ACTIONS", "").casefold() != "true":
        raise FrontierError("native frontier runs are allowed only on ephemeral GitHub Actions runners")
    raw = os.environ.get("RUNNER_TEMP")
    if not raw:
        raise FrontierError("RUNNER_TEMP is required")
    root = pathlib.Path(raw).resolve(strict=True)
    if not root.is_dir():
        raise FrontierError("RUNNER_TEMP is not a directory")
    return root


def contained(root: pathlib.Path, candidate: pathlib.Path) -> pathlib.Path:
    root = root.resolve(strict=True)
    candidate = candidate.resolve(strict=False)
    try:
        candidate.relative_to(root)
    except ValueError as exc:
        raise FrontierError("temporary path escaped RUNNER_TEMP") from exc
    return candidate


def validate(args: argparse.Namespace, ci: Any) -> pathlib.Path:
    temp = runner_temp()
    if args.origin not in COMPILER or args.compiler != COMPILER[args.origin]:
        raise FrontierError("origin and compiler do not match a supported public profile")
    system, arch = HOST[args.origin]
    if platform.system() != system or ci.normalized_arch(platform.machine()) != arch:
        raise FrontierError("origin does not match this runner's OS and architecture")
    if args.config not in ("Debug", "RelWithDebInfo"):
        raise FrontierError("unsupported build configuration")
    if args.simd not in (4, 8) or args.workers not in (1, 2, 4):
        raise FrontierError("unsupported SIMD or worker profile")
    if not 1 <= args.parallel <= 8 or not 1 <= args.timeout <= 1800:
        raise FrontierError("parallel must be 1..8 and timeout 1..1800 seconds")
    ci.validate_compiler_selection(args.compiler)
    ci.build_flags(args.config, args.simd)
    if args.simd == 8:
        ci.require_avx2_support()
    ci.verified_source(ROOT / "upstream")
    return temp


def validate_compiler_ids(origin: str, ids: dict[str, str]) -> None:
    values = [str(ids.get(key, "")).strip().casefold() for key in ("c", "cxx")]
    if any(not value for value in values):
        raise FrontierError("CMake did not identify both C and CXX compilers")
    if origin == "windows-msvc-x64":
        valid = all(v.startswith("msvc") for v in values)
    elif origin == "linux-gcc-x64":
        valid = all(v.startswith("gnu") for v in values)
    elif origin == "linux-clang-arm64":
        valid = all(v.startswith("clang") for v in values)
    else:
        valid = all(v.startswith(("appleclang", "clang")) for v in values)
    if not valid:
        raise FrontierError("CMake compiler IDs do not match the selected profile")


def run_bounded(argv: list[str], *, cwd: pathlib.Path, out_path: pathlib.Path,
                err_path: pathlib.Path, timeout: int, limit: int) -> dict[str, Any]:
    started = time.monotonic()
    killed = None
    with out_path.open("wb") as out, err_path.open("wb") as err:
        try:
            proc = subprocess.Popen(argv, cwd=str(cwd), stdin=subprocess.DEVNULL,
                                    stdout=out, stderr=err, shell=False)
        except OSError as exc:
            raise FrontierError("could not start " + pathlib.Path(argv[0]).name + ": " + str(exc)) from exc
        end = started + timeout
        while proc.poll() is None:
            if time.monotonic() >= end:
                killed = "timeout"
                proc.kill()
                break
            if out_path.stat().st_size > limit or err_path.stat().st_size > limit:
                killed = "output_limit"
                proc.kill()
                break
            time.sleep(0.1)
        try:
            code = proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            proc.kill()
            code = proc.wait(timeout=10)
    elapsed = round(time.monotonic() - started, 3)
    out_size, err_size = out_path.stat().st_size, err_path.stat().st_size
    overflow = killed == "output_limit" or out_size > limit or err_size > limit
    out_bytes = out_path.read_bytes() if out_size <= limit else b""
    err_bytes = err_path.read_bytes() if err_size <= limit else b""
    return {
        "returncode": code, "timed_out": killed == "timeout", "overflow": overflow,
        "elapsed_seconds": elapsed, "stdout": out_bytes, "stderr": err_bytes,
        "stdout_bytes": out_size, "stderr_bytes": err_size,
        "stdout_sha256": digest(out_bytes) if out_size <= limit else None,
        "stderr_sha256": digest(err_bytes) if err_size <= limit else None,
    }


def log_phase(name: str, result: dict[str, Any]) -> None:
    emit({"schema": "box3d-ci-frontier-phase-v1", "phase": name,
          "returncode": result["returncode"], "timed_out": result["timed_out"],
          "output_limit": result["overflow"], "elapsed_seconds": result["elapsed_seconds"],
          "stdout_bytes": result["stdout_bytes"], "stdout_sha256": result["stdout_sha256"],
          "stderr_bytes": result["stderr_bytes"], "stderr_sha256": result["stderr_sha256"]})
    if result["returncode"] or result["timed_out"] or result["overflow"]:
        tail = (result["stdout"] + result["stderr"])[-8_000:].decode("utf-8", errors="replace")
        if tail:
            sys.stdout.write(tail + ("" if tail.endswith("\n") else "\n"))


def executable(build: pathlib.Path, target: str) -> pathlib.Path:
    suffix = ".exe" if os.name == "nt" else ""
    matches = list((build / "bin").rglob(target + suffix))
    if len(matches) != 1 or not matches[0].is_file():
        raise FrontierError("expected exactly one executable for target " + target)
    return matches[0].resolve(strict=True)


def parse_jsonl(raw: bytes, profile: str, expected_driver: str) -> tuple[list[dict[str, Any]], list[str]]:
    errors: list[str] = []
    try:
        text = raw.decode("utf-8", errors="strict")
    except UnicodeDecodeError as exc:
        return [], [f"stdout is not UTF-8: {exc}"]
    lines = text.splitlines()
    if not lines:
        return [], ["driver emitted no JSONL records"]
    records: list[dict[str, Any]] = []
    for index, line in enumerate(lines):
        if not line:
            errors.append(f"blank JSONL record at line {index}")
            continue
        if len(line.encode("utf-8")) > MAX_LINE:
            errors.append(f"JSONL line {index} exceeds {MAX_LINE} bytes")
            continue
        try:
            row = json.loads(line)
        except json.JSONDecodeError as exc:
            errors.append(f"invalid JSONL at line {index}: {exc}")
            continue
        if not isinstance(row, dict):
            errors.append(f"JSONL line {index} is not an object")
            continue
        if row.get("schema") != "box3d-frontier-v1":
            errors.append(f"unexpected JSON schema at line {index}")
        if row.get("profile") != profile:
            errors.append(f"profile mismatch at line {index}")
        name = row.get("frontier", row.get("driver"))
        allowed = {"cache" if expected_driver == "cache-plain" else expected_driver}
        if name is None or name not in allowed:
            errors.append(f"driver identity mismatch at line {index}")
        if "workers_requested" in row and row["workers_requested"] != int(profile.rsplit("/w", 1)[1]):
            errors.append(f"workers requested mismatch at line {index}")
        status = row.get("status")
        if status is not None and str(status).casefold() not in NONFAIL_STATUSES:
            errors.append(f"driver reported a failing or unrecognized status at line {index}")
        records.append(row)
    return records, errors


def validate_driver_records(records: list[dict[str, Any]], label: str, workers: int,
                            simd: int) -> list[str]:
    errors: list[str] = []
    if not records:
        return ["driver emitted no records"]
    if label in ("cache", "cache-plain"):
        trials = [row for row in records if row.get("type") == "trial"]
        if len(trials) != 14 or len(trials) != len(records):
            errors.append(f"cache driver must emit exactly 14 trial records; got {len(trials)} of {len(records)}")
        expected_roster = {
            (case, mode) for case in ("contacts", "joints", "lifecycle")
            for mode in ("none", "zero_sign", "positive")
        } | {("torsion", mode) for mode in ("none", "positive", "ulp")} | {
            ("overflow", mode) for mode in ("none", "positive")
        }
        expected_instrumented = label == "cache"
        keys: set[tuple[str, str, int]] = set()
        for row in trials:
            case, mode, trial = row.get("case"), row.get("mode"), row.get("trial")
            if not isinstance(case, str) or not isinstance(mode, str) or type(trial) is not int:
                errors.append("cache trial key has invalid types")
                continue
            key = (case, mode, trial)
            if key in keys:
                errors.append("cache driver emitted a duplicate (case, mode, trial) key")
            keys.add(key)
            if row.get("status") != "complete" or row.get("instrumented") is not expected_instrumented:
                errors.append("cache trial status or instrumentation identity is incorrect")
            if row.get("workers_requested") != workers or row.get("simd_actual") != simd:
                errors.append("cache trial worker or SIMD profile does not match the requested profile")
            if row.get("workers_actual") != workers:
                errors.append("cache trial did not use the requested worker count")
            gates = row.get("gates")
            if not isinstance(gates, dict):
                errors.append("cache trial is missing its gates object")
            else:
                required_true = ("activation", "locality_and_readback", "reference_sham_exact",
                                 "prefix_exact", "instrumentation_contract")
                if any(gates.get(name) is not True for name in required_true):
                    errors.append("cache trial failed an activation, locality, control, prefix, or contract gate")
                hook_fields = ("all_hook_stages", "raw_loaded_matches", "prepared_used_matches")
                expected_hook_value = expected_instrumented
                if any(gates.get(name) is not expected_hook_value for name in hook_fields):
                    errors.append("cache trial hook gates do not match the selected instrumented/plain target")
            projections = row.get("body_projection_digests")
            if (not isinstance(projections, list) or len(projections) != 3 or
                    any(not isinstance(item, str) or len(item) != 16 or
                        any(ch not in "0123456789abcdefABCDEF" for ch in item) for item in projections)):
                errors.append("cache trial lacks three valid body projection digests")
        if {(case, mode) for case, mode, _ in keys} != expected_roster or \
                {trial for _, _, trial in keys} != set(range(1, 15)):
            errors.append("cache trial case/mode roster does not match the declared 14 cases")
    elif label in ("rollback", "concurrency"):
        if label == "concurrency":
            cases = [row.get("case") for row in records]
            if len(cases) != 2 or any(not isinstance(case, str) for case in cases) or \
                    set(cases) != {"islands", "dense"}:
                errors.append("concurrency must emit exactly one islands and one dense case record")
        if label == "rollback" and (len(records) != 1 or records[0].get("case") != "mixed"):
            errors.append("rollback must emit exactly one mixed-case record")
        for row in records:
            if "status" in row and str(row["status"]).casefold() not in ("complete", "pass", "success"):
                errors.append("rollback/concurrency emitted an incomplete result")
            if "workers_requested" in row and row["workers_requested"] != workers:
                errors.append("rollback/concurrency worker count does not match the requested profile")
            if "simd_width_actual" in row and row["simd_width_actual"] != simd:
                errors.append("rollback/concurrency SIMD width does not match the requested profile")
            gates = row.get("gates")
            if isinstance(gates, dict) and any(value is not True for value in gates.values()):
                errors.append("rollback/concurrency has a false or malformed gate")
    elif label in ("numeric", "quality"):
        summaries = [row for row in records if row.get("type") == "summary"]
        if len(summaries) != 1 or str(summaries[0].get("status", "")).casefold() != "pass":
            errors.append("numeric/quality requires exactly one passing summary record")
        if any(row.get("type") == "result" and
               str(row.get("status", "")).casefold() not in NONFAIL_STATUSES for row in records):
            errors.append("numeric/quality has a failing or unknown result status")
        if any("workers_requested" in row and row["workers_requested"] != workers for row in records):
            errors.append("numeric/quality worker count does not match the requested profile")
        if label == "quality" and any("actual_workers" in row and row["actual_workers"] != workers
                                      for row in records):
            errors.append("quality driver did not use the requested worker count")
        if any("simd_actual" in row and row["simd_actual"] != simd for row in records):
            errors.append("numeric/quality reported SIMD width does not match the requested profile")
    return errors


def cache_parity(instrumented: list[dict[str, Any]], plain: list[dict[str, Any]]) -> list[str]:
    def indexed(rows: list[dict[str, Any]]) -> dict[tuple[str, str, int], list[str]]:
        result: dict[tuple[str, str, int], list[str]] = {}
        for row in rows:
            if row.get("type") != "trial":
                continue
            case, mode, trial = row.get("case"), row.get("mode"), row.get("trial")
            if not isinstance(case, str) or not isinstance(mode, str) or type(trial) is not int:
                continue
            key = (case, mode, trial)
            projections = row.get("body_projection_digests")
            if key in result or not isinstance(projections, list):
                continue
            result[key] = projections
        return result
    left, right = indexed(instrumented), indexed(plain)
    errors = []
    if len(left) != 14 or len(right) != 14 or set(left) != set(right):
        errors.append("instrumented/plain cache trial rosters differ or are incomplete")
    for key in sorted(set(left) & set(right)):
        if left[key] != right[key]:
            errors.append(f"body projection digests differ for cache trial {key!r}")
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
    zlib_bytes = zlib.compress(raw, level=9)
    xz_bytes = lzma.compress(raw, format=lzma.FORMAT_XZ, preset=6)
    if len(xz_bytes) < len(zlib_bytes):
        compressed, encoding = xz_bytes, "xz+base64"
    else:
        compressed, encoding = zlib_bytes, "zlib+base64"
    if used + len(compressed) > MAX_PROFILE_COMPRESSED:
        return used, size, len(compressed), "profile compressed traces exceed 8 MiB"
    used += len(compressed)
    encoded = base64.b64encode(compressed).decode("ascii")
    chunks = [encoded[i:i + CHUNK_CHARS] for i in range(0, len(encoded), CHUNK_CHARS)]
    emit({"schema": TRACE_SCHEMA, "profile": profile, "origin": origin, "driver": driver,
          "bytes": len(raw), "sha256": digest(raw), "compressed_bytes": len(compressed),
          "compressed_sha256": digest(compressed), "encoding": encoding,
          "chunk_chars": CHUNK_CHARS, "chunk_count": len(chunks)})
    for index, chunk in enumerate(chunks):
        emit({"schema": CHUNK_SCHEMA, "profile": profile, "origin": origin,
              "driver": driver, "index": index, "payload": chunk})
    return used, size, len(compressed), None


def invoke_driver(label: str, target: str, identity: str, exe: pathlib.Path,
                  scratch: pathlib.Path, temp: pathlib.Path, profile: str,
                  origin: str, workers: int, simd: int, timeout: int, quality_timing: bool,
                  compressed_used: int) -> tuple[dict[str, Any], int, list[dict[str, Any]]]:
    logs = scratch / "logs"
    logs.mkdir(exist_ok=True)
    trace = scratch / (label + ".trace")
    argv = [str(exe), "--profile", profile, "--workers", str(workers),
            "--case", "all", "--trace", str(trace)]
    if label == "quality" and quality_timing:
        argv.append("--timing")
    result = run_bounded(argv, cwd=ROOT, out_path=logs / (label + ".stdout"),
                         err_path=logs / (label + ".stderr"), timeout=timeout,
                         limit=MAX_DRIVER_OUTPUT)
    raw_stdout = result["stdout"]
    if raw_stdout:
        sys.stdout.buffer.write(raw_stdout)
        if not raw_stdout.endswith(b"\n"):
            sys.stdout.buffer.write(b"\n")
        sys.stdout.buffer.flush()
    records, errors = parse_jsonl(raw_stdout, profile, identity)
    errors.extend(validate_driver_records(records, label, workers, simd))
    statuses = [str(r.get("status", "")).casefold() for r in records if "status" in r]
    gates_ok = not errors
    trace_error = None
    trace_raw = trace_compressed = 0
    if result["timed_out"] or result["overflow"]:
        trace_error = "driver exceeded time or output limit"
    else:
        compressed_used, trace_raw, trace_compressed, trace_error = trace_to_logs(
            trace, temp=temp, profile=profile, origin=origin, driver=label,
            used=compressed_used)
    ok = (result["returncode"] == 0 and not result["timed_out"] and not result["overflow"]
          and not errors and gates_ok and trace_error is None)
    summary = {
        "schema": "box3d-ci-frontier-driver-v1", "profile": profile, "driver": label,
        "executable": target, "returncode": result["returncode"],
        "timed_out": result["timed_out"], "output_limit": result["overflow"],
        "elapsed_seconds": result["elapsed_seconds"], "stdout_bytes": result["stdout_bytes"],
        "stdout_sha256": result["stdout_sha256"], "stderr_bytes": result["stderr_bytes"],
        "stderr_sha256": result["stderr_sha256"], "jsonl_records": len(records),
        "reported_statuses": statuses, "jsonl_errors": errors, "reported_gates_pass": gates_ok,
        "trace_bytes": trace_raw, "trace_compressed_bytes": trace_compressed,
        "trace_error": trace_error, "local_execution_gate": ok,
        "cross_host_comparison": "deferred_to_private_audit",
    }
    emit(summary)
    if not ok and result["stderr"]:
        tail = result["stderr"][-8_000:].decode("utf-8", errors="replace")
        sys.stdout.write(tail + ("" if tail.endswith("\n") else "\n"))
    return summary, compressed_used, records


def input_hashes() -> dict[str, str]:
    paths = [
        ROOT / "frontier-cache.c", ROOT / "frontier-rollback.c",
        ROOT / "frontier-concurrency.c", ROOT / "frontier-numeric.c",
        ROOT / "frontier-quality.c", ROOT / "causal-replay.c", CI_PATH,
        ROOT / "causal_ci.py", ROOT / "stress_ci.py", FRONTIER / "CMakeLists.txt",
    ]
    for extra in (FRONTIER / "instrument_cache.py", FRONTIER / "frontier-hook.h"):
        if extra.is_file():
            paths.append(extra)
    missing = [p.relative_to(ROOT).as_posix() for p in paths if not p.is_file()]
    if missing:
        raise FrontierError("frontier source inputs are missing: " + ", ".join(missing))
    return {p.relative_to(ROOT).as_posix(): digest(p.read_bytes()) for p in paths}


def run(args: argparse.Namespace) -> int:
    ci = load_ci()
    temp = validate(args, ci)
    profile = f"{args.origin}/{args.config}/simd{args.simd}/w{args.workers}"
    hashes = input_hashes()
    with tempfile.TemporaryDirectory(prefix="box3d-frontier-", dir=str(temp)) as tmp:
        scratch = pathlib.Path(tmp).resolve(strict=True)
        contained(temp, scratch)
        build = scratch / "build"
        configure_args = (["cmake", "-S", str(FRONTIER), "-B", str(build)]
                          + ci.build_flags(args.config, args.simd)
                          + ci.cmake_compiler_args(args.compiler, None)
                          + ["-DPython3_EXECUTABLE=" + sys.executable])
        configured = run_bounded(configure_args, cwd=ROOT,
                                 out_path=scratch / "configure.stdout",
                                 err_path=scratch / "configure.stderr",
                                 timeout=args.timeout, limit=MAX_BUILD_OUTPUT)
        log_phase("configure", configured)
        if configured["returncode"] or configured["timed_out"] or configured["overflow"]:
            emit({"schema": RUN_SCHEMA, "profile": profile, "status": "failed",
                  "phase": "configure", "source_hashes": hashes})
            return 1
        compiler_ids = ci.cmake_ids(configured["stdout"].decode("utf-8", errors="replace"))
        validate_compiler_ids(args.origin, compiler_ids)
        emit({"schema": "box3d-ci-frontier-configured-v1", "profile": profile,
              "origin": args.origin, "compiler": args.compiler, "config": args.config,
              "simd": args.simd, "compiler_ids": compiler_ids,
              "ci_helper_sha256": CI_SHA256, "upstream_pin": PIN, "source_hashes": hashes})
        summaries = []
        records_by_driver: dict[str, list[dict[str, Any]]] = {}
        build_gates: dict[str, bool] = {}
        compressed_used = 0
        for label, target, identity in DRIVERS:
            build_args = ["cmake", "--build", str(build), "--config", args.config,
                          "--parallel", str(args.parallel), "--target", target]
            built = run_bounded(build_args, cwd=ROOT,
                                out_path=scratch / ("build-" + label + ".stdout"),
                                err_path=scratch / ("build-" + label + ".stderr"),
                                timeout=args.timeout, limit=MAX_BUILD_OUTPUT)
            log_phase("build-" + label, built)
            build_ok = not (built["returncode"] or built["timed_out"] or built["overflow"])
            build_gates[label] = build_ok
            if not build_ok:
                summary = {"schema": "box3d-ci-frontier-driver-v1", "profile": profile,
                           "driver": label, "executable": target, "build_gate": False,
                           "local_execution_gate": False,
                           "error": "target build failed; remaining driver targets will still be attempted",
                           "cross_host_comparison": "deferred_to_private_audit"}
                emit(summary)
                summaries.append(summary)
                records_by_driver[label] = []
                continue
            try:
                exe = executable(build, target)
                summary, compressed_used, records = invoke_driver(
                    label, target, identity, exe, scratch, temp, profile, args.origin,
                    args.workers, args.simd, args.timeout, args.quality_timing, compressed_used)
                records_by_driver[label] = records
            except (FrontierError, OSError, subprocess.SubprocessError) as exc:
                summary = {"schema": "box3d-ci-frontier-driver-v1", "profile": profile,
                           "driver": label, "executable": target, "build_gate": True,
                           "local_execution_gate": False, "error": str(exc),
                           "cross_host_comparison": "deferred_to_private_audit"}
                emit(summary)
                records_by_driver[label] = []
            else:
                summary["build_gate"] = True
            summaries.append(summary)
        parity_errors = cache_parity(records_by_driver.get("cache", []),
                                     records_by_driver.get("cache-plain", []))
        parity_ok = not parity_errors
        emit({"schema": "box3d-ci-frontier-cache-parity-v1", "profile": profile,
              "status": "pass" if parity_ok else "failed", "paired_trials": 14,
              "body_projection_digests_equal": parity_ok, "errors": parity_errors,
              "comparison_scope": "same_profile_instrumented_vs_pristine_cache_twin"})
        passed = (len(summaries) == len(DRIVERS) and all(build_gates.values()) and parity_ok and all(
            row.get("local_execution_gate") is True for row in summaries)
        )
        emit({"schema": RUN_SCHEMA, "profile": profile,
              "status": "pass" if passed else "failed",
              "build_gate": all(build_gates.values()), "build_gates": build_gates,
              "driver_count": len(summaries),
              "driver_gates": {row["driver"]: row.get("local_execution_gate") is True
                               for row in summaries},
              "trace_compressed_bytes": compressed_used,
              "trace_compressed_limit_bytes": MAX_PROFILE_COMPRESSED,
              "quality_timing_requested": args.quality_timing,
              "cross_host_comparison": "deferred_to_private_audit",
              "source_hashes": hashes})
        return 0 if passed else 1


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Run bounded public Box3D frontier tests on ephemeral GitHub Actions runners.")
    subs = parser.add_subparsers(dest="command", required=True)
    cmd = subs.add_parser("run", help="build and execute every frontier diagnostic")
    cmd.add_argument("--origin", choices=tuple(COMPILER), required=True)
    cmd.add_argument("--compiler", choices=("gcc", "clang", "msvc"), required=True)
    cmd.add_argument("--config", choices=("Debug", "RelWithDebInfo"), required=True)
    cmd.add_argument("--simd", type=int, choices=(4, 8), required=True)
    cmd.add_argument("--workers", type=int, choices=(1, 2, 4), required=True)
    cmd.add_argument("--parallel", type=int, default=2)
    cmd.add_argument("--timeout", type=int, default=900)
    cmd.add_argument("--quality-timing", action="store_true")
    args = parser.parse_args()
    try:
        return run(args)
    except (FrontierError, OSError, subprocess.SubprocessError, ValueError) as exc:
        emit({"schema": RUN_SCHEMA, "status": "failed", "error": str(exc),
              "cross_host_comparison": "deferred_to_private_audit"})
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
