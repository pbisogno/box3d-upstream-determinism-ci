from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import os
import pathlib
import platform
import subprocess
import sys
import tempfile
from typing import Any

ROOT = pathlib.Path(__file__).resolve().parent
STRESS_PATH = ROOT / "stress_ci.py"
STRESS_SHA256 = "a9e5f7fb65c0a9de6dc59ea24ef7845f328a6243cfa24659e61121b6978b422d"
TRACE_SCHEMA = "box3d-ci-causal-trace-block-v1"
CHUNK_SCHEMA = "box3d-ci-causal-trace-chunk-v1"
TRACE_BLOCK_LIMIT = 2 * 1024 * 1024
TRACE_CHUNK_CHARS = 20_000
EXPECTED_COMPILER = {
    "windows-msvc-x64": "msvc",
    "linux-gcc-x64": "gcc",
    "linux-clang-arm64": "clang",
    "macos-clang-arm64": "clang",
    "macos-clang-x64": "clang",
}
EXPECTED_HOST = {
    "windows-msvc-x64": ("Windows", "x86_64"),
    "linux-gcc-x64": ("Linux", "x86_64"),
    "linux-clang-arm64": ("Linux", "arm64"),
    "macos-clang-arm64": ("Darwin", "arm64"),
    "macos-clang-x64": ("Darwin", "x86_64"),
}
CASE_IDS = {"contacts": 1, "joints": 2, "lifecycle": 3}
PAIR_SCHEMA = "box3d-causal-pair-v1"
TRIAL_POINTS = (
    ("contacts", 47), ("contacts", 129), ("contacts", 399),
    ("joints", 128), ("joints", 399),
    ("lifecycle", 52), ("lifecycle", 399),
)
PAIR_GATES = (
    "workers", "warmstarting", "finite_observations", "sham_exact", "prefix_exact",
    "activation", "readback", "only_zero_sign_changed", "expected_comparison_counts",
    "solver_eligibility", "positive_control_sensitive",
)
PAIR_EFFECTS = {"no_selected_effect", "signed_zero_only", "nonzero_or_discrete_effect", "not_established"}
PAIR_METRICS = (
    "differing_ticks", "signed_zero_cells", "nonzero_float_cells", "integer_cells", "structure_cells",
    "body_differing_ticks", "body_signed_zero_cells", "body_nonzero_float_cells",
    "body_integer_cells", "body_structure_cells",
)
INTERVENTION_FIELDS = (
    "baseline_eligible", "sham_eligible", "flip_eligible", "flipped", "confirmed",
    "object_bytes_verified", "positive_before", "negative_before", "awake_graph", "sleeping", "other",
    "target_field_tag", "public_alias_verified", "warmstarting", "next_step_retained",
    "next_step_awake_graph", "next_step_warmstarting", "after_step_retained",
    "after_step_sign_retained", "impulse_readback_nonzero",
)
INTERVENTION_BOOL_FIELDS = {
    "public_alias_verified", "warmstarting", "next_step_warmstarting", "impulse_readback_nonzero",
}


def load_frozen_stress():
    source = STRESS_PATH.read_bytes()
    actual = hashlib.sha256(source).hexdigest()
    if actual != STRESS_SHA256:
        raise RuntimeError("frozen stress helper SHA-256 mismatch")
    spec = importlib.util.spec_from_file_location("box3d_frozen_stress_ci", STRESS_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError("could not load frozen stress helper")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


STRESS = load_frozen_stress()


def validate_compiler_ids(origin: str, compiler_ids: dict[str, str]) -> None:
    identifiers = [str(compiler_ids.get(key, "")).strip().lower() for key in ("c", "cxx")]
    if any(not identifier for identifier in identifiers):
        raise RuntimeError("CMake compiler identifiers are missing")
    if origin == "windows-msvc-x64":
        matches = all(identifier.startswith("msvc") for identifier in identifiers)
    elif origin == "linux-gcc-x64":
        matches = all(identifier.startswith("gnu") for identifier in identifiers)
    elif origin == "linux-clang-arm64":
        matches = all(identifier.startswith("clang") for identifier in identifiers)
    else:
        matches = all(identifier.startswith(("appleclang", "clang")) for identifier in identifiers)
    if not matches:
        raise RuntimeError("CMake compiler identifiers do not match the selected public runner profile")


def emit_block(origin: str, case: str, block_index: int, start_state: int,
               state_bytes: list[bytes], expected_block_hash: str) -> dict[str, Any]:
    state_count = len(state_bytes)
    if state_count < 1 or state_count > STRESS.FIRST_BLOCK_STATES:
        raise RuntimeError("trace block has an invalid state count")
    if start_state != block_index * STRESS.FIRST_BLOCK_STATES:
        raise RuntimeError("trace block start does not match its sequence index")
    payload = b"".join(state_bytes)
    block_hash = STRESS.sha256(payload)
    if block_hash != expected_block_hash:
        raise RuntimeError("trace block payload hash does not match the frozen parser")
    frames = state_count - 1
    raw = STRESS.HEADER.pack(STRESS.TRACE_MAGIC, CASE_IDS[case], start_state, frames) + payload
    if len(raw) > TRACE_BLOCK_LIMIT:
        raise RuntimeError("trace block raw envelope exceeds 2 MiB")
    encoded = STRESS.encode_bytes(raw)
    if len(encoded.encode("ascii")) > TRACE_BLOCK_LIMIT:
        raise RuntimeError("trace block encoded envelope exceeds 2 MiB")
    chunks = [encoded[offset:offset + TRACE_CHUNK_CHARS]
              for offset in range(0, len(encoded), TRACE_CHUNK_CHARS)]
    if not chunks or any(not chunk or len(chunk) > TRACE_CHUNK_CHARS for chunk in chunks):
        raise RuntimeError("trace block could not be split into bounded log chunks")

    summary = {
        "schema": TRACE_SCHEMA,
        "case": case,
        "origin": origin,
        "block_index": block_index,
        "start_state": start_state,
        "state_count": state_count,
        "frames": frames,
        "bytes": len(raw),
        "sha256": STRESS.sha256(raw),
        "block_sha256": block_hash,
        "encoding": "zlib+base64",
        "chunk_chars": TRACE_CHUNK_CHARS,
        "chunk_count": len(chunks),
    }
    print(json.dumps(summary, sort_keys=True, separators=(",", ":")), flush=True)
    for index, chunk in enumerate(chunks):
        print(json.dumps({
            "schema": CHUNK_SCHEMA,
            "case": case,
            "origin": origin,
            "start_state": start_state,
            "index": index,
            "payload": chunk,
        }, sort_keys=True, separators=(",", ":")), flush=True)
    return summary


def runner_temp() -> pathlib.Path:
    if os.environ.get("GITHUB_ACTIONS", "").lower() != "true":
        raise RuntimeError("native capture/intervention is restricted to ephemeral GitHub Actions runners")
    raw = os.environ.get("RUNNER_TEMP")
    if not raw:
        raise RuntimeError("RUNNER_TEMP is required")
    path = pathlib.Path(raw).resolve()
    if not path.is_dir():
        raise RuntimeError("RUNNER_TEMP is not an existing directory")
    return path


def validate_profile(args: argparse.Namespace, *, require_builddir: bool = False) -> pathlib.Path:
    runner_root = runner_temp()
    if args.origin not in EXPECTED_COMPILER:
        raise RuntimeError("origin is not one of the five supported public runner labels")
    if args.compiler != EXPECTED_COMPILER[args.origin]:
        raise RuntimeError("origin and compiler profile do not match")
    expected_system, expected_arch = EXPECTED_HOST[args.origin]
    actual_arch = STRESS.CI.normalized_arch(platform.machine())
    if platform.system() != expected_system or actual_arch != expected_arch:
        raise RuntimeError("origin does not match the current runner operating system and architecture")
    if args.simd not in (4, 8) or args.workers not in (1, 2, 4):
        raise RuntimeError("unsupported SIMD or worker profile")
    if args.parallel < 1 or args.timeout < 1:
        raise RuntimeError("parallel and timeout values must be positive")
    STRESS.CI.validate_compiler_selection(args.compiler)
    STRESS.CI.build_flags(args.config, args.simd)
    if args.simd == 8 and args.command == "intervene":
        STRESS.CI.require_avx2_support()
    if require_builddir:
        build_component = pathlib.PurePath(args.builddir)
        if (build_component.is_absolute() or len(build_component.parts) != 1 or
                build_component.parts[0] in (".", "..")):
            raise RuntimeError("builddir must be one child-directory name under the runner temp directory")
    return runner_root


def capture(args: argparse.Namespace) -> int:
    runner_root = validate_profile(args, require_builddir=True)
    case_summaries = []
    total_blocks = 0
    with tempfile.TemporaryDirectory(prefix="box3d-causal-capture-", dir=str(runner_root)) as scratch:
        scratch_root = pathlib.Path(scratch)
        args.build = str(scratch_root / args.builddir)
        executable, compiler_ids = STRESS.get_driver(args, args.simd)
        validate_compiler_ids(args.origin, compiler_ids)
        for case in STRESS.CASES:
            checkpoint = STRESS.CHECKPOINTS[case]
            case_dir = scratch_root / "captures" / case
            metadata, recording_path, reference_path, trace_path = STRESS.invoke_capture(
                executable, case, checkpoint, args.workers, case_dir, args.timeout)
            full = STRESS.parse_trace(trace_path, expected_case=case,
                                      expected_checkpoint=0, expected_frames=4096)
            encoded_states = full.get("encodedStates")
            block_hashes = full.get("blockHashes")
            if (full.get("frames") != 4096 or len(encoded_states) != 4097 or
                    len(block_hashes) != 9 or len(full.get("states")) != 4097):
                raise RuntimeError("frozen parser did not return the complete 4097-state trace")
            emitted = []
            for block_index, start_state in enumerate(range(0, 4097, STRESS.FIRST_BLOCK_STATES)):
                state_bytes = encoded_states[start_state:start_state + STRESS.FIRST_BLOCK_STATES]
                expected = block_hashes[block_index]
                if (expected.get("firstState") != start_state or
                        expected.get("stateCount") != len(state_bytes)):
                    raise RuntimeError("frozen parser block boundaries are inconsistent")
                emitted.append(emit_block(args.origin, case, block_index, start_state,
                                          state_bytes, expected["sha256"]))
            if len(emitted) != 9 or emitted[-1]["state_count"] != 1:
                raise RuntimeError("trace did not produce eight 512-state blocks and one final state")
            total_blocks += len(emitted)
            case_summaries.append({
                "case": case,
                "checkpoint": checkpoint,
                "capture_status": metadata["status"],
                "capture_gates": metadata["gates"],
                "observations": metadata["observations"],
                "worker_checks": metadata["worker_checks"],
                "workers_actual": metadata["workers_actual"],
                "recording_bytes": recording_path.stat().st_size,
                "reference_bytes": reference_path.stat().st_size,
                "full_trace_bytes": full["bytes"],
                "full_trace_sha256": full["sha256"],
                "full_trace_states": len(encoded_states),
                "block_sha256": [block["sha256"] for block in block_hashes],
                "emitted_block_count": len(emitted),
            })
    passed = len(case_summaries) == 3 and total_blocks == 27 and all(
        row["capture_status"] == "pass" and row["workers_actual"] == args.workers and
        row["full_trace_states"] == 4097 and row["emitted_block_count"] == 9
        for row in case_summaries)
    summary = {
        "schema": "box3d-ci-causal-capture-v1",
        "revision": STRESS.PIN,
        "origin": args.origin,
        "configuration": args.config,
        "simd": args.simd,
        "workers_requested": args.workers,
        "workers_actual": args.workers,
        "cmake_compilers": compiler_ids,
        "cases": case_summaries,
        "full_trace_blocks": total_blocks,
        "status": "complete" if passed else "gate_failed",
    }
    print(json.dumps(summary, sort_keys=True, separators=(",", ":")), flush=True)
    return 0 if passed else 1


def build_causal_driver(args: argparse.Namespace, build_dir: pathlib.Path) -> tuple[pathlib.Path, dict[str, str]]:
    source = (ROOT / "upstream").resolve()
    STRESS.CI.verified_source(source)
    STRESS.CI.validate_compiler_selection(args.compiler)
    if not STRESS.CI.shutil.which("cmake"):
        raise RuntimeError("CMake is not available on PATH")
    configure = ["cmake", "-S", str(ROOT / "causal"), "-B", str(build_dir)]
    configure += STRESS.CI.build_flags(args.config, args.simd)
    configure += STRESS.CI.cmake_compiler_args(args.compiler, args.generator)
    configured = STRESS.run(configure, args.timeout)
    if configured.returncode:
        print(configured.stdout[-12000:], file=sys.stderr, end="")
        raise RuntimeError("causal-replay CMake configure failed")
    compiler_ids = STRESS.CI.cmake_ids(configured.stdout)
    validate_compiler_ids(args.origin, compiler_ids)
    built = STRESS.run(["cmake", "--build", str(build_dir), "--config", args.config,
                        "--target", "causal-replay", "--parallel", str(args.parallel)], args.timeout)
    if built.returncode:
        print(built.stdout[-12000:], file=sys.stderr, end="")
        raise RuntimeError("causal-replay build failed")
    config_dir = args.config if platform.system() == "Windows" else ""
    executable = build_dir / "bin" / config_dir / (
        "causal-replay.exe" if platform.system() == "Windows" else "causal-replay")
    if not executable.is_file() and platform.system() == "Windows":
        executable = build_dir / "bin" / "causal-replay.exe"
    if not executable.is_file():
        raise RuntimeError("built causal-replay executable was not found")
    return executable.resolve(), compiler_ids


def parse_pair_output(stdout: str) -> tuple[str, dict[str, Any]]:
    matches = []
    for line in stdout.splitlines():
        candidate = line.strip()
        if not candidate.startswith("{"):
            continue
        try:
            value = json.loads(candidate)
        except json.JSONDecodeError:
            continue
        if isinstance(value, dict) and value.get("schema") == PAIR_SCHEMA:
            matches.append((candidate, value))
    if len(matches) != 1:
        raise RuntimeError("causal-replay did not emit exactly one paired-result JSON line")
    return matches[0]


def compact_pair(pair: dict[str, Any], args: argparse.Namespace, compiler_ids: dict[str, str],
                 exit_code: int, native_line: str) -> tuple[dict[str, Any], bool]:
    errors = []
    expected_kind = "positive_control" if args.scope == "impulse" else "cache_zero_sign"
    expected_counts = {
        "steps_completed": args.horizon,
        "workers_requested": args.workers,
        "control_states_compared": args.horizon + 2,
        "pre_intervention_states_compared": args.tick + 1,
        "future_states_compared": args.horizon - args.tick,
    }
    expected_fields = {
        "case": args.case,
        "scope": args.scope,
        "intervention_kind": expected_kind,
        "intervention_tick": args.tick,
        "horizon": args.horizon,
    }
    for key, expected in expected_fields.items():
        if pair.get(key) != expected:
            errors.append(f"{key}_mismatch")
    for key, expected in expected_counts.items():
        if pair.get(key) != expected:
            errors.append(f"{key}_mismatch")
    actual_workers = pair.get("workers_actual")
    if actual_workers != [args.workers, args.workers, args.workers]:
        errors.append("workers_actual_mismatch")
    if pair.get("effect") not in PAIR_EFFECTS:
        errors.append("effect_invalid")
    gates = pair.get("gates")
    if not isinstance(gates, dict) or any(gates.get(name) is not True for name in PAIR_GATES):
        errors.append("one_or_more_native_gates_failed")
    applicability = pair.get("gate_applicability")
    expected_applicability = {
        "only_zero_sign_changed": args.scope != "impulse",
        "solver_eligibility": args.scope != "impulse",
        "positive_control_sensitive": args.scope == "impulse",
    }
    if applicability != expected_applicability:
        errors.append("gate_applicability_mismatch")
    if pair.get("status") == "complete" and pair.get("effect") == "not_established":
        errors.append("effect_not_established")
    if pair.get("status") != "complete" or exit_code != 0:
        errors.append("native_trial_incomplete")

    intervention = pair.get("intervention")
    if not isinstance(intervention, dict):
        intervention = {}
        errors.append("intervention_metrics_missing")
    else:
        missing = [name for name in INTERVENTION_FIELDS if name not in intervention]
        if missing:
            errors.append("intervention_metrics_incomplete")
        for name in INTERVENTION_FIELDS:
            if name not in intervention:
                continue
            value = intervention[name]
            if name in INTERVENTION_BOOL_FIELDS:
                if type(value) is not bool:
                    errors.append("intervention_metric_type_invalid")
                    break
            elif type(value) is not int or value < 0:
                errors.append("intervention_metric_type_invalid")
                break
    immediate = pair.get("immediate")
    future = pair.get("future")
    if not isinstance(immediate, dict) or not isinstance(future, dict):
        immediate = immediate if isinstance(immediate, dict) else {}
        future = future if isinstance(future, dict) else {}
        errors.append("comparison_metrics_missing")
    else:
        for label, section in (("immediate", immediate), ("future", future)):
            if any(type(section.get(name)) is not int or section[name] < 0 for name in PAIR_METRICS):
                errors.append(f"{label}_metrics_incomplete")
    if args.scope == "impulse":
        if (args.case != "contacts" or args.tick != 129 or
                intervention.get("impulse_readback_nonzero") is not True or
                future.get("body_nonzero_float_cells", 0) <= 0):
            errors.append("positive_control_sensitivity_missing")
    else:
        if (intervention.get("flipped", 0) <= 0 or
                intervention.get("confirmed", 0) != intervention.get("flipped") or
                intervention.get("object_bytes_verified", 0) != intervention.get("flipped")):
            errors.append("cache_flip_readback_missing")

    summary = {
        "schema": "box3d-ci-causal-intervention-row-v1",
        "revision": STRESS.PIN,
        "origin": args.origin,
        "configuration": args.config,
        "simd": args.simd,
        "workers_requested": args.workers,
        "cmake_compilers": compiler_ids,
        "case": pair.get("case"),
        "scope": pair.get("scope"),
        "intervention_kind": pair.get("intervention_kind"),
        "intervention_tick": pair.get("intervention_tick"),
        "horizon": pair.get("horizon"),
        "steps_completed": pair.get("steps_completed"),
        "workers_actual": actual_workers,
        "control_states_compared": pair.get("control_states_compared"),
        "pre_intervention_states_compared": pair.get("pre_intervention_states_compared"),
        "future_states_compared": pair.get("future_states_compared"),
        "effect": pair.get("effect"),
        "gate_applicability": applicability,
        "gates": gates,
        "intervention": {key: intervention.get(key) for key in INTERVENTION_FIELDS},
        "immediate": {key: immediate.get(key) for key in PAIR_METRICS},
        "future": {key: future.get(key) for key in PAIR_METRICS},
        "control_first_difference": pair.get("control_first_difference"),
        "prefix_first_difference": pair.get("prefix_first_difference"),
        "nonfinite_tick": pair.get("nonfinite_tick"),
        "nonfinite_field_tag": pair.get("nonfinite_field_tag"),
        "native_exit_code": exit_code,
        "native_json_sha256": hashlib.sha256(native_line.encode("utf-8")).hexdigest(),
        "validation_errors": errors,
        "status": "complete" if not errors else "gate_failed",
    }
    return summary, not errors


def trial_schedule() -> list[tuple[str, str, int]]:
    return [(case, scope, tick) for case, tick in TRIAL_POINTS for scope in ("single", "all")] + [
        ("contacts", "impulse", 129)
    ]


def trial_error_row(args: argparse.Namespace, trial: tuple[str, str, int],
                    compiler_ids: dict[str, str] | None, status: str, detail: str,
                    exit_code: int | None = None) -> dict[str, Any]:
    case, scope, tick = trial
    detail = detail[:400]
    for private_path in (os.environ.get("RUNNER_TEMP", ""), str(ROOT)):
        if private_path:
            detail = detail.replace(private_path, "<runner-temp>")
    return {
        "schema": "box3d-ci-causal-intervention-row-v1",
        "revision": STRESS.PIN,
        "origin": args.origin,
        "configuration": args.config,
        "simd": args.simd,
        "workers_requested": args.workers,
        "cmake_compilers": compiler_ids,
        "case": case,
        "scope": scope,
        "intervention_tick": tick,
        "horizon": args.horizon,
        "status": status,
        "native_exit_code": exit_code,
        "validation_errors": [detail],
    }


def intervene(args: argparse.Namespace) -> int:
    runner_root = validate_profile(args)
    trials = trial_schedule()
    compiler_ids = None
    executable = None
    build_error = None
    with tempfile.TemporaryDirectory(prefix="box3d-causal-intervene-", dir=str(runner_root)) as scratch:
        try:
            executable, compiler_ids = build_causal_driver(args, pathlib.Path(scratch) / "build")
        except (OSError, RuntimeError, ValueError, KeyError, TypeError, subprocess.SubprocessError) as exc:
            build_error = f"{type(exc).__name__}: {exc}"
        rows = []
        for case, scope, tick in trials:
            trial = (case, scope, tick)
            trial_values = dict(vars(args))
            trial_values.update(case=case, scope=scope, tick=tick)
            trial_args = argparse.Namespace(**trial_values)
            if build_error is not None or executable is None:
                row = trial_error_row(trial_args, trial, compiler_ids,
                                      "not_run_build_failed", build_error or "causal-replay unavailable")
                rows.append(row)
                print(json.dumps(row, sort_keys=True, separators=(",", ":")), flush=True)
                continue
            command = [str(executable), "--case", case, "--scope", scope,
                       "--tick", str(tick), "--workers", str(args.workers),
                       "--horizon", str(args.horizon)]
            result = None
            try:
                result = STRESS.run(command, args.timeout)
                native_line, pair = parse_pair_output(result.stdout)
                print(native_line, flush=True)
                row, _ = compact_pair(pair, trial_args, compiler_ids, result.returncode, native_line)
            except (OSError, RuntimeError, ValueError, KeyError, TypeError, subprocess.SubprocessError) as exc:
                exit_code = None if result is None else result.returncode
                row = trial_error_row(trial_args, trial, compiler_ids,
                                      "process_or_json_error", f"{type(exc).__name__}: {exc}", exit_code)
            rows.append(row)
            print(json.dumps(row, sort_keys=True, separators=(",", ":")), flush=True)
    passed = (len(rows) == 15 and len(trials) == 15 and build_error is None and
              all(row.get("status") == "complete" for row in rows))
    summary = {
        "schema": "box3d-ci-causal-intervene-v1",
        "revision": STRESS.PIN,
        "origin": args.origin,
        "configuration": args.config,
        "simd": args.simd,
        "workers_requested": args.workers,
        "horizon": args.horizon,
        "cmake_compilers": compiler_ids,
        "expected_trials": 15,
        "executed_trials": sum(row.get("status") != "not_run_build_failed" for row in rows),
        "completed_trials": sum(row.get("status") == "complete" for row in rows),
        "failed_trials": sum(row.get("status") != "complete" for row in rows),
        "trials": [{"case": row.get("case"), "scope": row.get("scope"),
                    "intervention_tick": row.get("intervention_tick"), "status": row.get("status"),
                    "effect": row.get("effect"), "native_exit_code": row.get("native_exit_code"),
                    "validation_errors": row.get("validation_errors", [])} for row in rows],
        "status": "complete" if passed else "gate_failed",
    }
    print(json.dumps(summary, sort_keys=True, separators=(",", ":")), flush=True)
    return 0 if passed else 1

def positive_int(value: str) -> int:
    parsed = int(value)
    if parsed < 1:
        raise argparse.ArgumentTypeError("value must be positive")
    return parsed


def add_profile_options(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--origin", choices=tuple(EXPECTED_COMPILER), required=True)
    parser.add_argument("--compiler", choices=("gcc", "clang", "msvc", "clang-cl"), required=True)
    parser.add_argument("--config", choices=("Debug", "RelWithDebInfo"), default="RelWithDebInfo")
    parser.add_argument("--simd", type=int, choices=(4, 8), default=4)
    parser.add_argument("--workers", type=int, choices=(1, 2, 4), default=1)
    parser.add_argument("--parallel", type=positive_int, default=2)
    parser.add_argument("--timeout", type=positive_int, default=900)
    parser.add_argument("--generator")


def main() -> int:
    parser = argparse.ArgumentParser(description="Bounded public Box3D causal capture/replay CI helper.")
    sub = parser.add_subparsers(dest="command", required=True)
    capture_parser = sub.add_parser("capture", help="capture and emit all exact states from the three public scenes")
    add_profile_options(capture_parser)
    capture_parser.add_argument("--builddir", default="causal-capture-build")
    capture_parser.set_defaults(func=capture)
    intervention_parser = sub.add_parser("intervene", help="run all fifteen paired public causal intervention trials")
    add_profile_options(intervention_parser)
    intervention_parser.add_argument("--horizon", type=positive_int, default=8192)
    intervention_parser.set_defaults(func=intervene)
    args = parser.parse_args()
    try:
        return args.func(args)
    except (OSError, RuntimeError, ValueError, KeyError, TypeError, subprocess.SubprocessError) as exc:
        detail = str(exc)
        for private_path in (os.environ.get("RUNNER_TEMP", ""), str(ROOT)):
            if private_path:
                detail = detail.replace(private_path, "<runner-temp>")
        print(json.dumps({"schema": "box3d-ci-causal-error-v1",
                          "error_type": type(exc).__name__, "detail": detail[:600]},
                         sort_keys=True, separators=(",", ":")), file=sys.stderr, flush=True)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())