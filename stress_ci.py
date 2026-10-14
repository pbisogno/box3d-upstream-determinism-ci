from __future__ import annotations

import argparse
import base64
import hashlib
import importlib.util
import json
import os
import pathlib
import platform
import re
import struct
import subprocess
import sys
import tempfile
import zlib
from typing import Any

ROOT = pathlib.Path(__file__).resolve().parent
PIN = "5d83df83ab47172c2c745b44315e7538ead02397"
CASES = ("contacts", "joints", "lifecycle")
CHECKPOINTS = {"contacts": 64, "joints": 2048, "lifecycle": 3968}
ORIGINS = {
    "PUBLIC_RECORD_WINDOWS": "windows-msvc-x64",
    "PUBLIC_RECORD_LINUX_X64": "linux-gcc-x64",
    "PUBLIC_RECORD_LINUX_ARM64": "linux-clang-arm64",
    "PUBLIC_RECORD_MACOS_ARM64": "macos-clang-arm64",
    "PUBLIC_RECORD_MACOS_INTEL": "macos-clang-x64",
}
RAW_LIMIT = 300 * 1024
CHUNK_SIZE = 20000
CHUNK_COUNT = 6
RECORD_LIMIT = CHUNK_SIZE * CHUNK_COUNT
EMPTY_CHUNK_SENTINEL = "."
HEADER = struct.Struct("<IIII")
STATE_COUNT = struct.Struct("<I")
CELL = struct.Struct("<II")
TRACE_MAGIC = 0x31525453  # STR1
CAPTURE_SCHEMA = "box3d-stress-capture-v1"
VERIFY_SCHEMA = "box3d-stress-verify-v1"
RECORD_SCHEMA = "box3d-public-stress-recording-v1"


def load_ci():
    path = ROOT / "ci.py"
    spec = importlib.util.spec_from_file_location("box3d_ci", path)
    if spec is None or spec.loader is None:
        raise RuntimeError("could not load the existing generic ci.py helper")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


CI = load_ci()


def sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def run(argv: list[str], timeout: int) -> subprocess.CompletedProcess[str]:
    return CI.run(argv, timeout=timeout)


def parse_json_line(stdout: str, expected_schema: str) -> dict[str, Any]:
    lines = [line.strip() for line in stdout.splitlines() if line.strip()]
    objects = []
    for line in lines:
        if not line.startswith("{"):
            continue
        try:
            value = json.loads(line)
        except json.JSONDecodeError:
            continue
        if isinstance(value, dict):
            objects.append(value)
    if len(objects) != 1 or objects[0].get("schema") != expected_schema:
        raise RuntimeError(f"expected exactly one {expected_schema} JSON object from stress driver")
    return objects[0]


def emit_driver_failure(action: str, case: str, exit_code: int,
                        driver_result: dict[str, Any] | None, *, gate_failed: bool = False) -> None:
    """Expose bounded public driver telemetry when a native gate fails."""
    print(json.dumps({"schema": "box3d-ci-stress-driver-failure-v1", "action": action,
                      "case": case, "exitCode": exit_code, "gateValidationFailed": gate_failed,
                      "driver_result": driver_result}, sort_keys=True, separators=(",", ":")))


def get_driver(args: argparse.Namespace, simd: int) -> tuple[pathlib.Path, dict[str, str]]:
    source = (ROOT / "upstream").resolve()
    CI.verified_source(source)
    CI.validate_compiler_selection(args.compiler)
    if not CI.shutil.which("cmake"):
        raise RuntimeError("CMake is not available on PATH")
    if simd == 8:
        CI.require_avx2_support()
    build_arg = pathlib.Path(args.build)
    build = (ROOT / build_arg).resolve() if not build_arg.is_absolute() else build_arg.resolve()
    configure = ["cmake", "-S", str(ROOT), "-B", str(build)] + CI.build_flags(args.config, simd)
    configure += CI.cmake_compiler_args(args.compiler, args.generator)
    configured = run(configure, args.timeout)
    if configured.returncode:
        print(configured.stdout[-12000:], file=sys.stderr, end="")
        raise RuntimeError("stress-replay CMake configure failed")
    built = run(["cmake", "--build", str(build), "--config", args.config,
                 "--target", "stress-replay", "--parallel", str(args.parallel)], args.timeout)
    if built.returncode:
        print(built.stdout[-12000:], file=sys.stderr, end="")
        raise RuntimeError("stress-replay build failed")
    config_dir = args.config if platform.system() == "Windows" else ""
    exe = build / "bin" / config_dir / ("stress-replay.exe" if platform.system() == "Windows" else "stress-replay")
    if not exe.is_file() and platform.system() == "Windows":
        exe = build / "bin" / "stress-replay.exe"
    if not exe.is_file():
        raise RuntimeError("built stress-replay executable was not found")
    return exe.resolve(), CI.cmake_ids(configured.stdout)


def parse_trace(path: pathlib.Path, *, expected_case: str, expected_checkpoint: int | None,
                expected_frames: int) -> dict[str, Any]:
    data = path.read_bytes()
    if len(data) < HEADER.size:
        raise RuntimeError(f"{path.name} is shorter than the trace header")
    magic, case_id, checkpoint, frames = HEADER.unpack_from(data)
    case_ids = {"contacts": 1, "joints": 2, "lifecycle": 3}
    if magic != TRACE_MAGIC or case_id != case_ids[expected_case]:
        raise RuntimeError(f"{path.name} has an unexpected STR1 magic or case")
    if frames != expected_frames or (expected_checkpoint is not None and checkpoint != expected_checkpoint):
        raise RuntimeError(f"{path.name} has unexpected checkpoint/frame count")
    cursor = HEADER.size
    states = []
    encoded_states = []
    for _ in range(frames + 1):
        state_start = cursor
        if cursor + STATE_COUNT.size > len(data):
            raise RuntimeError(f"{path.name} has a truncated state cell count")
        cell_count = STATE_COUNT.unpack_from(data, cursor)[0]
        cursor += STATE_COUNT.size
        if cell_count > 1_000_000 or cursor + cell_count * CELL.size > len(data):
            raise RuntimeError(f"{path.name} has an invalid/truncated cell vector")
        cells = []
        seen_tags = set()
        for _ in range(cell_count):
            tag, bits = CELL.unpack_from(data, cursor)
            cursor += CELL.size
            if tag in seen_tags:
                raise RuntimeError(f"{path.name} state contains a duplicate cell tag")
            seen_tags.add(tag)
            cells.append((tag, bits))
        states.append(cells)
        encoded_states.append(data[state_start:cursor])
    if cursor != len(data):
        raise RuntimeError(f"{path.name} has trailing bytes")
    block_hashes = []
    for start in range(0, len(encoded_states), 512):
        block = b"".join(encoded_states[start:start + 512])
        block_hashes.append({"firstState": start, "stateCount": len(encoded_states[start:start + 512]),
                             "sha256": sha256(block)})
    return {"path": str(path), "bytes": len(data), "sha256": sha256(data),
            "case": expected_case, "checkpoint": checkpoint, "frames": frames,
            "states": states, "encodedStates": encoded_states, "blockHashes": block_hashes}


def assert_projection(reference: dict[str, Any], full: dict[str, Any], checkpoint: int) -> None:
    start = checkpoint
    end = start + len(reference["states"])
    if end > len(full["states"]) or reference["states"] != full["states"][start:end]:
        raise RuntimeError("128-update reference is not an exact projection of the 4096-tick full trace at checkpoint")


def validate_capture(meta: dict[str, Any], case: str, checkpoint: int, workers: int,
                     recording: pathlib.Path, reference: pathlib.Path, full_trace: pathlib.Path) -> tuple[dict[str, Any], dict[str, Any], dict[str, Any]]:
    required = {"schema": CAPTURE_SCHEMA, "status": "pass", "case": case,
                "total_ticks": 4096, "checkpoint": checkpoint, "frames": 128,
                "reference_states": 129, "full_trace_states": 4097,
                "workers_requested": workers, "workers_actual": workers,
                "recording_validated": True, "first_divergent_tick": -1}
    for field, expected in required.items():
        if meta.get(field) != expected:
            raise RuntimeError(f"capture {case} {field}={meta.get(field)!r}, expected {expected!r}")
    for path, field in ((recording, "recording_bytes"), (reference, "reference_bytes")):
        if not path.is_file():
            raise RuntimeError(f"capture did not create {path.name}")
        if path.stat().st_size != meta.get(field):
            raise RuntimeError(f"capture {field} does not match {path.name} file size")
        if not 0 < path.stat().st_size <= RAW_LIMIT:
            raise RuntimeError(f"{path.name} must be in (0, {RAW_LIMIT}] bytes")
    gates = meta.get("gates")
    expected_gates = {"workers", "warm_contacts", "sleep_wake", "lifecycle_commands", "joints"}
    if not isinstance(gates, dict) or set(gates) != expected_gates or any(value is not True for value in gates.values()):
        raise RuntimeError(f"capture {case} did not pass every reported gate")
    observations = meta.get("observations")
    if not isinstance(observations, dict):
        raise RuntimeError("capture observations object is missing")
    observation_fields = ("warm_contact_ticks", "joint_ticks", "sleep_transitions", "wake_transitions", "creates", "destroys", "stale_handles_invalid")
    if any(not isinstance(observations.get(k), int) for k in observation_fields):
        raise RuntimeError("capture observations are missing required integer fields")
    if observations["sleep_transitions"] <= 0 or observations["wake_transitions"] <= 0:
        raise RuntimeError("capture must observe both sleep and wake transitions")
    if case == "contacts" and observations["warm_contact_ticks"] <= 0:
        raise RuntimeError("contacts capture lacks warm contact ticks")
    if case == "joints" and observations["joint_ticks"] <= 0:
        raise RuntimeError("joints capture lacks joint activity")
    if case == "lifecycle" and not (observations["creates"] > observations["destroys"] > 0 and observations["stale_handles_invalid"] == observations["destroys"]):
        raise RuntimeError("lifecycle capture lacks create/destroy/stale-handle observations")
    if not isinstance(meta.get("worker_checks"), int) or meta["worker_checks"] < 4097:
        raise RuntimeError("capture did not check the actual worker count for every captured tick")
    ref = parse_trace(reference, expected_case=case, expected_checkpoint=checkpoint, expected_frames=128)
    full = parse_trace(full_trace, expected_case=case, expected_checkpoint=0, expected_frames=4096)
    assert_projection(ref, full, checkpoint)
    if not recording.read_bytes()[:4]:
        raise RuntimeError("empty recording header")
    return meta, ref, full


def invoke_capture(exe: pathlib.Path, case: str, checkpoint: int, workers: int,
                   directory: pathlib.Path, timeout: int) -> tuple[dict[str, Any], pathlib.Path, pathlib.Path, pathlib.Path]:
    directory.mkdir(parents=True, exist_ok=True)
    recording = directory / f"{case}.b3rec"
    reference = pathlib.Path(str(recording) + ".ref")
    full_trace = pathlib.Path(str(recording) + ".full-trace")
    result = run([str(exe), "capture", str(recording), "--case", case,
                  "--checkpoint", str(checkpoint), "--workers", str(workers)], timeout)
    try:
        meta = parse_json_line(result.stdout, CAPTURE_SCHEMA)
    except RuntimeError:
        emit_driver_failure("capture", case, result.returncode, None, gate_failed=result.returncode == 0)
        raise
    if result.returncode:
        emit_driver_failure("capture", case, result.returncode, meta)
        raise RuntimeError(f"capture {case} driver exited {result.returncode}: {meta.get('status')}")
    try:
        validate_capture(meta, case, checkpoint, workers, recording, reference, full_trace)
    except (OSError, RuntimeError):
        emit_driver_failure("capture", case, result.returncode, meta, gate_failed=True)
        raise
    return meta, recording, reference, full_trace


def validate_verify(meta: dict[str, Any], case: str, checkpoint: int, workers: int, cycles: int,
                    reference_path: pathlib.Path, observed_path: pathlib.Path) -> dict[str, Any]:
    if meta.get("schema") != VERIFY_SCHEMA or meta.get("status") != "pass":
        raise RuntimeError(f"verify {case} did not return {VERIFY_SCHEMA}/pass")
    fixed = {"case": case, "checkpoint": checkpoint, "frames": 128, "cycles": cycles,
             "workers_requested": workers, "workers_actual": workers,
             "recording_validated": True, "observed_trace_states": 129,
             "first_divergent_tick": -1, "first_divergent_field_tag": 0}
    for field, expected in fixed.items():
        if meta.get(field) != expected:
            raise RuntimeError(f"verify {case} {field}={meta.get(field)!r}, expected {expected!r}")
    expected_counts = {"restart_count": 1 + cycles, "seek_count": cycles, "states_compared": 129 + 194 * cycles, "fresh_players": 1 + (cycles + 15) // 16}
    for field, expected in expected_counts.items():
        if meta.get(field) != expected:
            raise RuntimeError(f"verify {case} {field}={meta.get(field)!r}, expected {expected}")
    if not isinstance(meta.get("worker_checks"), int) or meta["worker_checks"] < 1 + cycles:
        raise RuntimeError(f"verify {case} did not check worker count after every restore")
    keyframe_budget = meta.get("keyframe_budget_bytes")
    keyframe_peak = meta.get("keyframe_peak_bytes")
    if (type(keyframe_budget) is not int or type(keyframe_peak) is not int or
            keyframe_budget <= 0 or keyframe_peak < 0 or keyframe_peak > keyframe_budget):
        raise RuntimeError(f"verify {case} has missing or invalid keyframe budget telemetry")
    expected_trace = reference_path.read_bytes()
    observed = observed_path.read_bytes()
    if observed != expected_trace:
        raise RuntimeError("observed trace is not byte-identical to the reference trace")
    parsed = parse_trace(observed_path, expected_case=case, expected_checkpoint=checkpoint, expected_frames=128)
    return {"status": "pass", "statesCompared": meta["states_compared"],
            "workerChecks": meta["worker_checks"], "observedTraceSha256": parsed["sha256"],
            "referenceSha256": sha256(expected_trace), "traceEqual": True,
            "workersRequested": meta["workers_requested"], "workersActual": meta["workers_actual"],
            "cycles": meta["cycles"], "freshPlayers": meta["fresh_players"],
            "restartCount": meta["restart_count"], "seekCount": meta["seek_count"],
            "keyframeBudgetBytes": keyframe_budget, "keyframePeakBytes": keyframe_peak}


def invoke_verify(exe: pathlib.Path, recording: pathlib.Path, reference: pathlib.Path,
                  case: str, checkpoint: int, workers: int, cycles: int,
                  timeout: int) -> tuple[dict[str, Any], dict[str, Any]]:
    observed_path = pathlib.Path(str(recording) + ".observed-trace")
    if observed_path.exists():
        observed_path.unlink()
    result = run([str(exe), "verify", str(recording), "--reference", str(reference),
                  "--workers", str(workers), "--cycles", str(cycles)], timeout)
    try:
        meta = parse_json_line(result.stdout, VERIFY_SCHEMA)
    except RuntimeError:
        emit_driver_failure("verify", case, result.returncode, None, gate_failed=result.returncode == 0)
        raise
    if result.returncode:
        emit_driver_failure("verify", case, result.returncode, meta)
        raise RuntimeError(f"verify {case} driver exited {result.returncode}: {meta.get('status')}")
    if not observed_path.is_file():
        emit_driver_failure("verify", case, result.returncode, meta, gate_failed=True)
        raise RuntimeError("verify did not write its observed-trace sidecar")
    try:
        verified = validate_verify(meta, case, checkpoint, workers, cycles, reference, observed_path)
    except (OSError, RuntimeError):
        emit_driver_failure("verify", case, result.returncode, meta, gate_failed=True)
        raise
    return meta, verified


def encode_bytes(data: bytes) -> str:
    return base64.b64encode(zlib.compress(data, level=9)).decode("ascii")


def make_record(origin: str, case: str, checkpoint: int, capture: dict[str, Any],
                recording: bytes, reference: bytes, full: dict[str, Any]) -> dict[str, Any]:
    return {"schema": RECORD_SCHEMA, "revision": PIN, "origin": origin, "case": case,
            "checkpoint": checkpoint,
            "recording": {"encoding": "zlib+base64", "bytes": len(recording), "sha256": sha256(recording), "payload": encode_bytes(recording)},
            "reference": {"encoding": "zlib+base64", "bytes": len(reference), "sha256": sha256(reference), "payload": encode_bytes(reference)},
            "capture": capture,
            "full_trace": {"bytes": full["bytes"], "sha256": full["sha256"], "states": full["frames"] + 1, "block_states": 512, "block_sha256": [block["sha256"] for block in full["blockHashes"]]}}


def append_output(path: pathlib.Path, key: str, value: str) -> None:
    with path.open("a", encoding="utf-8", newline="\n") as stream:
        stream.write(f"{key}={value}\n")


def split_transport_record(serialized: str) -> list[str]:
    if not serialized or not serialized.isascii():
        raise RuntimeError("transfer record must be nonempty ASCII JSON")
    if len(serialized) > RECORD_LIMIT:
        raise RuntimeError(f"transfer record is {len(serialized)} chars (limit {RECORD_LIMIT})")
    chunks = [serialized[start:start + CHUNK_SIZE] for start in range(0, len(serialized), CHUNK_SIZE)]
    if len(chunks) > CHUNK_COUNT:
        raise RuntimeError(f"transfer record cannot fit in {CHUNK_COUNT} chunks")
    return chunks + [EMPTY_CHUNK_SENTINEL] * (CHUNK_COUNT - len(chunks))


def join_transport_chunks(chunks: list[str], label: str) -> str:
    if len(chunks) != CHUNK_COUNT:
        raise RuntimeError(f"{label} must provide exactly {CHUNK_COUNT} chunks")
    normalized = []
    for index, chunk in enumerate(chunks):
        if not chunk:
            raise RuntimeError(f"{label}_{index} is empty; expected data or the sentinel")
        normalized.append("" if chunk == EMPTY_CHUNK_SENTINEL else chunk)
    chunks = normalized
    saw_empty = False
    for index, chunk in enumerate(chunks):
        if len(chunk) > CHUNK_SIZE:
            raise RuntimeError(f"{label}_{index} exceeds the {CHUNK_SIZE}-character chunk limit")
        if not chunk:
            saw_empty = True
        elif saw_empty:
            raise RuntimeError(f"{label}_{index} is nonempty after an empty tail chunk")
        if not chunk.isascii():
            raise RuntimeError(f"{label}_{index} contains non-ASCII transport data")
    serialized = "".join(chunks)
    if not serialized or len(serialized) > RECORD_LIMIT:
        raise RuntimeError(f"{label} chunks do not contain a complete bounded record")
    return serialized


def produce(args: argparse.Namespace) -> int:
    origin = os.environ.get("PUBLIC_ORIGIN", "")
    if not re.fullmatch(r"[a-z0-9][a-z0-9_-]{0,63}", origin):
        raise RuntimeError("PUBLIC_ORIGIN must be a generic stable runner label")
    output_path = os.environ.get("GITHUB_OUTPUT")
    if not output_path:
        raise RuntimeError("GITHUB_OUTPUT is required for producer mode")
    exe, compiler_ids = get_driver(args, 4)
    with tempfile.TemporaryDirectory(prefix="box3d-stress-producer-") as scratch:
        root = pathlib.Path(scratch)
        for case in CASES:
            checkpoint = CHECKPOINTS[case]
            case_dir = root / case
            cap, recording_path, ref_path, full_path = invoke_capture(
                exe, case, checkpoint, 1, case_dir, args.timeout)
            _, local_verify = invoke_verify(exe, recording_path, ref_path, case, checkpoint,
                                            1, 4, args.timeout)
            recording = recording_path.read_bytes()
            reference = ref_path.read_bytes()
            full = parse_trace(full_path, expected_case=case, expected_checkpoint=0, expected_frames=4096)
            record = make_record(origin, case, checkpoint, cap, recording, reference, full)
            encoded = json.dumps(record, sort_keys=True, separators=(",", ":"))
            chunks = split_transport_record(encoded)
            for index, chunk in enumerate(chunks):
                append_output(pathlib.Path(output_path), f"{case}_{index}", chunk)
            print(json.dumps({"schema": "box3d-ci-stress-produce-case-v1", "origin": origin,
                              "case": case, "recording_bytes": len(recording),
                              "reference_bytes": len(reference), "recording_sha256": record["recording"]["sha256"],
                              "reference_sha256": record["reference"]["sha256"],
                              "full_trace_sha256": full["sha256"], "full_trace_blocks": len(full["blockHashes"]),
                              "capture_metadata": cap, "serialized_output_characters": len(encoded),
                              "chunk_characters": [0 if chunk == EMPTY_CHUNK_SENTINEL else len(chunk) for chunk in chunks],
                              "empty_chunk_sentinel": EMPTY_CHUNK_SENTINEL,
                              "local_verify": local_verify, "payload_stdout": False},
                             sort_keys=True, separators=(",", ":")))
    output_keys = [f"{case}_{index}" for case in CASES for index in range(CHUNK_COUNT)]
    print(json.dumps({"schema": "box3d-ci-stress-produce-v1", "revision": PIN,
                      "origin": origin, "cases": list(CASES), "output_keys": output_keys,
                      "chunk_count_per_case": CHUNK_COUNT, "chunk_size_characters": CHUNK_SIZE,
                      "case_record_limit_characters": RECORD_LIMIT,
                      "empty_chunk_sentinel": EMPTY_CHUNK_SENTINEL,
                      "cmake_compilers": compiler_ids, "payload_stdout": False},
                     sort_keys=True, separators=(",", ":")))
    return 0


def decode_payload(record: dict[str, Any], object_field: str) -> bytes:
    blob = record.get(object_field)
    if not isinstance(blob, dict) or blob.get("encoding") != "zlib+base64":
        raise RuntimeError(f"invalid {object_field} encoding")
    try:
        packed = base64.b64decode(blob["payload"], validate=True)
        inflater = zlib.decompressobj()
        data = inflater.decompress(packed, RAW_LIMIT + 1)
    except (KeyError, ValueError, TypeError, zlib.error) as exc:
        raise RuntimeError(f"invalid {object_field} payload") from exc
    if len(data) > RAW_LIMIT or inflater.unconsumed_tail or not inflater.eof or inflater.unused_data:
        raise RuntimeError(f"{object_field} exceeds raw limit or has invalid zlib framing")
    if not data or len(data) != blob.get("bytes") or sha256(data) != blob.get("sha256"):
        raise RuntimeError(f"{object_field} byte count/SHA-256 mismatch")
    return data


def validate_embedded_capture(capture: Any, case: str, checkpoint: int,
                              recording_size: int, reference_size: int) -> None:
    if not isinstance(capture, dict):
        raise RuntimeError("producer capture metadata is missing")
    expected = {"schema": CAPTURE_SCHEMA, "status": "pass", "case": case,
                "total_ticks": 4096, "checkpoint": checkpoint, "frames": 128,
                "recording_bytes": recording_size, "reference_bytes": reference_size,
                "reference_states": 129, "full_trace_states": 4097,
                "workers_requested": 1, "workers_actual": 1,
                "recording_validated": True, "first_divergent_tick": -1}
    for field, value in expected.items():
        if capture.get(field) != value:
            raise RuntimeError(f"producer capture {field} mismatch")
    gates = capture.get("gates")
    expected_gates = {"workers", "warm_contacts", "sleep_wake", "lifecycle_commands", "joints"}
    if not isinstance(gates, dict) or set(gates) != expected_gates or any(value is not True for value in gates.values()):
        raise RuntimeError("producer capture gates are missing or failed")
    observations = capture.get("observations")
    names = ("warm_contact_ticks", "joint_ticks", "sleep_transitions", "wake_transitions",
             "creates", "destroys", "stale_handles_invalid")
    if not isinstance(observations, dict) or any(not isinstance(observations.get(name), int) for name in names):
        raise RuntimeError("producer capture observations are incomplete")
    if observations["sleep_transitions"] <= 0 or observations["wake_transitions"] <= 0:
        raise RuntimeError("producer capture did not include sleep and wake transitions")
    if case == "contacts" and observations["warm_contact_ticks"] <= 0:
        raise RuntimeError("producer contacts capture lacks warm contacts")
    if case == "joints" and observations["joint_ticks"] <= 0:
        raise RuntimeError("producer joints capture lacks joint activity")
    if case == "lifecycle" and not (observations["creates"] > observations["destroys"] > 0 and
                                    observations["stale_handles_invalid"] == observations["destroys"]):
        raise RuntimeError("producer lifecycle capture lacks lifecycle observations")
    if not isinstance(capture.get("worker_checks"), int) or capture["worker_checks"] < 4097:
        raise RuntimeError("producer did not report worker checks for all ticks")


def validate_remote_record(record: Any, case: str, origin: str) -> tuple[dict[str, Any], bytes, bytes]:
    label = f"{origin}/{case}"
    if not isinstance(record, dict):
        raise RuntimeError(f"{label} must contain one JSON object")
    if record.get("schema") != RECORD_SCHEMA or record.get("revision") != PIN:
        raise RuntimeError(f"{label} has unexpected schema/revision")
    if record.get("origin") != origin or record.get("case") != case:
        raise RuntimeError(f"{label} origin/case mismatch")
    checkpoint = CHECKPOINTS[case]
    if record.get("checkpoint") != checkpoint:
        raise RuntimeError(f"{label} checkpoint mismatch")
    recording = decode_payload(record, "recording")
    reference = decode_payload(record, "reference")
    validate_embedded_capture(record.get("capture"), case, checkpoint, len(recording), len(reference))
    full = record.get("full_trace")
    if (not isinstance(full, dict) or full.get("states") != 4097 or full.get("block_states") != 512 or
            type(full.get("bytes")) is not int or full["bytes"] <= 0 or
            not isinstance(full.get("sha256"), str) or not re.fullmatch(r"[0-9a-f]{64}", full["sha256"]) or
            not isinstance(full.get("block_sha256"), list) or len(full["block_sha256"]) != 9 or
            any(not isinstance(item, str) or not re.fullmatch(r"[0-9a-f]{64}", item) for item in full["block_sha256"])):
        raise RuntimeError(f"{label} full-trace metadata is malformed")
    return record, recording, reference


def receive(args: argparse.Namespace) -> int:
    chunks = []
    for index in range(CHUNK_COUNT):
        name = f"PUBLIC_CHUNK_{index}"
        value = os.environ.get(name)
        if value is None:
            raise RuntimeError(f"{name} is missing; all six chunk variables are required")
        chunks.append(value)
    serialized = join_transport_chunks(chunks, "PUBLIC_CHUNK")
    try:
        record = json.loads(serialized)
    except json.JSONDecodeError as exc:
        raise RuntimeError("received chunks do not form valid JSON") from exc
    validated, recording, reference = validate_remote_record(record, args.case, args.origin)
    encoded = serialized.encode("ascii")
    target = ROOT / "stress-transfer" / args.case / f"{args.origin}.json"
    target.parent.mkdir(parents=True, exist_ok=True)
    if target.exists():
        raise RuntimeError("received-record destination already exists")
    target.write_bytes(encoded)
    print(json.dumps({"schema": "box3d-ci-stress-receive-v1", "case": args.case,
                      "origin": args.origin, "record_json_bytes": len(encoded),
                      "recording_bytes": len(recording), "recording_sha256": sha256(recording),
                      "reference_bytes": len(reference), "reference_sha256": sha256(reference),
                      "output_keys": [f"{args.case}_{index}" for index in range(CHUNK_COUNT)],
                      "payload_stdout": False}, sort_keys=True, separators=(",", ":")))
    return 0


def load_remote_records(case: str) -> list[tuple[dict[str, Any], bytes, bytes]]:
    result = []
    for origin in ORIGINS.values():
        path = ROOT / "stress-transfer" / case / f"{origin}.json"
        if not path.is_file():
            raise RuntimeError(f"received record is missing for {origin}/{case}")
        if path.stat().st_size > RECORD_LIMIT:
            raise RuntimeError(f"received record for {origin}/{case} exceeds its bound")
        raw = path.read_bytes()
        if not raw:
            raise RuntimeError(f"received record for {origin}/{case} is empty")
        try:
            serialized = raw.decode("ascii")
            record = json.loads(serialized)
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise RuntimeError(f"received record for {origin}/{case} is not ASCII JSON") from exc
        result.append(validate_remote_record(record, case, origin))
    return result


def compare_record_full_trace(record: dict[str, Any], local_full: dict[str, Any]) -> dict[str, Any]:
    full = record.get("full_trace")
    if not isinstance(full, dict):
        return {"matches": False, "reason": "metadata_missing"}
    expected_blocks = full.get("block_sha256", [])
    actual_blocks = [block["sha256"] for block in local_full["blockHashes"]]
    first_block = next((index for index, (expected, actual) in enumerate(zip(expected_blocks, actual_blocks))
                        if expected != actual), None)
    matches = (full.get("bytes") == local_full["bytes"] and full.get("states") == 4097 and
               full.get("sha256") == local_full["sha256"] and expected_blocks == actual_blocks and
               full.get("block_states") == 512)
    diagnostic = {"matches": matches, "producerSha256": full.get("sha256"),
                  "localSha256": local_full["sha256"], "producerBytes": full.get("bytes"),
                  "localBytes": local_full["bytes"], "blockStates": full.get("block_states"),
                  "firstDifferingBlock": first_block}
    if first_block is not None:
        diagnostic["producerBlockSha256"] = expected_blocks[first_block]
        diagnostic["localBlockSha256"] = actual_blocks[first_block]
    elif expected_blocks != actual_blocks:
        diagnostic["blockCountMismatch"] = {"producer": len(expected_blocks), "local": len(actual_blocks)}
    return diagnostic


def compare_trace_projection(expected_path: pathlib.Path, actual_path: pathlib.Path,
                             case: str, checkpoint: int) -> dict[str, Any] | None:
    expected_bytes = expected_path.read_bytes()
    actual_bytes = actual_path.read_bytes()
    expected = parse_trace(expected_path, expected_case=case, expected_checkpoint=checkpoint, expected_frames=128)
    actual = parse_trace(actual_path, expected_case=case, expected_checkpoint=checkpoint, expected_frames=128)
    for state_index, (left, right) in enumerate(zip(expected["states"], actual["states"])):
        left_fields, right_fields = dict(left), dict(right)
        for tag in sorted(set(left_fields) | set(right_fields)):
            expected_bits, actual_bits = left_fields.get(tag), right_fields.get(tag)
            if expected_bits != actual_bits:
                return {"matches": False, "tick": checkpoint + state_index, "tag": tag,
                        "expectedBits": None if expected_bits is None else f"0x{expected_bits:08x}",
                        "actualBits": None if actual_bits is None else f"0x{actual_bits:08x}",
                        "expectedSha256": expected["sha256"], "actualSha256": actual["sha256"]}
    if expected_bytes != actual_bytes:
        first = next((i for i, (left, right) in enumerate(zip(expected_bytes, actual_bytes)) if left != right),
                     min(len(expected_bytes), len(actual_bytes)))
        return {"matches": False, "tick": None, "tag": None,
                "expectedBits": None, "actualBits": None, "firstDifferingByte": first,
                "expectedByte": expected_bytes[first] if first < len(expected_bytes) else None,
                "actualByte": actual_bytes[first] if first < len(actual_bytes) else None,
                "expectedSha256": expected["sha256"], "actualSha256": actual["sha256"],
                "note": "decoded states match, but serialized trace bytes differ"}
    return None


def validate_bad_magic(exe: pathlib.Path, recording: bytes, reference: pathlib.Path,
                       case: str, workers: int, cycles: int, timeout: int) -> dict[str, Any]:
    with tempfile.TemporaryDirectory(prefix="box3d-stress-negative-") as temp:
        temp_root = pathlib.Path(temp)
        corrupt = bytearray(recording)
        if not corrupt:
            raise RuntimeError("cannot create bad-magic control from empty recording")
        corrupt[0] ^= 0x01
        path = temp_root / "bad-magic.b3rec"
        path.write_bytes(corrupt)
        result = run([str(exe), "verify", str(path), "--reference", str(reference),
                      "--workers", str(workers), "--cycles", str(cycles)], timeout)
        meta = parse_json_line(result.stdout, VERIFY_SCHEMA)
        passed = result.returncode == 2 and meta.get("status") == "load_rejected"
        return {"status": "load_rejected" if passed else "failed", "exitCode": result.returncode,
                "driverStatus": meta.get("status")}


def consume(args: argparse.Namespace) -> int:
    case = args.case
    remotes = load_remote_records(case)
    exe, compiler_ids = get_driver(args, args.simd)
    with tempfile.TemporaryDirectory(prefix="box3d-stress-consumer-") as scratch:
        case_dir = pathlib.Path(scratch)
        checkpoint = CHECKPOINTS[case]
        local_meta, local_recording, local_ref_path, local_full_path = invoke_capture(
            exe, case, checkpoint, args.workers, case_dir / "local", args.timeout)
        local_ref = local_ref_path.read_bytes()
        local_full = parse_trace(local_full_path, expected_case=case, expected_checkpoint=0, expected_frames=4096)
        comparisons = []
        for record, recording, reference in remotes:
            remote_ref_path = case_dir / f"{record['origin']}.ref"
            remote_record_path = case_dir / f"{record['origin']}.b3rec"
            remote_ref_path.write_bytes(reference)
            remote_record_path.write_bytes(recording)
            projection_difference = compare_trace_projection(local_ref_path, remote_ref_path, case, checkpoint)
            reference_match = reference == local_ref and projection_difference is None
            full_trace_diagnostic = compare_record_full_trace(record, local_full)
            full_match = full_trace_diagnostic["matches"]
            observed_path = pathlib.Path(str(remote_record_path) + ".observed-trace")
            if observed_path.exists():
                observed_path.unlink()
            verify_run = None
            try:
                verify_run = run([str(exe), "verify", str(remote_record_path), "--reference", str(remote_ref_path),
                                  "--workers", str(args.workers), "--cycles", str(args.cycles)], args.timeout)
                verify_meta = parse_json_line(verify_run.stdout, VERIFY_SCHEMA)
                driver_status = verify_meta.get("status")
                if verify_run.returncode == 0 and driver_status == "pass":
                    try:
                        verify_result = validate_verify(verify_meta, case, checkpoint, args.workers,
                                                         args.cycles, remote_ref_path, observed_path)
                    except (OSError, RuntimeError) as exc:
                        verify_result = {"status": "gate_failed", "exitCode": verify_run.returncode,
                                         "driverStatus": driver_status, "detail": str(exc)}
                else:
                    statuses = {2: "load_rejected", 3: "player_rejected", 4: "diverged",
                                5: "gate_failed", 6: "reference_rejected", 64: "usage_error"}
                    verify_result = {"status": statuses.get(verify_run.returncode, "unexpected_exit"),
                                     "exitCode": verify_run.returncode, "driverStatus": driver_status,
                                     "firstDivergentTick": verify_meta.get("first_divergent_tick"),
                                     "firstDivergentFieldTag": verify_meta.get("first_divergent_field_tag"),
                                     "field": verify_meta.get("field"),
                                     "expectedBits": verify_meta.get("expected_bits"),
                                     "actualBits": verify_meta.get("actual_bits")}
            except (OSError, RuntimeError, subprocess.SubprocessError) as exc:
                verify_result = {"status": "invalid_result", "exitCode": None if verify_run is None else verify_run.returncode,
                                 "detail": str(exc)}
            try:
                negative = validate_bad_magic(exe, recording, remote_ref_path, case,
                                              args.workers, args.cycles, args.timeout)
            except (OSError, RuntimeError, subprocess.SubprocessError) as exc:
                negative = {"status": "failed", "detail": str(exc)}
            comparisons.append({"origin": record["origin"], "recordingBytes": len(recording),
                                "recordingSha256": sha256(recording), "referenceSha256": sha256(reference),
                                "fullTraceSha256Match": full_match, "referenceProjectionMatch": reference_match,
                                "fullTraceDiagnostic": full_trace_diagnostic,
                                "referenceProjectionDiagnostic": projection_difference,
                                "verify": verify_result, "negativeControl": negative})
        summary = {"schema": "box3d-ci-stress-consume-v1", "revision": PIN, "case": case,
                   "configuration": args.config, "requestedSimdWidth": args.simd,
                   "workersRequested": args.workers, "workersActual": local_meta["workers_actual"],
                   "cycles": args.cycles, "localCapture": {"status": local_meta["status"],
                   "captureMetadata": local_meta,
                   "fullTraceSha256": local_full["sha256"], "fullTraceBytes": local_full["bytes"],
                   "fullTraceBlockSha256": [block["sha256"] for block in local_full["blockHashes"]],
                   "referenceSha256": sha256(local_ref)},
                   "cmakeCompilers": compiler_ids, "producerComparisons": comparisons,
                   "allAccepted": len(comparisons) == 5 and all(item["verify"]["status"] == "pass" and
                      item["negativeControl"]["status"] == "load_rejected" and item["fullTraceSha256Match"] and
                      item["referenceProjectionMatch"] for item in comparisons)}
    print(json.dumps(summary, sort_keys=True, separators=(",", ":")))
    return 0 if summary["allAccepted"] else 1


def stress(args: argparse.Namespace) -> int:
    exe, compiler_ids = get_driver(args, 4)
    runs = []
    with tempfile.TemporaryDirectory(prefix="box3d-stress-100cycles-") as temp:
        root = pathlib.Path(temp)
        for case in CASES:
            checkpoint = CHECKPOINTS[case]
            case_dir = root / case
            capture_meta, recording, reference, full_path = invoke_capture(
                exe, case, checkpoint, args.workers, case_dir, args.timeout)
            full_trace = parse_trace(full_path, expected_case=case, expected_checkpoint=0,
                                     expected_frames=4096)
            verify_meta, verify = invoke_verify(exe, recording, reference, case, checkpoint,
                                                args.workers, args.cycles, args.timeout)
            runs.append({"case": case, "workersActual": capture_meta["workers_actual"],
                         "captureMetadata": capture_meta,
                         "fullTraceSha256": full_trace["sha256"], "fullTraceBytes": full_trace["bytes"],
                         "fullTraceBlockSha256": [block["sha256"] for block in full_trace["blockHashes"]],
                         "cycles": args.cycles, "verify": verify,
                         "restarts": verify_meta["restart_count"], "seeks": verify_meta["seek_count"]})
    summary = {"schema": "box3d-ci-stress-local-v1", "revision": PIN,
               "configuration": args.config, "simd": 4, "workersRequested": args.workers,
               "cyclesPerCase": args.cycles, "totalCaseCycles": len(CASES) * args.cycles,
               "cmakeCompilers": compiler_ids, "cases": runs,
               "allPassed": len(runs) == 3 and all(row["verify"]["status"] == "pass" for row in runs)}
    print(json.dumps(summary, sort_keys=True, separators=(",", ":")))
    return 0 if summary["allPassed"] else 1


def common_build_args(parser: argparse.ArgumentParser, *, simd: bool) -> None:
    parser.add_argument("--compiler", choices=("gcc", "clang", "msvc", "clang-cl"), required=True)
    parser.add_argument("--generator")
    parser.add_argument("--config", choices=("Debug", "RelWithDebInfo"), default="RelWithDebInfo")
    parser.add_argument("--build", default="stress-build")
    parser.add_argument("--parallel", type=int, default=2)
    parser.add_argument("--timeout", type=int, default=900)
    if simd:
        parser.add_argument("--simd", type=int, choices=(4, 8), required=True)


def main() -> int:
    parser = argparse.ArgumentParser(description="Public Box3D recording stress/replay CI wrapper.")
    sub = parser.add_subparsers(dest="command", required=True)
    producer = sub.add_parser("produce", help="capture and export contacts, joints, and lifecycle records")
    common_build_args(producer, simd=False)
    producer.set_defaults(func=produce)
    receiver = sub.add_parser("receive", help="validate one bounded public producer record into job-local storage")
    receiver.add_argument("--case", choices=CASES, required=True)
    receiver.add_argument("--origin", choices=tuple(ORIGINS.values()), required=True)
    receiver.set_defaults(func=receive)
    consumer = sub.add_parser("consume", help="compare one public case against all five producer origins")
    common_build_args(consumer, simd=True)
    consumer.add_argument("--case", choices=CASES, required=True)
    consumer.add_argument("--workers", type=int, choices=(1, 2, 4), default=1)
    consumer.add_argument("--cycles", type=int, choices=(4,), default=4)
    consumer.set_defaults(func=consume)
    soak = sub.add_parser("stress", help="run three bounded same-build restore/replay workloads")
    common_build_args(soak, simd=False)
    soak.add_argument("--workers", type=int, choices=(1, 2, 4), default=1)
    soak.add_argument("--cycles", type=int, choices=(100,), default=100)
    soak.set_defaults(func=stress)
    args = parser.parse_args()
    if (getattr(args, "parallel", 1) < 1 or getattr(args, "timeout", 1) < 1 or
            (hasattr(args, "cycles") and args.cycles < 1)):
        parser.error("parallel, timeout, and cycles must be positive")
    try:
        return args.func(args)
    except (OSError, RuntimeError, subprocess.SubprocessError, ValueError) as exc:
        print(json.dumps({"schema": "box3d-ci-stress-error-v1", "error": str(exc)},
                         sort_keys=True, separators=(",", ":")), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
