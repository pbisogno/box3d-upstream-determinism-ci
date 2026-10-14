from __future__ import annotations

import argparse
import base64
import hashlib
import json
import os
import pathlib
import platform
import re
import shutil
import subprocess
import sys
import tempfile
import zlib
from typing import Any

PINNED_REVISION = "5d83df83ab47172c2c745b44315e7538ead02397"


def run(argv: list[str], *, timeout: int = 120) -> subprocess.CompletedProcess[str]:
    return subprocess.run(argv, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                          errors="replace", timeout=timeout)


def normalized_arch(value: str) -> str:
    value = value.lower().replace(" ", "")
    return {"amd64": "x86_64", "x64": "x86_64", "aarch64": "arm64", "arm64": "arm64"}.get(value, value)


def linux_cpu() -> dict[str, Any]:
    result = run(["lscpu", "--json"], timeout=15)
    if result.returncode:
        return {"source": "lscpu", "available": False}
    try:
        rows = json.loads(result.stdout).get("lscpu", [])
    except (json.JSONDecodeError, AttributeError):
        return {"source": "lscpu", "available": False}
    values = {str(row.get("field", "")).rstrip(":"): row.get("data") for row in rows}
    return {
        "source": "lscpu --json",
        "vendor": values.get("Vendor ID"),
        "model": values.get("Model name"),
        "architecture": normalized_arch(str(values.get("Architecture") or platform.machine())),
        "logical_cpus": as_int(values.get("CPU(s)")),
        "cores_per_socket": as_int(values.get("Core(s) per socket")),
        "sockets": as_int(values.get("Socket(s)")),
        "features_available": bool(values.get("Flags")),
        "features": sorted(str(values.get("Flags") or "").split()),
    }


def as_int(value: Any) -> int | None:
    try:
        return int(str(value).strip())
    except (TypeError, ValueError):
        return None


def windows_cpu() -> dict[str, Any]:
    shell = shutil.which("pwsh") or shutil.which("powershell")
    if not shell:
        return {"source": "Win32_Processor CIM and .NET intrinsics", "available": False}
    script = r"""
$ErrorActionPreference = 'Stop'
$processors = @(Get-CimInstance Win32_Processor | Select-Object Name,Manufacturer,Architecture,NumberOfCores,NumberOfLogicalProcessors,InstructionSet)
$features = [ordered]@{}
foreach ($group in @(@('X86', @('Sse2','Avx2','Fma','Avx512F')), @('Arm', @('AdvSimd')))) {
  foreach ($name in $group[1]) {
    $type = [type]::GetType("System.Runtime.Intrinsics.$($group[0]).$name, System.Private.CoreLib", $false)
    if ($type) {
      $property = $type.GetProperty('IsSupported')
      if ($property) { $features[$name.ToLowerInvariant()] = [bool]$property.GetValue($null) }
    }
  }
}
[PSCustomObject]@{ processors = $processors; features = $features } | ConvertTo-Json -Compress -Depth 4
"""
    result = run([shell, "-NoProfile", "-NonInteractive", "-Command", script], timeout=20)
    if result.returncode:
        return {"source": "Win32_Processor CIM and .NET intrinsics", "available": False}
    try:
        data = json.loads(result.stdout)
        items = data.get("processors", [])
        features = data.get("features", {})
        if isinstance(items, dict):
            items = [items]
    except (json.JSONDecodeError, AttributeError):
        return {"source": "Win32_Processor CIM and .NET intrinsics", "available": False}
    return {
        "source": "Win32_Processor CIM and .NET intrinsics",
        "features_available": bool(features),
        "features": features or None,
        "processors": [
            {
                "vendor": item.get("Manufacturer"),
                "model": item.get("Name"),
                "architecture_code": item.get("Architecture"),
                "cores": item.get("NumberOfCores"),
                "logical_cpus": item.get("NumberOfLogicalProcessors"),
                "instruction_set_code": item.get("InstructionSet"),
            }
            for item in items if isinstance(item, dict)
        ],
    }

def mac_cpu() -> dict[str, Any]:
    keys = ["machdep.cpu.brand_string", "machdep.cpu.vendor", "machdep.cpu.features",
            "machdep.cpu.leaf7_features", "hw.ncpu", "hw.physicalcpu"]
    values: dict[str, str] = {}
    for key in keys:
        result = run(["sysctl", "-n", key], timeout=10)
        if result.returncode == 0:
            values[key] = result.stdout.strip()
    features = set()
    for key in ("machdep.cpu.features", "machdep.cpu.leaf7_features"):
        features.update(values.get(key, "").split())
    return {
        "source": "sysctl",
        "vendor": values.get("machdep.cpu.vendor"),
        "model": values.get("machdep.cpu.brand_string"),
        "architecture": normalized_arch(platform.machine()),
        "logical_cpus": as_int(values.get("hw.ncpu")),
        "physical_cpus": as_int(values.get("hw.physicalcpu")),
        "features_available": bool(values.get("machdep.cpu.features") or values.get("machdep.cpu.leaf7_features")),
        "features": sorted(features),
    }


def compiler_summary(log_path: str | None) -> dict[str, str]:
    if not log_path:
        return {}
    text = pathlib.Path(log_path).read_text(encoding="utf-8", errors="replace")
    found: dict[str, str] = {}
    patterns = {
        "c": r"(?:The )?C compiler identification is\s+([^\r\n]+)",
        "cxx": r"(?:The )?CXX compiler identification is\s+([^\r\n]+)",
    }
    for key, pattern in patterns.items():
        match = re.search(pattern, text, re.IGNORECASE)
        if match:
            found[key] = match.group(1).strip()
    return found


def cpu_inventory() -> dict[str, Any]:
    system = platform.system()
    if system == "Linux":
        return linux_cpu()
    if system == "Windows":
        return windows_cpu()
    if system == "Darwin":
        return mac_cpu()
    return {"source": "unsupported platform", "available": False}


def inventory(args: argparse.Namespace) -> int:
    system = platform.system()
    actual_arch = normalized_arch(platform.machine())
    matches = args.expected_arch is None or actual_arch == args.expected_arch
    output = {
        "schema": "box3d-ci-inventory-v1",
        "platform": {"system": system, "architecture": actual_arch,
                     "expected_architecture": args.expected_arch, "architecture_matches": matches},
        "cpu": cpu_inventory(),
        "cmake_compilers": compiler_summary(args.compiler_log),
    }
    print(json.dumps(output, sort_keys=True, separators=(",", ":")))
    return 0 if matches else 1


def require_avx2_support() -> None:
    cpu = cpu_inventory()
    features = cpu.get("features")
    if isinstance(features, list):
        normalized = {str(item).lower() for item in features}
        supported = bool(normalized & {"avx2", "avx2.0"})
    elif isinstance(features, dict):
        supported = features.get("avx2") is True
    else:
        supported = False
    if not cpu.get("features_available") or not supported:
        raise RuntimeError("SIMD8 requires an observable AVX2=true host feature")


def verified_source(source: pathlib.Path) -> None:
    revision = run(["git", "-C", str(source), "rev-parse", "HEAD"], timeout=15)
    if revision.returncode or revision.stdout.strip() != PINNED_REVISION:
        raise RuntimeError("source is not the pinned upstream revision")
    dirty = run(["git", "-C", str(source), "status", "--porcelain"], timeout=15)
    if dirty.returncode or dirty.stdout.strip():
        raise RuntimeError("upstream source checkout is modified")


def validate_compiler_selection(compiler: str | None) -> None:
    if platform.system() == "Windows" and compiler not in (None, "msvc", "clang-cl"):
        raise RuntimeError("Windows supports only the MSVC or clang-cl compiler selection")
    if platform.system() != "Windows" and compiler in ("msvc", "clang-cl"):
        raise RuntimeError("MSVC and clang-cl selections are available only on Windows")


def build_flags(config: str, simd: int) -> list[str]:
    if simd == 8 and normalized_arch(platform.machine()) != "x86_64":
        raise RuntimeError("AVX2/SIMD8 variant requires an x86_64 runner")
    return ["-DCMAKE_BUILD_TYPE=" + config, "-DBUILD_SHARED_LIBS=OFF",
            "-DBOX3D_SAMPLES=OFF", "-DBOX3D_BENCHMARKS=OFF", "-DBOX3D_DOCS=OFF",
            "-DBOX3D_UNIT_TESTS=ON", "-DBOX3D_PROFILE=OFF", "-DBOX3D_SANITIZE=OFF",
            "-DBOX3D_VALIDATE=ON", "-DBOX3D_DOUBLE_PRECISION=OFF",
            "-DBOX3D_DISABLE_SIMD=OFF", "-DBOX3D_AVX2=" + ("ON" if simd == 8 else "OFF")]


def cmake_compiler_args(compiler: str | None, generator: str | None) -> list[str]:
    validate_compiler_selection(compiler)
    result: list[str] = []
    if platform.system() == "Windows":
        result += ["-G", generator or "Visual Studio 17 2022", "-A", "x64"]
        if compiler == "clang-cl":
            result += ["-T", "ClangCL"]
    elif compiler:
        cc, cxx = ("gcc", "g++") if compiler == "gcc" else ("clang", "clang++")
        result += ["-DCMAKE_C_COMPILER=" + cc, "-DCMAKE_CXX_COMPILER=" + cxx]
    return result


def cmake_ids(output: str) -> dict[str, str]:
    found: dict[str, str] = {}
    for key, pattern in (("c", r"(?:The )?C compiler identification is\s+([^\r\n]+)"),
                         ("cxx", r"(?:The )?CXX compiler identification is\s+([^\r\n]+)")):
        match = re.search(pattern, output, re.IGNORECASE)
        if match:
            found[key] = match.group(1).strip()
    return found


def check_test_markers(name: str, output: str, simd: int) -> dict[str, Any]:
    if f"test passed: {name}" not in output:
        raise RuntimeError(f"required {name} pass marker missing")
    passed = set(re.findall(r"^\s*subtest passed: ([^\r\n]+)", output, re.MULTILINE))
    skipped = {item.split(",", 1)[0].strip()
               for item in re.findall(r"^\s*subtest skipped: ([^\r\n]+)", output, re.MULTILINE)}
    expected = {
        "DeterminismTest": {"CrossPlatformTest", "MultithreadingTest", "WavePileTest",
                            "QuerySpawnTest", "MeshDropTest", "SIMDWidthTest"},
        "RecordingTest": {"CrossWidthReplay"},
    }[name]
    for marker in expected:
        if marker in ("SIMDWidthTest", "CrossWidthReplay") and simd == 4:
            if marker not in passed and marker not in skipped:
                raise RuntimeError(f"{marker} did not report pass or skip")
            continue
        if marker in skipped:
            raise RuntimeError(f"required {marker} subtest was skipped for SIMD{simd}")
        if marker not in passed:
            raise RuntimeError(f"required {marker} pass marker missing")
    if simd == 8 and ("SIMDWidthTest" in skipped or "CrossWidthReplay" in skipped):
        raise RuntimeError("SIMD8 requires SIMDWidthTest and CrossWidthReplay to execute")
    return {"passed_subtests": sorted(passed), "skipped_subtests": sorted(skipped)}


def test_upstream(args: argparse.Namespace) -> int:
    project_root = pathlib.Path(__file__).resolve().parent
    source_arg = pathlib.Path(args.source)
    source = (project_root / source_arg).resolve() if not source_arg.is_absolute() else source_arg.resolve()
    build_arg = pathlib.Path(args.build or f"build-tests-simd{args.simd}")
    build = (project_root / build_arg).resolve() if not build_arg.is_absolute() else build_arg.resolve()
    verified_source(source)
    validate_compiler_selection(args.compiler)
    if not shutil.which("cmake"):
        raise RuntimeError("CMake is not available on PATH")
    config, simd = args.config, args.simd
    if simd == 8:
        require_avx2_support()
    configure = ["cmake", "-S", str(source), "-B", str(build)] + build_flags(config, simd)
    configure += cmake_compiler_args(args.compiler, args.generator)
    result = run(configure, timeout=args.timeout)
    if result.returncode:
        print(result.stdout[-12000:], end="", file=sys.stderr)
        return result.returncode
    compiler_ids = cmake_ids(result.stdout)
    build_result = run(["cmake", "--build", str(build), "--config", config,
                        "--target", "test", "--parallel", str(args.parallel)], timeout=args.timeout)
    if build_result.returncode:
        print(build_result.stdout[-12000:], end="", file=sys.stderr)
        return build_result.returncode
    exe = build / "bin" / (config if platform.system() == "Windows" else "") / ("test.exe" if platform.system() == "Windows" else "test")
    if not exe.is_file() and platform.system() == "Windows":
        exe = build / "bin" / "test.exe"
    if not exe.is_file():
        raise RuntimeError("expected upstream test executable was not found")
    runs = []
    for name in ("DeterminismTest", "RecordingTest", "DeterminismTest", "RecordingTest", None):
        test_result = run([str(exe)] + ([name] if name else []), timeout=args.timeout)
        output = test_result.stdout
        if test_result.returncode:
            print(output[-12000:], end="", file=sys.stderr)
            return test_result.returncode
        if name:
            markers = check_test_markers(name, output, simd)
        else:
            if ("test passed: DeterminismTest" not in output or
                    "test passed: RecordingTest" not in output or
                    "All Box3D tests passed!" not in output):
                raise RuntimeError("required complete-suite pass markers missing")
            det_markers = check_test_markers("DeterminismTest", output, simd)
            rec_markers = check_test_markers("RecordingTest", output, simd)
            markers = {"determinism": det_markers, "recording": rec_markers}
        runs.append({"test": name or "all", "returncode": test_result.returncode,
                     "markers": markers})
    print(json.dumps({"schema": "box3d-ci-test-v1", "revision": PINNED_REVISION,
                      "configuration": config, "requested_simd_width": simd,
                      "box3d_avx2": ("ON" if simd == 8 else "OFF"),
                      "cmake_compilers": compiler_ids, "runs": runs},
                     sort_keys=True, separators=(",", ":")))
    return 0


def build_overlay_replay(args: argparse.Namespace, *, build_dir: pathlib.Path) -> tuple[pathlib.Path, dict[str, str]]:
    project_root = pathlib.Path(__file__).resolve().parent
    source = project_root / "upstream"
    verified_source(source)
    validate_compiler_selection(args.compiler)
    if not shutil.which("cmake"):
        raise RuntimeError("CMake is not available on PATH")
    configure = ["cmake", "-S", str(project_root), "-B", str(build_dir)] + build_flags(args.config, 4)
    configure += cmake_compiler_args(args.compiler, args.generator)
    result = run(configure, timeout=args.timeout)
    if result.returncode:
        print(result.stdout[-12000:], end="", file=sys.stderr)
        raise RuntimeError("portable-replay CMake configure failed")
    build_result = run(["cmake", "--build", str(build_dir), "--config", args.config,
                        "--target", "portable-replay", "--parallel", str(args.parallel)], timeout=args.timeout)
    if build_result.returncode:
        print(build_result.stdout[-12000:], end="", file=sys.stderr)
        raise RuntimeError("portable-replay build failed")
    config_dir = args.config if platform.system() == "Windows" else ""
    exe = build_dir / "bin" / config_dir / ("portable-replay.exe" if platform.system() == "Windows" else "portable-replay")
    if not exe.is_file() and platform.system() == "Windows":
        exe = build_dir / "bin" / "portable-replay.exe"
    if not exe.is_file():
        raise RuntimeError("built portable-replay executable was not found")
    return exe, cmake_ids(result.stdout)


def capture_replay(exe: pathlib.Path, timeout: int) -> dict[str, Any]:
    exe = exe.resolve()
    if not exe.is_file():
        raise RuntimeError("portable-replay executable was not found")
    with tempfile.TemporaryDirectory(prefix="box3d-replay-") as temp_dir:
        recording = pathlib.Path(temp_dir) / "upstream-test.b3rec"
        captured = run([str(exe), "capture", str(recording)], timeout=timeout)
        if captured.returncode:
            print(captured.stdout[-12000:], end="", file=sys.stderr)
            raise RuntimeError("portable-replay capture command failed")
        cap = re.search(
            r"^CAPTURE frames=128 bytes=(\d+) seed_contacts=(\d+) seed_awake=(\d+) final_contacts=(\d+) final_awake=(\d+) sleep_frame=(-?\d+)\s*$",
            captured.stdout, re.MULTILINE)
        if not cap or int(cap.group(1)) <= 0 or int(cap.group(2)) < 1:
            print(captured.stdout[-12000:], end="", file=sys.stderr)
            raise RuntimeError("capture failed required frame, byte, or contact gate")
        if not recording.is_file() or recording.stat().st_size <= 0:
            raise RuntimeError("capture did not produce a nonempty recording")
        if int(cap.group(1)) != recording.stat().st_size:
            raise RuntimeError("capture byte-count marker does not match the recording file size")
        if recording.stat().st_size > 300 * 1024:
            raise RuntimeError("recording exceeds the 300 KiB public job-output limit")
        replayed = run([str(exe), "replay", str(recording)], timeout=timeout)
        if replayed.returncode:
            print(replayed.stdout[-12000:], end="", file=sys.stderr)
            raise RuntimeError("portable-replay replay command failed")
        accepted = re.search(
            r"^REPLAY_ACCEPTED frames=128 seed_contacts=(\d+) seed_awake=(\d+)\s*$",
            replayed.stdout, re.MULTILINE)
        passed = re.search(
            r"^REPLAY_PASS frames=128 first_divergent_frame=-1 seek=64->128 final_contacts=(\d+) final_awake=(\d+) sleep_frame=(-?\d+)\s*$",
            replayed.stdout, re.MULTILINE)
        if not accepted or not passed or int(accepted.group(1)) < 1:
            print(replayed.stdout[-12000:], end="", file=sys.stderr)
            raise RuntimeError("replay failed acceptance, deterministic frame, or seek marker gate")
        capture_values = tuple(int(cap.group(i)) for i in range(2, 7))
        replay_values = (int(accepted.group(1)), int(accepted.group(2)),
                         int(passed.group(1)), int(passed.group(2)), int(passed.group(3)))
        if capture_values != replay_values:
            raise RuntimeError("captured and replayed seed/final contacts, awake counts, or sleep frame differ")
        recording_bytes = recording.read_bytes()
        return {
            "capture": {"frames": 128, "bytes": len(recording_bytes),
                        "seed_contacts": capture_values[0], "seed_awake": capture_values[1],
                        "final_contacts": capture_values[2], "final_awake": capture_values[3],
                        "sleep_frame": capture_values[4]},
            "replay": {"frames": 128, "first_divergent_frame": -1,
                       "seed_contacts": replay_values[0], "seed_awake": replay_values[1],
                       "final_contacts": replay_values[2], "final_awake": replay_values[3],
                       "sleep_frame": replay_values[4], "seek": "64->128"},
            "state_metrics_match": True,
            "recording_removed": True,
            "recording_base64": base64.b64encode(recording_bytes).decode("ascii"),
        }


def replay_driver(args: argparse.Namespace) -> int:
    result = capture_replay(pathlib.Path(args.exe), args.timeout)
    result.pop("recording_base64", None)
    print(json.dumps({"schema": "box3d-ci-replay-v1", **result},
                     sort_keys=True, separators=(",", ":")))
    return 0


def safe_label(value: str) -> bool:
    return re.fullmatch(r"[a-z0-9][a-z0-9_-]{0,63}", value) is not None


def capture_producer(args: argparse.Namespace) -> int:
    origin = args.origin or os.environ.get("PUBLIC_ORIGIN", "")
    if not safe_label(origin) or not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", args.output_key):
        raise RuntimeError("PUBLIC_ORIGIN/origin and output key must be simple non-identifying labels")
    output_path = os.environ.get("GITHUB_OUTPUT")
    if not output_path:
        raise RuntimeError("GITHUB_OUTPUT is required for the producer mode")
    project_root = pathlib.Path(__file__).resolve().parent
    build_dir = pathlib.Path(args.build).resolve()
    exe, compiler_ids = build_overlay_replay(args, build_dir=build_dir)
    proof = capture_replay(exe, args.timeout)
    payload = proof.pop("recording_base64")
    raw = base64.b64decode(payload, validate=True)
    if not 0 < len(raw) <= 300 * 1024:
        raise RuntimeError("recording exceeds the 300 KiB public job-output limit")
    packed = base64.b64encode(zlib.compress(raw, level=9)).decode("ascii")
    record = {"schema": "box3d-public-recording-v1", "revision": PINNED_REVISION,
              "origin": origin, "encoding": "zlib+base64", "bytes": len(raw),
              "capture": proof["capture"], "sha256": hashlib.sha256(raw).hexdigest(),
              "payload": packed}
    encoded = json.dumps(record, sort_keys=True, separators=(",", ":"))
    if len(encoded) > 6500:
        print(json.dumps({"schema": "box3d-ci-size-limit-v1", "origin": origin,
                          "observed_characters": len(encoded), "limit_characters": 6500},
                         sort_keys=True, separators=(",", ":")))
        raise RuntimeError(f"compressed recording output is {len(encoded)} characters; per-origin limit is 6500")
    with open(output_path, "a", encoding="utf-8", newline="\n") as output:
        output.write(args.output_key + "=" + encoded + "\n")
    print(json.dumps({"schema": "box3d-ci-capture-v1", "record": {k: v for k, v in record.items() if k != "payload"},
                      "cmake_compilers": compiler_ids, "local_proof": proof,
                      "output_key": args.output_key, "payload_stdout": False},
                     sort_keys=True, separators=(",", ":")))
    return 0


def extract_record_objects(value: Any) -> list[dict[str, Any]]:
    found: list[dict[str, Any]] = []
    if isinstance(value, str):
        try:
            parsed = json.loads(value)
        except json.JSONDecodeError:
            return found
        return extract_record_objects(parsed)
    if isinstance(value, dict):
        if "schema" in value and "payload" in value:
            return [value]
        outputs = value.get("outputs")
        if isinstance(outputs, dict):
            for item in outputs.values():
                found.extend(extract_record_objects(item))
            return found
        for item in value.values():
            found.extend(extract_record_objects(item))
    elif isinstance(value, list):
        for item in value:
            found.extend(extract_record_objects(item))
    return found


def decode_public_records(raw_json: str) -> list[tuple[dict[str, Any], bytes]]:
    if len(raw_json) > 30000:
        raise RuntimeError(f"PUBLIC_RECORDINGS_JSON is {len(raw_json)} characters; limit is 30000")
    try:
        source = json.loads(raw_json)
    except json.JSONDecodeError as exc:
        raise RuntimeError("PUBLIC_RECORDINGS_JSON is not valid JSON") from exc
    records = extract_record_objects(source)
    if len(records) != 4:
        raise RuntimeError(f"expected exactly four public producer recordings; found {len(records)}")
    decoded: list[tuple[dict[str, Any], bytes]] = []
    origins = set()
    for record in records:
        if record.get("schema") != "box3d-public-recording-v1" or record.get("revision") != PINNED_REVISION:
            raise RuntimeError("producer recording has an unexpected schema or upstream revision")
        origin = record.get("origin")
        if not isinstance(origin, str) or not safe_label(origin) or origin in origins:
            raise RuntimeError("producer origins must be unique simple labels")
        origins.add(origin)
        if record.get("encoding") != "zlib+base64":
            raise RuntimeError(f"producer recording {origin} has an unsupported transport encoding")
        try:
            packed = base64.b64decode(record["payload"], validate=True)
        except (ValueError, TypeError) as exc:
            raise RuntimeError(f"producer recording {origin} has invalid compressed base64") from exc
        try:
            inflater = zlib.decompressobj()
            data = inflater.decompress(packed, 300 * 1024 + 1)
            if inflater.unconsumed_tail or len(data) > 300 * 1024 or not inflater.eof or inflater.unused_data:
                raise RuntimeError(f"producer recording {origin} exceeds the raw limit or has invalid zlib framing")
        except zlib.error as exc:
            raise RuntimeError(f"producer recording {origin} has invalid zlib data") from exc
        if not 0 < len(data) <= 300 * 1024 or record.get("bytes") != len(data):
            raise RuntimeError(f"producer recording {origin} has an invalid byte count")
        capture = record.get("capture")
        required_capture = ("frames", "bytes", "seed_contacts", "seed_awake", "final_contacts", "final_awake", "sleep_frame")
        if not isinstance(capture, dict) or any(not isinstance(capture.get(k), int) for k in required_capture):
            raise RuntimeError(f"producer recording {origin} has invalid capture metrics")
        if capture["frames"] != 128 or capture["bytes"] != len(data) or capture["seed_contacts"] < 1:
            raise RuntimeError(f"producer recording {origin} failed its capture metrics gate")
        digest = hashlib.sha256(data).hexdigest()
        if record.get("sha256") != digest:
            raise RuntimeError(f"producer recording {origin} SHA-256 does not match")
        decoded.append((record, data))
    return sorted(decoded, key=lambda pair: pair[0]["origin"])


def consume_one(exe: pathlib.Path, record: dict[str, Any], data: bytes, timeout: int) -> dict[str, Any]:
    summary: dict[str, Any] = {"origin": record["origin"], "bytes": len(data), "sha256": record["sha256"]}
    with tempfile.TemporaryDirectory(prefix="box3d-consume-") as temp_dir:
        file_path = pathlib.Path(temp_dir) / (record["origin"] + ".b3rec")
        file_path.write_bytes(data)
        result = run([str(exe), "replay", str(file_path)], timeout=timeout)
        accepted = re.search(r"^REPLAY_ACCEPTED frames=128 seed_contacts=(\d+) seed_awake=(\d+)", result.stdout, re.MULTILINE)
        passed = re.search(r"^REPLAY_PASS frames=128 first_divergent_frame=-1 seek=64->128 final_contacts=(\d+) final_awake=(\d+) sleep_frame=(-?\d+)", result.stdout, re.MULTILINE)
        negative: dict[str, Any] | None = None
        if result.returncode == 0 and accepted and passed:
            # One minimal negative control: corrupt only the first magic byte in a copy.
            corrupt = bytearray(data)
            corrupt[0] ^= 0x01
            negative_path = pathlib.Path(temp_dir) / (record["origin"] + "-bad-magic.b3rec")
            negative_path.write_bytes(corrupt)
            negative_result = run([str(exe), "replay", str(negative_path)], timeout=timeout)
            negative_ok = negative_result.returncode == 2 and "LOAD_REJECTED" in negative_result.stdout
            negative = {"status": "passed" if negative_ok else "failed",
                        "exit_code": negative_result.returncode,
                        "load_rejected_marker": "LOAD_REJECTED" in negative_result.stdout}
    if result.returncode in (2, 3):
        summary.update({"status": "rejected", "rejection_code": result.returncode,
                        "negative_control": "not_run_after_rejection"})
        return summary
    if result.returncode == 4:
        match = re.search(r"^REPLAY_DIVERGED first_divergent_frame=(-?\d+)", result.stdout, re.MULTILINE)
        summary.update({"status": "diverged", "first_divergent_frame": int(match.group(1)) if match else None,
                        "negative_control": "not_run_after_divergence"})
        return summary
    if result.returncode != 0:
        summary.update({"status": "gate_failure", "exit_code": result.returncode,
                        "negative_control": "not_run_after_gate_failure"})
        return summary
    if not accepted or not passed:
        summary.update({"status": "gate_failure", "exit_code": 0,
                        "negative_control": "not_run_after_gate_failure"})
        return summary
    replay_metrics = {"frames": 128, "seed_contacts": int(accepted.group(1)),
                      "seed_awake": int(accepted.group(2)), "final_contacts": int(passed.group(1)),
                      "final_awake": int(passed.group(2)), "sleep_frame": int(passed.group(3))}
    source_metrics = record["capture"]
    metric_keys = ("frames", "seed_contacts", "seed_awake", "final_contacts", "final_awake", "sleep_frame")
    metrics_match = all(replay_metrics[k] == source_metrics[k] for k in metric_keys)
    negative_passed = bool(negative and negative["status"] == "passed")
    overall = "pass" if metrics_match and negative_passed else ("state_mismatch" if not metrics_match else "negative_control_failed")
    summary.update({"status": overall, "frames": replay_metrics["frames"], "first_divergent_frame": -1,
                    "seed_contacts": replay_metrics["seed_contacts"], "seed_awake": replay_metrics["seed_awake"],
                    "final_contacts": replay_metrics["final_contacts"], "final_awake": replay_metrics["final_awake"],
                    "sleep_frame": replay_metrics["sleep_frame"], "capture_metrics_match": metrics_match,
                    "negative_control": negative})
    return summary

def consume_producers(args: argparse.Namespace) -> int:
    input_json = os.environ.get("PUBLIC_RECORDINGS_JSON")
    if not input_json:
        raise RuntimeError("PUBLIC_RECORDINGS_JSON is required for consumer mode")
    records = decode_public_records(input_json)
    build_dir = pathlib.Path(args.build).resolve()
    exe, compiler_ids = build_overlay_replay(args, build_dir=build_dir)
    # Run every valid source recording before returning failure so incompatibilities are comparable.
    results = [consume_one(exe, record, data, args.timeout) for record, data in records]
    summary = {"schema": "box3d-ci-consume-v1", "revision": PINNED_REVISION,
               "cmake_compilers": compiler_ids, "results": results,
               "all_accepted": all(item["status"] == "pass" and
                                    isinstance(item.get("negative_control"), dict) and
                                    item["negative_control"].get("status") == "passed"
                                    for item in results)}
    print(json.dumps(summary, sort_keys=True, separators=(",", ":")))
    return 0 if summary["all_accepted"] else 1


def make_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run the unchanged public Box3D test suite on preinstalled tools.")
    sub = parser.add_subparsers(dest="command", required=True)
    inv = sub.add_parser("inventory", help="emit privacy-limited CPU and architecture inventory JSON")
    inv.add_argument("--compiler-log", help="optional CMake configure log to summarize compiler IDs")
    inv.add_argument("--expected-arch", choices=("x86_64", "arm64"), required=True,
                     help="fail if the observed architecture differs")
    inv.set_defaults(func=inventory)
    test = sub.add_parser("test", help="configure, build, and repeat upstream tests for one SIMD variant")
    test.add_argument("--source", default="upstream", help="clean checkout of the pinned upstream Box3D revision")
    test.add_argument("--build", help="build directory; defaults to a distinct SIMD variant directory")
    test.add_argument("--config", choices=("Debug", "RelWithDebInfo"), default="Debug")
    test.add_argument("--simd", type=int, choices=(4, 8), required=True, help="4 disables AVX2; 8 enables the upstream AVX2 option")
    test.add_argument("--compiler", choices=("gcc", "clang", "msvc", "clang-cl"), required=True)
    test.add_argument("--generator", help="optional CMake generator")
    test.add_argument("--parallel", type=int, default=2)
    test.add_argument("--timeout", type=int, default=900)
    test.set_defaults(func=test_upstream)
    replay = sub.add_parser("replay", help="capture and replay the public Box3D recording in fresh processes")
    replay.add_argument("--exe", required=True, help="portable-replay executable built from public upstream sources")
    replay.add_argument("--timeout", type=int, default=900)
    replay.set_defaults(func=replay_driver)
    capture = sub.add_parser("capture", help="build a public source recording and export it as a bounded job output")
    capture.add_argument("--compiler", choices=("gcc", "clang", "msvc", "clang-cl"), required=True)
    capture.add_argument("--generator")
    capture.add_argument("--config", choices=("Debug", "RelWithDebInfo"), default="Debug")
    capture.add_argument("--build", default="capture-build")
    capture.add_argument("--origin", help="stable generic runner label; defaults to PUBLIC_ORIGIN")
    capture.add_argument("--output-key", default="recording")
    capture.add_argument("--parallel", type=int, default=2)
    capture.add_argument("--timeout", type=int, default=900)
    capture.set_defaults(func=capture_producer)
    consume = sub.add_parser("consume", help="replay all four producer records from PUBLIC_RECORDINGS_JSON")
    consume.add_argument("--compiler", choices=("gcc", "clang", "msvc", "clang-cl"), required=True)
    consume.add_argument("--generator")
    consume.add_argument("--config", choices=("Debug", "RelWithDebInfo"), default="Debug")
    consume.add_argument("--build", default="consume-build")
    consume.add_argument("--parallel", type=int, default=2)
    consume.add_argument("--timeout", type=int, default=900)
    consume.set_defaults(func=consume_producers)
    return parser


def main() -> int:
    args = make_parser().parse_args()
    try:
        return args.func(args)
    except (OSError, RuntimeError, subprocess.SubprocessError) as exc:
        print(json.dumps({"schema": "box3d-ci-error-v1", "error": str(exc)}, sort_keys=True), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())

