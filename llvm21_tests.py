from __future__ import annotations

# Run the existing public upstream test gate with an explicitly selected,
# ephemeral LLVM toolchain. No engine, test, or frozen helper modifications.
# Selection docs: https://learn.microsoft.com/en-us/cpp/build/clang-support-msbuild
# https://cmake.org/cmake/help/latest/variable/CMAKE_VS_GLOBALS.html
import argparse
import hashlib
import importlib.util
import json
import os
import pathlib
import platform
import re
import subprocess
import sys
from typing import Any

sys.dont_write_bytecode = True
ROOT = pathlib.Path(__file__).resolve().parent
VERSION = "21.1.8"
CI_SHA256 = "a3ade53fc8bcb6a98d0b1816c2dfcb2c999155c8b54606a68446dd4270f9aa3f"


def frozen_helper_hash() -> str:
    digest = hashlib.sha256((ROOT / "ci.py").read_bytes()).hexdigest()
    if digest != CI_SHA256:
        raise RuntimeError("the frozen ci.py helper does not match its approved SHA-256")
    return digest


def load_ci():
    frozen_helper_hash()
    spec = importlib.util.spec_from_file_location("box3d_frozen_ci", ROOT / "ci.py")
    if spec is None or spec.loader is None:
        raise RuntimeError("could not load the frozen upstream test helper")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def inside(path: pathlib.Path, parent: pathlib.Path) -> bool:
    return path != parent and path.is_relative_to(parent)


def runner_paths(root_arg: str, build_arg: str | None, config: str) -> tuple[pathlib.Path, pathlib.Path]:
    if os.environ.get("GITHUB_ACTIONS") != "true" or platform.system() != "Windows":
        raise RuntimeError("native tests require an ephemeral Windows GitHub Actions runner")
    temp_value = os.environ.get("RUNNER_TEMP")
    if not temp_value:
        raise RuntimeError("RUNNER_TEMP is required")
    temp = pathlib.Path(temp_value).resolve(strict=True)
    llvm = pathlib.Path(root_arg).resolve(strict=True)
    if not llvm.is_dir() or not inside(llvm, temp):
        raise RuntimeError("LLVM root must be a subdirectory of RUNNER_TEMP")
    if any(character in str(llvm) for character in (';', '<', '>', chr(34), chr(13), chr(10))):
        raise RuntimeError("LLVM root cannot contain CMake-list or MSBuild-XML delimiters")
    compiler = llvm / "bin" / "clang-cl.exe"
    if not compiler.is_file() or not inside(compiler.resolve(strict=True), llvm):
        raise RuntimeError("the selected LLVM root does not contain its own clang-cl.exe")
    build = pathlib.Path(build_arg).resolve() if build_arg else temp / ("box3d-llvm21-tests-" + config.lower())
    build = build.resolve()
    if not inside(build, temp) or build == llvm or build.is_relative_to(llvm) or llvm.is_relative_to(build):
        raise RuntimeError("build directory must be separate from LLVM and inside RUNNER_TEMP")
    if build.exists() and (not build.is_dir() or any(build.iterdir())):
        raise RuntimeError("the LLVM test build directory must be fresh and empty")
    return llvm, build


def cmake_value(text: str, variable: str) -> str:
    matches = re.findall(r"^set\(" + re.escape(variable) + r' "([^"\r\n]*)"\)\s*$', text, re.MULTILINE)
    if len(matches) != 1:
        raise RuntimeError("missing or ambiguous CMake compiler metadata: " + variable)
    return matches[0]


def verify_compilers(ci: Any, build: pathlib.Path, llvm: pathlib.Path, output: str) -> dict[str, Any]:
    banners = ci.cmake_ids(output)
    expected = (llvm / "bin" / "clang-cl.exe").resolve(strict=True)
    verified = {}
    for language, key in (("C", "c"), ("CXX", "cxx")):
        if not re.match(r"^Clang 21\.1\.8(?:\s|$)", banners.get(key, "")):
            raise RuntimeError("CMake did not identify " + language + " as Clang " + VERSION)
        paths = list((build / "CMakeFiles").glob("*/CMake" + language + "Compiler.cmake"))
        if len(paths) != 1:
            raise RuntimeError("expected one fresh CMake " + language + " compiler metadata file")
        metadata = paths[0].read_text(encoding="utf-8")
        prefix = "CMAKE_" + language + "_"
        compiler_id = cmake_value(metadata, prefix + "COMPILER_ID")
        version = cmake_value(metadata, prefix + "COMPILER_VERSION")
        simulate_id = cmake_value(metadata, prefix + "SIMULATE_ID")
        declared = pathlib.Path(cmake_value(metadata, prefix + "COMPILER"))
        if compiler_id != "Clang" or version != VERSION or simulate_id != "MSVC":
            raise RuntimeError("CMake " + language + " compiler ID/version/frontend does not match the approved toolchain")
        if not declared.is_absolute() or declared.name.lower() != "clang-cl.exe":
            raise RuntimeError("CMake " + language + " compiler must be an absolute clang-cl.exe path")
        actual = declared.resolve(strict=True)
        if not inside(actual, llvm) or actual != expected:
            raise RuntimeError("CMake selected a " + language + " compiler outside the extracted LLVM root")
        verified[key] = {"id": compiler_id, "version": version, "simulate_id": simulate_id,
                         "path": str(declared), "sha256": hashlib.sha256(actual.read_bytes()).hexdigest(),
                         "metadata_sha256": hashlib.sha256(paths[0].read_bytes()).hexdigest()}
    return verified


def execute(ci: Any, args: argparse.Namespace, llvm: pathlib.Path, build: pathlib.Path) -> int:
    original_args = ci.cmake_compiler_args
    original_run = ci.run
    selection: dict[str, Any] | None = None
    configure_output = ""
    built = False

    def compiler_args(compiler: str | None, generator: str | None) -> list[str]:
        if compiler != "clang-cl" or generator != "Visual Studio 17 2022":
            raise RuntimeError("only the fixed Visual Studio ClangCL selection is allowed")
        return original_args(compiler, generator) + [
            "-DCMAKE_VS_GLOBALS=LLVMInstallDir=" + llvm.as_posix() + ";LLVMToolsVersion=" + VERSION]

    def guarded_run(argv: list[str], *, timeout: int = 120):
        nonlocal selection, configure_output, built
        cmake = pathlib.Path(argv[0]).name.lower() in ("cmake", "cmake.exe")
        if cmake and "--build" in argv:
            requested = pathlib.Path(argv[argv.index("--build") + 1]).resolve()
            if requested != build or selection is None:
                raise RuntimeError("CMake build cannot run before the approved compiler selection gate")
            # Check metadata again immediately before the native build.
            if verify_compilers(ci, build, llvm, configure_output) != selection:
                raise RuntimeError("compiler metadata changed between configure and build")
            built = True
        result = original_run(argv, timeout=timeout)
        if cmake and "-S" in argv and "-B" in argv and result.returncode == 0:
            requested = pathlib.Path(argv[argv.index("-B") + 1]).resolve()
            if requested != build:
                raise RuntimeError("configure used an unexpected build directory")
            configure_output = result.stdout
            selection = verify_compilers(ci, build, llvm, configure_output)
            print(json.dumps({"schema": "box3d-ci-llvm21-selection-v1", "version": VERSION,
                              "llvm_root": str(llvm), "compilers": selection,
                              "before_build": True}, sort_keys=True, separators=(",", ":")))
        return result

    test_args = argparse.Namespace(source=args.source, build=str(build), config=args.config,
                                   simd=8, compiler="clang-cl", generator="Visual Studio 17 2022",
                                   parallel=args.parallel, timeout=args.timeout)
    ci.cmake_compiler_args = compiler_args
    ci.run = guarded_run
    try:
        code = ci.test_upstream(test_args)
        frozen_helper_hash()
        if code == 0 and (selection is None or not built):
            raise RuntimeError("upstream tests returned without the required configure/build gates")
        print(json.dumps({"schema": "box3d-ci-llvm21-result-v1", "status": "pass" if code == 0 else "failed",
                          "exit_code": code, "version": VERSION, "configuration": args.config,
                          "requested_simd_width": 8, "box3d_avx2": "ON", "frozen_ci_sha256": CI_SHA256,
                          "compilers": selection}, sort_keys=True, separators=(",", ":")))
        return code
    finally:
        ci.cmake_compiler_args = original_args
        ci.run = original_run


def main() -> int:
    parser = argparse.ArgumentParser(description="Run unchanged upstream SIMD8 tests using approved ephemeral LLVM 21.1.8.")
    parser.add_argument("--llvm-root", required=True)
    parser.add_argument("--config", choices=("Debug", "RelWithDebInfo"), required=True)
    parser.add_argument("--source", default="upstream")
    parser.add_argument("--build", help="optional fresh directory under RUNNER_TEMP")
    parser.add_argument("--parallel", type=int, default=2)
    parser.add_argument("--timeout", type=int, default=900)
    args = parser.parse_args()
    if args.parallel < 1 or args.timeout < 1:
        parser.error("parallel and timeout must be positive")
    try:
        llvm, build = runner_paths(args.llvm_root, args.build, args.config)
        return execute(load_ci(), args, llvm, build)
    except (OSError, RuntimeError, subprocess.SubprocessError) as exc:
        print(json.dumps({"schema": "box3d-ci-llvm21-error-v1", "error": str(exc)},
                         sort_keys=True, separators=(",", ":")), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
