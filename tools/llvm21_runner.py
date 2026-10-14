#!/usr/bin/env python3
"""Fetch the pinned LLVM toolchain into an ephemeral Windows Actions runner."""

from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile
import time
import urllib.error
import urllib.request


LLVM_URL = (
    "https://github.com/llvm/llvm-project/releases/download/"
    "llvmorg-21.1.8/LLVM-21.1.8-win64.exe"
)
LLVM_VERSION = "21.1.8"
LLVM_ARCHIVE_NAME = "LLVM-21.1.8-win64.exe"
LLVM_ARCHIVE_BYTES = 374_129_373
LLVM_ARCHIVE_SHA256 = "7a5386c26497db1691f320121e5b113364dd0274b98e55f15f4dbc00c0450113"
INSTALL_DIR_NAME = "box3d-llvm-21.1.8"
EXTRACT_DIR_NAME = "extracted"
CHUNK_BYTES = 1024 * 1024
TRANSFER_TIMEOUT_SECONDS = 30 * 60
HTTP_READ_TIMEOUT_SECONDS = 60


class RunnerError(RuntimeError):
    """A controlled failure that is safe to report in the Actions log."""


def _within(parent: Path, candidate: Path) -> Path:
    """Resolve candidate and require it to be strictly beneath parent."""
    base = parent.resolve(strict=True)
    resolved = candidate.resolve(strict=False)
    try:
        relative = resolved.relative_to(base)
    except ValueError as exc:
        raise RunnerError(f"path escaped runner temp: {candidate}") from exc
    if not relative.parts:
        raise RunnerError(f"path must be below runner temp: {candidate}")
    return resolved


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while True:
            block = stream.read(CHUNK_BYTES)
            if not block:
                break
            digest.update(block)
    return digest.hexdigest()


def _verify_archive(path: Path) -> None:
    if path.is_symlink() or not path.is_file():
        raise RunnerError("pinned LLVM archive path is not a regular file")
    actual_bytes = path.stat().st_size
    if actual_bytes != LLVM_ARCHIVE_BYTES:
        raise RunnerError(
            f"LLVM archive size mismatch: expected {LLVM_ARCHIVE_BYTES}, got {actual_bytes}"
        )
    actual_sha256 = _sha256_file(path)
    if actual_sha256 != LLVM_ARCHIVE_SHA256:
        raise RunnerError("LLVM archive SHA-256 mismatch")


def _download_archive(root: Path) -> Path:
    archive = _within(root, root / LLVM_ARCHIVE_NAME)
    if archive.is_symlink():
        raise RunnerError("refusing a symlink at the pinned LLVM archive path")
    if archive.exists():
        _verify_archive(archive)
        return archive

    fd, temporary_name = tempfile.mkstemp(
        prefix=".llvm-21.1.8-", suffix=".partial", dir=str(root)
    )
    temporary = _within(root, Path(temporary_name))
    committed = False
    started = time.monotonic()
    digest = hashlib.sha256()
    total = 0
    try:
        request = urllib.request.Request(
            LLVM_URL,
            headers={"User-Agent": "box3d-upstream-ci/llvm-runner"},
            method="GET",
        )
        with os.fdopen(fd, "wb") as output:
            try:
                response_context = urllib.request.urlopen(
                    request, timeout=HTTP_READ_TIMEOUT_SECONDS
                )
            except (urllib.error.URLError, TimeoutError) as exc:
                raise RunnerError(f"LLVM download failed: {exc}") from exc
            with response_context as response:
                status = getattr(response, "status", 200)
                if status != 200:
                    raise RunnerError(f"LLVM download returned HTTP {status}")
                length_header = response.headers.get("Content-Length")
                if length_header is not None:
                    try:
                        declared_length = int(length_header)
                    except ValueError as exc:
                        raise RunnerError("invalid LLVM Content-Length header") from exc
                    if declared_length != LLVM_ARCHIVE_BYTES:
                        raise RunnerError(
                            "LLVM Content-Length does not match the pinned package size"
                        )
                while True:
                    if time.monotonic() - started > TRANSFER_TIMEOUT_SECONDS:
                        raise RunnerError("LLVM download exceeded its time limit")
                    block = response.read(CHUNK_BYTES)
                    if not block:
                        break
                    total += len(block)
                    if total > LLVM_ARCHIVE_BYTES:
                        raise RunnerError("LLVM download exceeded the pinned package size")
                    output.write(block)
                    digest.update(block)
            output.flush()
            os.fsync(output.fileno())

        if total != LLVM_ARCHIVE_BYTES:
            raise RunnerError(
                f"LLVM download size mismatch: expected {LLVM_ARCHIVE_BYTES}, got {total}"
            )
        if digest.hexdigest() != LLVM_ARCHIVE_SHA256:
            raise RunnerError("downloaded LLVM archive SHA-256 mismatch")
        if archive.exists() or archive.is_symlink():
            raise RunnerError("LLVM archive appeared during download; refusing overwrite")
        os.rename(temporary, archive)
        committed = True
        _verify_archive(archive)
        return archive
    finally:
        if not committed:
            try:
                temporary.unlink(missing_ok=True)
            except OSError:
                pass


def _find_7zip() -> Path:
    candidates: list[Path] = []
    found = shutil.which("7z")
    if found:
        candidates.append(Path(found))
    program_files = os.environ.get("ProgramFiles", r"C:\Program Files")
    candidates.append(Path(program_files) / "7-Zip" / "7z.exe")

    for candidate in candidates:
        try:
            resolved = candidate.resolve(strict=True)
        except OSError:
            continue
        if resolved.is_file():
            return resolved
    raise RunnerError("preinstalled 7-Zip was not found")


def _find_clang_cl(extract_root: Path) -> tuple[Path, Path] | None:
    matches: list[Path] = []
    for candidate in extract_root.rglob("clang-cl.exe"):
        if candidate.name.casefold() != "clang-cl.exe":
            continue
        if candidate.parent.name.casefold() != "bin":
            continue
        resolved = candidate.resolve(strict=True)
        _within(extract_root, resolved)
        if resolved.is_file():
            matches.append(resolved)
    if len(matches) > 1:
        raise RunnerError("LLVM extraction contains multiple bin/clang-cl.exe files")
    if not matches:
        return None
    compiler = matches[0]
    return compiler.parent.parent, compiler


def _extract_archive(archive: Path, root: Path) -> tuple[Path, Path]:
    extract_root = _within(root, root / EXTRACT_DIR_NAME)
    if extract_root.is_symlink():
        raise RunnerError("refusing a symlink at the LLVM extraction path")
    seven_zip = _find_7zip()

    if extract_root.exists():
        found = _find_clang_cl(extract_root)
        if found is None:
            raise RunnerError("existing LLVM extraction is incomplete; refusing overwrite")
        return found

    extract_root.mkdir()
    _within(root, extract_root)
    command = [
        str(seven_zip),
        "x",
        str(archive),
        f"-o{extract_root}",
        "-y",
        "-bd",
    ]
    try:
        completed = subprocess.run(
            command,
            check=False,
            timeout=10 * 60,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
            text=True,
            encoding="utf-8",
            errors="replace",
        )
    except subprocess.TimeoutExpired as exc:
        raise RunnerError("7-Zip extraction exceeded its time limit") from exc
    except OSError as exc:
        raise RunnerError(f"could not run preinstalled 7-Zip: {exc}") from exc

    if completed.returncode != 0:
        detail = (completed.stderr or "").strip()[-2000:]
        raise RunnerError(
            f"7-Zip extraction failed with exit code {completed.returncode}: {detail}"
        )
    found = _find_clang_cl(extract_root)
    if found is None:
        raise RunnerError("extracted package does not contain bin/clang-cl.exe")
    return found


def _verify_compiler(compiler: Path) -> str:
    try:
        completed = subprocess.run(
            [str(compiler), "--version"],
            check=False,
            timeout=30,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            encoding="utf-8",
            errors="replace",
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        raise RunnerError(f"could not verify clang-cl version: {exc}") from exc
    version_output = completed.stdout or ""
    if completed.returncode != 0 or not re.search(
        r"(?im)^\s*clang version 21\.1\.8(?:\b|[- ])", version_output
    ):
        raise RunnerError("extracted clang-cl is not LLVM 21.1.8")
    return version_output.splitlines()[0].strip()


def _write_github_outputs(llvm_root: Path, compiler: Path) -> None:
    output_name = os.environ.get("GITHUB_OUTPUT")
    if not output_name:
        raise RunnerError("GITHUB_OUTPUT is not set")
    output_path = Path(output_name)
    if output_path.is_symlink() or not output_path.is_file():
        raise RunnerError("GITHUB_OUTPUT is not an existing regular file")

    values = {
        "llvm_root": str(llvm_root),
        "clang_cl": str(compiler),
    }
    if any("\n" in value or "\r" in value for value in values.values()):
        raise RunnerError("runner paths cannot be represented safely in GITHUB_OUTPUT")
    with output_path.open("a", encoding="utf-8", newline="\n") as output:
        for key, value in values.items():
            output.write(f"{key}={value}\n")


def main() -> int:
    try:
        if os.name != "nt" or sys.platform != "win32":
            raise RunnerError("this LLVM helper is Windows-only")
        if os.environ.get("GITHUB_ACTIONS", "").casefold() != "true":
            raise RunnerError("this LLVM helper runs only on GitHub Actions")
        runner_temp_name = os.environ.get("RUNNER_TEMP")
        if not runner_temp_name:
            raise RunnerError("RUNNER_TEMP is not set")
        runner_temp = Path(runner_temp_name).resolve(strict=True)
        if not runner_temp.is_dir():
            raise RunnerError("RUNNER_TEMP is not a directory")

        root_candidate = runner_temp / INSTALL_DIR_NAME
        if root_candidate.is_symlink():
            raise RunnerError("refusing a symlink at the LLVM runner-temp path")
        root_candidate.mkdir(exist_ok=True)
        root = _within(runner_temp, root_candidate)
        if not root.is_dir():
            raise RunnerError("LLVM runner-temp path is not a directory")

        archive = _download_archive(root)
        llvm_root, compiler = _extract_archive(archive, root)
        version_line = _verify_compiler(compiler)
        _write_github_outputs(llvm_root, compiler)
        print(
            json.dumps(
                {
                    "schema": "box3d-llvm-runner-v1",
                    "status": "ready",
                    "version": LLVM_VERSION,
                    "archive_bytes": LLVM_ARCHIVE_BYTES,
                    "archive_sha256": LLVM_ARCHIVE_SHA256,
                    "llvm_root": str(llvm_root),
                    "clang_cl": str(compiler),
                    "version_line": version_line,
                },
                sort_keys=True,
            )
        )
        return 0
    except (RunnerError, OSError) as exc:
        print(
            json.dumps(
                {
                    "schema": "box3d-llvm-runner-v1",
                    "status": "error",
                    "error": str(exc),
                },
                sort_keys=True,
            ),
            file=sys.stderr,
        )
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
