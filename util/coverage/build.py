# Copyright (c) 2026 The Regents of The University of California
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met: redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer;
# redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution;
# neither the name of the copyright holders nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""Build once and share verified GCC coverage binaries within a campaign."""

import argparse
import hashlib
import json
import os
import re
import shutil
import subprocess
import sys
import tarfile
from pathlib import (
    Path,
    PurePosixPath,
)

SOURCE_SUFFIXES = {".gcno", ".cc", ".hh", ".c", ".h", ".inc"}


def target_name(target):
    """Validate a relative TestLib binary target and return its artifact key."""
    if not re.fullmatch(r"build/[A-Za-z0-9_]+/gem5\.(opt|debug|fast)", target):
        raise ValueError(f"Invalid coverage build target: {target}")
    return target.removeprefix("build/").replace("/", "-")


def relative_target(root, target):
    path = Path(target)
    if path.is_absolute():
        path = path.relative_to(root)
    result = path.as_posix()
    target_name(result)
    return result


def discover(root, test_dir="gem5", lengths=("quick", "long", "very-long")):
    targets = set()
    for length in lengths:
        result = subprocess.check_output(
            [
                sys.executable,
                "main.py",
                "list",
                test_dir,
                f"--length={length}",
                "--host=x86_64",
                "--gcov=per-test",
                "--build-targets",
                "-q",
            ],
            cwd=root / "tests",
            text=True,
        )
        targets.update(
            relative_target(root, line)
            for line in result.splitlines()
            if line.strip()
        )
    return sorted(targets)


def configuration(root, target):
    target_name(target)
    directory = Path(target).parent.name
    choices = sorted(
        (p.name for p in (root / "build_opts").iterdir() if p.is_file()),
        key=len,
        reverse=True,
    )
    for name in choices:
        if directory == name:
            return name, None
        if directory.startswith(name + "_"):
            return name, directory[len(name) + 1 :]
    raise ValueError(f"No build configuration for {target}")


def build(root, target, jobs):
    isa, protocol = configuration(root, target)
    directory = str(Path(target).parent)
    commands = [
        ["defconfig", directory, f"build_opts/{isa}"],
        ["setconfig", directory, "USE_TEST_OBJECTS=y"],
    ]
    if protocol:
        commands.append(
            ["setconfig", directory, f"RUBY_PROTOCOL_{protocol.upper()}=y"]
        )
    commands.append(["--gcov", target, f"-j{jobs}", "CXX=g++", "CC=gcc"])
    for command in commands:
        subprocess.run(
            ["scons", "--ignore-style", *command], cwd=root, check=True
        )


def stream_digest(stream):
    checksum = hashlib.sha256()
    for chunk in iter(lambda: stream.read(1024 * 1024), b""):
        checksum.update(chunk)
    return checksum.hexdigest()


def digest(path):
    with path.open("rb") as stream:
        return stream_digest(stream)


def package(root, target, image, output):
    name = target_name(target)
    if "@sha256:" not in image:
        raise ValueError(
            "Coverage builds require an immutable container digest"
        )
    directory = root / Path(target).parent
    binary = root / target
    if not binary.is_file():
        raise ValueError(f"Missing coverage executable: {target}")
    files = {binary}
    files.update(
        p
        for p in directory.rglob("*")
        if p.is_file()
        and not p.is_symlink()
        and (p.suffix in SOURCE_SUFFIXES or p.name == ".config")
        and not p.name.endswith(".py.gcno")
    )
    if not any(p.suffix == ".gcno" for p in files):
        raise ValueError("Coverage build has no coverage notes")
    manifest = {
        "schema_version": 1,
        "revision": subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=root, text=True
        ).strip(),
        "target": target,
        "build_root": str(root),
        "image": image,
        "compiler": subprocess.check_output(["g++", "--version"], text=True),
        "gcov": subprocess.check_output(["gcov", "--version"], text=True),
        "instrumentation": "gcc --gcov",
        "test_objects": True,
        "files": {
            p.relative_to(root).as_posix(): digest(p) for p in sorted(files)
        },
    }
    output.mkdir(parents=True, exist_ok=True)
    manifest_path = output / f"{name}.json"
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n")
    with tarfile.open(
        output / f"{name}.tar.gz", "w:gz", compresslevel=1
    ) as archive:
        for path in sorted(files):
            archive.add(path, arcname=path.relative_to(root), recursive=False)
    return manifest


def unpack(root, manifest_path, revision, image):
    manifest = json.loads(manifest_path.read_text())
    if manifest.get("schema_version") != 1:
        raise ValueError("Unsupported coverage build manifest")
    target_name(manifest["target"])
    if manifest["revision"] != revision or manifest["image"] != image:
        raise ValueError("Coverage build revision/container does not match")
    if not manifest.get("test_objects"):
        raise ValueError("Coverage build lacks TestLib objects")
    expected = manifest["files"]
    if manifest["target"] not in expected:
        raise ValueError("Coverage manifest does not include its executable")
    prefix = str(PurePosixPath(manifest["target"]).parent) + "/"
    for name in expected:
        path = PurePosixPath(name)
        if (
            path.is_absolute()
            or ".." in path.parts
            or not name.startswith(prefix)
        ):
            raise ValueError(f"Invalid build archive member: {name}")
    archive_path = manifest_path.with_suffix(".tar.gz")
    with tarfile.open(archive_path) as archive:
        members = archive.getmembers()
        if len(members) != len(expected) or {m.name for m in members} != set(
            expected
        ):
            raise ValueError("Coverage archive contents do not match manifest")
        # Check every checksum before installing any files in the build tree.
        for member in members:
            if not member.isfile():
                raise ValueError(
                    "Only regular coverage build files are allowed"
                )
            with archive.extractfile(member) as stream:
                actual = stream_digest(stream)
            if actual != expected[member.name]:
                raise ValueError(
                    f"Coverage build checksum mismatch: {member.name}"
                )
        for member in members:
            destination = root / member.name
            if not destination.resolve().is_relative_to(root.resolve()):
                raise ValueError("Build destination escapes checkout")
            destination.parent.mkdir(parents=True, exist_ok=True)
            with (
                archive.extractfile(member) as source,
                destination.open("wb") as dest,
            ):
                shutil.copyfileobj(source, dest)
        (root / manifest["target"]).chmod(0o755)
    # Retain the exact build identity alongside the binary for collectors.
    destination = (
        root / Path(manifest["target"]).parent / f"{manifest_path.stem}.json"
    )
    shutil.copyfile(manifest_path, destination)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path.cwd())
    commands = parser.add_subparsers(dest="command", required=True)
    plan = commands.add_parser("plan")
    plan.add_argument("--test-dir", default="gem5")
    plan.add_argument("--length", choices=("quick", "long", "very-long"))
    plan.add_argument("--pattern", action="store_true")
    compile_parser = commands.add_parser("build")
    compile_parser.add_argument("target")
    compile_parser.add_argument("--jobs", type=int, default=os.cpu_count())
    compile_parser.add_argument("--image", required=True)
    compile_parser.add_argument("--output", type=Path, required=True)
    install = commands.add_parser("unpack")
    install.add_argument("directory", type=Path)
    install.add_argument("--revision", required=True)
    install.add_argument("--image", required=True)
    args = parser.parse_args()
    root = args.root.resolve()
    if args.command == "plan":
        lengths = (
            (args.length,) if args.length else ("quick", "long", "very-long")
        )
        targets = discover(root, args.test_dir, lengths)
        if args.pattern:
            print(
                "coverage-build-@(" + "|".join(map(target_name, targets)) + ")"
                if targets
                else ""
            )
        else:
            print(json.dumps(targets))
    elif args.command == "build":
        build(root, args.target, args.jobs)
        package(root, args.target, args.image, args.output)
    else:
        manifests = sorted(args.directory.glob("*.json"))
        if not manifests:
            raise ValueError("No coverage build artifacts were downloaded")
        for manifest in manifests:
            unpack(root, manifest, args.revision, args.image)


if __name__ == "__main__":
    main()
