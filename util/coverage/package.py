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

"""Retain report inputs separately from compressed native counter archives."""

import argparse
import gzip
import json
import shutil
import tarfile
from pathlib import Path


def package(source, status, output, raw_output, native=False):
    output.mkdir(parents=True, exist_ok=True)
    if status.is_file():
        shutil.copyfile(status, output / "status.json")
    else:
        # Preserve a visible failure if execution never reached its first stage.
        (output / "status.json").write_text(
            json.dumps({"stages": {"test": "failure"}}) + "\n"
        )
    if not source.exists():
        return
    if not native:
        for path in source.rglob("*.json"):
            if path.name not in {
                "coverage.json",
                "python-coverage.json",
                "baseline.json",
            }:
                continue
            dest = output / "coverage" / path.relative_to(source)
            dest = dest.with_suffix(dest.suffix + ".gz")
            dest.parent.mkdir(parents=True, exist_ok=True)
            with path.open("rb") as data, gzip.open(dest, "wb") as compressed:
                shutil.copyfileobj(data, compressed)
    raw_output.parent.mkdir(parents=True, exist_ok=True)
    with tarfile.open(raw_output, "w:gz") as archive:
        # tarfile preserves hard links, so shared notes are stored only once.
        if native:
            for path in sorted(source.rglob("*")):
                if (
                    path.is_file()
                    and not path.is_symlink()
                    and (
                        path.suffix in {".gcno", ".gcda"}
                        or path.name == ".config"
                    )
                ):
                    archive.add(path, arcname=str(path))
        else:
            archive.add(source, arcname="coverage")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("--status", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--raw-output", required=True, type=Path)
    parser.add_argument("--native", action="store_true")
    args = parser.parse_args()
    package(
        args.source, args.status, args.output, args.raw_output, args.native
    )


if __name__ == "__main__":
    main()
