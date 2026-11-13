# Copyright (c) 2026 Boyana Norris
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
# contributors may be used to endorse or promote products derived from this
# software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

"""Build the gfx942 regression programs and their guest launch scripts."""

import argparse
import base64
import gzip
import os
import subprocess
from pathlib import Path

PROGRAMS = ("division", "lds_exchange")


def build(rocm_path, output_dir):
    rocm_path = rocm_path.resolve()
    compiler = rocm_path / "lib" / "llvm" / "bin" / "clang++"
    if not compiler.is_file():
        raise FileNotFoundError(f"ROCm clang++ was not found at {compiler}")
    output_dir.mkdir(parents=True, exist_ok=True)
    for program in PROGRAMS:
        binary = output_dir / program
        source = Path(__file__).resolve().parent / f"{program}.hip"
        command = [
            str(compiler),
            "-x",
            "hip",
            f"--rocm-path={rocm_path}",
            "--offload-arch=gfx942",
            "-std=c++17",
            "-O2",
            "-Wall",
            "-Wextra",
            str(source),
            "-o",
            str(binary),
            f"-L{rocm_path / 'lib'}",
            "-lamdhip64",
            "-Wl,-rpath,/opt/rocm/lib",
        ]
        log_path = output_dir / f"{program}.build.log"
        with log_path.open("w") as log:
            subprocess.run(
                command,
                cwd=output_dir,
                stdout=log,
                stderr=subprocess.STDOUT,
                check=True,
            )
        payload = base64.b64encode(
            gzip.compress(binary.read_bytes(), mtime=0)
        ).decode("ascii")
        # make_gpu_app transfers one file to the guest. Package the binary
        # in a shell script so no disk-image changes or downloads are needed.
        wrapper = output_dir / f"{program}.sh"
        wrapper.write_text(
            "#!/bin/bash\n"
            "set -eu\n"
            "app_dir=$(mktemp -d)\n"
            "trap 'rm -rf \"$app_dir\"' EXIT\n"
            "base64 -d <<'GEM5_GPU_ISA_BINARY' | gzip -d > "
            f'"$app_dir/{program}"\n'
            f"{payload}\n"
            "GEM5_GPU_ISA_BINARY\n"
            f'chmod +x "$app_dir/{program}"\n'
            f'"$app_dir/{program}"\n'
        )
        wrapper.chmod(0o755)
        print(f"Built {wrapper}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--rocm-path",
        type=Path,
        default=Path(os.environ.get("ROCM_PATH", "/opt/rocm")),
        help=(
            "ROCm toolchain compatible with the GPUFS guest "
            "(default: ROCM_PATH)"
        ),
    )
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    build(args.rocm_path, args.output_dir.resolve())


if __name__ == "__main__":
    main()
