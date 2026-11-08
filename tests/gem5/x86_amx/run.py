#
# Copyright (c) 2026 Jiaqi Wang
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

"""Execute raw AMX encodings on Atomic, Timing, and O3 (no guest compiler)."""

import argparse
import json
import resource
import struct
import subprocess
from pathlib import Path

MARKER = b"AMX_CASE_OK\n"


def make_elf(path, instruction, address=0x402000):
    # One load segment: ELF headers, code at +0x1000, tilecfg at +0x2000.
    image = bytearray(0x2100)
    ident = b"\x7fELF\x02\x01\x01" + bytes(9)
    image[:64] = struct.pack(
        "<16sHHIQQQIHHHHHH",
        ident,
        2,
        62,
        1,
        0x401000,
        64,
        0,
        0,
        64,
        56,
        1,
        0,
        0,
        0,
    )
    image[64:120] = struct.pack(
        "<IIQQQQQQ",
        1,
        7,
        0,
        0x400000,
        0x400000,
        len(image),
        len(image),
        0x1000,
    )
    image[0x2000] = 1
    for tile in range(8):
        image[0x2010 + 2 * tile] = 64
        image[0x2030 + tile] = 8
    # Load a valid configuration before exercising the target encoding.
    code = b"\x48\xbf" + struct.pack("<Q", 0x402000)
    code += bytes.fromhex("c4e2784907")
    code += b"\x48\xbf" + struct.pack("<Q", address)
    code += bytes.fromhex(instruction)
    # Write a marker only after the instruction completes; then exit(0).
    code += bytes.fromhex("b801000000bf0100000048be")
    code += struct.pack("<Q", 0x402080)
    code += b"\xba" + struct.pack("<I", len(MARKER))
    code += bytes.fromhex("0f05b83c00000031ff0f05")
    image[0x1000 : 0x1000 + len(code)] = code
    image[0x2080 : 0x2080 + len(MARKER)] = MARKER
    path.write_bytes(image)
    path.chmod(0o755)


def cases():
    # Expected outcomes follow SDM AMX-E1/E2/E5/E6 and operand roles.
    forms = {
        "load": "c4e2784907",
        "store": "c4e2794907",
        "zero": "c4e27b49c0",
        "release": "c4e27849c0",
    }
    for name, encoding in forms.items():
        yield name, encoding, "ok", 0x402000
        for prefix in ("66", "f2", "f3", "f0", "40", "48"):
            yield f"{name}-prefix-{prefix}", prefix + encoding, "ud", 0x402000
        for prefix in ("67", "2e", "4067"):
            yield f"{name}-prefix-{prefix}", prefix + encoding, "ok", 0x402000
        for field, mask in (("W", 0x80), ("L", 4), ("vvvv", 8)):
            raw = bytearray.fromhex(encoding)
            raw[2] ^= mask
            yield f"{name}-{field}", raw.hex(), "ud", 0x402000
        raw = bytearray.fromhex(encoding)
        raw[1] ^= 0x80
        yield f"{name}-R", raw.hex(), (
            "ud" if name == "zero" else "ok"
        ), 0x402000
        for field, mask in (("X", 0x40), ("B", 0x20)):
            if field == "B" and name in ("load", "store"):
                continue  # B names a real base register in memory forms.
            raw = bytearray.fromhex(encoding)
            raw[1] ^= mask
            yield f"{name}-{field}", raw.hex(), "ok", 0x402000
    for name, encoding in (
        ("load-F3", "c4e27a4907"),
        ("release-F3", "c4e27a49c0"),
    ):
        yield name, encoding, "ud", 0x402000
    for name, encoding in (("load", "c4e2784907"), ("store", "c4e2794907")):
        yield name + "-noncanonical-DS", encoding, "gp", 0x0100000000000000
        # Upper-address wraparound is canonical; the unmapped page faults.
        yield name + "-upper-wrap", encoding, "pf", 0xFFFFFFFFFFFFFFF0
        # mov rbp,rdi; use [rbp+0], whose default segment is SS.
        ss = "4889fd" + encoding[:-2] + "4500"
        yield name + "-noncanonical-SS", ss, "ss", 0x0100000000000000


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gem5", type=Path, required=True)
    parser.add_argument("--outdir", type=Path, required=True)
    parser.add_argument(
        "--cpus",
        nargs="+",
        choices=("atomic", "timing", "o3"),
        default=("atomic", "timing", "o3"),
    )
    args = parser.parse_args()
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))
    config = Path(__file__).with_name("config.py")
    results = []
    faults = {
        "ud": "Unrecognized/invalid instruction executed",
        "gp": "fault (General-Protection)",
        "ss": "fault (Stack)",
        "pf": "unmapped address",
    }
    for cpu in args.cpus:
        for name, encoding, expected, address in cases():
            directory = args.outdir / cpu / name
            directory.mkdir(parents=True, exist_ok=True)
            binary = directory / "probe.elf"
            make_elf(binary, encoding, address)
            command = [
                str(args.gem5.resolve()),
                "-d",
                str(directory),
                str(config.resolve()),
                "--binary",
                str(binary.resolve()),
                "--cpu",
                cpu,
            ]
            result = subprocess.run(
                command, capture_output=True, text=True, timeout=90
            )
            output = result.stdout + result.stderr
            (directory / "output.log").write_text(output)
            passed = (
                result.returncode == 0 and MARKER.decode() in output
                if expected == "ok"
                else result.returncode != 0 and faults[expected] in output
            )
            results.append(
                {
                    "cpu": cpu,
                    "case": name,
                    "expected": expected,
                    "returncode": result.returncode,
                    "passed": passed,
                }
            )
            if not passed:
                print(
                    f"FAIL {cpu}/{name}: see {directory / 'output.log'}",
                    flush=True,
                )
    (args.outdir / "results.json").write_text(
        json.dumps(results, indent=2) + "\n"
    )
    failed = sum(not row["passed"] for row in results)
    print(
        f"{len(results) - failed}/{len(results)} AMX instruction tests passed"
    )
    return int(failed != 0)


if __name__ == "__main__":
    raise SystemExit(main())
