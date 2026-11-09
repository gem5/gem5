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


def make_elf(path, instruction, address=0x402000, operation="load"):
    # One load segment: ELF headers, code at +0x1000, test data at +0x2000.
    image = bytearray(0x2400)
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
    initial = bytearray(64)
    initial[0] = 1
    initial[1] = 3  # TILEZERO must reset this nonzero restart field.
    for tile in range(8):
        initial[16 + 2 * tile] = 64
        initial[48 + tile] = 8
    target = initial.copy()
    target[1] = 5
    target[16] = 32
    target[48] = 4  # A no-op LDTILECFG must not pass a round trip.
    expected = initial.copy()
    if operation == "load":
        expected = target
    elif operation == "palette-zero":
        target[0] = 0  # Other bytes are intentionally not zero.
        expected = bytes(64)
    elif operation == "zero":
        expected[1] = 0
    elif operation == "release":
        expected = bytes(64)
    image[0x2000:0x2040] = target
    image[0x2040:0x2080] = initial
    # The store destination and its guards are distinct from both configs.
    image[0x2100:0x2150] = bytes([0xA5]) * 80
    image[0x2200:0x2250] = bytes([0xA5]) * 8 + expected + bytes([0xA5]) * 8
    image[0x2300 : 0x2300 + len(MARKER)] = MARKER

    def mov_rdi(value):
        return b"\x48\xbf" + struct.pack("<Q", value)

    code = mov_rdi(0x402040) + bytes.fromhex("c4e2784907")
    if operation == "store" and address == 0x402000:
        address = 0x402108
    code += mov_rdi(address) + bytes.fromhex(instruction)
    if operation != "store":
        code += mov_rdi(0x402108) + bytes.fromhex("c4e2794907")
    # Compare all 64 output bytes and both guards, not merely execution.
    code += mov_rdi(0x402100)
    code += b"\x48\xbe" + struct.pack("<Q", 0x402200)
    code += bytes.fromhex("b950000000fcf3a6")  # ecx=80; cld; repe cmpsb
    code += bytes.fromhex("0f85")
    branch = len(code)
    code += bytes(4)
    code += bytes.fromhex("b801000000bf0100000048be")
    code += struct.pack("<Q", 0x402300)
    code += b"\xba" + struct.pack("<I", len(MARKER))
    code += bytes.fromhex("0f05b83c00000031ff0f05")
    failure = len(code)
    code += bytes.fromhex("b83c000000bf010000000f05")  # exit(1), no marker
    code = bytearray(code)
    code[branch : branch + 4] = struct.pack("<i", failure - branch - 4)
    image[0x1000 : 0x1000 + len(code)] = code
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
    yield "load-stack", "4889fdc4e278494500", "ok", 0x402000
    yield "store-stack", "4889fdc4e279494500", "ok", 0x402000
    yield "load-palette-zero", forms["load"], "ok", 0x402000
    for tile in range(1, 8):
        yield f"zero-tmm{tile}", f"c4e27b49{0xc0 + 8 * tile:02x}", "ok", 0x402000
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
            operation = name.split("-", 1)[0]
            if name == "load-palette-zero":
                operation = "palette-zero"
            make_elf(binary, encoding, address, operation)
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
