# Copyright (c) 2026 The Board of Trustees of the Leland Stanford
# Junior University
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

import os
from argparse import ArgumentParser

from m5.objects import Process
from m5.util import addToPath

addToPath("../../")

from common import Options


def make_parser() -> ArgumentParser:
    parser = ArgumentParser()
    parser.add_argument(
        "--chdir",
        default=".",
        help="Set working directory of simulated process",
    )
    parser.add_argument("--max-stack-size", type=str, default="64MB")
    Options.addCommonOptions(parser)
    Options.addSEOptions(parser)
    parser.add_argument("cmd", help="Executable to simulate")
    parser.add_argument(
        "args", nargs="*", help="Arguments to pass to executable"
    )

    # FIXME: Shouldn't hard-code this. Should generate the path at build
    # time, like other
    # m5 paths.
    gem5_root = os.path.dirname(
        os.path.dirname(os.path.dirname(os.path.dirname(__file__)))
    )
    parser.add_argument(
        "--pin",
        default=os.path.join(gem5_root, "pin", "pin"),
        help="Path to Intel Pin executable",
    )
    parser.add_argument(
        "--pin-tool",
        default=os.path.join(gem5_root, "pintool", "build", "libclient.so"),
        help="Path to host PinTool",
    ),
    parser.add_argument(
        "--pin-guest",
        default=os.path.join(gem5_root, "pintool", "build", "guest"),
        help="Path to Pin guest",
    )

    parser.add_argument("--stdin", default="/dev/stdin")
    parser.add_argument("--stdout", default="stdout.txt")
    parser.add_argument("--stderr", default="stderr.txt")

    return parser


# Shift the address space down, to avoid collisions with Pin's
# internal mappings.
ADDR_SPACE_SHIFT = -0x100000000000


def make_process(args) -> Process:
    process = Process(pid=100)
    process.addrSpaceShift = ADDR_SPACE_SHIFT
    process.executable = args.cmd
    process.cwd = os.path.abspath(args.chdir)
    # process.gid = os.getgid()
    process.maxStackSize = args.max_stack_size

    # Clear out the environment.
    process.env = []

    process.cmd = [args.cmd, *args.args]

    # Get stdin path.
    process.input = (
        args.stdin
        if os.path.isabs(args.stdin)
        else os.path.join(args.chdir, args.stdin)
    )
    process.output = args.stdout
    process.errout = args.stderr

    return process
