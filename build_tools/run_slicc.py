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

import argparse
import os
import sys


def remove_stale_protocol_outputs(slicc, output_dir):
    protocol_dir_path = os.path.join(output_dir, slicc.protocol)

    if not os.path.isdir(protocol_dir_path):
        return

    expected = set()
    for filename in slicc.files():
        if os.path.dirname(filename) == slicc.protocol:
            expected.add(os.path.basename(filename))

    expected.add(f"{slicc.protocol}ProtocolInfo.hh")
    for filename in os.listdir(protocol_dir_path):
        filepath = os.path.join(protocol_dir_path, filename)
        if not os.path.isfile(filepath):
            continue

        if filename.endswith(".py.cc"):
            continue

        if (
            filename.endswith((".cc", ".hh", ".py"))
            and filename not in expected
        ):
            os.remove(filepath)

            if filename.endswith(".cc"):
                objpath = filepath[:-3] + ".o"
                if os.path.exists(objpath):
                    os.remove(objpath)
            elif filename.endswith(".py"):
                for suffix in (".cc", ".o", ".pyo"):
                    sidecar = filepath + suffix
                    if os.path.exists(sidecar):
                        os.remove(sidecar)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--src-root", required=True)
    parser.add_argument("--code-path", required=True)
    parser.add_argument("--protocol-base", required=True)
    parser.add_argument("--html-path")
    parser.add_argument("--verbose", action="store_true")
    parser.add_argument("--include", action="append", default=[])
    parser.add_argument("slicc_files", nargs="+")
    args = parser.parse_args()

    sys.path.append(os.path.join(args.src_root, "src", "mem"))
    sys.path.append(os.path.join(args.src_root, "ext", "ply"))

    from slicc.parser import SLICC

    for slicc_file in args.slicc_files:
        slicc = SLICC(
            slicc_file,
            [os.path.join(args.protocol_base, "RubySlicc_interfaces.slicc")],
            args.protocol_base,
            verbose=args.verbose,
        )
        slicc.process()
        remove_stale_protocol_outputs(slicc, args.code_path)
        slicc.writeCodeFiles(args.code_path, args.include)
        if args.html_path:
            slicc.writeHTMLFiles(args.html_path)


if __name__ == "__main__":
    main()
