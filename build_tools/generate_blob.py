#!/usr/bin/env python3
# Copyright (c) 2021 Google, Inc.
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

"""Standalone blob generator for the CMake build.

Generates both a C++ header (.hh) and source (.cc) file that embed
a binary file as a byte array, making it accessible to C++ code.

This is the command-line equivalent of the SCons blob builder in
site_scons/gem5_scons/builders/blob.py.

Usage:
    generate_blob.py SYMBOL INPUT_FILE OUTPUT_CC OUTPUT_HH INCLUDE_PATH

Arguments:
    SYMBOL       - C++ symbol name for the blob array
    INPUT_FILE   - Path to the binary file to embed
    OUTPUT_CC    - Path to write the generated .cc file
    OUTPUT_HH    - Path to write the generated .hh file
    INCLUDE_PATH - Include path for the .hh file in the .cc file
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from blob import bytesToCppArray
from code_formatter import code_formatter


def generate(symbol, input_file, output_cc, output_hh, include_path):
    with open(input_file, "rb") as f:
        data = f.read()

    # Ensure output directories exist
    os.makedirs(os.path.dirname(output_hh), exist_ok=True)
    os.makedirs(os.path.dirname(output_cc), exist_ok=True)

    # Generate header
    hh_code = code_formatter()
    hh_code(
        """\
#include <cstddef>
#include <cstdint>

namespace gem5
{
namespace Blobs
{

extern const std::size_t ${symbol}_len;
extern const std::uint8_t ${symbol}[];

} // namespace Blobs
} // namespace gem5
"""
    )
    hh_code.write(output_hh)

    # Generate source
    cc_code = code_formatter()
    cc_code(
        """\
#include "${include_path}"

namespace gem5
{
namespace Blobs
{

const std::size_t ${symbol}_len = ${{len(data)}};
"""
    )
    bytesToCppArray(cc_code, symbol, data)
    cc_code(
        """
} // namespace Blobs
} // namespace gem5
"""
    )
    cc_code.write(output_cc)


if __name__ == "__main__":
    if len(sys.argv) != 6:
        print(
            f"Usage: {sys.argv[0]} SYMBOL INPUT_FILE OUTPUT_CC OUTPUT_HH INCLUDE_PATH",
            file=sys.stderr,
        )
        sys.exit(1)
    generate(sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4], sys.argv[5])
