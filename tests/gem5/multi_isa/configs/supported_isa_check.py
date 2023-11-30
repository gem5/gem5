# Copyright (c) 2022 The Regents of the University of California
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

"""
This is a very simple script to test the output given by
`gem5.runtime.get_supported_isas`
"""

import argparse
import os

from gem5.isas import (
    get_isa_from_str,
    get_isas_str_set,
)
from gem5.runtime import get_supported_isas

parser = argparse.ArgumentParser(
    description="A simple script used to check the output of "
    "`gem5.runtime.get_supported_isas`"
)

parser.add_argument(
    "-e",
    "--expected-isa",
    type=str,
    choices=get_isas_str_set(),
    required=True,
    help="An ISA expected to be included in the binary. If not returned by "
    "`get_supported_isas`, a non-zero exit code will be returned by the "
    "script",
)

args = parser.parse_args()
supported_isas = get_supported_isas()
expected_isa = get_isa_from_str(args.expected_isa)

if expected_isa in supported_isas:
    exit(0)

print(f"ISA expected: {args.expected_isa}")

supported_isas_str = ""
for isa in supported_isas:
    supported_isas += f"{os.linesep}{isa.value}"
print(f"get_supported_isas() returned:{supported_isas}")

exit(1)
