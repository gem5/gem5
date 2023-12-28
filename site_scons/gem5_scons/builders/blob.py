# -*- mode:python -*-

# Copyright (c) 2018, 2020 ARM Limited
#
# The license below extends only to copyright in the software and shall
# not be construed as granting a license to any other intellectual
# property including but not limited to intellectual property relating
# to a hardware implementation of the functionality of the software
# licensed hereunder.  You may use the software subject to the license
# terms below provided that you ensure that this notice is replicated
# unmodified and in its entirety in all distributions of the software,
# modified or unmodified, in source code or in binary form.
#
# Copyright (c) 2004-2005 The Regents of The University of Michigan
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

import os.path

import SCons.Node.Python
from blob import bytesToCppArray
from code_formatter import code_formatter
from gem5_scons import (
    MakeAction,
    Transform,
)


def build_blob(target, source, env):
    """
    Embed an arbitrary blob into the gem5 executable,
    and make it accessible to C++ as a byte array.
    """

    with open(str(source[0]), "rb") as f:
        data = f.read()
    symbol = str(source[1])
    cc, hh = target

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
    hh_code.write(str(hh))

    include_path = os.path.relpath(hh.abspath, env["BUILDDIR"])

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
    cc_code.write(str(cc))


blob_action = MakeAction(build_blob, Transform("EMBED BLOB"))


def blob_emitter(target, source, env):
    symbol = str(target[0])
    cc_file = env.File(symbol + ".cc")
    hh_file = env.File(symbol + ".hh")
    return [cc_file, hh_file], [source, SCons.Node.Python.Value(symbol)]


def Blob(env):
    blob_builder = env.Builder(action=blob_action, emitter=blob_emitter)
    env.Append(BUILDERS={"Blob": blob_builder})
