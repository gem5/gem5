# -*- mode:python -*-
# Copyright (c) 2026 Arm Limited
# All rights reserved.
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

from lib.chi import read_shared

import m5
from m5.tlm_chi.utils import Resp

# Fixed batch: one cache line per request.
BATCH_BASE_ADDR = 0x0000_0000
BATCH_SIZE = 16
CACHELINE_SIZE = 64


TRANSACTIONS = []


def _find_simple_memory(system):
    if hasattr(system, "mem_ctrls") and system.mem_ctrls:
        return system.mem_ctrls[0]

    raise RuntimeError("Could not find SimpleMemory controller in system")


def _resolve_total_stat(simobj, stat_base_name):
    stat = simobj.resolveStat(stat_base_name)
    if stat is None:
        raise RuntimeError(f"Unable to resolve stat '{stat_base_name}'")

    return int(stat.total)


def simple_memory_verifier(system, expected_reads, expected_bytes):
    mem = _find_simple_memory(system)

    reads = _resolve_total_stat(mem, "numReads")
    bytes_read = _resolve_total_stat(mem, "bytesRead")
    writes = _resolve_total_stat(mem, "numWrites")

    assert (
        reads == expected_reads
    ), f"SimpleMemory numReads mismatch: expected={expected_reads}, actual={reads}"
    assert bytes_read == expected_bytes, (
        "SimpleMemory bytesRead mismatch: "
        f"expected={expected_bytes}, actual={bytes_read}"
    )
    assert (
        writes == 0
    ), f"SimpleMemory numWrites mismatch: expected=0, actual={writes}"


def test_all(system, generator):
    for idx in range(BATCH_SIZE):
        addr = BATCH_BASE_ADDR + idx * CACHELINE_SIZE
        TRANSACTIONS.append(read_shared(generator, addr, idx, Resp.RESP_UC))

    yield lambda sys: simple_memory_verifier(
        sys,
        expected_reads=BATCH_SIZE,
        expected_bytes=BATCH_SIZE * CACHELINE_SIZE,
    )
