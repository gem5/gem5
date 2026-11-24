# Copyright (c) 2026 Google LLC
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


def upgrader(cpt):
    """
    Update RISC-V PLIC checkpoints to the input-queue and per-source
    claim model.

    The PLIC used to delay its output by output_latency and track one
    claimed interrupt per context (lastID). It now delays interrupt
    source inputs and tracks claimed interrupts per source in a bitmask
    (claimedInt).

    - Convert each non-zero lastID entry to a bit in claimedInt.
    - Drop the queued outputs (n_outputs and output_*). The PLIC
      recomputes its output from the restored state on unserialize.
    - Start with an empty input queue (n_inputs = 0).
    """
    import re

    for sec in cpt.sections():
        if not (
            cpt.has_option(sec, "lastID") and cpt.has_option(sec, "n_outputs")
        ):
            continue

        n_src32 = sum(
            1 for key in cpt.options(sec) if re.fullmatch(r"pending\d+", key)
        )
        claimed = [0] * n_src32
        for src_id in map(int, cpt.get(sec, "lastID").split()):
            if src_id:
                claimed[src_id >> 5] |= 1 << (src_id & 0x1F)
        cpt.set(sec, "claimedInt", " ".join(str(x) for x in claimed))
        cpt.remove_option(sec, "lastID")

        for i in range(int(cpt.get(sec, "n_outputs"))):
            for key in ("output_tick", "output_id", "output_pri"):
                cpt.remove_option(sec, f"{key}{i}")
        cpt.remove_option(sec, "n_outputs")
        cpt.set(sec, "n_inputs", "0")
