# Copyright (c) 2026 Aalhad Sawane
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
Regression test for partial-line DMA writes with the CHI protocol.

The test runs MemTest testers on the L1 caches and on a Ruby DMA port of
the stdlib CHI `PrivateL1CacheHierarchy`, with all testers accessing the
same cache lines. It checks that the home node handles `WriteUniquePtl`
requests to lines shared by the L1 caches.
"""

from testlib import (
    absdirpath,
    constants,
    joinpath,
)

from gem5.suite import gem5_verify_config

for num_testers in (2, 4):
    gem5_verify_config(
        name=f"test-chi-dma-partial-write-{num_testers}",
        fixtures=(),
        verifiers=(),
        config=joinpath(
            absdirpath(__file__),
            "configs",
            "chi-dma-partial-write.py",
        ),
        config_args=[
            f"--num-testers={num_testers}",
            "--max-loads=10000",
        ],
        valid_isas=(constants.all_compiled_tag,),
        valid_hosts=constants.supported_hosts,
        length=constants.quick_tag,
    )
