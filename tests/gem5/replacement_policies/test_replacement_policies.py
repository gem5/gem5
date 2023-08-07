# Copyright (c) 2022 Jarvis Jia, Jing Qu, Matt Sinclair, & Mingyuan Xiang
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

import os

from testlib import *


def test_replacement_policy(config_name: str, config_path: str) -> None:
    name = f"test-replacement-policy-{config_name}"

    verifiers = (
        verifier.MatchStdoutNoPerf(joinpath(getcwd(), "ref", config_name[7:])),
    )
    gem5_verify_config(
        name=name,
        fixtures=(),
        verifiers=verifiers,
        config=joinpath(
            config.base_dir,
            "tests",
            "gem5",
            "replacement_policies",
            "configs",
            "run_replacement_policy.py",
        ),
        config_args=[config_name, config_path],
        valid_isas=(constants.null_tag,),
        protocol="MI_example",
        valid_hosts=constants.supported_hosts,
        length=constants.long_tag,
    )


def create_replacement_policy_tests(traces):
    this_dir = os.path.dirname(__file__)
    for trace in traces:
        config_name = trace.split(".")[0]
        config_path = os.path.join(this_dir, trace)
        test_replacement_policy(config_name, config_path)


traces = [
    "traces/fifo_test1_ld.py",
    "traces/fifo_test2_ld.py",
    "traces/lru_test3_ld.py",
    "traces/lru_test4_ld.py",
    "traces/lfu_test1_ld.py",
    "traces/lfu_test2_ld.py",
    "traces/lfu_test3_ld.py",
    "traces/lip_test1_ld.py",
    "traces/lru_test1_ld.py",
    "traces/lru_test2_ld.py",
    "traces/mru_test1_ld.py",
    "traces/mru_test2_ld.py",
    "traces/nru_test1_ld.py",
    "traces/rrip_test1_ld.py",
    "traces/rrip_test2_ld.py",
    "traces/second_chance_test1_ld.py",
    "traces/second_chance_test2_ld.py",
    "traces/second_chance_test3_ld.py",
    "traces/tree_plru_test1_ld.py",
    "traces/tree_plru_test2_ld.py",
    "traces/tree_plru_test3_ld.py",
    "traces/fifo_test1_st.py",
    "traces/fifo_test2_st.py",
    "traces/lru_test3_st.py",
    "traces/lru_test4_st.py",
    "traces/lfu_test1_st.py",
    "traces/lfu_test2_st.py",
    "traces/lfu_test3_st.py",
    "traces/lip_test1_st.py",
    "traces/lru_test1_st.py",
    "traces/lru_test2_st.py",
    "traces/mru_test1_st.py",
    "traces/mru_test2_st.py",
    "traces/nru_test1_st.py",
    "traces/rrip_test1_st.py",
    "traces/rrip_test2_st.py",
    "traces/second_chance_test1_st.py",
    "traces/second_chance_test2_st.py",
    "traces/second_chance_test3_st.py",
    "traces/tree_plru_test1_st.py",
    "traces/tree_plru_test2_st.py",
    "traces/tree_plru_test3_st.py",
]
create_replacement_policy_tests(traces)
