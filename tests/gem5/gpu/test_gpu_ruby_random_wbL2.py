# Copyright (c) 2023 The Board of Regents of the University of Wisconsin
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
from testlib import *

"""
This file contains random tests for the Ruby GPU protocols with a WB L2 cache.
"""

# This test will first run the GPU protocol random tester -- it should take
# about 30 seconds to run and provides good coverage for the coherence
# protocol.
#
# Input choices (some are default and thus implicit):
# - use small cache size to encourage races
# - use small system size to encourage races since more requests per CU (and
#   faster sim)
# - use small address range to encourage more races
# - use small episode length to encourage more races
# - 50K tests runs in ~30 seconds with reasonably good coverage
# - num-dmas = 0 because VIPER doesn't support partial cache line writes, which
#   DMAs need
gem5_verify_config(
    name="ruby-gpu-random-test-wbL2-perCheckin",
    fixtures=(),
    verifiers=(),
    config=joinpath(
        config.base_dir, "configs", "example", "ruby_gpu_random_test.py"
    ),
    config_args=["--WB_L2", "--test-length", "50000", "--num-dmas", "0"],
    valid_isas=(constants.vega_x86_tag,),
    valid_hosts=constants.supported_hosts,
    length=constants.long_tag,
)


# This test will run the GPU protocol random tester in nightly -- it should
# take about 30 minutes to run and provides good coverage for the coherence
# protocol.
#
# Input choices (some are default and thus implicit):
#  - use small cache size to encourage races
#  - use small system size to encourage races since more requests per CU (and
#    faster sim)
#  - use small address range to encourage more races
#  - use small episode length to encourage more races
#  - 5M tests runs in ~30 minutes with reasonably good coverage
#  - num-dmas = 0 because VIPER doesn't support partial cache line writes,
#    which DMAs need
gem5_verify_config(
    name="ruby-gpu-random-test-wbL2-nightly",
    fixtures=(),
    verifiers=(),
    config=joinpath(
        config.base_dir, "configs", "example", "ruby_gpu_random_test.py"
    ),
    config_args=["--WB_L2", "--test-length", "5000000", "--num-dmas", "0"],
    valid_isas=(constants.vega_x86_tag,),
    valid_hosts=constants.supported_hosts,
    length=constants.long_tag,
)
