# Copyright (c) 2021-2023 The Regents of the University of California
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

from testlib import (
    config,
    constants,
    gem5_verify_config,
    joinpath,
    log,
)
from testlib.configuration import constants

log.test_log.message(
    "PARSEC tests are disabled. This is due to our GitHub "
    "Actions self-hosted runners only having 60GB of disk space. The "
    "PARSEC Disk image is too big to use.",
    level=log.LogLevel.Warn,
)
# 'False' is used to disable the tests.
if False:  # os.access("/dev/kvm", mode=os.R_OK | os.W_OK):
    # The x86-parsec-benchmarks uses KVM cores, this test will therefore only
    # be run on systems that support KVM.
    gem5_verify_config(
        name="test-gem5-library-example-x86-parsec-benchmarks",
        fixtures=(),
        verifiers=(),
        config=joinpath(
            config.base_dir,
            "configs",
            "example",
            "gem5_library",
            "x86-parsec-benchmarks.py",
        ),
        config_args=["--benchmark", "blackscholes", "--size", "simsmall"],
        valid_isas=(constants.all_compiled_tag,),
        valid_hosts=(constants.host_x86_64_tag,),
        length=constants.long_tag,
        uses_kvm=True,
    )

if os.access("/dev/kvm", mode=os.R_OK | os.W_OK):
    # The x86-npb-benchmarks uses KVM cores, this test will therefore only be
    # run on systems that support KVM.
    gem5_verify_config(
        name="test-gem5-library-example-x86-npb-benchmarks",
        fixtures=(),
        verifiers=(),
        config=joinpath(
            config.base_dir,
            "configs",
            "example",
            "gem5_library",
            "x86-npb-benchmarks.py",
        ),
        config_args=[
            "--benchmark",
            "npb-bt-a",
            "--ticks",
            "5000000000",
        ],
        valid_isas=(constants.all_compiled_tag,),
        valid_hosts=(constants.host_x86_64_tag,),
        length=constants.long_tag,
        uses_kvm=True,
    )

if os.access("/dev/kvm", mode=os.R_OK | os.W_OK):
    # The x86-gapbs-benchmarks uses KVM cores, this test will therefore only
    # be run on systems that support KVM.
    gem5_verify_config(
        name="test-gem5-library-example-x86-gapbs-benchmarks",
        fixtures=(),
        verifiers=(),
        config=joinpath(
            config.base_dir,
            "configs",
            "example",
            "gem5_library",
            "x86-gapbs-benchmarks.py",
        ),
        config_args=["--benchmark", "gapbs-bfs-test"],
        valid_isas=(constants.all_compiled_tag,),
        valid_hosts=(constants.host_x86_64_tag,),
        length=constants.long_tag,
        uses_kvm=True,
    )
