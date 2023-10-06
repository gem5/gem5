# Copyright (c) 2021 Huawei International
# Copyright (c) 2021 Arm Limited
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
"""
Test file for the hdf5 stats.
It just runs an SE simulation with the hdf5 stats and checks that the
simulation succeeds and the stats file exists.
No specific checks on the stats are performed.

**Important Note**: This test has a major design flaw, noted here:
https://gem5.atlassian.net/browse/GEM5-1073.
It will not run if the build/ARM/gem5.opt has not been built. As this is not
built prior to this test being processed during the Weekly run, this test is
not run.
"""
import os
import re

from testlib import *

if config.bin_path:
    resource_path = config.bin_path
else:
    resource_path = joinpath(absdirpath(__file__), "..", "resources")


def have_hdf5():
    have_hdf5_file = os.path.join(
        config.base_dir,
        "build",
        constants.arm_tag,
        "config",
        "have_hdf5.hh",
    )
    if not os.path.exists(have_hdf5_file):
        # This will most likely happen if the file has yet to have been
        # compiled. It should be noted that this case is likely. This is not
        # a good test as checking if hdf5 is available requires compilation
        # which is not assumed to be true at this stage in the test.
        return False
    with open(have_hdf5_file) as f:
        content = f.read()

    result = re.match("#define HAVE_HDF5 ([0-1])", content)
    if not result:
        raise Exception(f"Unable to find the HAVE_HDF5 in {have_hdf5_file}")
    else:
        return result.group(1) == "1"


if have_hdf5():
    ok_exit_regex = re.compile(
        r"Exiting @ tick \d+ because exiting with last active thread context",
    )
    ok_verifier = verifier.MatchRegex(ok_exit_regex)

    # FIXME: flaky, should check return code instead...
    # See: https://gem5.atlassian.net/browse/GEM5-1099
    err_regex = re.compile(
        r"RuntimeError: Failed creating H5::DataSet \w+; .*",
    )
    err_verifier = verifier.NoMatchRegex(err_regex, True, False)

    h5_verifier = verifier.CheckH5StatsExist()

    gem5_verify_config(
        name="hdf5_test",
        verifiers=[ok_verifier, err_verifier, h5_verifier],
        fixtures=(),
        config=joinpath(
            config.base_dir,
            "tests",
            "gem5",
            "stats",
            "configs",
            "simple_binary_run.py",
        ),
        config_args=[
            "arm-hello64-static",
            "--resource-directory",
            resource_path,
        ],
        gem5_args=["--stats-file=h5://stats.h5"],
        valid_isas=(constants.all_compiled_tag,),
    )
