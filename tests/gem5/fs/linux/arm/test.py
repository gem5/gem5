# Copyright (c) 2019-2020 ARM Limited
# All rights reserved
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

'''
Arm FS simulation tests
'''

from os.path import join as joinpath

from testlib import *

arm_fs_quick_tests = [
    'realview64-simple-atomic',
    'realview64-simple-atomic-dual',
    'realview64-simple-atomic-checkpoint',
    'realview64-simple-timing',
    'realview64-simple-timing-dual',
    'realview64-switcheroo-atomic',
    'realview64-switcheroo-timing',
]

arm_fs_long_tests = [
    'realview-simple-atomic',
    'realview-simple-atomic-checkpoint',
    'realview-simple-timing',
    'realview-switcheroo-atomic',
    'realview-switcheroo-timing',
    'realview-o3',
    'realview-minor',
    'realview-switcheroo-noncaching-timing',
    'realview-switcheroo-o3',
    'realview-switcheroo-full',
    'realview64-o3',
    'realview64-o3-dual',
    'realview64-minor',
    'realview64-minor-dual',
    'realview64-switcheroo-o3',
    'realview64-switcheroo-full',
    'realview-simple-timing-ruby',
    'realview64-simple-timing-ruby',
    'realview64-simple-timing-dual-ruby',


    # The following tests fail. These are recorded in the GEM5-640 and GEM5-364
    # Jira issues.
    #
    # https://gem5.atlassian.net/browse/GEM5-640
    #'realview-simple-atomic-dual',
    #'realview-simple-timing-dual',
    #'realview-o3-dual',
    #'realview-minor-dual',
    #'realview-simple-timing-dual-ruby',
    #
    # https://gem5.atlassian.net/browse/GEM5-364
    #'realview-o3-checker',
    #'realview64-o3-checker',
]

tarball = 'aarch-system-201901106.tar.bz2'
url = config.resource_url + "/arm/" + tarball
filepath = os.path.dirname(os.path.abspath(__file__))
path = joinpath(config.bin_path, 'arm')
arm_fs_binaries = DownloadedArchive(url, path, tarball)

for name in arm_fs_quick_tests:
    args = [
        joinpath(config.base_dir, 'tests', 'gem5', 'configs', name + '.py'),
        path,
        config.base_dir
    ]
    gem5_verify_config(
        name=name,
        verifiers=(), # Add basic stat verifiers
        config=joinpath(filepath, 'run.py'),
        config_args=args,
        valid_isas=(constants.arm_tag,),
        length=constants.quick_tag,
        fixtures=(arm_fs_binaries,)
    )

for name in arm_fs_long_tests:
    args = [
        joinpath(config.base_dir, 'tests', 'gem5', 'configs', name + '.py'),
        path,
        config.base_dir
    ]
    gem5_verify_config(
        name=name,
        verifiers=(), # TODO: Add basic stat verifiers
        config=joinpath(filepath, 'run.py'),
        config_args=args,
        valid_isas=(constants.arm_tag,),
        length=constants.long_tag,
        fixtures=(arm_fs_binaries,)
    )
