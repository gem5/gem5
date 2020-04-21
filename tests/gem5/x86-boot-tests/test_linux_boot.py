# Copyright (c) 2020 The Regents of the University of California
# All Rights Reserved.
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


if config.bin_path:
    base_path = config.bin_path
else:
    base_path = joinpath(absdirpath(__file__), '..', 'resources',
            'ubuntu-boot')

image_url = config.resource_url + '/images/x86/ubuntu-18-04/base.img'
kernel_url = config.resource_url + '/kernels/x86/static/vmlinux-4.19.83'

image_name = 'ubuntu-18-04-base.img'
kernel_name = 'vmlinux-4.19.83' # 4.19 is LTS (Projected EOL: Dec, 2020)

image = DownloadedProgram(image_url, base_path, image_name)
kernel = DownloadedProgram(kernel_url, base_path, kernel_name)


def test_boot(cpu_type, num_cpus, boot_type):
    gem5_verify_config(
        name = 'test-ubuntu_boot-' + cpu_type + '_cpu-' + num_cpus + '_cpus-'
               + boot_type + '_boot',
        verifiers = (),
        fixtures = (image, kernel,),
        config = joinpath(joinpath(absdirpath(__file__), 'run_exit.py')),
        config_args = [
            '--kernel', joinpath(base_path, kernel_name),
            '--disk', joinpath(base_path, image_name),
            '--cpu-type', cpu_type,
            '--num-cpus', num_cpus,
            '--boot-type', boot_type,
        ],
        valid_isas = ('X86',),
        valid_hosts = constants.supported_hosts,
        length = constants.long_tag,
    )

# Test every CPU type
cpu_types = ('atomic', 'simple',)
for cpu_type in cpu_types:
    test_boot(cpu_type, '1', 'init')

# Test a multicore system
test_boot('atomic', '4', 'systemd')
