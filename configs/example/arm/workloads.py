# Copyright (c) 2020 ARM Limited
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
#
import inspect

import m5
from common.ObjectList import ObjectList
from common.SysPaths import binary
from common.SysPaths import disk
from m5.objects import *
from m5.options import *


class ArmBaremetal(ArmFsWorkload):
    """Baremetal workload"""

    dtb_addr = 0

    def __init__(self, obj, system, **kwargs):
        super(ArmBaremetal, self).__init__(**kwargs)

        self.object_file = obj


class ArmTrustedFirmware(ArmFsWorkload):
    """
    Arm Trusted Firmware (TFA) workload.

    It models the firmware design described at:

    https://trustedfirmware-a.readthedocs.io/en/latest/design/firmware-design.html

    The Workload is expecting to find a set of firmare images under
    the M5_PATH/binaries path. Those images are:
        * bl1.bin (BL1 = Stage 1 Bootloader)
        * fip.bin (FIP = Firmware Image Package):
            BL2, BL31, BL33 binaries compiled under a singe package

    These are the results of the compilation of Arm Trusted Firmware.
    https://github.com/ARM-software/arm-trusted-firmware

    """

    dtb_addr = 0

    def __init__(self, obj, system, **kwargs):
        super(ArmTrustedFirmware, self).__init__(**kwargs)

        self.extras = [binary("bl1.bin"), binary("fip.bin")]
        self.extras_addrs = [
            system.realview.bootmem.range.start,
            system.realview.flash0.range.start,
        ]

        # Arm Trusted Firmware will provide a PSCI implementation
        system._have_psci = True


class _WorkloadList(ObjectList):
    def _add_objects(self):
        """Add all sub-classes of the base class in the object hierarchy."""
        modname = sys.modules[__name__]
        for name, cls in inspect.getmembers(modname, self._is_obj_class):
            self._sub_classes[name] = cls


workload_list = _WorkloadList(getattr(m5.objects, "ArmFsWorkload", None))
