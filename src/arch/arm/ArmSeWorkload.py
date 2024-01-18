# Copyright 2020 Google Inc.
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

from m5.objects.Workload import SEWorkload
from m5.params import *


class ArmSEWorkload(SEWorkload):
    type = "ArmSEWorkload"
    cxx_header = "arch/arm/se_workload.hh"
    cxx_class = "gem5::ArmISA::SEWorkload"
    abstract = True


class ArmEmuLinux(ArmSEWorkload):
    type = "ArmEmuLinux"
    cxx_header = "arch/arm/linux/se_workload.hh"
    cxx_class = "gem5::ArmISA::EmuLinux"

    @classmethod
    def _is_compatible_with(cls, obj):
        return obj.get_arch() in (
            "arm64",
            "arm",
            "thumb",
        ) and obj.get_op_sys() in ("linux", "unknown")


class ArmEmuFreebsd(ArmSEWorkload):
    type = "ArmEmuFreebsd"
    cxx_header = "arch/arm/freebsd/se_workload.hh"
    cxx_class = "gem5::ArmISA::EmuFreebsd"

    @classmethod
    def _is_compatible_with(cls, obj):
        return (
            obj.get_arch() in ("arm64", "arm", "thumb")
            and obj.get_op_sys() == "freebsd"
        )
