# Copyright 2022 Google, Inc.
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

from m5.objects.E820 import X86E820Entry
from m5.objects.QemuFwCfg import (
    QemuFwCfgIo,
    QemuFwCfgItem,
)
from m5.params import *


def x86IOAddress(port):
    IO_address_space_base = 0x8000000000000000
    return IO_address_space_base + port


class X86QemuFwCfg(QemuFwCfgIo):
    selector_addr = x86IOAddress(0x510)


class QemuFwCfgItemE820(QemuFwCfgItem):
    type = "QemuFwCfgItemE820"
    cxx_class = "gem5::qemu::FwCfgItemFactory<gem5::qemu::FwCfgItemE820>"
    cxx_template_params = ["class ItemType"]
    cxx_header = "dev/x86/qemu_fw_cfg.hh"

    # There is a fixed index for this file.
    index = 0x8003
    arch_specific = True
    path = "etc/e820"

    entries = VectorParam.X86E820Entry("entries for the e820 table")
