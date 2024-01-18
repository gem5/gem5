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

from m5.objects.Device import PioDevice
from m5.objects.SimObject import SimObject
from m5.params import *


class QemuFwCfgItem(SimObject):
    type = "QemuFwCfgItem"
    cxx_class = "gem5::qemu::FwCfgItemFactoryBase"
    cxx_header = "dev/qemu/fw_cfg.hh"
    abstract = True

    # The path this item will be listed under in the firmware config directory.
    arch_specific = Param.Bool(False, "if this item is archiecture specific")
    index = Param.Unsigned(0, "Fixed index, or 0 for automatic")
    path = Param.String("Path to item in the firmware config directory")


class QemuFwCfgItemFile(QemuFwCfgItem):
    type = "QemuFwCfgItemFile"
    cxx_class = "gem5::qemu::FwCfgItemFactory<gem5::qemu::FwCfgItemFile>"
    cxx_template_params = ["class ItemType"]
    cxx_header = "dev/qemu/fw_cfg.hh"

    # The path to the file that will be used to populate this item.
    file = Param.String("Path to file to export")


class QemuFwCfgItemString(QemuFwCfgItem):
    type = "QemuFwCfgItemString"
    cxx_class = "gem5::qemu::FwCfgItemFactory<gem5::qemu::FwCfgItemString>"
    cxx_template_params = ["class ItemType"]
    cxx_header = "dev/qemu/fw_cfg.hh"

    # The string which directly populates this item.
    string = Param.String("String to export")


class QemuFwCfgItemBytes(QemuFwCfgItem):
    type = "QemuFwCfgItemBytes"
    cxx_class = "gem5::qemu::FwCfgItemFactory<gem5::qemu::FwCfgItemBytes>"
    cxx_template_params = ["class ItemType"]
    cxx_header = "dev/qemu/fw_cfg.hh"

    data = VectorParam.UInt8("Bytes to export")


class QemuFwCfg(PioDevice):
    type = "QemuFwCfg"
    cxx_class = "gem5::qemu::FwCfg"
    cxx_header = "dev/qemu/fw_cfg.hh"
    abstract = True

    items = VectorParam.QemuFwCfgItem(
        [], "Items exported by the firmware config device"
    )


class QemuFwCfgIo(QemuFwCfg):
    type = "QemuFwCfgIo"
    cxx_class = "gem5::qemu::FwCfgIo"
    cxx_header = "dev/qemu/fw_cfg.hh"

    # The selector register is 16 bits wide, and little endian. The data
    # register must be one port ahead of the selector.
    selector_addr = Param.Addr("IO port for the selector register")


class QemuFwCfgMmio(QemuFwCfg):
    type = "QemuFwCfgMmio"
    cxx_class = "gem5::qemu::FwCfgMmio"
    cxx_header = "dev/qemu/fw_cfg.hh"

    # The selector register is 16 bits wide, and big endian.
    selector_addr = Param.Addr("Memory address for the selector register")

    # The data register is 8, 16, 32 or 64 bits wide.
    data_addr_range = Param.AddrRange(
        "Memory address range for the data register"
    )
