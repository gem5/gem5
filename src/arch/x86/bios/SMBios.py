# Copyright (c) 2008 The Hewlett-Packard Development Company
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
from m5.params import *
from m5.SimObject import SimObject


class X86SMBiosSMBiosStructure(SimObject):
    type = "X86SMBiosSMBiosStructure"
    cxx_class = "gem5::X86ISA::smbios::SMBiosStructure"
    cxx_header = "arch/x86/bios/smbios.hh"
    abstract = True


class Characteristic(Enum):
    map = {
        "Unknown": 2,
        "Unsupported": 3,
        "ISA": 4,
        "MCA": 5,
        "EISA": 6,
        "PCI": 7,
        "PCMCIA": 8,
        "PnP": 9,
        "APM": 10,
        "Flash": 11,
        "Shadow": 12,
        "VL_Vesa": 13,
        "ESCD": 14,
        "CDBoot": 15,
        "SelectBoot": 16,
        "Socketed": 17,
        "PCMCIABoot": 18,
        "EDD": 19,
        "NEC9800": 20,
        "Toshiba": 21,
        "Floppy_5_25_360KB": 22,
        "Floppy_5_25_1_2MB": 23,
        "Floppy_3_5_720KB": 24,
        "Floppy_3_5_2_88MB": 25,
        "PrintScreen": 26,
        "Keyboard8024": 27,
        "Serial": 28,
        "Printer": 29,
        "CGA_Mono": 30,
        "NEC_PC_98": 31,
    }


class ExtCharacteristic(Enum):
    map = {
        "ACPI": 0,
        "USBLegacy": 1,
        "AGP": 2,
        "I20Boot": 3,
        "LS_120Boot": 4,
        "ZIPBoot": 5,
        "FirewireBoot": 6,
        "SmartBattery": 7,
        "BootSpec": 8,
        "NetServiceBoot": 9,
        "TargetContent": 10,
    }


class X86SMBiosBiosInformation(X86SMBiosSMBiosStructure):
    type = "X86SMBiosBiosInformation"
    cxx_class = "gem5::X86ISA::smbios::BiosInformation"
    cxx_header = "arch/x86/bios/smbios.hh"

    vendor = Param.String("", "vendor name string")
    version = Param.String("", "version string")
    starting_addr_segment = Param.UInt16(
        0, "segment location of bios starting address"
    )
    release_date = Param.String("06/08/2008", "release date")
    rom_size = Param.UInt8(0, "rom size")
    characteristics = VectorParam.Characteristic(
        [], "bios characteristic bit vector"
    )
    characteristic_ext_bytes = VectorParam.ExtCharacteristic(
        [], "extended bios characteristic bit vector"
    )
    major = Param.UInt8(0, "major version number")
    minor = Param.UInt8(0, "minor version number")
    emb_cont_firmware_major = Param.UInt8(
        0, "embedded controller firmware major version number"
    )

    emb_cont_firmware_minor = Param.UInt8(
        0, "embedded controller firmware minor version number"
    )


class X86SMBiosSMBiosTable(SimObject):
    type = "X86SMBiosSMBiosTable"
    cxx_class = "gem5::X86ISA::smbios::SMBiosTable"
    cxx_header = "arch/x86/bios/smbios.hh"

    major_version = Param.UInt8(2, "major version number")
    minor_version = Param.UInt8(5, "minor version number")

    structures = VectorParam.X86SMBiosSMBiosStructure([], "smbios structures")
