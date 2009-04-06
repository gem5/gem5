# Copyright (c) 2008 The Hewlett-Packard Development Company
# All rights reserved.
#
# Redistribution and use of this software in source and binary forms,
# with or without modification, are permitted provided that the
# following conditions are met:
#
# The software must be used only for Non-Commercial Use which means any
# use which is NOT directed to receiving any direct monetary
# compensation for, or commercial advantage from such use.  Illustrative
# examples of non-commercial use are academic research, personal study,
# teaching, education and corporate research & development.
# Illustrative examples of commercial use are distributing products for
# commercial advantage and providing services using the software for
# commercial advantage.
#
# If you wish to use this software or functionality therein that may be
# covered by patents for commercial use, please contact:
#     Director of Intellectual Property Licensing
#     Office of Strategy and Technology
#     Hewlett-Packard Company
#     1501 Page Mill Road
#     Palo Alto, California  94304
#
# Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.  Redistributions
# in binary form must reproduce the above copyright notice, this list of
# conditions and the following disclaimer in the documentation and/or
# other materials provided with the distribution.  Neither the name of
# the COPYRIGHT HOLDER(s), HEWLETT-PACKARD COMPANY, nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.  No right of
# sublicense is granted herewith.  Derivatives of the software and
# output created using the software may be prepared, but only for
# Non-Commercial Uses.  Derivatives of the software may be shared with
# others provided: (i) the others agree to abide by the list of
# conditions herein which includes the Non-Commercial Use restrictions;
# and (ii) such Derivatives of the software include the above copyright
# notice to acknowledge the contribution from this software where
# applicable, this list of conditions and the disclaimer below.
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
# Authors: Gabe Black

from m5.params import *
from m5.SimObject import SimObject

class X86SMBiosSMBiosStructure(SimObject):
    type = 'X86SMBiosSMBiosStructure'
    cxx_class = 'X86ISA::SMBios::SMBiosStructure'
    abstract = True

class Characteristic(Enum):
    map = {'Unknown' : 2,
           'Unsupported' : 3,
           'ISA' : 4,
           'MCA' : 5,
           'EISA' : 6,
           'PCI' : 7,
           'PCMCIA' : 8,
           'PnP' : 9,
           'APM' : 10,
           'Flash' : 11,
           'Shadow' : 12,
           'VL_Vesa' : 13,
           'ESCD' : 14,
           'CDBoot' : 15,
           'SelectBoot' : 16,
           'Socketed' : 17,
           'PCMCIABoot' : 18,
           'EDD' : 19,
           'NEC9800' : 20,
           'Toshiba' : 21,
           'Floppy_5_25_360KB' : 22,
           'Floppy_5_25_1_2MB' : 23,
           'Floppy_3_5_720KB' : 24,
           'Floppy_3_5_2_88MB' : 25,
           'PrintScreen' : 26,
           'Keyboard8024' : 27,
           'Serial' : 28,
           'Printer' : 29,
           'CGA_Mono' : 30,
           'NEC_PC_98' : 31
    }

class ExtCharacteristic(Enum):
    map = {'ACPI' : 0,
           'USBLegacy' : 1,
           'AGP' : 2,
           'I20Boot' : 3,
           'LS_120Boot' : 4,
           'ZIPBoot' : 5,
           'FirewireBoot' : 6,
           'SmartBattery' : 7,
           'BootSpec' : 8,
           'NetServiceBoot' : 9,
           'TargetContent' : 10
    }

class X86SMBiosBiosInformation(X86SMBiosSMBiosStructure):
    type = 'X86SMBiosBiosInformation'
    cxx_class = 'X86ISA::SMBios::BiosInformation'

    vendor = Param.String("", "vendor name string")
    version = Param.String("", "version string")
    starting_addr_segment = \
        Param.UInt16(0, "segment location of bios starting address")
    release_date = Param.String("06/08/2008", "release date")
    rom_size = Param.UInt8(0, "rom size")
    characteristics = VectorParam.Characteristic([],
            "bios characteristic bit vector")
    characteristic_ext_bytes = VectorParam.ExtCharacteristic([],
            "extended bios characteristic bit vector")
    major = Param.UInt8(0, "major version number")
    minor = Param.UInt8(0, "minor version number")
    emb_cont_firmware_major = Param.UInt8(0,
            "embedded controller firmware major version number")

    emb_cont_firmware_minor = Param.UInt8(0,
            "embedded controller firmware minor version number")

class X86SMBiosSMBiosTable(SimObject):
    type = 'X86SMBiosSMBiosTable'
    cxx_class = 'X86ISA::SMBios::SMBiosTable'

    major_version = Param.UInt8(2, "major version number")
    minor_version = Param.UInt8(5, "minor version number")

    structures = VectorParam.X86SMBiosSMBiosStructure([], "smbios structures")
