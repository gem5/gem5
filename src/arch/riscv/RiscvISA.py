# Copyright (c) 2012 ARM Limited
# Copyright (c) 2014 Sven Karlsson
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
# Copyright (c) 2016 RISC-V Foundation
# Copyright (c) 2016 The University of Virginia
# Copyright (c) 2023 The Regents of the University of California
# Copyright (c) 2024 University of Rostock
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

from m5.objects.BaseISA import BaseISA
from m5.params import (
    Enum,
    Param,
    String,
    UInt32,
    VectorParam,
)
from m5.util import warn


class RiscvVectorLength(UInt32):
    min = 8
    max = 65536

    def _check(self):
        super()._check()

        # VLEN needs to be a whole power of 2. We already know value is
        # not zero. Hence:
        if self.value & (self.value - 1) != 0:
            raise TypeError("VLEN is not a power of 2: %d" % self.value)


class RiscvVectorElementLength(UInt32):
    min = 8
    max = 64

    def _check(self):
        super()._check()

        # ELEN needs to be a whole power of 2. We already know value is
        # not zero. Hence:
        if self.value & (self.value - 1) != 0:
            raise TypeError("ELEN is not a power of 2: %d" % self.value)


class RiscvInternalString(String):
    cmd_line_settable = False


class RiscvType(Enum):
    vals = ["RV32", "RV64"]


class PrivilegeModeSet(Enum):
    vals = [
        "M",  # Machine privilege mode only
        "MU",  # Machine and user privlege modes
        "MSU",  # Machine, supervisor and user modes
        "MHSU",  # Machine, hypervisor, supervisor and user modes
    ]


class RiscvProfile(Enum):
    vals = [
        "RVI20U32",
        "RVI20U64",
        "RVA20U64",
        "RVA20S64",
        "RVA22U64",
        "RVA22S64",
        "RVA23U64",
        "RVA23S64",
        "RVB23U64",
        "RVB23S64",
    ]


# Ratified standard extensions from RISC-V Unified DB
# spec/std/isa/ext. This registry is a recognition catalog plus gem5
# support metadata; implemented=True is the support/reporting claim.
_EXTENSION_REGISTRY = {
    "A": {"implemented": True},
    "B": {"implemented": True},
    "C": {"implemented": True},
    "D": {"implemented": True},
    "F": {"implemented": True},
    "H": {"implemented": True},
    "I": {
        "implemented": True,
        "notes": "Base integer ISA is emitted from the selected profile.",
    },
    "M": {"implemented": True},
    "Q": {"implemented": False},
    "S": {"implemented": False},
    "Sdext": {"implemented": False},
    "Sdtrig": {"implemented": False},
    "Sha": {"implemented": False},
    "Shcounterenw": {"implemented": True},
    "Shgatpa": {"implemented": False},
    "Shtvala": {"implemented": False},
    "Shvsatpa": {"implemented": False},
    "Shvstvala": {"implemented": False},
    "Shvstvecd": {"implemented": True},
    "Sm": {"implemented": False},
    "Smaia": {"implemented": False},
    "Smcdeleg": {"implemented": False},
    "Smcntrpmf": {"implemented": False},
    "Smcsrind": {"implemented": False},
    "Smctr": {"implemented": False},
    "Smdbltrp": {"implemented": False},
    "Smepmp": {"implemented": False},
    "Smmpm": {"implemented": False},
    "Smnpm": {"implemented": False},
    "Smrnmi": {"implemented": True},
    "Smstateen": {"implemented": False},
    "Ssaia": {"implemented": False},
    "Ss1p11": {"implemented": False},
    "Ss1p12": {"implemented": False},
    "Ss1p13": {"implemented": False},
    "Ssccfg": {"implemented": False},
    "Ssccptr": {"implemented": False},
    "Sscofpmf": {"implemented": False},
    "Sscounterenw": {"implemented": True},
    "Sscsrind": {"implemented": False},
    "Ssctr": {"implemented": False},
    "Ssdbltrp": {"implemented": False},
    "Ssnpm": {"implemented": False},
    "Sspm": {"implemented": False},
    "Ssqosid": {"implemented": False},
    "Ssstateen": {"implemented": False},
    "Ssstrict": {"implemented": False},
    "Sstc": {"implemented": False},
    "Sstvala": {"implemented": True},
    "Sstvecd": {"implemented": True},
    "Sstvecv": {"implemented": False},
    "Ssu32xl": {"implemented": False},
    "Ssu64xl": {"implemented": True},
    "Ssube": {"implemented": False},
    "Supm": {"implemented": False},
    "Sv32": {"implemented": False},
    "Sv39": {"implemented": True},
    "Sv48": {"implemented": False},
    "Sv57": {"implemented": False},
    "Svade": {"implemented": False},
    "Svadu": {"implemented": False},
    "Svbare": {"implemented": True},
    "Svinval": {"implemented": False},
    "Svnapot": {
        "implemented": True,
        "isa": "svnapot",
    },
    "Svpbmt": {"implemented": False},
    "Svrsw60t59b": {"implemented": False},
    "Svvptc": {"implemented": False},
    "U": {"implemented": False},
    "V": {"implemented": True},
    "Za128rs": {"implemented": False},
    "Za64rs": {"implemented": False},
    "Zaamo": {"implemented": False},
    "Zabha": {"implemented": False},
    "Zacas": {"implemented": False},
    "Zalasr": {"implemented": False},
    "Zalrsc": {"implemented": False},
    "Zama16b": {"implemented": False},
    "Zawrs": {"implemented": False},
    "Zba": {"implemented": True},
    "Zbb": {"implemented": True},
    "Zbc": {"implemented": True},
    "Zbkb": {"implemented": True},
    "Zbkc": {"implemented": True},
    "Zbkx": {"implemented": True},
    "Zbs": {"implemented": True},
    "Zca": {"implemented": True},
    "Zcb": {"implemented": True},
    "Zcd": {"implemented": True},
    "Zce": {"implemented": False},
    "Zcf": {"implemented": True},
    "Zclsd": {"implemented": False},
    "Zcmop": {"implemented": False},
    "Zcmp": {"implemented": True},
    "Zcmt": {"implemented": True},
    "Zdinx": {"implemented": False},
    "Zfa": {"implemented": True},
    "Zfbfmin": {"implemented": True},
    "Zfh": {"implemented": True},
    "Zfhmin": {"implemented": True},
    "Zfinx": {"implemented": False},
    "Zhinx": {"implemented": False},
    "Zhinxmin": {"implemented": False},
    "Zic64b": {"implemented": True},
    "Zicbom": {"implemented": True},
    "Zicbop": {"implemented": True},
    "Zicboz": {"implemented": True},
    "Ziccamoa": {"implemented": False},
    "Ziccamoc": {"implemented": False},
    "Ziccif": {"implemented": False},
    "Zicclsm": {"implemented": False},
    "Ziccrse": {"implemented": False},
    "Zicfilp": {"implemented": False},
    "Zicfiss": {"implemented": False},
    "Zicntr": {"implemented": True},
    "Zicond": {"implemented": True},
    "Zicsr": {"implemented": True},
    "Zifencei": {"implemented": True},
    "Zihintntl": {"implemented": False},
    "Zihintpause": {"implemented": True},
    "Zihpm": {"implemented": True},
    "Zilsd": {"implemented": False},
    "Zimop": {"implemented": False},
    "Zk": {"implemented": False},
    "Zkn": {"implemented": False},
    "Zknd": {"implemented": True},
    "Zkne": {"implemented": True},
    "Zknh": {"implemented": True},
    "Zkr": {"implemented": False},
    "Zks": {"implemented": False},
    "Zksed": {"implemented": True},
    "Zksh": {"implemented": True},
    "Zkt": {"implemented": False},
    "Zmmul": {"implemented": False},
    "Ztso": {"implemented": False},
    "Zvbb": {"implemented": False},
    "Zvbc": {"implemented": True},
    "Zve32f": {"implemented": True},
    "Zve32x": {"implemented": True},
    "Zve64d": {"implemented": True},
    "Zve64f": {"implemented": True},
    "Zve64x": {"implemented": True},
    "Zvfbfmin": {"implemented": True},
    "Zvfbfwma": {"implemented": True},
    "Zvfh": {"implemented": True},
    "Zvfhmin": {"implemented": True},
    "Zvkb": {"implemented": False},
    "Zvkg": {"implemented": False},
    "Zvkn": {"implemented": False},
    "Zvknc": {"implemented": False},
    "Zvkned": {"implemented": False},
    "Zvkng": {"implemented": False},
    "Zvknha": {"implemented": False},
    "Zvknhb": {"implemented": False},
    "Zvks": {"implemented": False},
    "Zvksc": {"implemented": False},
    "Zvksed": {"implemented": False},
    "Zvksg": {"implemented": False},
    "Zvksh": {"implemented": False},
    "Zvkt": {"implemented": False},
    "Zvl1024b": {"implemented": False},
    "Zvl128b": {"implemented": False},
    "Zvl256b": {"implemented": False},
    "Zvl32b": {"implemented": False},
    "Zvl512b": {"implemented": False},
    "Zvl64b": {"implemented": False},
}

_SINGLE_LETTER_EXTENSION_ORDER = "MAFDQLCBKJTPVH"
_MULTI_LETTER_EXTENSION_PREFIX_ORDER = {
    "Z": 0,
    "S": 1,
    "H": 2,
    "X": 3,
}


def _ordered_extension_names(extensions):
    def sort_key(extension):
        if len(extension) == 1:
            order = _SINGLE_LETTER_EXTENSION_ORDER.find(extension)
            if order != -1:
                return (0, order, extension)
            return (0, len(_SINGLE_LETTER_EXTENSION_ORDER), extension)

        return (
            1,
            _MULTI_LETTER_EXTENSION_PREFIX_ORDER.get(extension[0], 4),
            extension.lower(),
        )

    return tuple(sorted(extensions, key=sort_key))


def _profile(parent=(), add=(), remove=()):
    extensions = list(parent)
    for extension in remove:
        if extension in extensions:
            extensions.remove(extension)
    for extension in add:
        if extension not in extensions:
            extensions.append(extension)
    return tuple(extensions)


_RVI20 = ()
_RVA20U64 = _profile(
    _RVI20,
    (
        "M",
        "A",
        "F",
        "D",
        "C",
        "Zicsr",
        "Zicntr",
        "Ziccif",
        "Ziccrse",
        "Ziccamoa",
        "Za128rs",
        "Zicclsm",
    ),
)
_RVA20S64 = _profile(
    _RVA20U64,
    (
        "Zifencei",
        "Ss1p11",
        "Svbare",
        "Sv39",
        "Svade",
        "Ssccptr",
        "Sstvecd",
        "Sstvala",
    ),
)
_RVA22U64 = _profile(
    _RVA20U64,
    (
        "Zihpm",
        "Za64rs",
        "Zihintpause",
        "Zba",
        "Zbb",
        "Zbs",
        "Zic64b",
        "Zicbom",
        "Zicbop",
        "Zicboz",
        "Zfhmin",
        "Zkt",
    ),
    remove=("Za128rs",),
)
_RVA22S64 = _profile(
    _RVA22U64,
    (
        "Zifencei",
        "Ss1p12",
        "Svbare",
        "Sv39",
        "Svade",
        "Ssccptr",
        "Sstvecd",
        "Sstvala",
        "Sscounterenw",
        "Svpbmt",
        "Svinval",
    ),
)
_RVA23U64 = _profile(
    _RVA22U64,
    (
        "V",
        "Zvfhmin",
        "Zvbb",
        "Zvkt",
        "Zihintntl",
        "Zicond",
        "Zimop",
        "Zcmop",
        "Zcb",
        "Zfa",
        "Zawrs",
        "Supm",
    ),
)
_RVA23S64 = _profile(
    _RVA23U64,
    (
        "Zifencei",
        "Ss1p13",
        "Svbare",
        "Sv39",
        "Svade",
        "Ssccptr",
        "Sstvecd",
        "Sstvala",
        "Sscounterenw",
        "Svpbmt",
        "Svinval",
        "Svnapot",
        "Sstc",
        "Sscofpmf",
        "Ssnpm",
        "Ssu64xl",
        "Sha",
        "H",
        "Ssstateen",
        "Shcounterenw",
        "Shvstvala",
        "Shtvala",
        "Shvstvecd",
        "Shvsatpa",
        "Shgatpa",
    ),
)
_RVB23U64 = _profile(
    _RVA22U64,
    (
        "Zihintntl",
        "Zicond",
        "Zimop",
        "Zcmop",
        "Zcb",
        "Zfa",
        "Zawrs",
    ),
    remove=("Zfhmin",),
)
_RVB23S64 = _profile(
    _RVB23U64,
    (
        "Zifencei",
        "Ss1p13",
        "Svnapot",
        "Svbare",
        "Sv39",
        "Svade",
        "Ssccptr",
        "Sstvecd",
        "Sstvala",
        "Sscounterenw",
        "Svpbmt",
        "Svinval",
        "Sstc",
        "Sscofpmf",
        "Ssu64xl",
    ),
)

_PROFILE_EXTENSIONS = {
    "RVI20U32": _RVI20,
    "RVI20U64": _RVI20,
    "RVA20U64": _RVA20U64,
    "RVA20S64": _RVA20S64,
    "RVA22U64": _RVA22U64,
    "RVA22S64": _RVA22S64,
    "RVA23U64": _RVA23U64,
    "RVA23S64": _RVA23S64,
    "RVB23U64": _RVB23U64,
    "RVB23S64": _RVB23S64,
}


def _validate_extension_tables():
    known_extensions = set(_EXTENSION_REGISTRY)
    invalid_entries = sorted(
        extension
        for extension, attributes in _EXTENSION_REGISTRY.items()
        if not isinstance(attributes.get("implemented"), bool)
    )
    if invalid_entries:
        raise ValueError(
            "RISC-V extension registry entries missing implemented "
            "metadata: %s" % ", ".join(invalid_entries)
        )

    profile_extensions = {
        extension
        for extensions in _PROFILE_EXTENSIONS.values()
        for extension in extensions
    }
    unknown_profile_extensions = sorted(profile_extensions - known_extensions)
    if unknown_profile_extensions:
        raise ValueError(
            "Profile RISC-V extensions missing from registry: %s"
            % ", ".join(unknown_profile_extensions)
        )


_validate_extension_tables()


_WARNED_PROFILE_CONFIGS = set()
_WARNED_UNIMPLEMENTED_EXTRAS = set()


class RiscvISA(BaseISA):
    type = "RiscvISA"
    cxx_class = "gem5::RiscvISA::ISA"
    cxx_header = "arch/riscv/isa.hh"

    riscv_profile = Param.RiscvProfile(
        "RVA23S64",
        "RISC-V application profile used for extension reporting. Profile "
        "selection does not gate instruction decoding or execution; it only "
        "selects which supported mandatory profile extensions gem5 reports "
        "to software.",
    )
    extra_extensions = VectorParam.String(
        [
            "Zbc",
            "Zbkb",
            "Zbkc",
            "Zbkx",
            # "Zcf",        # RV32-only and implied by C+F
            # "Zcmp",       # C+D implies Zcd which is used by default
            # "Zcmt",       # C+D implies Zcd which is used by default
            "Zfbfmin",
            "Zfh",
            "Zknd",
            "Zkne",
            "Zknh",
            "Zksed",
            "Zksh",
            "Zvbc",
            "Zvfbfmin",
            "Zvfbfwma",
            "Zvfh",
            # "Smrnmi",     # Not enabled by default, changes trapping behavior
        ],
        "Implemented extensions to expose in addition to the selected "
        "profile.",
    )
    reported_extensions = VectorParam.RiscvInternalString(
        [],
        "Python-computed extension list passed to C++ for reporting.",
    )

    vlen = Param.RiscvVectorLength(
        256,
        "Length of each vector register in bits. \
        VLEN in Ch. 2 of RISC-V vector spec",
    )
    elen = Param.RiscvVectorElementLength(
        64,
        "Length of each vector element in bits. \
        ELEN in Ch. 2 of RISC-V vector spec",
    )
    privilege_mode_set = Param.PrivilegeModeSet(
        "MSU",  # set MHSU to enable hypervisor (H-extension)
        # No timing CPUs are supported in MHSU currently
        # *ONLY THE ATOMIC CPU / ATOMIC MEMORY* is supported
        # PTW does not yet implement timing walks with H extension on
        # (P.S.: look at the change for MIDELEG in isa.cc:readMiscReg
        # if working with old bbl bootloader)
        "The combination of privilege modes \
        in Privilege Levels section of RISC-V privileged spec",
    )

    wfi_resume_on_pending = Param.Bool(
        False,
        "If wfi_resume_on_pending is set to True, the hart will resume "
        "execution when interrupt becomes pending. The local enabled status "
        "is not considered.\n"
        "If wfi_resume_on_pending is set to False, the hart will only "
        "resume the execution when an locally enabled interrupt becomes "
        "pending.",
    )

    def _effective_riscv_type(self):
        profile = self.riscv_profile.value
        if profile.endswith("U32"):
            return "RV32"
        if profile.endswith("U64") or profile.endswith("S64"):
            return "RV64"
        raise ValueError(
            "Cannot determine XLEN from RISC-V profile: %s" % profile
        )

    def _configured_extensions(self):
        profile = self.riscv_profile.value
        mandatory = set(_PROFILE_EXTENSIONS[profile])
        requested = {
            extension.value if hasattr(extension, "value") else extension
            for extension in self.extra_extensions
        }
        effective = mandatory | requested
        if "Zvfbfwma" in effective:
            effective.update(("Zfbfmin", "Zvfbfmin"))
        return mandatory, requested, effective

    def _is_supported(self, extension):
        if extension == "H" and self.privilege_mode_set.value != "MHSU":
            return False
        return _EXTENSION_REGISTRY[extension]["implemented"]

    def _validate_extensions(self):
        mandatory, requested, effective = self._configured_extensions()
        for extension in mandatory | requested:
            if extension not in _EXTENSION_REGISTRY:
                raise ValueError(
                    "Unknown standard RISC-V extension: %s" % extension
                )

        bf16_extensions = effective & {
            "Zfbfmin",
            "Zvfbfmin",
            "Zvfbfwma",
        }
        if bf16_extensions and "F" not in effective:
            extensions = ", ".join(_ordered_extension_names(bf16_extensions))
            if len(bf16_extensions) == 1:
                raise ValueError(
                    "RISC-V extension %s requires the F extension" % extensions
                )
            raise ValueError(
                "RISC-V extensions %s require the F extension" % extensions
            )

        vector_bf16_extensions = effective & {
            "Zvfbfmin",
            "Zvfbfwma",
        }
        if vector_bf16_extensions and not effective.intersection(
            ("V", "Zve32f")
        ):
            extensions = ", ".join(
                _ordered_extension_names(vector_bf16_extensions)
            )
            if len(vector_bf16_extensions) == 1:
                raise ValueError(
                    "RISC-V extension %s requires V or Zve32f" % extensions
                )
            raise ValueError(
                "RISC-V extensions %s require V or Zve32f" % extensions
            )

        unimplemented_extras = sorted(
            extension
            for extension in requested - mandatory
            if not _EXTENSION_REGISTRY[extension]["implemented"]
        )
        if unimplemented_extras:
            key = tuple(unimplemented_extras)
            if key not in _WARNED_UNIMPLEMENTED_EXTRAS:
                _WARNED_UNIMPLEMENTED_EXTRAS.add(key)
                warn(
                    "Ignoring RISC-V extensions that are recognized but not "
                    "implemented/reportable: %s",
                    ", ".join(unimplemented_extras),
                )

        missing = [
            extension
            for extension in mandatory
            if not self._is_supported(extension)
        ]
        if missing:
            key = (
                self.riscv_profile.value,
                tuple(_ordered_extension_names(missing)),
            )
            if key not in _WARNED_PROFILE_CONFIGS:
                _WARNED_PROFILE_CONFIGS.add(key)
                warn(
                    "RISC-V profile %s has mandatory extensions not "
                    "implemented/reportable by this gem5 configuration: %s",
                    self.riscv_profile.value,
                    ", ".join(_ordered_extension_names(missing)),
                )
        return mandatory, requested, effective

    def _reported_extension_set(self, effective):
        reportable = {
            extension
            for extension in effective
            if self._is_supported(extension)
        }

        if {"Zba", "Zbb", "Zbs"}.issubset(reportable):
            reportable.add("B")
        if "C" in reportable:
            reportable.add("Zca")
        if {"C", "D"}.issubset(reportable):
            reportable.add("Zcd")
        if self._effective_riscv_type() == "RV32" and {"C", "F"}.issubset(
            reportable
        ):
            reportable.add("Zcf")

        return reportable

    def reports_extension(self, extension):
        _, _, effective = self._validate_extensions()
        if extension not in _EXTENSION_REGISTRY:
            return False
        return extension in self._reported_extension_set(effective)

    def get_reported_extensions(self):
        _, _, effective = self._validate_extensions()
        return list(
            _ordered_extension_names(self._reported_extension_set(effective))
        )

    def get_missing_profile_extensions(self):
        mandatory, _, _ = self._validate_extensions()
        return [
            extension
            for extension in mandatory
            if not self._is_supported(extension)
        ]

    def get_isa_string(self):
        base_extensions = []
        # check for the base ISA type
        if self._effective_riscv_type() == "RV32":
            base_extensions.append("rv32")
        elif self._effective_riscv_type() == "RV64":
            base_extensions.append("rv64")

        reported_extensions = self.get_reported_extensions()
        suppressed_extensions = set()
        if "B" in reported_extensions:
            suppressed_extensions.update(("Zba", "Zbb", "Zbs"))
        if "C" in reported_extensions:
            suppressed_extensions.update(("Zca", "Zcd", "Zcf"))

        single_letter = []
        multi_letter = []
        for extension in reported_extensions:
            if extension == "I" or extension in suppressed_extensions:
                continue
            if len(extension) == 1:
                single_letter.append(extension.lower())
            else:
                multi_letter.append(extension)

        isa_string = "".join(base_extensions + ["i"] + single_letter)

        for extension in multi_letter:
            isa_string += "_" + _EXTENSION_REGISTRY[extension].get(
                "isa", extension
            )

        return isa_string

    def getCCParams(self):
        self.reported_extensions = self.get_reported_extensions()
        return super().getCCParams()
