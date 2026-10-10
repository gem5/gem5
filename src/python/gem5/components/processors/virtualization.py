# Copyright (c) 2026 The Regents of The University of California
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

"""Host-dependent selection of a hardware-virtualized CPU backend."""

import os
import platform
import subprocess
from dataclasses import dataclass
from typing import Optional

from m5.defines import buildEnv

from ...isas import ISA
from .cpu_types import CPUTypes


def _host_isa(machine: str) -> Optional[ISA]:
    machine = machine.lower()
    if machine in ("x86_64", "amd64"):
        return ISA.X86
    if machine in ("aarch64", "arm64", "armv7l"):
        return ISA.ARM
    if machine in ("riscv64", "riscv"):
        return ISA.RISCV
    return None


def _kvm_isa(value: str) -> Optional[ISA]:
    return {
        "x86": ISA.X86,
        "arm": ISA.ARM,
        "riscv": ISA.RISCV,
    }.get(value.lower())


def _apple_hypervisor_available() -> bool:
    try:
        return (
            subprocess.check_output(
                ("sysctl", "-n", "kern.hv_support"), text=True
            ).strip()
            == "1"
        )
    except (OSError, subprocess.CalledProcessError):
        return False


@dataclass(frozen=True)
class VirtualizationHost:
    """Host and build facts consumed by the virtualized CPU resolver."""

    operating_system: str
    isa: Optional[ISA]
    kvm_device_exists: bool
    kvm_device_accessible: bool
    kvm_compiled: bool
    kvm_isa: Optional[ISA]
    apple_virt_compiled: bool
    apple_hypervisor_available: bool

    @classmethod
    def detect(cls) -> "VirtualizationHost":
        operating_system = platform.system()
        return cls(
            operating_system=operating_system,
            isa=_host_isa(platform.machine()),
            kvm_device_exists=os.path.exists("/dev/kvm"),
            kvm_device_accessible=os.access(
                "/dev/kvm", mode=os.R_OK | os.W_OK
            ),
            kvm_compiled=buildEnv.get("USE_KVM", False),
            kvm_isa=_kvm_isa(buildEnv.get("KVM_ISA", "")),
            apple_virt_compiled=buildEnv.get("USE_APPLE_VIRT", False),
            apple_hypervisor_available=(
                operating_system == "Darwin" and _apple_hypervisor_available()
            ),
        )


def resolve_virtualized_cpu_type(
    isa: ISA,
    num_cores: int,
    host: Optional[VirtualizationHost] = None,
) -> CPUTypes:
    """Resolve the host's usable virtualization backend without fallback."""

    if num_cores <= 0:
        raise ValueError("The number of cores must be a positive integer")
    if isa is None:
        raise ValueError(
            "A virtualized processor requires an explicit guest ISA"
        )

    host = host or VirtualizationHost.detect()
    if host.isa is None:
        raise ValueError(
            "Hardware virtualization is unsupported on host machine type "
            f"'{platform.machine()}'"
        )
    if isa != host.isa:
        raise ValueError(
            f"Hardware virtualization requires the guest ISA ({isa.name}) "
            f"to match the host ISA ({host.isa.name})"
        )

    if host.operating_system == "Linux":
        if not host.kvm_compiled:
            raise ValueError(
                "This gem5 binary was built without KVM support for the host "
                f"{host.isa.name} ISA"
            )
        if host.kvm_isa != isa:
            compiled = host.kvm_isa.name if host.kvm_isa else "none"
            raise ValueError(
                f"This gem5 binary contains KVM support for {compiled}, not "
                f"the requested {isa.name} ISA"
            )
        if not host.kvm_device_exists:
            raise ValueError(
                "KVM is unavailable because /dev/kvm does not exist; enable "
                "KVM in the host kernel or virtual-machine configuration"
            )
        if not host.kvm_device_accessible:
            raise ValueError(
                "KVM is unavailable because /dev/kvm is not readable and "
                "writable by the current user"
            )
        return CPUTypes.KVM

    if host.operating_system == "Darwin":
        if isa != ISA.ARM:
            raise ValueError(
                "Apple virtualization currently supports only AArch64 guests"
            )
        if num_cores != 1:
            raise ValueError(
                "Apple virtualization currently supports exactly one core"
            )
        if not host.apple_virt_compiled:
            raise ValueError(
                "This gem5 binary was built without Apple virtualization "
                "support"
            )
        if not host.apple_hypervisor_available:
            raise ValueError(
                "Apple Hypervisor.framework is unavailable on this host"
            )
        return CPUTypes.APPLE_VIRT

    raise ValueError(
        "No gem5 hardware-virtualization backend is supported on host "
        f"operating system '{host.operating_system}'"
    )
