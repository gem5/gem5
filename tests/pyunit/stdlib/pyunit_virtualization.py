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

import unittest
from unittest.mock import patch

from gem5.components.processors.cpu_types import CPUTypes
from gem5.components.processors.simple_switchable_processor import (
    SimpleSwitchableProcessor,
)
from gem5.components.processors.virtualization import (
    VirtualizationHost,
    _host_isa,
    resolve_virtualized_cpu_type,
)
from gem5.components.processors.virtualized_processor import (
    VirtualizedProcessor,
    VirtualizedSwitchableProcessor,
)
from gem5.isas import ISA


def host(
    operating_system: str,
    isa: ISA,
    *,
    kvm_device_exists: bool = True,
    kvm_device_accessible: bool = True,
    kvm_compiled: bool = True,
    kvm_isa: ISA = None,
    apple_virt_compiled: bool = True,
    apple_hypervisor_available: bool = True,
) -> VirtualizationHost:
    return VirtualizationHost(
        operating_system=operating_system,
        isa=isa,
        kvm_device_exists=kvm_device_exists,
        kvm_device_accessible=kvm_device_accessible,
        kvm_compiled=kvm_compiled,
        kvm_isa=kvm_isa or isa,
        apple_virt_compiled=apple_virt_compiled,
        apple_hypervisor_available=apple_hypervisor_available,
    )


class VirtualizedCPUResolverTest(unittest.TestCase):
    def test_supported_host_machine_names(self):
        cases = {
            "x86_64": ISA.X86,
            "amd64": ISA.X86,
            "aarch64": ISA.ARM,
            "arm64": ISA.ARM,
            "armv7l": ISA.ARM,
            "riscv64": ISA.RISCV,
        }
        for machine, isa in cases.items():
            with self.subTest(machine=machine):
                self.assertEqual(isa, _host_isa(machine))

    def test_linux_native_isas_resolve_to_kvm(self):
        for isa in (ISA.X86, ISA.ARM, ISA.RISCV):
            with self.subTest(isa=isa):
                self.assertEqual(
                    CPUTypes.KVM,
                    resolve_virtualized_cpu_type(
                        isa=isa,
                        num_cores=4,
                        host=host("Linux", isa),
                    ),
                )

    def test_apple_silicon_resolves_to_apple_virt(self):
        self.assertEqual(
            CPUTypes.APPLE_VIRT,
            resolve_virtualized_cpu_type(
                isa=ISA.ARM,
                num_cores=1,
                host=host("Darwin", ISA.ARM),
            ),
        )

    def test_guest_and_host_isa_must_match(self):
        with self.assertRaisesRegex(ValueError, "guest ISA.*host ISA"):
            resolve_virtualized_cpu_type(
                isa=ISA.ARM,
                num_cores=1,
                host=host("Linux", ISA.X86),
            )

    def test_guest_isa_must_be_explicit(self):
        with self.assertRaisesRegex(ValueError, "explicit guest ISA"):
            resolve_virtualized_cpu_type(
                isa=None,
                num_cores=1,
                host=host("Linux", ISA.X86),
            )

    def test_kvm_binary_must_match_isa(self):
        with self.assertRaisesRegex(ValueError, "KVM support for X86"):
            resolve_virtualized_cpu_type(
                isa=ISA.ARM,
                num_cores=1,
                host=host("Linux", ISA.ARM, kvm_isa=ISA.X86),
            )

    def test_kvm_must_be_compiled(self):
        with self.assertRaisesRegex(ValueError, "built without KVM support"):
            resolve_virtualized_cpu_type(
                isa=ISA.X86,
                num_cores=1,
                host=host("Linux", ISA.X86, kvm_compiled=False),
            )

    def test_kvm_device_errors_are_distinct(self):
        with self.assertRaisesRegex(ValueError, "/dev/kvm does not exist"):
            resolve_virtualized_cpu_type(
                isa=ISA.X86,
                num_cores=1,
                host=host("Linux", ISA.X86, kvm_device_exists=False),
            )
        with self.assertRaisesRegex(ValueError, "not readable and writable"):
            resolve_virtualized_cpu_type(
                isa=ISA.X86,
                num_cores=1,
                host=host("Linux", ISA.X86, kvm_device_accessible=False),
            )

    def test_apple_virt_constraints_report_actionable_errors(self):
        cases = (
            (2, host("Darwin", ISA.ARM), "exactly one core"),
            (
                1,
                host("Darwin", ISA.ARM, apple_virt_compiled=False),
                "built without Apple virtualization",
            ),
            (
                1,
                host(
                    "Darwin",
                    ISA.ARM,
                    apple_hypervisor_available=False,
                ),
                "Hypervisor.framework is unavailable",
            ),
        )
        for num_cores, facts, message in cases:
            with self.subTest(message=message):
                with self.assertRaisesRegex(ValueError, message):
                    resolve_virtualized_cpu_type(
                        isa=ISA.ARM,
                        num_cores=num_cores,
                        host=facts,
                    )

    def test_unsupported_operating_system_does_not_fall_back(self):
        with self.assertRaisesRegex(ValueError, "FreeBSD"):
            resolve_virtualized_cpu_type(
                isa=ISA.X86,
                num_cores=1,
                host=host("FreeBSD", ISA.X86),
            )

    def test_switchable_processor_exposes_all_configured_cores(self):
        processor = SimpleSwitchableProcessor(
            starting_core_type=CPUTypes.ATOMIC,
            switch_core_type=CPUTypes.TIMING,
            num_cores=2,
            isa=ISA.X86,
            clk_freq="3GHz",
        )
        self.assertEqual(2, len(processor.get_cores()))
        self.assertEqual(4, len(processor.get_all_cores()))

    def test_virtualized_processor_resolves_its_core_type(self):
        with patch(
            "gem5.components.processors.virtualized_processor."
            "resolve_virtualized_cpu_type",
            return_value=CPUTypes.ATOMIC,
        ):
            processor = VirtualizedProcessor(
                num_cores=2,
                isa=ISA.X86,
                clk_freq="3GHz",
            )
        self.assertEqual(
            [CPUTypes.ATOMIC, CPUTypes.ATOMIC],
            [core.get_type() for core in processor.get_cores()],
        )

    def test_virtualized_switchable_processor_resolves_starting_core(self):
        with patch(
            "gem5.components.processors.virtualized_processor."
            "resolve_virtualized_cpu_type",
            return_value=CPUTypes.ATOMIC,
        ):
            processor = VirtualizedSwitchableProcessor(
                switch_core_type=CPUTypes.TIMING,
                num_cores=1,
                isa=ISA.X86,
                clk_freq="3GHz",
            )
        self.assertEqual(CPUTypes.ATOMIC, processor.get_cores()[0].get_type())

    def test_virtualized_switchable_target_must_be_simulated(self):
        with self.assertRaisesRegex(ValueError, "must be a simulated"):
            VirtualizedSwitchableProcessor(
                switch_core_type=CPUTypes.KVM,
                num_cores=1,
                isa=ISA.X86,
                clk_freq="3GHz",
            )


if __name__ == "__main__":
    unittest.main()
