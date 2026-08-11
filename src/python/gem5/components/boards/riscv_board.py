# Copyright (c) 2021 The Regents of the University of California
# Copyright (c) 2022 EXAscale Performance SYStems (EXAPSYS)
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

import ctypes
import errno
import fcntl
import os
import struct
from typing import (
    List,
    Optional,
    Set,
)

import m5
from m5.objects import (
    BadAddr,
    Bridge,
    CowDiskImage,
    Frequency,
    GenericRiscvPciHost,
    HiFive,
    IGbE_e1000,
    IOXBar,
    PciHost,
    PMAChecker,
    RawDiskImage,
    RiscvBootloaderKernelWorkload,
    RiscvMmioVirtIO,
    RiscvRTC,
    RiscvSystem,
    Root,
    SimObject,
    VirtIOBlock,
    VirtIORng,
)
from m5.params import (
    AddrRange,
    Port,
)
from m5.util import warn
from m5.util.fdthelper import (
    Fdt,
    FdtNode,
    FdtProperty,
    FdtPropertyStrings,
    FdtPropertyWords,
    FdtState,
)

from ...components.boards.se_binary_workload import SEBinaryWorkload
from ...isas import ISA
from ...resources.resource import AbstractResource
from ...utils.override import overrides
from ..cachehierarchies.abstract_cache_hierarchy import AbstractCacheHierarchy
from ..memory.abstract_memory_system import AbstractMemorySystem
from ..processors.abstract_processor import AbstractProcessor
from .abstract_board import AbstractBoard
from .kernel_disk_workload import KernelDiskWorkload


class RiscvBoard(
    RiscvSystem, AbstractBoard, KernelDiskWorkload, SEBinaryWorkload
):
    """
    A board capable of full system simulation for RISC-V.

    At a high-level, this is based on the HiFive Unmatched board from SiFive.

    This board assumes that you will be booting Linux.

    **Limitations**
    * Only works with classic caches
    """

    _default_timebase_frequency = 100000000
    _default_mmu_type = "riscv,sv39"
    _default_m5ops_base = 0x10010000
    _default_m5ops_size = 0x10000

    _kvmio = 0xAE
    _ioc_nrbits = 8
    _ioc_typebits = 8
    _ioc_sizebits = 14
    _ioc_nrshift = 0
    _ioc_typeshift = _ioc_nrshift + _ioc_nrbits
    _ioc_sizeshift = _ioc_typeshift + _ioc_typebits
    _ioc_dirshift = _ioc_sizeshift + _ioc_sizebits
    _ioc_none = 0
    _ioc_write = 1
    _ioc_read = 2
    _kvm_get_api_version = (_kvmio << _ioc_typeshift) | 0x00
    _kvm_create_vm = (_kvmio << _ioc_typeshift) | 0x01
    _kvm_create_vcpu = (_kvmio << _ioc_typeshift) | 0x41
    _kvm_get_one_reg = (
        ((_ioc_read | _ioc_write) << _ioc_dirshift)
        | (16 << _ioc_sizeshift)
        | (_kvmio << _ioc_typeshift)
        | 0xAB
    )
    _kvm_expected_api_version = 12
    _kvm_reg_riscv = 0x8000000000000000
    _kvm_reg_size_u64 = 0x0030000000000000
    _kvm_reg_riscv_timer = 0x04 << 24
    _kvm_reg_riscv_vector = 0x09 << 24
    _kvm_timer_frequency_reg = (
        _kvm_reg_riscv | _kvm_reg_size_u64 | _kvm_reg_riscv_timer
    )
    _kvm_vector_vlenb_reg = (
        _kvm_reg_riscv | _kvm_reg_size_u64 | _kvm_reg_riscv_vector | 4
    )

    def __init__(
        self,
        clk_freq: str,
        processor: AbstractProcessor,
        memory: AbstractMemorySystem,
        cache_hierarchy: AbstractCacheHierarchy,
    ) -> None:
        super().__init__()
        AbstractBoard.__init__(
            self, clk_freq, processor, memory, cache_hierarchy
        )
        # Kernel-disk workloads may query default kernel args before the
        # full-system board setup assigns the MMIO m5ops window.
        self.m5ops_base = self._default_m5ops_base
        self._kvm_probe_cache = None
        self._kvm_host_freq_cache = None
        self._host_timebase_freq_cache = None

        if processor.get_isa() != ISA.RISCV:
            raise Exception(
                "The RISCVBoard requires a processor using the"
                "RISCV ISA. Current processor ISA: "
                f"'{processor.get_isa().name}'."
            )

    def _has_kvm_cores(self) -> bool:
        return any(
            core.is_kvm_core() for core in self.get_processor().get_cores()
        )

    @staticmethod
    def _read_int_file(path: str) -> Optional[int]:
        try:
            with open(path) as f:
                return int(f.read().strip())
        except (OSError, ValueError):
            return None

    @staticmethod
    def _read_dt_u32(path: str) -> Optional[int]:
        try:
            with open(path, "rb") as f:
                data = f.read(4)
        except OSError:
            return None

        if len(data) != 4:
            return None

        return int.from_bytes(data, byteorder="big")

    @staticmethod
    def _read_cpu_list(path: str) -> Set[int]:
        try:
            with open(path) as f:
                spec = f.read().strip()
        except OSError:
            return set()

        cpus = set()
        for chunk in spec.replace(",", " ").split():
            chunk = chunk.strip()
            if not chunk:
                continue
            if "-" in chunk:
                start, end = chunk.split("-", 1)
                cpus.update(range(int(start), int(end) + 1))
            else:
                cpus.add(int(chunk))
        return cpus

    def _probe_kvm_one_reg(self, reg_id: int) -> Optional[int]:
        if not self._has_kvm_cores():
            return None

        kvm_fd = -1
        vm_fd = -1
        vcpu_fd = -1

        try:
            kvm_fd = os.open("/dev/kvm", os.O_RDWR | os.O_CLOEXEC)
            api_version = fcntl.ioctl(kvm_fd, self._kvm_get_api_version)
            if api_version != self._kvm_expected_api_version:
                raise OSError(
                    errno.ENOTSUP,
                    "unsupported KVM API version",
                )

            vm_fd = fcntl.ioctl(kvm_fd, self._kvm_create_vm, 0)
            vcpu_fd = fcntl.ioctl(vm_fd, self._kvm_create_vcpu, 0)

            value = struct.pack("Q", 0)
            value_buf = bytearray(value)
            addr = ctypes.addressof(ctypes.c_uint64.from_buffer(value_buf))
            one_reg = bytearray(struct.pack("QQ", reg_id, addr))
            fcntl.ioctl(vcpu_fd, self._kvm_get_one_reg, one_reg, True)
            return struct.unpack("Q", value_buf)[0]
        except OSError as err:
            if err.errno in (
                errno.ENOENT,
                errno.EINVAL,
                errno.ENOTSUP,
                errno.EOPNOTSUPP,
            ):
                return None
            raise
        finally:
            for fd in (vcpu_fd, vm_fd, kvm_fd):
                if fd >= 0:
                    os.close(fd)

    def _kvm_probe_data(self) -> dict:
        if self._kvm_probe_cache is None:
            self._kvm_probe_cache = {
                "timer_frequency": self._probe_kvm_one_reg(
                    self._kvm_timer_frequency_reg
                ),
                "vlenb": self._probe_kvm_one_reg(self._kvm_vector_vlenb_reg),
            }

        return self._kvm_probe_cache

    def _configured_riscv_isa(self):
        return self.get_processor().get_cores()[0].core.isa[0]

    def _cpu_timebase_frequency(self) -> int:
        if not self._has_kvm_cores():
            return self._default_timebase_frequency

        timer_frequency = self._kvm_probe_data()["timer_frequency"]
        if timer_frequency is not None:
            return timer_frequency

        host_timebase = self._detect_host_timebase_frequency()
        if host_timebase is not None:
            warn(
                "Unable to read the KVM RISC-V timer frequency; "
                "falling back to the host device-tree timebase-frequency "
                f"({host_timebase} Hz)."
            )
            return host_timebase

        warn(
            "Unable to read the KVM RISC-V timer frequency; "
            f"falling back to {self._default_timebase_frequency} Hz."
        )
        return self._default_timebase_frequency

    def _detect_host_timebase_frequency(self) -> Optional[int]:
        if self._host_timebase_freq_cache is not None:
            return self._host_timebase_freq_cache

        for path in (
            "/proc/device-tree/cpus/timebase-frequency",
            "/sys/firmware/devicetree/base/cpus/timebase-frequency",
        ):
            frequency = self._read_dt_u32(path)
            if frequency is not None:
                self._host_timebase_freq_cache = frequency
                return frequency

        return None

    @staticmethod
    def _decode_dt_string_list(data: bytes) -> List[str]:
        return [
            entry.decode()
            for entry in data.rstrip(b"\0").split(b"\0")
            if entry
        ]

    @staticmethod
    def _split_isa_string(isa: str):
        isa_tokens = isa.lower().split("_")
        return isa_tokens[0], isa_tokens[1:]

    def _detect_kvm_host_frequency(self) -> Optional[int]:
        if self._kvm_host_freq_cache is not None:
            return self._kvm_host_freq_cache

        affinity = None
        try:
            affinity = os.sched_getaffinity(0)
        except AttributeError:
            pass

        policies = []
        root = "/sys/devices/system/cpu/cpufreq"
        try:
            entries = sorted(
                (
                    entry
                    for entry in os.scandir(root)
                    if entry.is_dir() and entry.name.startswith("policy")
                ),
                key=lambda entry: entry.name,
            )
        except OSError:
            self._kvm_host_freq_cache = None
            return None

        for entry in entries:
            cpus = self._read_cpu_list(
                os.path.join(entry.path, "affected_cpus")
            )
            if affinity is not None and cpus and not (cpus & affinity):
                continue

            freq_khz = self._read_int_file(
                os.path.join(entry.path, "cpuinfo_max_freq")
            )
            if freq_khz is None:
                freq_khz = self._read_int_file(
                    os.path.join(entry.path, "scaling_cur_freq")
                )

            if freq_khz is not None:
                policies.append((entry.name, freq_khz * 1000))

        if not policies:
            self._kvm_host_freq_cache = None
            return None

        policy_freqs = sorted({freq for _, freq in policies})
        host_freq = max(policy_freqs)

        if len(policy_freqs) > 1:
            policy_desc = ", ".join(
                f"{name}={freq}Hz" for name, freq in policies
            )
            warn(
                "Detected multiple KVM host CPU frequencies in the current "
                f"affinity mask ({policy_desc}); using {host_freq}Hz for "
                "hostFreq. Pin gem5 to a homogeneous cpufreq policy or "
                "override hostFreq for more accurate perf-cycle scaling."
            )

        self._kvm_host_freq_cache = host_freq
        return host_freq

    def _configure_kvm_core_host_freq(self) -> None:
        if not self._has_kvm_cores():
            return

        host_freq = self._detect_kvm_host_frequency()
        if host_freq is None:
            return

        for core in self.get_processor().get_cores():
            if not core.is_kvm_core():
                continue

            sim_core = core.core
            if "hostFreq" in sim_core._values.local:
                continue

            sim_core.hostFreq = f"{host_freq}Hz"

    def _configure_kvm_core_rvv(self) -> None:
        if not self._has_kvm_cores():
            return

        need_vector_probe = any(
            core.is_kvm_core() and core.core.isa[0].enable_rvv.value
            for core in self.get_processor().get_cores()
        )
        if not need_vector_probe:
            return

        kvm_vlenb = self._kvm_probe_data()["vlenb"]
        if kvm_vlenb is None:
            m5.fatal(
                "RISC-V KVM RVV was enabled in the gem5 configuration, but "
                "the host KVM interface does not expose vector state."
            )

        kvm_vlen = kvm_vlenb * 8
        for core in self.get_processor().get_cores():
            if not core.is_kvm_core():
                continue

            isa = core.core.isa[0]
            if not isa.enable_rvv.value:
                continue

            if "vlen" in isa._values.local:
                if isa.vlen.value != kvm_vlen:
                    m5.fatal(
                        "Configured RISC-V KVM RVV VLEN (%d bits) does not "
                        "match the host KVM VLEN (%d bits). KVM exposes a "
                        "fixed VLENB, so set isa[0].vlen to %d or disable "
                        "RVV for the KVM CPU."
                        % (isa.vlen.value, kvm_vlen, kvm_vlen)
                    )
                continue

            isa.vlen = kvm_vlen

    def _configure_kvm_cores(self) -> None:
        self._configure_kvm_core_host_freq()
        self._configure_kvm_core_rvv()

    @overrides(AbstractBoard)
    def _setup_board(self) -> None:
        if self.is_fullsystem():
            self.workload = RiscvBootloaderKernelWorkload()

            # Contains a CLINT, PLIC, UART, and some functions for the dtb, etc.
            self.platform = HiFive()
            # Note: This only works with single threaded cores.
            self.platform.plic.hart_config = ",".join(
                ["MS" for _ in range(self.processor.get_num_cores())]
            )
            self.platform.attachPlic()
            self.platform.clint.num_threads = self.processor.get_num_cores()

            # Add the RTC
            self.platform.rtc = RiscvRTC(
                frequency=Frequency(f"{self._cpu_timebase_frequency()}Hz")
            )
            self.platform.clint.int_pin = self.platform.rtc.int_pin

            # Incoherent I/O bus
            self.iobus = IOXBar()
            self.iobus.badaddr_responder = BadAddr()
            self.iobus.default = self.iobus.badaddr_responder.pio
            self.m5ops_base = self._default_m5ops_base

            # The virtio disk
            self.disk = RiscvMmioVirtIO(
                vio=VirtIOBlock(),
                interrupt_id=0x8,
                pio_size=4096,
                pio_addr=0x10008000,
            )

            # The virtio rng
            self.rng = RiscvMmioVirtIO(
                vio=VirtIORng(),
                interrupt_id=0x8,
                pio_size=4096,
                pio_addr=0x10007000,
            )

            # Note: This overrides the platform's code because the platform
            # isn't general enough.
            self._on_chip_devices = [self.platform.clint, self.platform.plic]
            self._off_chip_devices = [self.platform.uart, self.disk, self.rng]

        else:
            # SE mode board setup
            pass

    def _setup_io_devices(self) -> None:
        """Connect the I/O devices to the I/O bus."""
        # Add PCI
        self.platform.pci_host.internal_connect()
        self.platform.pci_host.connect_upper_bus(self.iobus, True)

        # Add Ethernet card
        self.ethernet = IGbE_e1000(
            pci_dev=0, pci_func=0, InterruptLine=1, InterruptPin=1
        )

        self.platform.pci_host.connect_device(self.ethernet)

        if self.get_cache_hierarchy().is_ruby():
            for device in self._off_chip_devices + self._on_chip_devices:
                device.pio = self.iobus.mem_side_ports

        else:
            for device in self._off_chip_devices:
                device.pio = self.iobus.mem_side_ports
            for device in self._on_chip_devices:
                device.pio = self.get_cache_hierarchy().get_mem_side_port()

            self.bridge = Bridge(delay="10ns")
            self.bridge.mem_side_port = self.iobus.cpu_side_ports
            self.bridge.cpu_side_port = (
                self.get_cache_hierarchy().get_mem_side_port()
            )
            self.bridge.ranges = [
                AddrRange(dev.pio_addr, size=dev.pio_size)
                for dev in self._off_chip_devices
            ]
            self.bridge.ranges.append(
                AddrRange(self.m5ops_base, size=self._default_m5ops_size)
            )

            # PCI
            self.bridge.ranges.append(AddrRange(0x2F000000, size="16MiB"))
            self.bridge.ranges.append(AddrRange(0x30000000, size="256MiB"))
            self.bridge.ranges.append(AddrRange(0x40000000, size="512MiB"))

    def _setup_pma(self) -> None:
        """Set the PMA devices on each core."""

        uncacheable_range = [
            AddrRange(dev.pio_addr, size=dev.pio_size)
            for dev in self._on_chip_devices + self._off_chip_devices
        ]
        uncacheable_range.append(
            AddrRange(self.m5ops_base, size=self._default_m5ops_size)
        )

        # PCI
        uncacheable_range.append(AddrRange(0x2F000000, size="16MiB"))
        uncacheable_range.append(AddrRange(0x30000000, size="256MiB"))
        uncacheable_range.append(AddrRange(0x40000000, size="512MiB"))

        # TODO: Not sure if this should be done per-core like in the example
        for cpu in self.get_processor().get_cores():
            cpu.get_mmu().pma_checker = PMAChecker(
                uncacheable=uncacheable_range
            )

    @overrides(AbstractBoard)
    def has_dma_ports(self) -> bool:
        return False

    @overrides(AbstractBoard)
    def get_dma_ports(self) -> List[Port]:
        raise Exception(
            "Cannot execute `get_dma_ports()`: Board does not have DMA ports "
            "to return. Use `has_dma_ports()` to check this."
        )

    @overrides(AbstractBoard)
    def has_io_bus(self) -> bool:
        return self.is_fullsystem()

    @overrides(AbstractBoard)
    def get_io_bus(self) -> IOXBar:
        if self.has_io_bus():
            return self.iobus
        else:
            raise Exception(
                "Cannot execute `get_io_bus()`: Board does not have an I/O "
                "bus to return. Use `has_io_bus()` to check this."
            )

    @overrides(AbstractBoard)
    def has_pci_host(self) -> bool:
        return self.is_fullsystem()

    @overrides(AbstractBoard)
    def get_pci_host(self) -> PciHost:
        if self.has_pci_host():
            return self.platform.pci_host
        else:
            raise Exception(
                "Cannot execute `get_pci_host()`: Board does not have a PCI "
                "host to return. Use `has_pci_host()` to check this."
            )

    @overrides(AbstractBoard)
    def has_coherent_io(self) -> bool:
        return self.is_fullsystem()

    @overrides(AbstractBoard)
    def get_mem_side_coherent_io_port(self) -> Port:
        if self.has_coherent_io():
            return self.iobus.mem_side_ports
        else:
            raise Exception(
                "Cannot execute `get_mem_side_coherent_io_port()`: Board does "
                "not have any I/O ports to return. Use `has_coherent_io()` to "
                "check this."
            )

    @overrides(AbstractBoard)
    def _setup_memory_ranges(self):
        memory = self.get_memory()
        mem_size = memory.get_size()
        self.mem_ranges = [AddrRange(start=0x80000000, size=mem_size)]
        memory.set_memory_range(self.mem_ranges)

    def generate_device_tree(self, outdir: str) -> None:
        """Creates the ``dtb`` and ``dts`` files.

        Creates two files in the outdir: ``device.dtb`` and ``device.dts``.

        :param outdir: Directory to output the files.
        """

        state = FdtState(addr_cells=2, size_cells=2, cpu_cells=1)
        root = FdtNode("/")
        root.append(state.addrCellsProperty())
        root.append(state.sizeCellsProperty())
        root.appendCompatible(["riscv-virtio"])

        for mem_range in self.mem_ranges:
            node = FdtNode(f"memory@{int(mem_range.start):x}")
            node.append(FdtPropertyStrings("device_type", ["memory"]))
            node.append(
                FdtPropertyWords(
                    "reg",
                    state.addrCells(mem_range.start)
                    + state.sizeCells(mem_range.size()),
                )
            )
            root.append(node)

        node = FdtNode(f"chosen")
        bootargs = self.workload.command_line
        node.append(FdtPropertyStrings("bootargs", [bootargs]))
        node.append(FdtPropertyStrings("stdout-path", ["/soc/uart@10000000"]))
        root.append(node)

        # See Documentation/devicetree/bindings/riscv/cpus.txt for details.
        cpus_node = FdtNode("cpus")
        cpus_state = FdtState(addr_cells=1, size_cells=0)
        cpus_node.append(cpus_state.addrCellsProperty())
        cpus_node.append(cpus_state.sizeCellsProperty())
        cpu_timebase_frequency = self._cpu_timebase_frequency()
        cpu_isa_string = self._configured_riscv_isa().get_isa_string()
        _, cpu_isa_exts = self._split_isa_string(cpu_isa_string)
        cpu_isa_exts_set = set(cpu_isa_exts)
        cpu_clock_frequency = self.clk_domain.clock[0].frequency
        cpu_cbom_block_size = self.get_cache_line_size()
        cpu_cboz_block_size = self.get_cache_line_size()
        cpus_node.append(
            FdtPropertyWords("timebase-frequency", [cpu_timebase_frequency])
        )

        for i, core in enumerate(self.get_processor().get_cores()):
            node = FdtNode(f"cpu@{i}")
            node.append(FdtPropertyStrings("device_type", "cpu"))
            node.append(FdtPropertyWords("reg", state.CPUAddrCells(i)))
            node.append(FdtPropertyStrings("mmu-type", "riscv,sv48"))
            if core.core.isa[0].reports_extension("Zicbom"):
                node.append(
                    FdtPropertyWords(
                        "riscv,cbom-block-size", cpu_cbom_block_size
                    )
                )
            if core.core.isa[0].reports_extension("Zicboz"):
                node.append(
                    FdtPropertyWords(
                        "riscv,cboz-block-size", cpu_cboz_block_size
                    )
                )
            node.append(FdtPropertyStrings("status", "okay"))
            node.append(FdtPropertyStrings("riscv,isa", cpu_isa_string))
            node.append(
                FdtPropertyWords("clock-frequency", cpu_clock_frequency)
            )
            node.appendCompatible(["riscv"])
            int_phandle = state.phandle(f"cpu@{i}.int_state")
            node.appendPhandle(f"cpu@{i}")

            int_node = FdtNode("interrupt-controller")
            int_state = FdtState(interrupt_cells=1)
            int_phandle = int_state.phandle(f"cpu@{i}.int_state")
            int_node.append(int_state.interruptCellsProperty())
            int_node.append(FdtProperty("interrupt-controller"))
            int_node.appendCompatible("riscv,cpu-intc")
            int_node.append(FdtPropertyWords("phandle", [int_phandle]))

            node.append(int_node)
            cpus_node.append(node)

        root.append(cpus_node)

        soc_node = FdtNode("soc")
        soc_state = FdtState(addr_cells=2, size_cells=2)
        soc_node.append(soc_state.addrCellsProperty())
        soc_node.append(soc_state.sizeCellsProperty())
        soc_node.append(FdtProperty("ranges"))
        soc_node.appendCompatible(["simple-bus"])

        # CLINT node
        clint = self.platform.clint
        clint_node = clint.generateBasicPioDeviceNode(
            soc_state, "clint", clint.pio_addr, clint.pio_size
        )
        int_extended = list()
        for i, core in enumerate(self.get_processor().get_cores()):
            phandle = soc_state.phandle(f"cpu@{i}.int_state")
            int_extended.append(phandle)
            int_extended.append(0x3)
            int_extended.append(phandle)
            int_extended.append(0x7)
        clint_node.append(
            FdtPropertyWords("interrupts-extended", int_extended)
        )
        clint_node.appendCompatible(["riscv,clint0"])
        soc_node.append(clint_node)

        # PLIC node
        plic = self.platform.plic
        plic_node = plic.generateBasicPioDeviceNode(
            soc_state, "plic", plic.pio_addr, plic.pio_size
        )

        int_state = FdtState(addr_cells=0, interrupt_cells=1)
        plic_node.append(int_state.addrCellsProperty())
        plic_node.append(int_state.interruptCellsProperty())

        phandle = int_state.phandle(plic)
        plic_node.append(FdtPropertyWords("phandle", [phandle]))
        plic_node.append(FdtPropertyWords("riscv,ndev", [plic.n_src - 1]))

        int_extended = list()
        cpu_id = 0
        phandle = int_state.phandle(f"cpu@{cpu_id}.int_state")
        for c in plic.hart_config:
            if c == ",":
                cpu_id += 1
                assert cpu_id < self.get_processor().get_num_cores()
                phandle = int_state.phandle(f"cpu@{cpu_id}.int_state")
            elif c == "S":
                int_extended.append(phandle)
                int_extended.append(0x9)
            elif c == "M":
                int_extended.append(phandle)
                int_extended.append(0xB)

        plic_node.append(FdtPropertyWords("interrupts-extended", int_extended))
        plic_node.append(FdtProperty("interrupt-controller"))
        plic_node.appendCompatible(["riscv,plic0"])

        soc_node.append(plic_node)

        # PCI
        pci_state = FdtState(
            addr_cells=3, size_cells=2, cpu_cells=1, interrupt_cells=1
        )
        pci_node = FdtNode("pci")

        if int(self.platform.pci_host.conf_device_bits) == 8:
            pci_node.appendCompatible("pci-host-cam-generic")
        elif int(self.platform.pci_host.conf_device_bits) == 12:
            pci_node.appendCompatible("pci-host-ecam-generic")
        else:
            m5.fatal("No compatibility string for the set conf_device_width")

        pci_node.append(FdtPropertyStrings("device_type", ["pci"]))

        # Cell sizes of child nodes/peripherals
        pci_node.append(pci_state.addrCellsProperty())
        pci_node.append(pci_state.sizeCellsProperty())
        pci_node.append(pci_state.interruptCellsProperty())
        # PCI address for CPU
        pci_node.append(
            FdtPropertyWords(
                "reg",
                soc_state.addrCells(self.platform.pci_host.conf_base)
                + soc_state.sizeCells(self.platform.pci_host.conf_size),
            )
        )

        # Ranges mapping
        # For now some of this is hard coded, because the PCI module does not
        # have a proper full understanding of the memory map, but adapting the
        # PCI module is beyond the scope of what I'm trying to do here.
        # Values are taken from the ARM VExpress_GEM5_V1 platform.
        ranges = []
        # Pio address range
        ranges += self.platform.pci_host.pciFdtAddr(space=1, addr=0)
        ranges += soc_state.addrCells(self.platform.pci_host.pci_pio_base)
        ranges += pci_state.sizeCells(0x10000)  # Fixed size

        # AXI memory address range
        ranges += self.platform.pci_host.pciFdtAddr(space=2, addr=0)
        ranges += soc_state.addrCells(self.platform.pci_host.pci_mem_base)
        ranges += pci_state.sizeCells(0x40000000)  # Fixed size
        pci_node.append(FdtPropertyWords("ranges", ranges))

        # Interrupt mapping
        plic_handle = int_state.phandle(plic)
        int_base = self.platform.pci_host.int_base

        interrupts = []

        for i in range(int(self.platform.pci_host.int_count)):
            interrupts += self.platform.pci_host.pciFdtAddr(
                device=i, addr=0
            ) + [int(i) + 1, plic_handle, int(int_base) + i]

        pci_node.append(FdtPropertyWords("interrupt-map", interrupts))

        int_count = int(self.platform.pci_host.int_count)
        if int_count & (int_count - 1):
            fatal("PCI interrupt count should be power of 2")

        intmask = self.platform.pci_host.pciFdtAddr(
            device=int_count - 1, addr=0
        ) + [0x0]
        pci_node.append(FdtPropertyWords("interrupt-map-mask", intmask))

        if self.platform.pci_host._dma_coherent:
            pci_node.append(FdtProperty("dma-coherent"))

        soc_node.append(pci_node)

        # UART node
        uart = self.platform.uart
        uart_node = uart.generateBasicPioDeviceNode(
            soc_state, "uart", uart.pio_addr, uart.pio_size
        )
        uart_node.append(
            FdtPropertyWords("interrupts", [self.platform.uart_int_id])
        )
        uart_node.append(FdtPropertyWords("clock-frequency", [0x384000]))
        uart_node.append(
            FdtPropertyWords("interrupt-parent", soc_state.phandle(plic))
        )
        uart_node.appendCompatible(["ns8250", "ns16550a"])
        soc_node.append(uart_node)

        # VirtIO MMIO disk node
        disk = self.disk
        disk_node = disk.generateBasicPioDeviceNode(
            soc_state, "virtio_mmio", disk.pio_addr, disk.pio_size
        )
        disk_node.append(FdtPropertyWords("interrupts", [disk.interrupt_id]))
        disk_node.append(
            FdtPropertyWords("interrupt-parent", soc_state.phandle(plic))
        )
        disk_node.appendCompatible(["virtio,mmio"])
        soc_node.append(disk_node)

        # VirtIO MMIO rng node
        rng = self.rng
        rng_node = rng.generateBasicPioDeviceNode(
            soc_state, "virtio_mmio", rng.pio_addr, rng.pio_size
        )
        rng_node.append(FdtPropertyWords("interrupts", [rng.interrupt_id]))
        rng_node.append(
            FdtPropertyWords("interrupt-parent", soc_state.phandle(plic))
        )
        rng_node.appendCompatible(["virtio,mmio"])
        soc_node.append(rng_node)

        root.append(soc_node)

        fdt = Fdt()
        fdt.add_rootnode(root)
        fdt.writeDtsFile(os.path.join(outdir, "device.dts"))
        fdt.writeDtbFile(os.path.join(outdir, "device.dtb"))

    @overrides(KernelDiskWorkload)
    def get_disk_device(self):
        return "/dev/vda"

    @overrides(AbstractBoard)
    def _pre_instantiate(self, full_system: Optional[bool] = None) -> Root:
        self._configure_kvm_cores()

        # This is a bit of a hack necessary to get the RiscDemoBoard working
        # At the time of writing the RiscvBoard does not support SE mode so
        # this branch looks pointless. However, the RiscvDemoBoard does and
        # needs this logic in place.
        #
        # This should be refactored in the future as part of a chance to have
        # all boards support both FS and SE modes.
        if self.is_fullsystem():
            if len(self._bootloader) > 0:
                self.workload.bootloader_addr = 0x80000000
                self.workload.bootloader_filename = self._bootloader[0]
                self.workload.kernel_addr = 0x80200000
                self.workload.entry_point = (
                    0x80000000  # Bootloader starting point
                )
            else:
                self.workload.kernel_addr = 0x80000000
                self.workload.entry_point = 0x80000000

            # Set up the device tree. We need to wait until pre-instantiate to
            # do this since the workload could change until this point.
            # Default DTB address if bbl is built with --with-dts option
            self.workload.dtb_addr = 0x87E00000

            self.generate_device_tree(m5.options.outdir)
            self.workload.dtb_filename = os.path.join(
                m5.options.outdir, "device.dtb"
            )

        return super()._pre_instantiate(full_system=full_system)

    @overrides(KernelDiskWorkload)
    def _add_disk_to_board(self, disk_image: AbstractResource):
        image = CowDiskImage(
            child=RawDiskImage(read_only=True), read_only=False
        )
        image.child.image_file = disk_image.get_local_path()
        self.disk.vio.image = image

        # Note: The below is a bit of a hack. We need to wait to generate the
        # device tree until after the disk is set up. Now that the disk and
        # workload are set, we can generate the device tree file.
        self._setup_io_devices()
        self._setup_pma()

    @overrides(KernelDiskWorkload)
    def get_default_kernel_args(self) -> List[str]:
        m5ops_base = int(getattr(self, "m5ops_base", self._default_m5ops_base))
        return [
            "console=ttyS0",
            "root={root_value}",
            "disk_device={disk_device}",
            f"gem5_bridge_baseaddr=0x{m5ops_base:x}",
            f"gem5_bridge_rangesize=0x{self._default_m5ops_size:x}",
            "rw",
        ]

    @overrides(SimObject)
    def createCCObject(self):
        """We override this function as it is called in ``m5.instantiate``. This
        means we can insert a check to ensure the ``_connect_things`` function
        has been run.
        """
        super()._connect_things_check()
        super().createCCObject()
