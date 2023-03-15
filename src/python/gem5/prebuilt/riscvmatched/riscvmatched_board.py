# Copyright (c) 2022 The Regents of the University of California
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

import os
import re

from typing import List, Optional

from gem5.utils.override import overrides
from gem5.components.boards.abstract_system_board import AbstractSystemBoard
from gem5.components.boards.kernel_disk_workload import KernelDiskWorkload
from gem5.components.boards.se_binary_workload import SEBinaryWorkload
from gem5.resources.resource import AbstractResource
from gem5.components.memory import SingleChannelDDR4_2400
from gem5.utils.requires import requires
from gem5.isas import ISA
from .riscvmatched_cache import RISCVMatchedCacheHierarchy
from .riscvmatched_processor import U74Processor
from gem5.isas import ISA

import m5

from m5.objects import (
    BadAddr,
    Bridge,
    PMAChecker,
    RiscvLinux,
    AddrRange,
    IOXBar,
    RiscvRTC,
    HiFive,
    IGbE_e1000,
    CowDiskImage,
    RawDiskImage,
    RiscvMmioVirtIO,
    VirtIOBlock,
    VirtIORng,
    Frequency,
    Port,
)

from m5.util.fdthelper import (
    Fdt,
    FdtNode,
    FdtProperty,
    FdtPropertyStrings,
    FdtPropertyWords,
    FdtState,
)


def U74Memory():
    """
    Memory for the U74 board.
    DDR4 Subsystem with 16GB of memory.
    Starts at 0x80000000.
    Details at: Section 23, page 195 of the datasheet.

    return: ChanneledMemory
    """
    memory = SingleChannelDDR4_2400("16GB")
    memory.set_memory_range(
        [AddrRange(start=0x80000000, size=memory.get_size())]
    )
    return memory


class RISCVMatchedBoard(
    AbstractSystemBoard, KernelDiskWorkload, SEBinaryWorkload
):
    """
    A board capable of full system simulation for RISC-V

    At a high-level, this is based on the HiFive Unmatched board from SiFive.
    Based on : src/python/gem5/components/boards/riscv_board.py

    This board assumes that you will be booting Linux for fullsystem emulation.

    The frequency of the RTC for the system is set to 1MHz.
    Details can be found on page 77, section 7.1 of the datasheet.

    Datasheet for inbuilt params can be found here: https://sifive.cdn.prismic.io/sifive/1a82e600-1f93-4f41-b2d8-86ed8b16acba_fu740-c000-manual-v1p6.pdf
    """

    def __init__(
        self,
        clk_freq: str = "1.2GHz",
        l2_size: str = "2MB",
        is_fs: bool = False,
    ) -> None:
        """

        :param clk_freq: The clock frequency of the system,
        default: 1.2GHz
        :param l2_size: The size of the L2 cache,
        default: 2MB
        :param is_fs: Whether the system is a full system or not,
        default: False (SE Mode)

        """
        requires(isa_required=ISA.RISCV)
        self._fs = is_fs

        cache_hierarchy = RISCVMatchedCacheHierarchy(l2_size=l2_size)

        memory = U74Memory()

        processor = U74Processor(is_fs=is_fs)
        super().__init__(
            clk_freq=clk_freq,  # real system is 1.0 to 1.5 GHz
            processor=processor,
            memory=memory,
            cache_hierarchy=cache_hierarchy,
        )

    @overrides(AbstractSystemBoard)
    def _setup_board(self) -> None:
        if self._fs:
            self.workload = RiscvLinux()

            # Contains a CLINT, PLIC, UART, and some functions for the dtb, etc.
            self.platform = HiFive()
            # Note: This only works with single threaded cores.
            self.platform.plic.n_contexts = self.processor.get_num_cores() * 2
            self.platform.attachPlic()
            self.platform.clint.num_threads = self.processor.get_num_cores()

            # Add the RTC
            self.platform.rtc = RiscvRTC(
                frequency=Frequency("100MHz")
            )  # page 77, section 7.1
            self.platform.clint.int_pin = self.platform.rtc.int_pin

            # Incoherent I/O bus
            self.iobus = IOXBar()
            self.iobus.badaddr_responder = BadAddr()
            self.iobus.default = self.iobus.badaddr_responder.pio

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

            # Note: This overrides the platform's code because the platform isn't
            # general enough.
            self._on_chip_devices = [self.platform.clint, self.platform.plic]
            self._off_chip_devices = [self.platform.uart, self.disk, self.rng]

        else:
            pass

    def _setup_io_devices(self) -> None:
        """Connect the I/O devices to the I/O bus in FS mode."""
        if self._fs:
            # Add PCI
            self.platform.pci_host.pio = self.iobus.mem_side_ports

            # Add Ethernet card
            self.ethernet = IGbE_e1000(
                pci_bus=0,
                pci_dev=0,
                pci_func=0,
                InterruptLine=1,
                InterruptPin=1,
            )

            self.ethernet.host = self.platform.pci_host
            self.ethernet.pio = self.iobus.mem_side_ports
            self.ethernet.dma = self.iobus.cpu_side_ports

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

                # PCI
                self.bridge.ranges.append(AddrRange(0x2F000000, size="16MB"))
                self.bridge.ranges.append(AddrRange(0x30000000, size="256MB"))
                self.bridge.ranges.append(AddrRange(0x40000000, size="512MB"))

    def _setup_pma(self) -> None:
        """Set the PMA devices on each core"""

        uncacheable_range = [
            AddrRange(dev.pio_addr, size=dev.pio_size)
            for dev in self._on_chip_devices + self._off_chip_devices
        ]

        # PCI
        uncacheable_range.append(AddrRange(0x2F000000, size="16MB"))
        uncacheable_range.append(AddrRange(0x30000000, size="256MB"))
        uncacheable_range.append(AddrRange(0x40000000, size="512MB"))

        # TODO: Not sure if this should be done per-core like in the example
        for cpu in self.get_processor().get_cores():
            cpu.get_mmu().pma_checker = PMAChecker(
                uncacheable=uncacheable_range
            )

    @overrides(AbstractSystemBoard)
    def has_dma_ports(self) -> bool:
        return False

    @overrides(AbstractSystemBoard)
    def get_dma_ports(self) -> List[Port]:
        raise NotImplementedError(
            "RISCVBoard does not have DMA Ports. "
            "Use `has_dma_ports()` to check this."
        )

    @overrides(AbstractSystemBoard)
    def has_io_bus(self) -> bool:
        return self._fs

    @overrides(AbstractSystemBoard)
    def get_io_bus(self) -> IOXBar:
        if self._fs:
            return self.iobus
        else:
            raise NotImplementedError(
                "HiFiveBoard does not have an IO bus. "
                "Use `has_io_bus()` to check this."
            )

    @overrides(AbstractSystemBoard)
    def has_coherent_io(self) -> bool:
        return self._fs

    @overrides(AbstractSystemBoard)
    def get_mem_side_coherent_io_port(self) -> Port:
        if self._fs:
            return self.iobus.mem_side_ports
        else:
            raise NotImplementedError(
                "HiFiveBoard does not have any I/O ports. Use has_coherent_io to "
                "check this."
            )

    @overrides(AbstractSystemBoard)
    def _setup_memory_ranges(self):
        """
        Starting range for the DDR memory is 0x80000000.

        Details can be found on page 201, section 23.2.3 of the datasheet.

        """
        if self._fs:
            memory = self.get_memory()
            mem_size = memory.get_size()
            self.mem_ranges = [AddrRange(start=0x80000000, size=mem_size)]
            memory.set_memory_range(self.mem_ranges)
        else:
            memory = self.get_memory()
            # The SE board just has one memory range that is the size of the
            # memory.
            self.mem_ranges = [AddrRange(memory.get_size())]
            memory.set_memory_range(self.mem_ranges)

    def generate_device_tree(self, outdir: str) -> None:
        """Creates the dtb and dts files.

        Creates two files in the outdir: 'device.dtb' and 'device.dts'

        :param outdir: Directory to output the files
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

        # See Documentation/devicetree/bindings/riscv/cpus.txt for details.
        cpus_node = FdtNode("cpus")
        cpus_state = FdtState(addr_cells=1, size_cells=0)
        cpus_node.append(cpus_state.addrCellsProperty())
        cpus_node.append(cpus_state.sizeCellsProperty())
        # Used by the CLINT driver to set the timer frequency. Value taken from
        # RISC-V kernel docs (Note: freedom-u540 is actually 1MHz)
        cpus_node.append(FdtPropertyWords("timebase-frequency", [100000000]))

        for i, core in enumerate(self.get_processor().get_cores()):
            node = FdtNode(f"cpu@{i}")
            node.append(FdtPropertyStrings("device_type", "cpu"))
            node.append(FdtPropertyWords("reg", state.CPUAddrCells(i)))
            node.append(FdtPropertyStrings("mmu-type", "riscv,sv48"))
            node.append(FdtPropertyStrings("status", "okay"))
            node.append(FdtPropertyStrings("riscv,isa", "rv64imafdc"))
            freq = self.clk_domain.clock[0].frequency
            node.append(FdtPropertyWords("clock-frequency", freq))
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
        for i, core in enumerate(self.get_processor().get_cores()):
            phandle = state.phandle(f"cpu@{i}.int_state")
            int_extended.append(phandle)
            int_extended.append(0xB)
            int_extended.append(phandle)
            int_extended.append(0x9)

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
        uart_node.appendCompatible(["ns8250"])
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

        # Default DTB address if bbl is built with --with-dts option
        self.workload.dtb_addr = 0x87E00000

        self.generate_device_tree(m5.options.outdir)
        self.workload.dtb_filename = os.path.join(
            m5.options.outdir, "device.dtb"
        )

    @overrides(KernelDiskWorkload)
    def get_default_kernel_args(self) -> List[str]:
        return ["console=ttyS0", "root={root_value}", "rw"]

    @overrides(KernelDiskWorkload)
    def set_kernel_disk_workload(
        self,
        kernel: AbstractResource,
        disk_image: AbstractResource,
        bootloader: Optional[AbstractResource] = None,
        readfile: Optional[str] = None,
        readfile_contents: Optional[str] = None,
        kernel_args: Optional[List[str]] = None,
        exit_on_work_items: bool = True,
    ) -> None:
        self.workload = RiscvLinux()
        KernelDiskWorkload.set_kernel_disk_workload(
            self=self,
            kernel=kernel,
            disk_image=disk_image,
            bootloader=bootloader,
            readfile=readfile,
            readfile_contents=readfile_contents,
            kernel_args=kernel_args,
            exit_on_work_items=exit_on_work_items,
        )
