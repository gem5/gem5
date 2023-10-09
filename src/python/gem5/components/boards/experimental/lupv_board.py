# Copyright (c) 2021 The Regents of the University of California
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
from typing import List

from ....utils.override import overrides
from ..abstract_system_board import AbstractSystemBoard
from ...processors.abstract_processor import AbstractProcessor
from ...memory.abstract_memory_system import AbstractMemorySystem
from ...cachehierarchies.abstract_cache_hierarchy import AbstractCacheHierarchy
from ..kernel_disk_workload import KernelDiskWorkload
from ....resources.resource import AbstractResource
from ....isas import ISA

import m5
from m5.objects import (
    Bridge,
    PMAChecker,
    RiscvLinux,
    RiscvRTC,
    AddrRange,
    IOXBar,
    Clint,
    Plic,
    Terminal,
    LupioBLK,
    LupioIPI,
    LupioPIC,
    LupioRNG,
    LupioRTC,
    LupioTMR,
    LupioTTY,
    LupioSYS,
    LupV,
    AddrRange,
    CowDiskImage,
    RawDiskImage,
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


class LupvBoard(AbstractSystemBoard, KernelDiskWorkload):
    """
    A board capable of full system simulation for RISC-V.
    This board uses a set of LupIO education-friendly devices.
    This board assumes that you will be booting Linux.

    **Limitations**
    * Only works with classic caches
    """

    def __init__(
        self,
        clk_freq: str,
        processor: AbstractProcessor,
        memory: AbstractMemorySystem,
        cache_hierarchy: AbstractCacheHierarchy,
    ) -> None:
        if cache_hierarchy.is_ruby():
            raise OSError("RiscvBoard is not compatible with Ruby")

        if processor.get_isa() != ISA.RISCV:
            raise Exception(
                "The LupvBoard requires a processor using the "
                "RISCV ISA. Current processor "
                f"ISA: '{processor.get_isa().name}'."
            )

        super().__init__(clk_freq, processor, memory, cache_hierarchy)

    @overrides(AbstractSystemBoard)
    def _setup_board(self) -> None:
        self.workload = RiscvLinux()

        # Initialize all the devices that we want to use on this board
        # Interrupt IDS for PIC Device
        self._excep_code = {
            "INT_SOFT_SUPER": 1,
            "INT_SOFT_MACHINE": 3,
            "INT_TIMER_SUPER": 5,
            "INT_TIMER_MACHINE": 7,
            "INT_EXT_SUPER": 9,
            "INT_EXT_MACHINE": 11,
        }
        self._int_ids = {"TTY": 0, "BLK": 1, "RNG": 2}

        # CLINT
        self.clint = Clint(pio_addr=0x2000000)

        # PLIC
        self.pic = Plic(pio_addr=0xC000000)

        # LUPIO IPI
        self.lupio_ipi = LupioIPI(
            pio_addr=0x20001000,
            int_type=self._excep_code["INT_SOFT_SUPER"],
            num_threads=self.processor.get_num_cores(),
        )

        # LUPIO PIC
        self.lupio_pic = LupioPIC(
            pio_addr=0x20002000,
            int_type=self._excep_code["INT_EXT_SUPER"],
            num_threads=self.processor.get_num_cores(),
        )

        # LupV Platform
        self.lupv = LupV(pic=self.lupio_pic, uart_int_id=self._int_ids["TTY"])

        # LUPIO BLK
        self.lupio_blk = LupioBLK(
            pio_addr=0x20000000,
            platform=self.lupv,
            int_id=self._int_ids["BLK"],
        )

        # LUPIO RNG
        self.lupio_rng = LupioRNG(
            pio_addr=0x20003000,
            platform=self.lupv,
            int_id=self._int_ids["RNG"],
        )

        # LUPIO RTC
        self.lupio_rtc = LupioRTC(pio_addr=0x20004000)

        # LUPIO SYS
        self.lupio_sys = LupioSYS(pio_addr=0x20005000)
        # LUPIO TMR
        self.lupio_tmr = LupioTMR(
            pio_addr=0x20006000,
            int_type=self._excep_code["INT_TIMER_SUPER"],
            num_threads=self.processor.get_num_cores(),
        )

        # LUPIO TTY
        self.lupio_tty = LupioTTY(
            pio_addr=0x20007000,
            platform=self.lupv,
            int_id=self._int_ids["TTY"],
        )
        self.terminal = Terminal()

        pic_srcs = [
            self._int_ids["TTY"],
            self._int_ids["BLK"],
            self._int_ids["RNG"],
        ]

        # Set the number of sources to the PIC as 0 because we've removed the
        # connections from all the external devices to the PIC, and moved them
        # to the LupioPIC.  The PIC and CLINT only remain on the board at this
        # point for our bbl to use upon startup, and will
        # remain unused during the simulation
        self.pic.n_src = 0
        self.pic.n_contexts = 0
        self.lupio_pic.n_src = max(pic_srcs) + 1
        self.lupio_pic.num_threads = self.processor.get_num_cores()

        self.lupio_tmr.num_threads = self.processor.get_num_cores()
        self.clint.num_threads = self.processor.get_num_cores()

        # Add the RTC
        # TODO: Why 100MHz? Does something else need to change when this does?
        self.rtc = RiscvRTC(frequency=Frequency("100MHz"))
        self.clint.int_pin = self.rtc.int_pin

        # Incoherent I/O bus
        self.iobus = IOXBar()

        self._on_chip_devices = [
            self.clint,
            self.pic,
            self.lupio_ipi,
            self.lupio_pic,
            self.lupio_tmr,
        ]
        self._off_chip_devices = [
            self.lupio_blk,
            self.lupio_tty,
            self.lupio_sys,
            self.lupio_rng,
            self.lupio_rtc,
        ]

    def _setup_io_devices(self) -> None:
        """Connect the I/O devices to the I/O bus"""
        for device in self._off_chip_devices:
            device.pio = self.iobus.mem_side_ports
        self.lupio_blk.dma = self.iobus.cpu_side_ports

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

    def _setup_pma(self) -> None:
        """Set the PMA devices on each core"""
        uncacheable_range = [
            AddrRange(dev.pio_addr, size=dev.pio_size)
            for dev in self._on_chip_devices + self._off_chip_devices
        ]
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
            "The LupvBoard does not have DMA Ports. "
            "Use `has_dma_ports()` to check this."
        )

    @overrides(AbstractSystemBoard)
    def has_io_bus(self) -> bool:
        return True

    @overrides(AbstractSystemBoard)
    def get_io_bus(self) -> IOXBar:
        return self.iobus

    def has_coherent_io(self) -> bool:
        return True

    def get_mem_side_coherent_io_port(self) -> Port:
        return self.iobus.mem_side_ports

    @overrides(AbstractSystemBoard)
    def _setup_memory_ranges(self):
        memory = self.get_memory()
        mem_size = memory.get_size()
        self.mem_ranges = [AddrRange(start=0x80000000, size=mem_size)]
        memory.set_memory_range(self.mem_ranges)

    def _generate_device_tree(self, outdir: str) -> None:
        """Creates the dtb and dts files.
        Creates two files in the outdir: 'device.dtb' and 'device.dts'
        :param outdir: Directory to output the files
        """
        state = FdtState(addr_cells=2, size_cells=2, cpu_cells=1)
        root = FdtNode("/")
        root.append(state.addrCellsProperty())
        root.append(state.sizeCellsProperty())
        root.appendCompatible(["luplab,lupv"])

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
            node.append(FdtPropertyStrings("riscv,isa", "rv64imafdcsu"))
            # TODO: Should probably get this from the core.
            freq = self.clk_domain.clock[0].frequency
            node.appendCompatible(["riscv"])
            int_phandle = state.phandle(f"cpu@{i}.int_state")
            int_node = FdtNode("interrupt-controller")
            int_state = FdtState(interrupt_cells=1)
            int_phandle = state.phandle(f"cpu@{i}.int_state")
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
        clint = self.clint
        clint_node = clint.generateBasicPioDeviceNode(
            soc_state, "clint", clint.pio_addr, clint.pio_size
        )

        clint_node.append(FdtPropertyStrings("status", "disable"))

        int_extended = list()
        for i, core in enumerate(self.get_processor().get_cores()):
            phandle = state.phandle(f"cpu@{i}.int_state")
            int_extended.append(phandle)
            int_extended.append(self._excep_code["INT_SOFT_MACHINE"])
            int_extended.append(phandle)
            int_extended.append(self._excep_code["INT_TIMER_MACHINE"])
        clint_node.append(
            FdtPropertyWords("interrupts-extended", int_extended)
        )
        clint_node.appendCompatible(["riscv,clint0"])
        soc_node.append(clint_node)

        # Clock
        clk_node = FdtNode("lupv-clk")
        clk_phandle = state.phandle(clk_node)
        clk_node.append(FdtPropertyWords("phandle", [clk_phandle]))
        clk_node.append(FdtPropertyWords("clock-frequency", [100000000]))
        clk_node.append(FdtPropertyWords("#clock-cells", [0]))
        clk_node.appendCompatible(["fixed-clock"])
        root.append(clk_node)

        # LupioTMR
        lupio_tmr = self.lupio_tmr
        lupio_tmr_node = lupio_tmr.generateBasicPioDeviceNode(
            soc_state, "lupio-tmr", lupio_tmr.pio_addr, lupio_tmr.pio_size
        )
        int_state = FdtState(addr_cells=0, interrupt_cells=1)
        lupio_tmr_node.append(FdtPropertyWords("clocks", [clk_phandle]))
        int_extended = list()
        for i, core in enumerate(self.get_processor().get_cores()):
            phandle = state.phandle(f"cpu@{i}.int_state")
            int_extended.append(phandle)
            int_extended.append(self._excep_code["INT_TIMER_SUPER"])
        lupio_tmr_node.append(
            FdtPropertyWords("interrupts-extended", int_extended)
        )
        lupio_tmr_node.appendCompatible(["lupio,tmr"])
        soc_node.append(lupio_tmr_node)

        # PLIC node
        plic = self.pic
        plic_node = plic.generateBasicPioDeviceNode(
            soc_state, "plic", plic.pio_addr, plic.pio_size
        )

        plic_node.append(FdtPropertyStrings("status", "disable"))

        int_state = FdtState(interrupt_cells=1)
        plic_node.append(int_state.interruptCellsProperty())

        phandle = int_state.phandle(plic)
        plic_node.append(FdtPropertyWords("phandle", [phandle]))
        plic_node.append(FdtPropertyWords("riscv,ndev", 0))

        int_extended = list()
        for i, core in enumerate(self.get_processor().get_cores()):
            phandle = state.phandle(f"cpu@{i}.int_state")
            int_extended.append(phandle)
            int_extended.append(self._excep_code["INT_EXT_MACHINE"])

        plic_node.append(FdtPropertyWords("interrupts-extended", int_extended))
        plic_node.append(FdtProperty("interrupt-controller"))
        plic_node.appendCompatible(["riscv,plic0"])

        soc_node.append(plic_node)

        # LupioIPI Device
        lupio_ipi = self.lupio_ipi
        lupio_ipi_node = lupio_ipi.generateBasicPioDeviceNode(
            soc_state, "lupio-ipi", lupio_ipi.pio_addr, lupio_ipi.pio_size
        )
        int_extended = list()
        for i, core in enumerate(self.get_processor().get_cores()):
            phandle = state.phandle(f"cpu@{i}.int_state")
            int_extended.append(phandle)
            int_extended.append(self._excep_code["INT_SOFT_SUPER"])
        lupio_ipi_node.append(
            FdtPropertyWords("interrupts-extended", int_extended)
        )
        lupio_ipi_node.append(FdtProperty("interrupt-controller"))
        lupio_ipi_node.appendCompatible(["lupio,ipi"])
        soc_node.append(lupio_ipi_node)

        # LupioPIC Device
        lupio_pic = self.lupio_pic
        lupio_pic_node = lupio_pic.generateBasicPioDeviceNode(
            soc_state, "lupio-pic", lupio_pic.pio_addr, lupio_pic.pio_size
        )
        int_state = FdtState(interrupt_cells=1)
        lupio_pic_node.append(int_state.interruptCellsProperty())
        phandle = state.phandle(lupio_pic)
        lupio_pic_node.append(FdtPropertyWords("phandle", [phandle]))
        int_extended = list()
        for i, core in enumerate(self.get_processor().get_cores()):
            phandle = state.phandle(f"cpu@{i}.int_state")
            int_extended.append(phandle)
            int_extended.append(self._excep_code["INT_EXT_SUPER"])
        lupio_pic_node.append(
            FdtPropertyWords("interrupts-extended", int_extended)
        )
        lupio_pic_node.append(FdtProperty("interrupt-controller"))
        lupio_pic_node.appendCompatible(["lupio,pic"])
        soc_node.append(lupio_pic_node)

        # LupioBLK Device
        lupio_blk = self.lupio_blk
        lupio_blk_node = lupio_blk.generateBasicPioDeviceNode(
            soc_state, "lupio-blk", lupio_blk.pio_addr, lupio_blk.pio_size
        )
        lupio_blk_node.appendCompatible(["lupio,blk"])
        lupio_blk_node.append(
            FdtPropertyWords(
                "interrupts-extended",
                [state.phandle(self.lupio_pic), self.lupio_blk.int_id],
            )
        )
        soc_node.append(lupio_blk_node)

        # LupioRNG Device
        lupio_rng = self.lupio_rng
        lupio_rng_node = lupio_rng.generateBasicPioDeviceNode(
            soc_state, "lupio-rng", lupio_rng.pio_addr, lupio_rng.pio_size
        )
        lupio_rng_node.appendCompatible(["lupio,rng"])
        lupio_rng_node.append(
            FdtPropertyWords(
                "interrupts-extended",
                [state.phandle(self.lupio_pic), self.lupio_rng.int_id],
            )
        )
        soc_node.append(lupio_rng_node)

        # LupioSYS Device
        lupio_sys = self.lupio_sys
        lupio_sys_node = lupio_sys.generateBasicPioDeviceNode(
            soc_state, "lupio-sys", lupio_sys.pio_addr, lupio_sys.pio_size
        )
        lupio_sys_node.appendCompatible(["syscon"])
        sys_phandle = state.phandle(self.lupio_sys)
        lupio_sys_node.append(FdtPropertyWords("phandle", [sys_phandle]))
        soc_node.append(lupio_sys_node)

        poweroff_node = FdtNode("poweroff")
        poweroff_node.appendCompatible(["syscon-poweroff"])
        poweroff_node.append(FdtPropertyWords("regmap", [sys_phandle]))
        poweroff_node.append(FdtPropertyWords("offset", [0x0]))
        poweroff_node.append(FdtPropertyWords("value", [1]))
        soc_node.append(poweroff_node)

        reboot_node = FdtNode("reboot")
        reboot_node.appendCompatible(["syscon-reboot"])
        reboot_node.append(FdtPropertyWords("regmap", [sys_phandle]))
        reboot_node.append(FdtPropertyWords("offset", [0x4]))
        reboot_node.append(FdtPropertyWords("value", [1]))
        soc_node.append(reboot_node)

        # LupioRTC Device
        lupio_rtc = self.lupio_rtc
        lupio_rtc_node = lupio_rtc.generateBasicPioDeviceNode(
            soc_state, "lupio-rtc", lupio_rtc.pio_addr, lupio_rtc.pio_size
        )
        lupio_rtc_node.appendCompatible(["lupio,rtc"])
        soc_node.append(lupio_rtc_node)

        # LupioTTY Device
        lupio_tty = self.lupio_tty
        lupio_tty_node = lupio_tty.generateBasicPioDeviceNode(
            soc_state, "lupio-tty", lupio_tty.pio_addr, lupio_tty.pio_size
        )
        lupio_tty_node.appendCompatible(["lupio,tty"])
        lupio_tty_node.append(
            FdtPropertyWords(
                "interrupts-extended",
                [state.phandle(self.lupio_pic), self.lupio_tty.int_id],
            )
        )
        soc_node.append(lupio_tty_node)

        root.append(soc_node)
        fdt = Fdt()
        fdt.add_rootnode(root)
        fdt.writeDtsFile(os.path.join(outdir, "device.dts"))
        fdt.writeDtbFile(os.path.join(outdir, "device.dtb"))

    @overrides(KernelDiskWorkload)
    def get_disk_device(self) -> str:
        return "/dev/lda"

    @overrides(KernelDiskWorkload)
    def get_default_kernel_args(self) -> List[str]:
        return [
            "console=ttyLIO0",
            "root={root_value}",
            "disk_device={disk_device}",
            "rw",
        ]

    @overrides(KernelDiskWorkload)
    def _add_disk_to_board(self, disk_image: AbstractResource) -> None:
        # Note: This must be called after set_workload because it looks for an
        # attribute named "disk" and connects

        # Set the disk image for the block device to use
        image = CowDiskImage(
            child=RawDiskImage(read_only=True), read_only=False
        )
        image.child.image_file = disk_image.get_local_path()
        self.lupio_blk.image = image

        self._setup_io_devices()
        self._setup_pma()

        # Default DTB address if bbl is built with --with-dts option
        self.workload.dtb_addr = 0x87E00000

        # We need to wait to generate the device tree until after the disk is
        # set up. Now that the disk and workload are set, we can generate the
        # device tree file.
        self._generate_device_tree(m5.options.outdir)
        self.workload.dtb_filename = os.path.join(
            m5.options.outdir, "device.dtb"
        )
