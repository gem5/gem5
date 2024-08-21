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

import m5
from m5.objects import (
    AddrRange,
    BadAddr,
    Bridge,
    CowDiskImage,
    Frequency,
    GenericRiscvPciHost,
    HiFive,
    IGbE_e1000,
    IOXBar,
    PMAChecker,
    Port,
    RawDiskImage,
    RiscvBootloaderKernelWorkload,
    RiscvMmioVirtIO,
    RiscvRTC,
    VirtIOBlock,
    VirtIORng,
)
from m5.util import warn

# from ...components.memory.single_channel import SingleChannelDDR3_1600
from gem5.components.memory import DualChannelDDR4_2400

from ...coherence_protocol import CoherenceProtocol
from ...components.boards.riscv_board import RiscvBoard
from ...components.boards.se_binary_workload import SEBinaryWorkload

# from ...components.cachehierarchies.ruby.mesi_two_level_cache_hierarchy import (
#     MESITwoLevelCacheHierarchy,
# )
from ...components.cachehierarchies.classic.private_l1_private_l2_walk_cache_hierarchy import (
    PrivateL1PrivateL2WalkCacheHierarchy,
)
from ...components.processors.cpu_types import CPUTypes
from ...components.processors.simple_processor import SimpleProcessor
from ...isas import ISA
from ...resources.resource import AbstractResource
from ...utils.override import overrides
from ...utils.requires import requires


class RiscvDemoBoard(RiscvBoard, SEBinaryWorkload):
    """
    This board is based on the X86DemoBoard.

    This prebuilt RISCV board is used for demonstration purposes. It simulates
    an RISCV 3GHz quad-core system with a 2GB DDR3_1600 memory system. A
    MESI_Two_Level cache hierarchy is set with an l1 data and instruction
    cache, each 32kB with an associativity of 8, and a single bank l2 cache of
    1MB with an associativity of 16.

    **DISCLAIMER**: This board is solely for demonstration purposes. This board
    is not known to be representative of any real-world system or produce
    reliable statistical results.

    """

    def __init__(self, is_fs):
        requires(
            isa_required=ISA.RISCV,
            # coherence_protocol_required=CoherenceProtocol.MESI_TWO_LEVEL,
        )

        warn(
            "The RiscvDemoBoard is solely for demonstration purposes. "
            "This board is not known to be be representative of any "
            "real-world system. Use with caution."
        )
        self._fs = is_fs

        # memory = SingleChannelDDR3_1600(size="2GiB")
        memory = DualChannelDDR4_2400(size="3GiB")

        processor = SimpleProcessor(
            # cpu_type=CPUTypes.TIMING, isa=ISA.RISCV, num_cores=4
            cpu_type=CPUTypes.TIMING,
            isa=ISA.RISCV,
            num_cores=2,
        )
        # cache_hierarchy = MESITwoLevelCacheHierarchy(
        #     l1d_size="32KiB",
        #     l1d_assoc=8,
        #     l1i_size="32KiB",
        #     l1i_assoc=8,
        #     l2_size="1MiB",
        #     l2_assoc=16,
        #     num_l2_banks=1,
        # )
        # Here we setup the parameters of the l1 and l2 caches.
        cache_hierarchy = PrivateL1PrivateL2WalkCacheHierarchy(
            l1d_size="16KiB", l1i_size="16KiB", l2_size="256KiB"
        )

        super().__init__(
            clk_freq="3GHz",
            processor=processor,
            memory=memory,
            cache_hierarchy=cache_hierarchy,
        )

    # Taken from riscv matched board. Below are functions that are needed to get
    # SE mode to work.
    @overrides(RiscvBoard)
    def _pre_instantiate(self):
        if self._fs:
            if len(self._bootloader) > 0:
                self.workload.bootloader_addr = 0x0
                self.workload.bootloader_filename = self._bootloader[0]
                self.workload.kernel_addr = 0x80200000
                self.workload.entry_point = (
                    0x80000000  # Bootloader starting point
                )
            else:
                self.workload.kernel_addr = 0x0
                self.workload.entry_point = 0x80000000

        self._connect_things()

    # @overrides(RiscvBoard)
    # def _add_disk_to_board(self, disk_image: AbstractResource):
    #     image = CowDiskImage(
    #         child=RawDiskImage(read_only=True), read_only=False
    #     )
    #     image.child.image_file = disk_image.get_local_path()
    #     self.disk.vio.image = image

    #     # Note: The below is a bit of a hack. We need to wait to generate the
    #     # device tree until after the disk is set up. Now that the disk and
    #     # workload are set, we can generate the device tree file.
    #     self._setup_io_devices()
    #     self._setup_pma()

    #     # Default DTB address if bbl is built with --with-dts option
    #     self.workload.dtb_addr = 0x87E00000

    #     self.generate_device_tree(m5.options.outdir)
    #     self.workload.dtb_filename = os.path.join(
    #         m5.options.outdir, "device.dtb"
    #     )

    @overrides(RiscvBoard)
    def _setup_board(self) -> None:
        if self._fs:
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

    @overrides(RiscvBoard)
    def has_io_bus(self) -> bool:
        return self._fs

    @overrides(RiscvBoard)
    def get_io_bus(self) -> IOXBar:
        if self._fs:
            return self.iobus
        else:
            raise NotImplementedError(
                "RiscvDemoBoard does not have an IO bus. "
                "Use `has_io_bus()` to check this."
            )

    @overrides(RiscvBoard)
    def has_coherent_io(self) -> bool:
        return self._fs

    @overrides(RiscvBoard)
    def get_mem_side_coherent_io_port(self) -> Port:
        if self._fs:
            return self.iobus.mem_side_ports
        else:
            raise NotImplementedError(
                "RiscvDemoBoard does not have any I/O ports. Use has_coherent_io to "
                "check this."
            )

    # def _setup_io_devices(self) -> None:
    #     """Connect the I/O devices to the I/O bus in FS mode."""
    #     if self._fs:
    #         # Add PCI
    #         self.platform.pci_host.pio = self.iobus.mem_side_ports

    #         # Add Ethernet card
    #         self.ethernet = IGbE_e1000(
    #             pci_bus=0,
    #             pci_dev=0,
    #             pci_func=0,
    #             InterruptLine=1,
    #             InterruptPin=1,
    #         )

    #         self.ethernet.host = self.platform.pci_host
    #         self.ethernet.pio = self.iobus.mem_side_ports
    #         self.ethernet.dma = self.iobus.cpu_side_ports

    #         if self.get_cache_hierarchy().is_ruby():
    #             for device in self._off_chip_devices + self._on_chip_devices:
    #                 device.pio = self.iobus.mem_side_ports

    #         else:
    #             for device in self._off_chip_devices:
    #                 device.pio = self.iobus.mem_side_ports
    #             for device in self._on_chip_devices:
    #                 device.pio = self.get_cache_hierarchy().get_mem_side_port()

    #             self.bridge = Bridge(delay="10ns")
    #             self.bridge.mem_side_port = self.iobus.cpu_side_ports
    #             self.bridge.cpu_side_port = (
    #                 self.get_cache_hierarchy().get_mem_side_port()
    #             )
    #             self.bridge.ranges = [
    #                 AddrRange(dev.pio_addr, size=dev.pio_size)
    #                 for dev in self._off_chip_devices
    #             ]

    #             # PCI
    #             self.bridge.ranges.append(AddrRange(0x2F000000, size="16MB"))
    #             self.bridge.ranges.append(AddrRange(0x30000000, size="256MB"))
    #             self.bridge.ranges.append(AddrRange(0x40000000, size="512MB"))
