# Copyright (c) 2022 The Regents of the University of California
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

from m5.objects import (
    Port,
    IOXBar,
    Bridge,
    BadAddr,
    Terminal,
    PciVirtIO,
    VncServer,
    AddrRange,
    ArmSystem,
    ArmRelease,
    ArmFsLinux,
    VirtIOBlock,
    CowDiskImage,
    RawDiskImage,
    VoltageDomain,
    SrcClockDomain,
    ArmDefaultRelease,
    VExpress_GEM5_Base,
    VExpress_GEM5_Foundation,
)

import os
import m5
from abc import ABCMeta
from ...isas import ISA
from typing import List
from m5.util import fatal
from ...utils.requires import requires
from ...utils.override import overrides
from .abstract_board import AbstractBoard
from ...resources.resource import AbstractResource
from .kernel_disk_workload import KernelDiskWorkload
from ..cachehierarchies.classic.no_cache import NoCache
from ..processors.abstract_processor import AbstractProcessor
from ..memory.abstract_memory_system import AbstractMemorySystem
from ..cachehierarchies.abstract_cache_hierarchy import AbstractCacheHierarchy


class ArmBoard(ArmSystem, AbstractBoard, KernelDiskWorkload):
    """
    A board capable of full system simulation for ARM instructions. It is based
    ARMv8.

    The board is based on Arm Motherboard Express uATX (V2M-P1), Arm
    CoreTile Express A15x2 (V2P-CA15) and on Armv8-A FVP Foundation platform
    v11.8, depending on the simulated platform. These boards are parts of ARM's
    Versatile(TM) Express family of boards.

    **Limitations**
    * The board currently does not support ruby caches.
    * stage2 walker ports are ignored.
    * This version does not support SECURITY extension.
    """

    __metaclass__ = ABCMeta

    def __init__(
        self,
        clk_freq: str,
        processor: AbstractProcessor,
        memory: AbstractMemorySystem,
        cache_hierarchy: AbstractCacheHierarchy,
        platform: VExpress_GEM5_Base = VExpress_GEM5_Foundation(),
        release: ArmRelease = ArmDefaultRelease(),
    ) -> None:
        super().__init__()
        AbstractBoard.__init__(
            self,
            clk_freq=clk_freq,
            processor=processor,
            memory=memory,
            cache_hierarchy=cache_hierarchy,
        )

        # This board requires ARM ISA to work.

        requires(isa_required=ISA.ARM)

        # Setting the voltage domain here.

        self.voltage_domain = self.clk_domain.voltage_domain

        # Setting up ARM release here. We use the ARM default release, which
        # corresponds to an ARMv8 system.

        self.release = release

        # RealView sets up most of the on-chip and off-chip devices and GIC
        # for the ARM board. These devices' iformation is also used to
        # generate the dtb file.

        self._setup_realview(platform)

        # ArmBoard's memory can only be setup once realview is initialized.

        self._setup_arm_memory_ranges()

        # Setting multi_proc of ArmSystem by counting the number of processors.

        if processor.get_num_cores() != 1:
            self.multi_proc = False
        else:
            self.multi_proc = True

    @overrides(AbstractBoard)
    def _setup_board(self) -> None:

        # This board is expected to run full-system simulation.
        # Loading ArmFsLinux() from `src/arch/arm/ArmFsWorkload.py`

        self.workload = ArmFsLinux()

        # We are fixing the following variable for the ArmSystem to work. The
        # security extension is checked while generating the dtb file in
        # realview. This board does not have security extention enabled.

        self._have_psci = False

        # highest_el_is_64 is set to True. True if the register width of the
        # highest implemented exception level is 64 bits.

        self.highest_el_is_64 = True

        # Setting up the voltage and the clock domain here for the ARM board.
        # The ArmSystem/RealView expects voltage_domain to be a parameter.
        # The voltage and the clock frequency are taken from the devices.py
        # file from configs/example/arm

        self.voltage_domain = VoltageDomain(voltage="1.0V")
        self.clk_domain = SrcClockDomain(
            clock="1GHz", voltage_domain=self.voltage_domain
        )

        # The ARM board supports both Terminal and VncServer.

        self.terminal = Terminal()
        self.vncserver = VncServer()

        # Incoherent I/O Bus

        self.iobus = IOXBar()
        self.iobus.badaddr_responder = BadAddr()
        self.iobus.default = self.iobus.badaddr_responder.pio

    def _setup_io_devices(self) -> None:
        """
        This method connects the I/O devices to the I/O bus.
        """

        # We setup the iobridge for the ARM Board. The default
        # cache_hierarchy's NoCache class has an iobridge has a latency of
        # 10. We are using an iobridge with latency = 50ns, taken from the
        # configs/example/arm/devices.py

        self.iobridge = Bridge(delay="50ns")
        self.iobridge.mem_side_port = self.iobus.cpu_side_ports
        self.iobridge.cpu_side_port = self.cache_hierarchy.get_mem_side_port()

        # We either have iocache or dmabridge depending upon the
        # cache_hierarchy. If we have "NoCache", then we use the dmabridge.
        # Otherwise, we use the iocache on the board.

        if isinstance(self.cache_hierarchy, NoCache) is False:

            # The ArmBoard does not support ruby caches.

            if self.get_cache_hierarchy().is_ruby():
                fatal("Ruby caches are not supported by the ArmBoard.")

            # The classic caches are setup in the  _setup_io_cache() method,
            # defined under the cachehierarchy class. Verified it with both
            # PrivateL1PrivateL2CacheHierarchy and PrivateL1CacheHierarchy
            # classes.

        else:

            # This corresponds to a machine without caches. We have a DMA
            # beidge in this case. Parameters of this bridge are also taken
            # from the common/example/arm/devices.py file.

            self.dmabridge = Bridge(delay="50ns", ranges=self.mem_ranges)

            self.dmabridge.mem_side_port = self.get_dma_ports()[0]
            self.dmabridge.cpu_side_port = self.get_dma_ports()[1]

        self.realview.attachOnChipIO(
            self.cache_hierarchy.membus, self.iobridge
        )
        self.realview.attachIO(self.iobus)

    def _setup_realview(self, platform) -> None:
        """
        Notes:
        The ARM Board has realview platform. Most of the on-chip and
        off-chip devices are setup by the RealView platform. Currently, there
        are 5 different types of realview platforms supported by the ArmBoard.

        :param platform: the user can specify the platform while instantiating
        an ArmBoard object.
        """

        # Currently, the ArmBoard supports VExpress_GEM5_V1,
        # VExpress_GEM5_V1_HDLcd and VExpress_GEM5_Foundation.
        # VExpress_GEM5_V2 and VExpress_GEM5_V2_HDLcd are not supported by the
        # ArmBoard.

        self.realview = platform

        # We need to setup the global interrupt controller (GIC) addr for the
        # realview system.

        if hasattr(self.realview.gic, "cpu_addr"):
            self.gic_cpu_addr = self.realview.gic.cpu_addr

    def _setup_io_cache(self):
        pass

    @overrides(AbstractBoard)
    def has_io_bus(self) -> bool:
        return True

    @overrides(AbstractBoard)
    def get_io_bus(self) -> IOXBar:
        return [self.iobus.cpu_side_ports, self.iobus.mem_side_ports]

    @overrides(AbstractBoard)
    def has_coherent_io(self) -> bool:
        return True

    @overrides(AbstractBoard)
    def get_mem_side_coherent_io_port(self) -> Port:
        return self.iobus.mem_side_ports

    @overrides(AbstractBoard)
    def has_dma_ports(self) -> bool:
        return True

    def _setup_coherent_io_bridge(self, board: AbstractBoard) -> None:
        pass

    @overrides(AbstractBoard)
    def get_dma_ports(self) -> List[Port]:
        return [
            self.cache_hierarchy.get_cpu_side_port(),
            self.iobus.mem_side_ports,
        ]

    @overrides(AbstractBoard)
    def connect_system_port(self, port: Port) -> None:
        self.system_port = port

    @overrides(KernelDiskWorkload)
    def get_disk_device(self):
        return "/dev/vda"

    @overrides(KernelDiskWorkload)
    def _add_disk_to_board(self, disk_image: AbstractResource):

        # We define the image.

        image = CowDiskImage(
            child=RawDiskImage(read_only=True), read_only=False
        )

        self.pci_devices = [PciVirtIO(vio=VirtIOBlock(image=image))]
        self.realview.attachPciDevice(self.pci_devices[0], self.iobus)

        # Now that the disk and workload are set, we can generate the device
        # tree file. We will generate the dtb file everytime the board is
        # boot-up.

        image.child.image_file = disk_image.get_local_path()

        # _setup_io_devices needs to be implemented.

        self._setup_io_devices()

        # Specifying the dtb file location to the workload.

        self.workload.dtb_filename = os.path.join(
            m5.options.outdir, "device.dtb"
        )

        # Calling generateDtb from class ArmSystem to add memory information to
        # the dtb file.

        self.generateDtb(self.workload.dtb_filename)

        # Finally we need to setup the bootloader for the ArmBoard. An ARM
        # system requires three inputs to simulate a full system: a disk image,
        # the kernel file and the bootloader file(s).

        self.realview.setupBootLoader(
            self, self.workload.dtb_filename, self._bootloader
        )

    def _get_memory_ranges(self, mem_size) -> list:
        """
        This method is taken from configs/example/arm/devices.py. It sets up
        all the memory ranges for the board.
        """
        mem_ranges = []

        for mem_range in self.realview._mem_regions:
            size_in_range = min(mem_size, mem_range.size())
            mem_ranges.append(
                AddrRange(start=mem_range.start, size=size_in_range)
            )

            mem_size -= size_in_range
            if mem_size == 0:
                return mem_ranges

        raise ValueError("Memory size too big for platform capabilities")

    @overrides(AbstractBoard)
    def _setup_memory_ranges(self) -> None:
        """
        The ArmBoard's memory can only be setup after realview is setup. Once
        realview is initialized, we call _setup_arm_memory_ranges() to
        correctly setup the memory ranges.
        """
        pass

    def _setup_arm_memory_ranges(self) -> None:

        # We setup the memory here. The memory size is specified in the run
        # script that the user uses.

        memory = self.get_memory()
        mem_size = memory.get_size()

        self.mem_ranges = self._get_memory_ranges(mem_size)
        memory.set_memory_range(self.mem_ranges)

    @overrides(KernelDiskWorkload)
    def get_default_kernel_args(self) -> List[str]:

        # The default kernel string is taken from the devices.py file.

        return [
            "console=ttyAMA0",
            "lpj=19988480",
            "norandmaps",
            "root={root_value}",
            "rw",
            "mem=%s" % self.get_memory().get_size(),
        ]
