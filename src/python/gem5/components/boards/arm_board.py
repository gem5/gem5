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

import os
from abc import ABCMeta
from typing import (
    List,
    Sequence,
    Tuple,
)

import m5
from m5.objects import (
    AddrRange,
    ArmDefaultRelease,
    ArmFsLinux,
    ArmRelease,
    ArmSystem,
    BadAddr,
    Bridge,
    CowDiskImage,
    GenericTimer,
    IOXBar,
    PciVirtIO,
    Port,
    RawDiskImage,
    SimObject,
    SrcClockDomain,
    Terminal,
    VExpress_GEM5_Base,
    VExpress_GEM5_Foundation,
    VExpress_GEM5_V1,
    VirtIOBlock,
    VncServer,
    VoltageDomain,
)

from ...isas import ISA
from ...resources.resource import AbstractResource
from ...utils.override import overrides
from ...utils.requires import requires
from ..cachehierarchies.abstract_cache_hierarchy import AbstractCacheHierarchy
from ..cachehierarchies.classic.no_cache import NoCache
from ..memory.abstract_memory_system import AbstractMemorySystem
from ..processors.abstract_processor import AbstractProcessor
from .abstract_board import AbstractBoard
from .kernel_disk_workload import KernelDiskWorkload


class ArmBoard(ArmSystem, AbstractBoard, KernelDiskWorkload):
    """
    A board capable of full system simulation for ARM instructions. It is based
    ARMv8.

    The board is based on Arm Motherboard Express uATX (V2M-P1), Arm
    CoreTile Express A15x2 (V2P-CA15) and on Armv8-A FVP Foundation platform
    v11.8, depending on the simulated platform. These boards are parts of ARM's
    Versatile(TM) Express family of boards.

    **Limitations**
    * stage2 walker ports are ignored.
    * KVM cores only work with VExpress_GEM5_V1
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
        # The platform and the clk has to be set before calling the super class
        self._platform = platform
        self._clk_freq = clk_freq

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

        # Setting up ARM release here. We use the ARM default release, which
        # corresponds to an ARMv8 system.
        self.release = release

        # Setting multi_proc of ArmSystem by counting the number of processors.
        if processor.get_num_cores() == 1:
            self.multi_proc = False
        else:
            self.multi_proc = True

    @overrides(AbstractBoard)
    def _set_fullsystem(self, is_fs: bool) -> None:
        self._is_fs = is_fs

        if self._is_fs:
            # This board is expected to run full-system simulation.
            # Loading ArmFsLinux() from `src/arch/arm/ArmFsWorkload.py`
            self.workload = ArmFsLinux()

    @overrides(AbstractBoard)
    def _setup_board(self) -> None:
        # We are fixing the following variable for the ArmSystem to work. The
        # security extension is checked while generating the dtb file in
        # realview. This board does not have security extension enabled.
        self._have_psci = False

        # highest_el_is_64 is set to True. True if the register width of the
        # highest implemented exception level is 64 bits.
        self.highest_el_is_64 = True

        # Setting up the voltage and the clock domain here for the ARM board.
        # The ArmSystem/RealView expects voltage_domain to be a parameter.
        # The voltage and the clock frequency are taken from the devices.py
        # file from configs/example/arm. We set the clock to the same frequency
        # as the user specified in the config script.
        self.voltage_domain = VoltageDomain(voltage="1.0V")
        self.clk_domain = SrcClockDomain(
            clock=self._clk_freq, voltage_domain=self.voltage_domain
        )

        # The ARM board supports both Terminal and VncServer.
        self.terminal = Terminal()
        self.vncserver = VncServer()

        # Incoherent I/O Bus
        self.iobus = IOXBar()
        self.iobus.badaddr_responder = BadAddr()
        self.iobus.default = self.iobus.badaddr_responder.pio

        # We now need to setup the dma_ports.
        self._dma_ports = None

        # RealView sets up most of the on-chip and off-chip devices and GIC
        # for the ARM board. These devices' information is also used to
        # generate the dtb file. We then connect the I/O devices to the
        # I/O bus.
        self._setup_io_devices()

        # Once the realview is setup, we can continue setting up the memory
        # ranges. ArmBoard's memory can only be setup once realview is
        # initialized.
        memory = self.get_memory()
        mem_size = memory.get_size()

        # The following code is taken from configs/example/arm/devices.py. It
        # sets up all the memory ranges for the board.
        self.mem_ranges = []
        success = False
        for mem_range in self.realview._mem_regions:
            size_in_range = min(mem_size, mem_range.size())
            self.mem_ranges.append(
                AddrRange(start=mem_range.start, size=size_in_range)
            )

            mem_size -= size_in_range
            if mem_size == 0:
                success = True
                break

        if success:
            memory.set_memory_range(self.mem_ranges)
        else:
            raise ValueError("Memory size too big for platform capabilities")

        # The PCI Devices. PCI devices can be added via the `_add_pci_device`
        # function.
        self._pci_devices = []

    def _setup_io_devices(self) -> None:
        """
        This method first sets up the platform. ARM uses ``realview`` platform.
        Most of the on-chip and off-chip devices are setup by the realview
        platform. Once realview is setup, we connect the I/O devices to the
        I/O bus.
        """

        # Currently, the ArmBoard supports VExpress_GEM5_V1,
        # VExpress_GEM5_V1_HDLcd and VExpress_GEM5_Foundation.
        # VExpress_GEM5_V2 and VExpress_GEM5_V2_HDLcd are not supported by the
        # ArmBoard.
        self.realview = self._platform

        # We need to setup the global interrupt controller (GIC) addr for the
        # realview system.
        if hasattr(self.realview.gic, "cpu_addr"):
            self.gic_cpu_addr = self.realview.gic.cpu_addr

        # For KVM cpus, we need to simulate the GIC.
        if any(core.is_kvm_core() for core in self.processor.get_cores()):
            # The following is taken from
            # `tests/fs/linux/arm/configs/arm_generic.py`:
            # Arm KVM regressions will use a simulated GIC. This means that in
            # order to work we need to remove the system interface of the
            # generic timer from the DTB and we need to inform the MuxingKvmGic
            # class to use the gem5 GIC instead of relying on the host one
            GenericTimer.generateDeviceTree = SimObject.generateDeviceTree
            self.realview.gic.simulate_gic = True

        # IO devices has to setup before incorporating the caches in the case
        # of ruby caches. Otherwise the DMA controllers are incorrectly
        # created. The IO device has to be attached first. This is done in the
        # realview class.
        if self.get_cache_hierarchy().is_ruby():
            # All the on-chip devices are attached in this method.
            self.realview.attachOnChipIO(
                self.iobus,
                dma_ports=self.get_dma_ports(),
                mem_ports=self.get_memory().get_mem_ports(),
            )
            self.realview.attachIO(self.iobus, dma_ports=self.get_dma_ports())

        else:
            # We either have iocache or dmabridge depending upon the
            # cache_hierarchy. If we have "NoCache", then we use the dmabridge.
            # Otherwise, we use the iocache on the board.

            # We setup the iobridge for the ARM Board. The default
            # cache_hierarchy's NoCache class has an iobridge has a latency
            # of 10. We are using an iobridge with latency = 50ns, taken
            # from the configs/example/arm/devices.py.
            self.iobridge = Bridge(delay="50ns")
            self.iobridge.mem_side_port = self.iobus.cpu_side_ports
            self.iobridge.cpu_side_port = (
                self.cache_hierarchy.get_mem_side_port()
            )

            if isinstance(self.cache_hierarchy, NoCache) is True:
                # This corresponds to a machine without caches. We have a DMA
                # bridge in this case. Parameters of this bridge are also taken
                # from the common/example/arm/devices.py file.
                self.dmabridge = Bridge(delay="50ns", ranges=self.mem_ranges)
                self.dmabridge.mem_side_port = (
                    self.cache_hierarchy.get_cpu_side_port()
                )
                self.dmabridge.cpu_side_port = self.iobus.mem_side_ports

            # The classic caches are setup in the  _setup_io_cache() method
            # defined under the cachehierarchy class. Verified it with both
            # PrivateL1PrivateL2CacheHierarchy and PrivateL1CacheHierarchy
            # classes.
            self.realview.attachOnChipIO(
                self.cache_hierarchy.membus, self.iobridge
            )
            self.realview.attachIO(self.iobus)

    @overrides(AbstractBoard)
    def get_mem_ports(self) -> Sequence[Tuple[AddrRange, Port]]:
        all_ports = [
            (self.realview.bootmem.range, self.realview.bootmem.port),
        ] + self.get_memory().get_mem_ports()

        return all_ports

    @overrides(AbstractBoard)
    def has_io_bus(self) -> bool:
        return True

    @overrides(AbstractBoard)
    def get_io_bus(self) -> IOXBar:
        return self.iobus

    @overrides(AbstractBoard)
    def has_coherent_io(self) -> bool:
        # The setup of the caches gets a little tricky here. We need to
        # override the default cache_hierarchy.iobridge due to different delay
        # values (see method _setup_io_devices()). One way to do it would be to
        # prevent creating cache_hierarchy.iobridge altogether. We trick
        # NoCache() to assume that this board has no coherent_io and we we
        # simply setup our own iobridge in the _setup_io_devices() method.
        if isinstance(self.cache_hierarchy, NoCache):
            return False
        # In all other cases, we use the default values setup in the
        # respective cache hierarchy class.
        return True

    @overrides(AbstractBoard)
    def get_mem_side_coherent_io_port(self) -> Port:
        return self.iobus.mem_side_ports

    @overrides(AbstractBoard)
    def has_dma_ports(self) -> bool:
        return True

    @overrides(AbstractBoard)
    def get_dma_ports(self) -> List[Port]:
        # The DMA ports differ depending upon the cache hierarchy. The method
        # self.set_dma_ports takes care of that. In the case of ruby caches,
        # this method should initially return an empty list.
        if self.cache_hierarchy.is_ruby():
            if self._dma_ports is None:
                self._dma_ports = []

        # _dma_ports should always be empty for classic caches.
        return self._dma_ports

    @overrides(AbstractBoard)
    def connect_system_port(self, port: Port) -> None:
        self.system_port = port

    @overrides(AbstractBoard)
    def _pre_instantiate(self):
        super()._pre_instantiate()

        # Add the PCI devices.
        self.pci_devices = self._pci_devices

        # The workload needs to know the dtb_file.
        self.workload.dtb_filename = self._get_dtb_filename()

        # Finally we need to setup the bootloader for the ArmBoard. An ARM
        # system requires three inputs to simulate a full system: a disk image,
        # the kernel file and the bootloader file(s).
        self.realview.setupBootLoader(
            self, self._get_dtb_filename(), self._bootloader
        )

        # Calling generateDtb from class ArmSystem to add memory information to
        # the dtb file.
        self.generateDtb(self._get_dtb_filename())

    def _get_dtb_filename(self) -> str:
        """Returns the ``dtb`` file location.

        .. note::

            This may be the ``_expected_`` file location when generated. A
            file may not exist at this location when this function is called.
        """

        return os.path.join(m5.options.outdir, "device.dtb")

    def _add_pci_device(self, pci_device: PciVirtIO) -> None:
        """Attaches the PCI Device to the board. All devices will be added to
        ``self.pci_device`` as a pre-instantiation setup.

        :param pci_device: The PCI Device to add.
        """
        self._pci_devices.append(pci_device)

        # For every PCI device, we need to get its dma_port so that we
        # can setup dma_controllers correctly.
        self.realview.attachPciDevice(
            pci_device, self.iobus, dma_ports=self.get_dma_ports()
        )

    @overrides(KernelDiskWorkload)
    def get_disk_device(self):
        return "/dev/vda"

    @overrides(KernelDiskWorkload)
    def _add_disk_to_board(self, disk_image: AbstractResource):
        self._image = CowDiskImage(
            child=RawDiskImage(
                read_only=True, image_file=disk_image.get_local_path()
            ),
            read_only=False,
        )

        self._add_pci_device(PciVirtIO(vio=VirtIOBlock(image=self._image)))

    @overrides(AbstractBoard)
    def _setup_memory_ranges(self) -> None:
        """
        The ArmBoard's memory can only be setup after realview is setup. We set
        this up in the ``_setup_board`` function.
        """
        pass

    @overrides(KernelDiskWorkload)
    def get_default_kernel_args(self) -> List[str]:
        # The default kernel string is taken from the devices.py file.
        return [
            "console=ttyAMA0",
            "lpj=19988480",
            "norandmaps",
            "root={root_value}",
            "disk_device={disk_device}",
            "rw",
            f"mem={self.get_memory().get_size()}",
        ]

    @overrides(SimObject)
    def createCCObject(self):
        """We override this function as it is called in ``m5.instantiate``. This
        means we can insert a check to ensure the ``_connect_things`` function
        has been run.
        """
        super()._connect_things_check()
        super().createCCObject()
