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

import inspect
from abc import (
    ABCMeta,
    abstractmethod,
)
from typing import (
    List,
    Optional,
    Sequence,
    Tuple,
)

from m5.objects import (
    AddrRange,
    ClockDomain,
    IOXBar,
    Port,
    SrcClockDomain,
    System,
    VoltageDomain,
)

from ...resources.resource import WorkloadResource
from ..modifier import Modifier
from .mem_mode import (
    MemMode,
    mem_mode_to_string,
)


class AbstractBoard:
    """The abstract board interface.

    Boards are used as the object which can connect together all other
    components. This abstract class defines the external interface that other
    boards must provide. Boards can be specialized for different ISAs or system
    designs (e.g., core counts, cache types, memory channels, I/O devices, etc).

    In addition to providing the place that system components are connected,
    the board also exposes an interface for the caches, processor, and memory
    to interact.

    The board also exposes an interface to set up I/O devices which needs to be
    specialized for each ISA and/or platform.

    Board inherits from System and can therefore be used as a System SimObject
    when required.
    """

    __metaclass__ = ABCMeta

    def __init__(
        self,
        clk_freq: str,
        processor: "AbstractProcessor",
        memory: "AbstractMemorySystem",
        cache_hierarchy: Optional["AbstractCacheHierarchy"],
    ) -> None:
        """
        :param clk_freq: The clock frequency for this board.
        :param processor: The processor for this board.
        :param memory: The memory for this board.
        :param cache_hierarchy: The Cache Hierarchy for this board.
                                In some boards caches can be optional. If so,
                                that board must override ``_connect_things``.
        """

        if not isinstance(self, System):
            raise Exception("A gem5 stdlib board must inherit from System.")

        # Set up the clock domain and the voltage domain.
        self.clk_domain = SrcClockDomain()
        self.clk_domain.clock = clk_freq
        self.clk_domain.voltage_domain = VoltageDomain()

        # Set the processor, memory, and cache hierarchy.
        self.processor = processor
        self.memory = memory
        self._cache_hierarchy = cache_hierarchy
        if cache_hierarchy is not None:
            self.cache_hierarchy = cache_hierarchy

        # This variable determines whether the board is to be executed in
        # full-system or syscall-emulation mode. This is set when the workload
        # is defined. Whether or not the board is to be run in FS mode is
        # determined by which kind of workload is set.
        self._is_fs = None

        # This variable is used to record the checkpoint directory which is
        # set when declaring the board's workload and then used by the
        # Simulator module.
        self._checkpoint = None

        # Setup the board and memory system's memory ranges.
        self._setup_memory_ranges()

        # Setup board properties unique to the board being constructed.
        self._setup_board()

        # A private variable to record whether `_connect_things` has been
        # been called.
        self._connect_things_called = False

        # A list to store the modifiers user might add to this board.
        self._modifiers = []

    def get_processor(self) -> "AbstractProcessor":
        """Get the processor connected to the board.

        :returns: The processor.
        """
        return self.processor

    def get_memory(self) -> "AbstractMemory":
        """Get the memory (RAM) connected to the board.

        :returns: The memory system.
        """
        return self.memory

    def get_mem_ports(self) -> Sequence[Tuple[AddrRange, Port]]:
        """Get the memory ports exposed on this board

        .. note::

            The ports should be returned such that the address ranges are
            in ascending order.
        """
        return self.get_memory().get_mem_ports()

    def get_cache_hierarchy(self) -> Optional["AbstractCacheHierarchy"]:
        """Get the cache hierarchy connected to the board.

        :returns: The cache hierarchy.
        """
        return self._cache_hierarchy

    def get_cache_line_size(self) -> int:
        """Get the size of the cache line.

        :returns: The size of the cache line size.
        """
        return self.cache_line_size

    def connect_system_port(self, port: Port) -> None:
        self.system_port = port

    def set_mem_mode(self, mem_mode: MemMode) -> None:
        """
        Set the memory mode of the board.

        :param mem_mode: The memory mode the board is to be set to.
        """
        self.mem_mode = mem_mode_to_string(mem_mode=mem_mode)

    def get_clock_domain(self) -> ClockDomain:
        """Get the clock domain.

        :returns: The clock domain.
        """
        return self.clk_domain

    def _set_fullsystem(self, is_fs: bool) -> None:
        """
        Sets whether this board is to be run in FS or SE mode. This is set
        via the workload (the workload specified determines whether this will
        be run in FS mode or not). This is not intended to be set in a
        configuration script ergo, it's private.

        :param is_fs: Set whether the board is to be run in FS mode or SE mode.
        """
        self._is_fs = is_fs

    def is_fullsystem(self) -> bool:
        """
        Returns ``True`` if the board is to be run in FS mode. Otherwise the board
        is to be run in Se mode. An exception will be thrown if this has not
        been set.

        This function is used by the Simulator module to setup the simulation
        correctly.
        """
        if self._is_fs == None:
            raise Exception(
                "The workload for this board not yet to be set. "
                "Whether the board is to be executed in FS or SE "
                "mode is determined by which 'set workload' "
                "function is run."
            )
        return self._is_fs

    def set_workload(self, workload: WorkloadResource) -> None:
        """
        Set the workload for this board to run.

        This function will take the workload specified and run the correct
        workload function (e.g., ``set_kernel_disk_workload``) with the correct
        parameters

        :param workload: The workload to be set to this board.
        """

        try:
            func = getattr(self, workload.get_function_str())
        except AttributeError:
            raise Exception(
                "This board does not support this workload type. "
                f"This board does not contain the necessary "
                f"`{workload.get_function_str()}` function"
            )

        func_signature = inspect.signature(func)
        for param_name in workload.get_parameters().keys():
            if param_name not in func_signature.parameters.keys():
                raise Exception(
                    "Workload specifies non-existent parameter "
                    f"`{param_name}` for function "
                    f"`{workload.get_function_str()}` "
                )

        func(**workload.get_parameters())

    def add_modifier(self, modifier: Modifier) -> None:
        """
        Add a modifier to the board.

        :param modifier: The modifier to add to the board.
        """
        self._modifiers.append(modifier)

    @abstractmethod
    def _setup_board(self) -> None:
        """
        This function is called in the AbstractBoard constructor, before the
        memory, processor, and cache hierarchy components are incorporated via
        ``_connect_thing()``, but after the ``_setup_memory_ranges()`` function.
        This function should be overridden by boards to specify components,
        connections unique to that board.
        """
        raise NotImplementedError

    # Technically `get_dma_ports` returns a list. This list could be empty to
    # indicate the presense of dma ports. Though I quite like having this
    # boolean to quickly check a board.
    @abstractmethod
    def has_dma_ports(self) -> bool:
        """Determine whether the board has DMA ports or not.

        :returns: ``True`` if the board has DMA ports, otherwise ``False``.
        """
        raise NotImplementedError

    @abstractmethod
    def get_dma_ports(self) -> List[Port]:
        """Get the board's Direct Memory Access ports.
        This abstract method must be implemented within the subclasses if they
        support DMA and/or full system simulation.

        :returns: A List of the Direct Memory Access ports.

        """
        raise NotImplementedError

    @abstractmethod
    def has_io_bus(self) -> bool:
        """Determine whether the board has an IO bus or not.

        :returns: ``True`` if the board has an IO bus, otherwise ``False``.
        """
        raise NotImplementedError

    @abstractmethod
    def get_io_bus(self) -> IOXBar:
        """Get the board's IO Bus.

        This abstract method must be implemented within the subclasses if they
        support DMA and/or full system simulation.

        The I/O bus is a non-coherent bus (in the classic caches). On the CPU
        side, it accepts requests meant for I/O devices. On the memory side, it
        forwards these requests to the devices (e.g., the interrupt
        controllers on each core).

        :returns: The I/O Bus.
        """
        raise NotImplementedError

    @abstractmethod
    def has_coherent_io(self) -> bool:
        """Determine whether the board needs coherent I/O

        :returns: ``True`` if the board needs coherent I/O, ``False`` otherwise.
        """
        raise NotImplementedError

    @abstractmethod
    def get_mem_side_coherent_io_port(self):
        """Get the memory-side coherent I/O port.

        This abstract method must be implemented if ``has_coherent_io`` is ``True``.

        This returns a *port* (not a bus) that should be connected to a
        CPU-side port for which coherent I/O (DMA) is issued.
        """
        raise NotImplementedError

    @abstractmethod
    def _setup_memory_ranges(self) -> None:
        """
        Set the memory ranges for this board and memory system.

        This is called in the constructor, prior to ``_setup_board`` and
        ``_connect_things``. It should query the board's memory to determine the
        size and the set the memory ranges on the memory system and on the
        board.

        The simplest implementation sets the board's memory range to the size
        of memory and memory system's range to be the same as the board. Full
        system implementations will likely need something more complicated.

        .. note::

            This *must* be called prior to the incorporation of the cache
            hierarchy (via ``_connect_things``) as cache hierarchies depend upon
            knowing the memory system's ranges.
        """
        raise NotImplementedError

    def _connect_things(self) -> None:
        """Connects all the components to the board.

        The order of this board is always:

        1. Connect the memory.
        2. Connect the cache hierarchy.
        3. Connect the processor.

        Developers may build upon this assumption when creating components.

        .. note::

            * The processor is incorporated after the cache hierarchy due to a bug
            noted here: https://gem5.atlassian.net/browse/GEM5-1113. Until this
            bug is fixed, this ordering must be maintained.
            * Once this function is called ``_connect_things_called`` *must* be set
            to ``True``.
        """

        if self._connect_things_called:
            raise Exception(
                "The `_connect_things` function has already been called."
            )

        # Incorporate the memory into the motherboard.
        self.get_memory().incorporate_memory(self)

        # Incorporate the cache hierarchy for the motherboard.
        if self.get_cache_hierarchy():
            self.get_cache_hierarchy().incorporate_cache(self)

        # Incorporate the processor into the motherboard.
        self.get_processor().incorporate_processor(self)

        self._connect_things_called = True

    def _post_instantiate(self):
        """Called to set up anything needed after ``m5.instantiate``."""
        self.get_processor()._post_instantiate()
        if self.get_cache_hierarchy():
            self.get_cache_hierarchy()._post_instantiate()
        self.get_memory()._post_instantiate()

    def _pre_instantiate(self):
        """To be called immediately before ``m5.instantiate``. This is where
        ``_connect_things`` is executed by default."""

        # Connect the memory, processor, and cache hierarchy.
        self._connect_things()

    def _apply_modifiers(self) -> None:
        for modifier in self._modifiers:
            modifier.apply(self)

    def _connect_things_check(self):
        """
        Here we check that connect things has been called and throw an
        Exception if it has not.

        Since v22.1 ``_connect_things`` function has
        been moved from the AbstractBoard constructor to the
        ``_pre_instantation`` function. Users who have used the gem5 stdlib
        components (i.e., boards which inherit from AbstractBoard) and the
        Simulator module should notice no change. Those who do not use the
        Simulator module and instead called ``m5.instantiate`` directly must
        call ``AbstractBoard._pre_instantation`` prior to ``_connect_things`` is
        called. In order to avoid confusion, this check has been incorporated
        and the Exception thrown explains the fix needed to convert old scripts
        to function with `v22.1`.

        This function is called in ``AbstractSystemBoard.createCCObject`` and
        ``ArmBoard.createCCObject``. Both these functions override
        ``SimObject.createCCObject``. We can not do that here as AbstractBoard
        does not inherit form System.
        """
        if not self._connect_things_called:
            raise Exception(
                """
AbstractBoard's ``_connect_things`` function has not been called. This is likely
due to not running a board outside of the gem5 Standard Library Simulator
module. If this is the case, this can be resolved by calling
``<AbstractBoard>._pre_instantiate()`` prior to ``m5.instantiate()``.
"""
            )
