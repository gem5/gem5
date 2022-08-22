# Copyright (c) 2009-2020, 2022 Arm Limited
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

from m5.SimObject import SimObject
from m5.objects.Device import PioDevice
from m5.params import Param, MaxAddr, NULL, VectorParam
from m5.proxy import Parent
from m5.util import fatal
from m5.util.fdthelper import FdtNode, FdtProperty, FdtPropertyWords, FdtState


class SystemCounter(SimObject):
    """
    Shared by both PE-implementations and memory-mapped timers. It provides a
    uniform view of system time through its counter value.

    Reference:
        Arm ARM (ARM DDI 0487E.a)
        D11.1.2 - The system counter
    """

    type = "SystemCounter"
    cxx_header = "dev/arm/generic_timer.hh"
    cxx_class = "gem5::SystemCounter"

    # Maximum of 1004 frequency entries, including end marker
    freqs = VectorParam.UInt32(
        [0x01800000],
        "Frequencies available for the "
        "system counter (in Hz). First element is the base frequency, "
        "following are alternative lower ones which must be exact divisors",
    )

    def generateDtb(self):
        if not self.freqs:
            fatal("No counter frequency to expose in DTB")
        return FdtPropertyWords("clock-frequency", [self.freqs[0]])


class GenericTimer(SimObject):
    """
    Architected timers per PE in the system. Each of them provides a physical
    counter, a virtual counter and several timers accessible from different
    exception levels and security states.

    Reference:
        Arm ARM (ARM DDI 0487E.a)
        D11.2 - The AArch64 view of the Generic Timer
        G6.2  - The AArch32 view of the Generic Timer
    """

    type = "GenericTimer"
    cxx_header = "dev/arm/generic_timer.hh"
    cxx_class = "gem5::GenericTimer"

    _freq_in_dtb = False

    system = Param.ArmSystem(Parent.any, "system")

    counter = Param.SystemCounter(Parent.any, "Global system counter")

    int_el1_phys = Param.ArmPPI("EL1 physical timer interrupt")
    int_el1_virt = Param.ArmPPI("EL1 virtual timer interrupt")
    int_el2_ns_phys = Param.ArmPPI("EL2 Non-secure physical timer interrupt")
    int_el2_ns_virt = Param.ArmPPI("EL2 Non-secure virtual timer interrupt")
    int_el2_s_phys = Param.ArmPPI("EL2 Secure physical timer interrupt")
    int_el2_s_virt = Param.ArmPPI("EL2 Secure virtual timer interrupt")
    int_el3_phys = Param.ArmPPI("EL3 physical timer interrupt")

    # This value should be in theory initialized by the highest
    # priviledged software. We do this in gem5 to avoid KVM
    # complications (the gem5 firmware won't run at highest EL)
    #
    # PLEASE note: change this parameter only if using the gem5 bootloader
    # Another real world bootloader might be changing the CNTFRQ register
    # value, so this initial value will be discarded
    cntfrq = Param.UInt64(0x1800000, "Value for the CNTFRQ timer register")

    def generateDeviceTree(self, state):
        node = FdtNode("timer")

        node.appendCompatible(
            ["arm,cortex-a15-timer", "arm,armv7-timer", "arm,armv8-timer"]
        )

        gic = self._parent.unproxy(self).gic
        node.append(
            FdtPropertyWords(
                "interrupts",
                self.int_el3_phys.generateFdtProperty(gic)
                + self.int_el1_phys.generateFdtProperty(gic)
                + self.int_el1_virt.generateFdtProperty(gic)
                + self.int_el2_ns_phys.generateFdtProperty(gic)
                + self.int_el2_ns_virt.generateFdtProperty(gic),
            )
        )

        if self._freq_in_dtb:
            node.append(self.counter.unproxy(self).generateDtb())

        yield node


class GenericTimerFrame(PioDevice):
    """
    Memory-mapped timer frame implementation. Controlled from GenericTimerMem,
    may be used by peripherals without a system register interface.

    Reference:
        Arm ARM (ARM DDI 0487E.a)
        I2.3.2 - The CNTBaseN and CNTEL0BaseN frames
    """

    type = "GenericTimerFrame"
    cxx_header = "dev/arm/generic_timer.hh"
    cxx_class = "gem5::GenericTimerFrame"

    _frame_num = 0

    counter = Param.SystemCounter(Parent.any, "Global system counter")

    cnt_base = Param.Addr("CNTBase register frame base")
    cnt_el0_base = Param.Addr(MaxAddr, "CNTEL0Base register frame base")

    int_phys = Param.ArmSPI("Physical Interrupt")
    int_virt = Param.ArmSPI("Virtual Interrupt")

    def generateDeviceTree(self, state, gic):
        node = FdtNode("frame@{:08x}".format(self.cnt_base.value))
        node.append(FdtPropertyWords("frame-number", self._frame_num))

        ints = self.int_phys.generateFdtProperty(gic)
        if self.int_virt != NULL:
            ints.extend(self.int_virt.generateFdtProperty(gic))
        node.append(FdtPropertyWords("interrupts", ints))

        reg = state.addrCells(self.cnt_base) + state.sizeCells(0x1000)
        if self.cnt_el0_base.value != MaxAddr:
            reg.extend(
                state.addrCells(self.cnt_el0_base) + state.sizeCells(0x1000)
            )
        node.append(FdtPropertyWords("reg", reg))

        return node


class GenericTimerMem(PioDevice):
    """
    System level implementation. It provides three main components:
    - Memory-mapped counter module: controls the system timer through the
      CNTControlBase frame, and provides its value through the CNTReadBase frame
    - Memory-mapped timer control module: controls the memory-mapped timers
    - Memory-mapped timers: implementations of the GenericTimer for system
      peripherals

    Reference:
        Arm ARM (ARM DDI 0487E.a)
        I2 - System Level Implementation of the Generic Timer
    """

    type = "GenericTimerMem"
    cxx_header = "dev/arm/generic_timer.hh"
    cxx_class = "gem5::GenericTimerMem"

    _freq_in_dtb = False

    counter = Param.SystemCounter(Parent.any, "Global system counter")

    cnt_control_base = Param.Addr("CNTControlBase register frame base")
    cnt_read_base = Param.Addr("CNTReadBase register frame base")
    cnt_ctl_base = Param.Addr("CNTCTLBase register frame base")

    # Maximum of 8 timer frames
    frames = VectorParam.GenericTimerFrame([], "Memory-mapped timer frames")

    def generateDeviceTree(self, state):
        node = self.generateBasicPioDeviceNode(
            state, "timer", self.cnt_ctl_base, 0x1000
        )
        node.appendCompatible(["arm,armv7-timer-mem"])
        node.append(state.addrCellsProperty())
        node.append(state.sizeCellsProperty())
        node.append(FdtProperty("ranges"))

        if self._freq_in_dtb:
            node.append(self.counter.unproxy(self).generateDtb())

        gic = self._parent.unproxy(self).gic

        for i, frame in enumerate(self.frames):
            frame._frame_num = i
            node.append(frame.generateDeviceTree(state, gic))

        yield node
