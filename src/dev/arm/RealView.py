# Copyright (c) 2009 ARM Limited
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
# Copyright (c) 2006-2007 The Regents of The University of Michigan
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
#
# Authors: Ali Saidi
#          Gabe Black

from m5.params import *
from m5.proxy import *
from Device import BasicPioDevice, PioDevice, IsaFake, BadAddr, DmaDevice
from Platform import Platform
from Terminal import Terminal
from Uart import Uart

class AmbaDevice(BasicPioDevice):
    type = 'AmbaDevice'
    abstract = True
    amba_id = Param.UInt32("ID of AMBA device for kernel detection")

class AmbaDmaDevice(DmaDevice):
    type = 'AmbaDmaDevice'
    abstract = True
    amba_id = Param.UInt32("ID of AMBA device for kernel detection")

class RealViewCtrl(BasicPioDevice):
    type = 'RealViewCtrl'
    proc_id = Param.UInt32(0x0C000000, "Platform ID")

class Gic(PioDevice):
    type = 'Gic'
    dist_addr = Param.Addr(0x1f001000, "Address for distributor")
    cpu_addr = Param.Addr(0x1f000100, "Address for cpu")
    dist_pio_delay = Param.Latency('10ns', "Delay for PIO r/w to distributor")
    cpu_pio_delay = Param.Latency('10ns', "Delay for PIO r/w to cpu")
    it_lines = Param.UInt32(128, "Number of interrupt lines supported (max = 1020)")

class AmbaFake(AmbaDevice):
    type = 'AmbaFake'
    ignore_access = Param.Bool(False, "Ignore reads/writes to this device, (e.g. IsaFake + AMBA)")
    amba_id = 0;

class Pl011(Uart):
    type = 'Pl011'
    gic = Param.Gic(Parent.any, "Gic to use for interrupting")
    int_num = Param.UInt32("Interrupt number that connects to GIC")
    end_on_eot = Param.Bool(False, "End the simulation when a EOT is received on the UART")
    int_delay = Param.Latency("100ns", "Time between action and interrupt generation by UART")

class Sp804(AmbaDevice):
    type = 'Sp804'
    gic = Param.Gic(Parent.any, "Gic to use for interrupting")
    int_num0 = Param.UInt32("Interrupt number that connects to GIC")
    clock0 = Param.Clock('1MHz', "Clock speed of the input")
    int_num1 = Param.UInt32("Interrupt number that connects to GIC")
    clock1 = Param.Clock('1MHz', "Clock speed of the input")
    amba_id = 0x00141804

class RealView(Platform):
    type = 'RealView'
    system = Param.System(Parent.any, "system")

class RealViewPBX(RealView):
    uart = Pl011(pio_addr=0x10009000, int_num=44)
    realview_io = RealViewCtrl(pio_addr=0x10000000)
    gic = Gic()
    timer0 = Sp804(int_num0=36, int_num1=36, pio_addr=0x10011000)
    timer1 = Sp804(int_num0=37, int_num1=37, pio_addr=0x10012000)

    l2x0_fake     = IsaFake(pio_addr=0x1f002000, pio_size=0xfff, warn_access="1")
    dmac_fake     = AmbaFake(pio_addr=0x10030000)
    uart1_fake    = AmbaFake(pio_addr=0x1000a000)
    uart2_fake    = AmbaFake(pio_addr=0x1000b000)
    uart3_fake    = AmbaFake(pio_addr=0x1000c000)
    smc_fake      = AmbaFake(pio_addr=0x100e1000)
    clcd_fake     = AmbaFake(pio_addr=0x10020000)
    sp810_fake    = AmbaFake(pio_addr=0x10001000, ignore_access=True)
    watchdog_fake = AmbaFake(pio_addr=0x10010000)
    gpio0_fake    = AmbaFake(pio_addr=0x10013000)
    gpio1_fake    = AmbaFake(pio_addr=0x10014000)
    gpio2_fake    = AmbaFake(pio_addr=0x10015000)
    ssp_fake      = AmbaFake(pio_addr=0x1000d000)
    sci_fake      = AmbaFake(pio_addr=0x1000e000)
    aaci_fake     = AmbaFake(pio_addr=0x10004000)
    mmc_fake      = AmbaFake(pio_addr=0x10005000)
    kmi0_fake     = AmbaFake(pio_addr=0x10006000)
    kmi1_fake     = AmbaFake(pio_addr=0x10007000)
    rtc_fake      = AmbaFake(pio_addr=0x10017000, amba_id=0x41031)



    # Attach I/O devices that are on chip
    def attachOnChipIO(self, bus):
       self.gic.pio = bus.port
       self.l2x0_fake.pio = bus.port

    # Attach I/O devices to specified bus object.  Can't do this
    # earlier, since the bus object itself is typically defined at the
    # System level.
    def attachIO(self, bus):
       self.uart.pio          = bus.port
       self.realview_io.pio   = bus.port
       self.timer0.pio        = bus.port
       self.timer1.pio        = bus.port
       self.dmac_fake.pio     = bus.port
       self.uart1_fake.pio    = bus.port
       self.uart2_fake.pio    = bus.port
       self.uart3_fake.pio    = bus.port
       self.smc_fake.pio      = bus.port
       self.clcd_fake.pio     = bus.port
       self.sp810_fake.pio    = bus.port
       self.watchdog_fake.pio = bus.port
       self.gpio0_fake.pio    = bus.port
       self.gpio1_fake.pio    = bus.port
       self.gpio2_fake.pio    = bus.port
       self.ssp_fake.pio      = bus.port
       self.sci_fake.pio      = bus.port
       self.aaci_fake.pio     = bus.port
       self.mmc_fake.pio      = bus.port
       self.kmi0_fake.pio     = bus.port
       self.kmi1_fake.pio     = bus.port
       self.rtc_fake.pio      = bus.port

class RealViewEB(RealView):
    uart = Pl011(pio_addr=0x10009000, int_num=44)
    realview_io = RealViewCtrl(pio_addr=0x10000000)
    gic = Gic(dist_addr=0x10041000, cpu_addr=0x10040000)
    timer0 = Sp804(int_num0=36, int_num1=36, pio_addr=0x10011000)
    timer1 = Sp804(int_num0=37, int_num1=37, pio_addr=0x10012000)

    l2x0_fake     = IsaFake(pio_addr=0x1f002000, pio_size=0xfff, warn_access="1")
    dmac_fake     = AmbaFake(pio_addr=0x10030000)
    uart1_fake    = AmbaFake(pio_addr=0x1000a000)
    uart2_fake    = AmbaFake(pio_addr=0x1000b000)
    uart3_fake    = AmbaFake(pio_addr=0x1000c000)
    smc_fake      = AmbaFake(pio_addr=0x100e1000)
    clcd_fake     = AmbaFake(pio_addr=0x10020000)
    sp810_fake    = AmbaFake(pio_addr=0x10001000, ignore_access=True)
    watchdog_fake = AmbaFake(pio_addr=0x10010000)
    gpio0_fake    = AmbaFake(pio_addr=0x10013000)
    gpio1_fake    = AmbaFake(pio_addr=0x10014000)
    gpio2_fake    = AmbaFake(pio_addr=0x10015000)
    ssp_fake      = AmbaFake(pio_addr=0x1000d000)
    sci_fake      = AmbaFake(pio_addr=0x1000e000)
    aaci_fake     = AmbaFake(pio_addr=0x10004000)
    mmc_fake      = AmbaFake(pio_addr=0x10005000)
    kmi0_fake     = AmbaFake(pio_addr=0x10006000)
    kmi1_fake     = AmbaFake(pio_addr=0x10007000)
    rtc_fake      = AmbaFake(pio_addr=0x10017000, amba_id=0x41031)



    # Attach I/O devices that are on chip
    def attachOnChipIO(self, bus):
       self.gic.pio = bus.port
       self.l2x0_fake.pio = bus.port

    # Attach I/O devices to specified bus object.  Can't do this
    # earlier, since the bus object itself is typically defined at the
    # System level.
    def attachIO(self, bus):
       self.uart.pio          = bus.port
       self.realview_io.pio   = bus.port
       self.timer0.pio        = bus.port
       self.timer1.pio        = bus.port
       self.dmac_fake.pio     = bus.port
       self.uart1_fake.pio    = bus.port
       self.uart2_fake.pio    = bus.port
       self.uart3_fake.pio    = bus.port
       self.smc_fake.pio      = bus.port
       self.clcd_fake.pio     = bus.port
       self.sp810_fake.pio    = bus.port
       self.watchdog_fake.pio = bus.port
       self.gpio0_fake.pio    = bus.port
       self.gpio1_fake.pio    = bus.port
       self.gpio2_fake.pio    = bus.port
       self.ssp_fake.pio      = bus.port
       self.sci_fake.pio      = bus.port
       self.aaci_fake.pio     = bus.port
       self.mmc_fake.pio      = bus.port
       self.kmi0_fake.pio     = bus.port
       self.kmi1_fake.pio     = bus.port
       self.rtc_fake.pio      = bus.port

