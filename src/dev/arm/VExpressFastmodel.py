# Copyright 2019 Google, Inc.
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
from m5.objects.FastModelGIC import FastModelGIC
from m5.objects.FastModelGIC import SCFastModelGIC
from m5.objects.Gic import ArmSPI
from m5.objects.RealView import HDLcd
from m5.objects.RealView import VExpress_GEM5_Base
from m5.objects.SubSystem import SubSystem


class VExpressFastmodel(VExpress_GEM5_Base):
    gic = FastModelGIC(
        sc_gic=SCFastModelGIC(
            its_count=1,
            its0_base=0x2E010000,
            reg_base=0x2C000000,
            reg_base_per_redistributor="0.0.0.0=0x2c010000",
            spi_count=988,
        ),
    )

    hdlcd = HDLcd(
        pxl_clk=VExpress_GEM5_Base.dcc.osc_pxl,
        pio_addr=0x2B000000,
        interrupt=ArmSPI(num=95),
    )

    # Remove original timer to prevent from possible conflict with Fastmodel
    # timer.
    generic_timer = SubSystem()
    generic_timer_mem = SubSystem()
    sys_counter = SubSystem()
    el2_watchdog = SubSystem()

    def _on_chip_devices(self):
        return [
            self.gic,
            self.hdlcd,
            self.system_watchdog,
            self.trusted_watchdog,
        ]

    def setupBootLoader(self, cur_sys, loc, boot_loader=None):
        if boot_loader is None:
            boot_loader = [loc("boot_v2.arm64")]
        super().setupBootLoader(cur_sys, boot_loader)
