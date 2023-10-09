# Copyright (c) 2015 ARM Limited
# All rights reserved
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


def upgrader(cpt):
    """HDLCD controller rewrite. Converted checkpoints cause the HDLCD
    model to start a new screen refresh and FIFO buffer fill immediately
    after they are loaded. Expect some timing differences."""

    import re

    if cpt.get("root", "isa", fallback="") != "arm":
        return

    option_names = {
        "int_rawstat": "int_rawstat_serial",
        "int_mask": "int_mask_serial",
        "fb_base": "fb_base",
        "fb_line_length": "fb_line_length",
        "fb_line_count": "fb_line_count_serial",
        "fb_line_pitch": "fb_line_pitch",
        "bus_options": "bus_options_serial",
        "v_sync": "v_sync_serial",
        "v_back_porch": "v_back_porch_serial",
        "v_data": "v_data_serial",
        "v_front_porch": "v_front_porch_serial",
        "h_sync": "h_sync_serial",
        "h_back_porch": "h_back_porch_serial",
        "h_data": "h_data_serial",
        "h_front_porch": "h_front_porch_serial",
        "polarities": "polarities_serial",
        "command": "command_serial",
        "pixel_format": "pixel_format_serial",
        "red_select": "red_select_serial",
        "green_select": "green_select_serial",
        "blue_select": "blue_select_serial",
    }

    for sec in cpt.sections():
        if re.search(r".*\.hdlcd$", sec):
            options = {}
            for new, old in list(option_names.items()):
                options[new] = cpt.get(sec, old)

            cpt.remove_section(sec)
            cpt.add_section(sec)
            for key, value in list(options.items()):
                cpt.set(sec, key, value)

            # Create a DMA engine section. The LCD controller will
            # initialize the DMA it after the next VSync, so we don't
            # care about the actual values
            sec_dma = f"{sec}.dmaEngine"
            cpt.add_section(sec_dma)
            cpt.set(sec_dma, "nextLineAddr", "0")
            cpt.set(sec_dma, "frameEnd", "0")
            cpt.set(sec_dma, "startAddr", "0")
            cpt.set(sec_dma, "endAddr", "0")
            cpt.set(sec_dma, "nextAddr", "0")
            cpt.set(sec_dma, "buffer", "")

    print(
        "Warning: Assuming that the HDLCD pixel clock and global frequency "
        "are still using their default values."
    )
    sec_osc = "system.realview.realview_io.osc_pxl"
    global_tick = 1e12
    pxl_freq = 137e6
    pxl_ticks = global_tick / pxl_freq
    if not cpt.has_section(sec_osc):
        cpt.add_section(sec_osc)
    cpt.set(sec_osc, "type", "RealViewOsc")
    cpt.set(sec_osc, "_clockPeriod", "%i" % pxl_ticks)
