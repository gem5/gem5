# Copyright (c) 2013-2015 ARM Limited
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
#
# Authors: Rene de Jong
#
import sys
from m5.params import *
from m5.proxy import *
from m5.objects.Device import DmaDevice
from m5.objects.AbstractNVM import *

class UFSHostDevice(DmaDevice):
    type = 'UFSHostDevice'
    cxx_header = "dev/arm/ufs_device.hh"
    pio_addr = Param.Addr("Address for SCSI configuration slave interface")
    pio_latency = Param.Latency("10ns", "Time between action and write/read \
       result by AMBA DMA Device")
    gic = Param.BaseGic(Parent.any, "Gic to use for interrupting")
    int_num = Param.UInt32("Interrupt number that connects to GIC")
    img_blk_size = Param.UInt32(512, "Size of one image block in bytes")
    # Every image that is added to the vector will generate a new logic unit
    # in the UFS device; Theoretically (when using the driver from Linux
    # kernel 3.9 onwards), this can be as many as eigth. Up to two have been
    # tested.
    image = VectorParam.DiskImage("Disk images")
    # Every logic unit can have its own flash dimensions. So the number of
    # images that have been provided in the image vector, should be equal to
    # the number of flash objects that are created. Each logic unit can have
    # its own flash dimensions; to allow the system to define a hetrogeneous
    # storage system.
    internalflash = VectorParam.AbstractNVM("Describes the internal flash")
    ufs_slots = Param.UInt32(32, "Number of commands that can be queued in \
        the Host controller (min: 1, max: 32)")

