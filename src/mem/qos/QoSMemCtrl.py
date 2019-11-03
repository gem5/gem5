# Copyright (c) 2018 ARM Limited
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
# Authors: Matteo Andreozzi

from m5.params import *
from m5.objects.AbstractMemory import AbstractMemory
from m5.objects.QoSTurnaround import *

# QoS Queue Selection policy used to select packets among same-QoS queues
class QoSQPolicy(Enum): vals = ["fifo", "lifo", "lrg"]

class QoSMemCtrl(AbstractMemory):
    type = 'QoSMemCtrl'
    cxx_header = "mem/qos/mem_ctrl.hh"
    cxx_class = 'QoS::MemCtrl'
    abstract = True

    ##### QoS support parameters ####

    # Number of priorities in the system
    qos_priorities = Param.Unsigned(1, "QoS priorities")

    # QoS scheduler policy: tags request with QoS priority value
    qos_policy = Param.QoSPolicy(NULL,
        "Memory Controller Requests QoS arbitration policy")

    # Select QoS driven turnaround policy
    # (direction switch triggered by highest priority buffer content)
    qos_turnaround_policy = Param.QoSTurnaroundPolicy(NULL,
        "Selects QoS driven turnaround policy")

    # QoS Queue Select policy: selects packets among same priority level
    # (only supported in QoSMemSinkCtrl)
    qos_q_policy = Param.QoSQPolicy('fifo',
        "Memory Controller Requests same-QoS selection policy")

    # flag to select QoS syncronised scheduling
    # (calls the scheduler on all masters at every packet arrival)
    qos_syncro_scheduler = Param.Bool(False,
        "Enables QoS syncronized scheduling")

    # flag to enable QoS priority escalation
    qos_priority_escalation = Param.Bool(False,
        "Enables QoS priority escalation")

    # Master ID to be mapped to service parameters in QoS schedulers
    qos_masters = VectorParam.String(['']* 16,
        "Master Names to be mapped to service parameters in QoS scheduler")
