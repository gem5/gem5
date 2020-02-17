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

from six import string_types

from m5.SimObject import *
from m5.params import *

# QoS scheduler policy used to serve incoming transaction
class QoSPolicy(SimObject):
    type = 'QoSPolicy'
    abstract = True
    cxx_header = "mem/qos/policy.hh"
    cxx_class = 'QoS::Policy'

class QoSFixedPriorityPolicy(QoSPolicy):
    type = 'QoSFixedPriorityPolicy'
    cxx_header = "mem/qos/policy_fixed_prio.hh"
    cxx_class = 'QoS::FixedPriorityPolicy'

    cxx_exports = [
        PyBindMethod('initMasterName'),
        PyBindMethod('initMasterObj'),
    ]

    _mpriorities = None

    def setMasterPriority(self, master, priority):
        if not self._mpriorities:
            self._mpriorities = []

        self._mpriorities.append([master, priority])

    def init(self):
        if not self._mpriorities:
            print("Error, use setMasterPriority to init masters/priorities\n");
            exit(1)
        else:
            for mprio in self._mpriorities:
                master = mprio[0]
                priority = mprio[1]
                if isinstance(master, string_types):
                    self.getCCObject().initMasterName(
                        master, int(priority))
                else:
                    self.getCCObject().initMasterObj(
                        master.getCCObject(), priority)

    # default fixed priority value for non-listed Masters
    qos_fixed_prio_default_prio = Param.UInt8(0,
        "Default priority for non-listed Masters")

class QoSPropFairPolicy(QoSPolicy):
    type = 'QoSPropFairPolicy'
    cxx_header = "mem/qos/policy_pf.hh"
    cxx_class = 'QoS::PropFairPolicy'

    cxx_exports = [
        PyBindMethod('initMasterName'),
        PyBindMethod('initMasterObj'),
    ]

    _mscores = None

    def setInitialScore(self, master, score):
        if not self._mscores:
            self._mscores = []

        self._mscores.append([master, score])

    def init(self):
        if not self._mscores:
            print("Error, use setInitialScore to init masters/scores\n");
            exit(1)
        else:
            for mprio in self._mscores:
                master = mprio[0]
                score = mprio[1]
                if isinstance(master, string_types):
                    self.getCCObject().initMasterName(
                        master, float(score))
                else:
                    self.getCCObject().initMasterObj(
                        master.getCCObject(), float(score))

    weight = Param.Float(0.5, "Pf score weight")
