# -*- mode:python -*-
# Copyright (c) 2024-2025 Arm Limited
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


from m5.objects.ClockedObject import ClockedObject
from m5.objects.TlmController import TlmController
from m5.params import *
from m5.SimObject import (
    PyBindMethod,
)
from m5.tlm_chi.port import (
    TlmSinkPort,
    TlmSourcePort,
)


class TlmGenerator(ClockedObject):
    type = "TlmGenerator"
    cxx_header = "mem/ruby/protocol/chi/tlm/generator.hh"
    cxx_class = "gem5::tlm::chi::TlmGenerator"

    cxx_exports = [
        PyBindMethod("scheduleTransaction"),
        PyBindMethod("enqueueTransaction"),
    ]

    _transactions = []

    def inject(self, payload, phase, when=None):
        from m5.tlm_chi.utils import Transaction

        transaction = Transaction(payload, phase)

        if when:
            self._transactions.append((when, transaction))
        else:
            self.getCCObject().enqueueTransaction(transaction)

        return transaction

    def init(self):
        for when, tr in self._transactions:
            self.getCCObject().scheduleTransaction(when, tr)

    cpu_id = Param.Int("TlmGenerator CPU identifier")
    tran_per_cycle = Param.Unsigned(
        2,
        "Number of transaction per cycle to be scheduled "
        "(For transactions injected with the inject method "
        "and not with injectAt, which forces a transaction to "
        "be injected at a specific tick overriding any clock "
        "based timing)",
    )
    max_pending_tran = OptionalParam.Unsigned(
        "Max number of pending transactions issued via the inject API"
    )
    in_port = TlmSinkPort("CHI TLM input/response port")
    out_port = TlmSourcePort("CHI TLM output/request port")
