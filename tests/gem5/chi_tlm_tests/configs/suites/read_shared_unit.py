# -*- mode:python -*-
# Copyright (c) 2024 Arm Limited
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

from m5.tlm_chi import *


def payload_gen():
    # Populating payload
    payload = TlmPayload()
    payload.address = 0x80000000
    payload.ns = True
    payload.size = Size.SIZE_64
    return payload


def phase_gen():
    # Populating phase
    phase = TlmPhase()
    phase.opcode = ReqOpcode.READ_SHARED
    phase.src_id = 0
    phase.tgt_id = 0
    phase.exp_comp_ack = True
    return phase


def channel_check(transaction):
    return expect_equal(transaction.phase.channel, Channel.DAT)


def opcode_check(transaction):
    return expect_equal(transaction.phase.opcode, DatOpcode.COMP_DATA)


def cacheline_check(transaction):
    return expect_equal(transaction.phase.resp, Resp.RESP_UC)


def data_id_check_gen(exp):
    def data_id_check(transaction):
        return expect_equal(transaction.phase.data_id, exp)

    return data_id_check


def wait_data(transaction):
    return True


def do_comp_ack(transaction):
    transaction.phase.channel = Channel.RSP
    transaction.phase.opcode = RspOpcode.COMP_ACK
    transaction.inject()
    return False


def test_all(generator):
    payload = payload_gen()
    phase = phase_gen()

    tran = generator.injectAt(10, payload, phase)
    tran.EXPECT(channel_check)
    tran.EXPECT(opcode_check)
    tran.EXPECT(cacheline_check)
    tran.EXPECT(data_id_check_gen(0))
    tran.DO_WAIT(wait_data)
    tran.EXPECT(channel_check)
    tran.EXPECT(opcode_check)
    tran.EXPECT(cacheline_check)
    tran.EXPECT(data_id_check_gen(2))
    tran.DO(do_comp_ack)
