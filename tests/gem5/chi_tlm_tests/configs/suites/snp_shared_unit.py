# -*- mode:python -*-
# Copyright (c) 2026 Arm Limited
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

import m5
from m5.objects.SnoopHandler import PySnoopHandler
from m5.tlm_chi.utils import *

ADDRESS = 0x80000000
CACHELINE_SIZE = 64
GENERATOR0_ID = 0
GENERATOR1_ID = 1
FIRST_TXN_ID = 0
SECOND_TXN_ID = 1

SNOOP_RESPONSES = []


def payload_gen(address):
    payload = TlmPayload()
    payload.address = address
    payload.ns = True
    payload.size = Size.SIZE_64
    return payload


def phase_gen(src_id, txn_id):
    phase = TlmPhase()
    phase.opcode = ReqOpcode.READ_SHARED
    phase.txn_id = txn_id
    phase.src_id = src_id
    phase.tgt_id = 0
    phase.exp_comp_ack = True
    return phase


def channel_check_gen(exp):
    def channel_check(transaction):
        return expect_equal(transaction.phase.channel, exp)

    return channel_check


def opcode_check_gen(exp):
    def opcode_check(transaction):
        return expect_equal(transaction.phase.opcode, exp)

    return opcode_check


def cacheline_check_gen(exp):
    def cacheline_check(transaction):
        return expect_equal(transaction.phase.resp, exp)

    return cacheline_check


def data_id_check_gen(exp):
    def data_id_check(transaction):
        return expect_equal(transaction.phase.data_id, exp)

    return data_id_check


def wait_data(transaction):
    return True


def cycles(num_cycles):
    def nothing(transaction):
        return True

    return nothing, num_cycles


def do_comp_ack(transaction):
    transaction.phase.channel = Channel.RSP
    transaction.phase.opcode = RspOpcode.COMP_ACK
    transaction.send()
    return False


def do_snp_resp_fwded_gen(resp, fwd_state):
    def do_snp_resp_sc(transaction):
        ret_to_src = transaction.payload.ret_to_src
        assert ret_to_src == False, "ret_to_src should be False"

        snoop_data = SNOOP_RESPONSES[0]
        transaction.phase.channel = Channel.RSP
        transaction.phase.opcode = RspOpcode.SNP_RESP_FWDED
        transaction.phase.resp = resp
        transaction.phase.fwd_state = fwd_state
        transaction.phase.txn_id = snoop_data["txn_id"]
        transaction.phase.tgt_id = snoop_data["src_id"]
        transaction.send()

    return do_snp_resp_sc


def do_snp_comp_data_gen(data_id, resp):
    def do_snp_comp_data(transaction):
        snoop_data = SNOOP_RESPONSES[0]
        transaction.phase.channel = Channel.DAT
        transaction.phase.opcode = DatOpcode.COMP_DATA
        transaction.phase.resp = resp
        transaction.phase.txn_id = snoop_data["txn_id"]
        transaction.phase.tgt_id = snoop_data["fwd_nid"]

        if data_id == 0:
            transaction.payload.byte_enable = 0x00000000FFFFFFFF
        else:
            transaction.payload.byte_enable = 0xFFFFFFFF00000000

        transaction.phase.data_id = data_id
        transaction.send()

    return do_snp_comp_data


def record_snoop(transaction):
    SNOOP_RESPONSES.append(
        {
            "address": ADDRESS,
            "txn_id": transaction.phase.txn_id,
            "fwd_nid": transaction.phase.fwd_nid,
            "src_id": transaction.phase.src_id,
        }
    )
    return True


def req_out_verifier(generator, expected):
    def verifier(*args):
        m5.stats.prepare()
        stat = generator.resolveStat("reqOut")
        actual = int(stat.total)
        assert actual == expected, (
            "TlmGenerator reqOut mismatch: "
            f"expected={expected}, actual={actual}"
        )

    return verifier


def snoop_seen_verifier(generator, expected):
    def verifier(*args):
        m5.stats.prepare()
        stat = generator.snp_handler.resolveStat("snoops.snoopSharedFwd")
        actual = int(stat.total)
        assert actual == expected, (
            "SnpShared callback count mismatch: "
            f"expected={expected}, actual={actual}"
        )

    return verifier


def init(board):
    cores = board.get_processor().get_cores()
    if len(cores) != 2:
        raise Exception("This suite should be run with 2 TLM generators")

    for generator in cores:
        generator.snp_handler = PySnoopHandler(discard_after_snoop=True)


def tear_down(board):
    m5.stats.reset()


def test_snp_resp_sc_fwded_sc(generators):
    generator0 = generators[0]
    generator1 = generators[1]

    tran = generator0.inject(
        payload_gen(ADDRESS), phase_gen(GENERATOR0_ID, FIRST_TXN_ID)
    )
    tran.ASSERT(channel_check_gen(Channel.DAT))
    tran.ASSERT(opcode_check_gen(DatOpcode.COMP_DATA))
    tran.ASSERT(cacheline_check_gen(Resp.RESP_UC))
    tran.ASSERT(data_id_check_gen(0))
    tran.DO_WAIT(wait_data)
    tran.ASSERT(channel_check_gen(Channel.DAT))
    tran.ASSERT(opcode_check_gen(DatOpcode.COMP_DATA))
    tran.ASSERT(cacheline_check_gen(Resp.RESP_UC))
    tran.ASSERT(data_id_check_gen(2))
    tran.DO(do_comp_ack)

    yield req_out_verifier(generator0, 1)

    snoop = generator0.snp_handler.add_snoop(ADDRESS)
    snoop.DO(record_snoop)
    snoop.DO(do_snp_comp_data_gen(0, Resp.RESP_SC))
    snoop.DO_WAIT_FOR(*cycles(1))
    snoop.DO(do_snp_comp_data_gen(2, Resp.RESP_SC))
    snoop.DO_WAIT_FOR(*cycles(1))
    snoop.DO(do_snp_resp_fwded_gen(Resp.RESP_SC, Resp.RESP_SC))

    tran = generator1.inject(
        payload_gen(ADDRESS), phase_gen(GENERATOR1_ID, SECOND_TXN_ID)
    )
    tran.ASSERT(channel_check_gen(Channel.DAT))
    tran.ASSERT(opcode_check_gen(DatOpcode.COMP_DATA))
    tran.ASSERT(cacheline_check_gen(Resp.RESP_SC))
    tran.ASSERT(data_id_check_gen(0))
    tran.DO_WAIT(wait_data)
    tran.ASSERT(channel_check_gen(Channel.DAT))
    tran.ASSERT(opcode_check_gen(DatOpcode.COMP_DATA))
    tran.ASSERT(cacheline_check_gen(Resp.RESP_SC))
    tran.ASSERT(data_id_check_gen(2))
    tran.DO(do_comp_ack)

    yield snoop_seen_verifier(generator0, 1)

    tear_down(None)


def test_snp_resp_sc_fwded_sd(generators):
    generator0 = generators[0]
    generator1 = generators[1]

    tran = generator0.inject(
        payload_gen(ADDRESS + 64), phase_gen(GENERATOR0_ID, FIRST_TXN_ID)
    )
    tran.ASSERT(channel_check_gen(Channel.DAT))
    tran.ASSERT(opcode_check_gen(DatOpcode.COMP_DATA))
    tran.ASSERT(cacheline_check_gen(Resp.RESP_UC))
    tran.ASSERT(data_id_check_gen(0))
    tran.DO_WAIT(wait_data)
    tran.ASSERT(channel_check_gen(Channel.DAT))
    tran.ASSERT(opcode_check_gen(DatOpcode.COMP_DATA))
    tran.ASSERT(cacheline_check_gen(Resp.RESP_UC))
    tran.ASSERT(data_id_check_gen(2))
    tran.DO(do_comp_ack)

    yield req_out_verifier(generator0, 1)

    snoop = generator0.snp_handler.add_snoop(ADDRESS + 64)
    snoop.DO(record_snoop)
    snoop.DO(do_snp_comp_data_gen(0, Resp.RESP_SD_PD))
    snoop.DO_WAIT_FOR(*cycles(1))
    snoop.DO(do_snp_comp_data_gen(2, Resp.RESP_SD_PD))
    snoop.DO_WAIT_FOR(*cycles(1))
    snoop.DO(do_snp_resp_fwded_gen(Resp.RESP_SC, Resp.RESP_SD_PD))

    tran = generator1.inject(
        payload_gen(ADDRESS + 64), phase_gen(GENERATOR1_ID, SECOND_TXN_ID)
    )
    tran.ASSERT(channel_check_gen(Channel.DAT))
    tran.ASSERT(opcode_check_gen(DatOpcode.COMP_DATA))
    tran.ASSERT(cacheline_check_gen(Resp.RESP_SD_PD))
    tran.ASSERT(data_id_check_gen(0))
    tran.DO_WAIT(wait_data)
    tran.ASSERT(channel_check_gen(Channel.DAT))
    tran.ASSERT(opcode_check_gen(DatOpcode.COMP_DATA))
    tran.ASSERT(cacheline_check_gen(Resp.RESP_SD_PD))
    tran.ASSERT(data_id_check_gen(2))
    tran.DO(do_comp_ack)

    yield snoop_seen_verifier(generator0, 1)

    tear_down(None)


def test_all(generators):
    yield from test_snp_resp_sc_fwded_sc(generators)

    yield from test_snp_resp_sc_fwded_sd(generators)
