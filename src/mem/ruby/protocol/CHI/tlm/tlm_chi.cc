/*
 * Copyright (c) 2024 Arm Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <ARM/TLM/arm_chi.h>

#include "python/pybind11/pybind.hh"
#include "sim/init.hh"

namespace py = pybind11;

using namespace ARM::CHI;

namespace gem5 {

namespace {

/** We are forced to define a custom deleter as the Payload
 * destructor is deleted and pybind11 will complain */
template <typename T>
struct PayloadDeleter
{
    void
    operator()(T * p) const
    {
        p->unref();
    }
};

void
tlm_chi_pybind(pybind11::module_ &m_internal)
{
    auto tlm_chi = m_internal.def_submodule("tlm_chi");
    py::class_<Payload, std::unique_ptr<Payload, PayloadDeleter<Payload>>>(
        tlm_chi, "TlmPayload")
        .def(py::init(&Payload::new_payload))
        .def_readwrite("address", &Payload::address)
        .def_property("size",
            [] (const Payload &p) { return p.size; },
            [] (Payload &p, SizeEnum val) { p.size = val; })
        .def_readwrite("lpid", &Payload::lpid)
        .def_property("ns",
            [] (const Payload &p) { return p.ns; },
            [] (Payload &p, bool val) { p.ns = val; })
        ;

    py::class_<Phase>(tlm_chi, "TlmPhase")
        .def(py::init<>())
        .def_readwrite("txn_id", &Phase::txn_id)
        .def_readwrite("src_id", &Phase::src_id)
        .def_readwrite("tgt_id", &Phase::tgt_id)
        .def_readwrite("opcode", &Phase::raw_opcode)
        .def_property("channel",
            [] (const Phase &p) { return p.channel; },
            [] (Phase &p, Channel val) { p.channel = val; })
        .def_property("sub_channel",
            [] (const Phase &p) { return p.sub_channel; },
            [] (Phase &p, uint8_t val) { p.sub_channel = val; })
        .def_property("lcrd",
            [] (const Phase &p) { return p.lcrd; },
            [] (Phase &p, bool val) { p.lcrd = val; })
        .def_property("exp_comp_ack",
            [] (const Phase &p) { return p.exp_comp_ack; },
            [] (Phase &p, bool val) { p.exp_comp_ack = val; })
        .def_property("snp_attr",
            [] (const Phase &p) { return p.snp_attr; },
            [] (Phase &p, bool val) { p.snp_attr = val; })
        .def_property("allow_retry",
            [] (const Phase &p) { return p.allow_retry; },
            [] (Phase &p, bool val) { p.allow_retry = val; })
        .def_property("do_dwt",
            [] (const Phase &p) { return p.do_dwt; },
            [] (Phase &p, bool val) { p.do_dwt = val; })
        .def_property("data_id",
            [] (const Phase &p) { return p.data_id; },
            [] (Phase &p, uint8_t val) { p.data_id = val; })
        .def_property("c_busy",
            [] (const Phase &p) { return p.c_busy; },
            [] (Phase &p, uint8_t val) { p.c_busy = val; })
        .def_property("pcrd_type",
            [] (const Phase &p) { return p.pcrd_type; },
            [] (Phase &p, uint8_t val) { p.pcrd_type = val; })
        .def_property("qos",
            [] (const Phase &p) { return p.qos; },
            [] (Phase &p, uint8_t val) { p.qos = val; })
        .def_property("resp",
            [] (const Phase &p) { return p.resp; },
            [] (Phase &p, Resp val) { p.resp = val; })
        .def_property("fwd_state",
            [] (const Phase &p) { return p.fwd_state; },
            [] (Phase &p, Resp val) { p.fwd_state = val; })
        .def_property("order",
            [] (const Phase &p) { return p.order; },
            [] (Phase &p, Order val) { p.order = val; })
        .def_property("resp_err",
            [] (const Phase &p) { return p.resp_err; },
            [] (Phase &p, RespErr val) { p.resp_err = val; })
        .def_property("tag_op",
            [] (const Phase &p) { return p.tag_op; },
            [] (Phase &p, TagOp val) { p.tag_op = val; })
        ;

    py::enum_<Resp>(tlm_chi, "Resp")
        .value("RESP_I", RESP_I)
        .value("RESP_SC", RESP_SC)
        .value("RESP_UC", RESP_UC)
        .value("RESP_UD", RESP_UD)
        .value("RESP_SD", RESP_SD)
        .value("RESP_I_PD", RESP_I_PD)
        .value("RESP_SC_PD", RESP_SC_PD)
        .value("RESP_UC_PD", RESP_UC_PD)
        .value("RESP_UD_PD", RESP_UD_PD)
        .value("RESP_SD_PD", RESP_SD_PD)
        ;

    py::enum_<RespErr>(tlm_chi, "RespErr")
        .value("RESP_ERR_OK", RESP_ERR_OK)
        .value("RESP_ERR_EXOK", RESP_ERR_EXOK)
        .value("RESP_ERR_DERR", RESP_ERR_DERR)
        .value("RESP_ERR_NDERR", RESP_ERR_NDERR)
        ;

    py::enum_<Order>(tlm_chi, "Order")
        .value("ORDER_NO_ORDER", ORDER_NO_ORDER)
        .value("ORDER_REQUEST_ACCEPTED", ORDER_REQUEST_ACCEPTED)
        .value("ORDER_REQUEST_ORDER", ORDER_REQUEST_ORDER)
        .value("ORDER_ENDPOINT_ORDER", ORDER_ENDPOINT_ORDER)
        ;

    py::enum_<TagOp>(tlm_chi, "TagOp")
        .value("TAG_OP_INVALID", TAG_OP_INVALID)
        .value("TAG_OP_TRANSFER", TAG_OP_TRANSFER)
        .value("TAG_OP_UPDATE", TAG_OP_UPDATE)
        .value("TAG_OP_MATCH", TAG_OP_MATCH)
        .value("TAG_OP_FETCH", TAG_OP_FETCH)
        ;

    py::enum_<SizeEnum>(tlm_chi, "Size")
        .value("SIZE_1", SIZE_1)
        .value("SIZE_2", SIZE_2)
        .value("SIZE_4", SIZE_4)
        .value("SIZE_8", SIZE_8)
        .value("SIZE_16", SIZE_16)
        .value("SIZE_32", SIZE_32)
        .value("SIZE_64", SIZE_64)
        ;

    py::enum_<Channel>(tlm_chi, "Channel")
        .value("REQ", CHANNEL_REQ)
        .value("DAT", CHANNEL_DAT)
        .value("RSP", CHANNEL_RSP)
        .value("SNP", CHANNEL_SNP)
        ;

    py::enum_<ReqOpcodeEnum>(tlm_chi, "ReqOpcode")
        .value("READ_SHARED", REQ_OPCODE_READ_SHARED)
        .value("READ_CLEAN", REQ_OPCODE_READ_CLEAN)
        .value("READ_ONCE", REQ_OPCODE_READ_ONCE)
        .value("READ_NO_SNP", REQ_OPCODE_READ_NO_SNP)
        .value("PCRD_RETURN", REQ_OPCODE_PCRD_RETURN)
        .value("READ_UNIQUE", REQ_OPCODE_READ_UNIQUE)
        .value("CLEAN_SHARED", REQ_OPCODE_CLEAN_SHARED)
        .value("CLEAN_INVALID", REQ_OPCODE_CLEAN_INVALID)
        .value("MAKE_INVALID", REQ_OPCODE_MAKE_INVALID)
        .value("CLEAN_UNIQUE", REQ_OPCODE_CLEAN_UNIQUE)
        .value("MAKE_UNIQUE", REQ_OPCODE_MAKE_UNIQUE)
        .value("EVICT", REQ_OPCODE_EVICT)
        .value("EO_BARRIER", REQ_OPCODE_EO_BARRIER)
        .value("EC_BARRIER", REQ_OPCODE_EC_BARRIER)
        .value("READ_NO_SNP_SEP", REQ_OPCODE_READ_NO_SNP_SEP)
        .value("CLEAN_SHARED_PERSIST_SEP", REQ_OPCODE_CLEAN_SHARED_PERSIST_SEP)
        .value("DVM_OP", REQ_OPCODE_DVM_OP)
        .value("WRITE_EVICT_FULL", REQ_OPCODE_WRITE_EVICT_FULL)
        .value("WRITE_CLEAN_PTL", REQ_OPCODE_WRITE_CLEAN_PTL)
        .value("WRITE_CLEAN_FULL", REQ_OPCODE_WRITE_CLEAN_FULL)
        .value("WRITE_UNIQUE_PTL", REQ_OPCODE_WRITE_UNIQUE_PTL)
        .value("WRITE_UNIQUE_FULL", REQ_OPCODE_WRITE_UNIQUE_FULL)
        .value("WRITE_BACK_PTL", REQ_OPCODE_WRITE_BACK_PTL)
        .value("WRITE_BACK_FULL", REQ_OPCODE_WRITE_BACK_FULL)
        .value("WRITE_NO_SNP_PTL", REQ_OPCODE_WRITE_NO_SNP_PTL)
        .value("WRITE_NO_SNP_FULL", REQ_OPCODE_WRITE_NO_SNP_FULL)
        .value("WRITE_UNIQUE_FULL_STASH", REQ_OPCODE_WRITE_UNIQUE_FULL_STASH)
        .value("WRITE_UNIQUE_PTL_STASH", REQ_OPCODE_WRITE_UNIQUE_PTL_STASH)
        .value("STASH_ONCE_SHARED", REQ_OPCODE_STASH_ONCE_SHARED)
        .value("STASH_ONCE_UNIQUE", REQ_OPCODE_STASH_ONCE_UNIQUE)
        ;

    py::enum_<RspOpcodeEnum>(tlm_chi, "RspOpcode")
        .value("RSP_LCRD_RETURN", RSP_OPCODE_RSP_LCRD_RETURN)
        .value("SNP_RESP", RSP_OPCODE_SNP_RESP)
        .value("COMP_ACK", RSP_OPCODE_COMP_ACK)
        .value("RETRY_ACK", RSP_OPCODE_RETRY_ACK)
        .value("COMP", RSP_OPCODE_COMP)
        .value("COMP_DBID_RESP", RSP_OPCODE_COMP_DBID_RESP)
        .value("DBID_RESP", RSP_OPCODE_DBID_RESP)
        .value("PCRD_GRANT", RSP_OPCODE_PCRD_GRANT)
        .value("READ_RECEIPT", RSP_OPCODE_READ_RECEIPT)
        .value("SNP_RESP_FWDED", RSP_OPCODE_SNP_RESP_FWDED)
        .value("TAG_MATCH", RSP_OPCODE_TAG_MATCH)
        .value("RESP_SEP_DATA", RSP_OPCODE_RESP_SEP_DATA)
        .value("PERSIST", RSP_OPCODE_PERSIST)
        .value("COMP_PERSIST", RSP_OPCODE_COMP_PERSIST)
        .value("DBID_RESP_ORD", RSP_OPCODE_DBID_RESP_ORD)
        .value("STASH_DONE", RSP_OPCODE_STASH_DONE)
        .value("COMP_STASH_DONE", RSP_OPCODE_COMP_STASH_DONE)
        .value("COMP_CMO", RSP_OPCODE_COMP_CMO)
        ;

    py::enum_<DatOpcodeEnum>(tlm_chi, "DatOpcode")
        .value("DAT_LCRD_RETURN", DAT_OPCODE_DAT_LCRD_RETURN)
        .value("SNP_RESP_DATA", DAT_OPCODE_SNP_RESP_DATA)
        .value("COPY_BACK_WR_DATA", DAT_OPCODE_COPY_BACK_WR_DATA)
        .value("NON_COPY_BACK_WR_DATA", DAT_OPCODE_NON_COPY_BACK_WR_DATA)
        .value("COMP_DATA", DAT_OPCODE_COMP_DATA)
        .value("SNP_RESP_DATA_PTL", DAT_OPCODE_SNP_RESP_DATA_PTL)
        .value("SNP_RESP_DATA_FWDED", DAT_OPCODE_SNP_RESP_DATA_FWDED)
        .value("WRITE_DATA_CANCEL", DAT_OPCODE_WRITE_DATA_CANCEL)
        .value("DATA_SEP_RESP", DAT_OPCODE_DATA_SEP_RESP)
        .value("NCB_WR_DATA_COMP_ACK", DAT_OPCODE_NCB_WR_DATA_COMP_ACK)
        ;

    py::enum_<SnpOpcodeEnum>(tlm_chi, "SnpOpcode")
        .value("SNP_LCRD_RETURN", SNP_OPCODE_SNP_LCRD_RETURN)
        .value("SNP_SHARED", SNP_OPCODE_SNP_SHARED)
        .value("SNP_CLEAN", SNP_OPCODE_SNP_CLEAN)
        .value("SNP_ONCE", SNP_OPCODE_SNP_ONCE)
        .value("SNP_NOT_SHARED_DIRTY", SNP_OPCODE_SNP_NOT_SHARED_DIRTY)
        .value("SNP_UNIQUE_STASH", SNP_OPCODE_SNP_UNIQUE_STASH)
        .value("SNP_MAKE_INVALID_STASH", SNP_OPCODE_SNP_MAKE_INVALID_STASH)
        .value("SNP_UNIQUE", SNP_OPCODE_SNP_UNIQUE)
        .value("SNP_CLEAN_SHARED", SNP_OPCODE_SNP_CLEAN_SHARED)
        .value("SNP_CLEAN_INVALID", SNP_OPCODE_SNP_CLEAN_INVALID)
        .value("SNP_MAKE_INVALID", SNP_OPCODE_SNP_MAKE_INVALID)
        .value("SNP_STASH_UNIQUE", SNP_OPCODE_SNP_STASH_UNIQUE)
        .value("SNP_STASH_SHARED", SNP_OPCODE_SNP_STASH_SHARED)
        .value("SNP_DVM_OP", SNP_OPCODE_SNP_DVM_OP)
        .value("SNP_QUERY", SNP_OPCODE_SNP_QUERY)
        .value("SNP_SHARED_FWD", SNP_OPCODE_SNP_SHARED_FWD)
        .value("SNP_CLEAN_FWD", SNP_OPCODE_SNP_CLEAN_FWD)
        .value("SNP_ONCE_FWD", SNP_OPCODE_SNP_ONCE_FWD)
        .value("SNP_NOT_SHARED_DIRTY_FWD", SNP_OPCODE_SNP_NOT_SHARED_DIRTY_FWD)
        .value("SNP_PREFER_UNIQUE", SNP_OPCODE_SNP_PREFER_UNIQUE)
        .value("SNP_PREFER_UNIQUE_FWD", SNP_OPCODE_SNP_PREFER_UNIQUE_FWD)
        .value("SNP_UNIQUE_FWD", SNP_OPCODE_SNP_UNIQUE_FWD)
        ;
}

EmbeddedPyBind embed_("tlm_chi", &tlm_chi_pybind);

} // namespace
} // namespace gem5
