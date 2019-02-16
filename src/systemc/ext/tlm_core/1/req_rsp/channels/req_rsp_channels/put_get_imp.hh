/*****************************************************************************

  Licensed to Accellera Systems Initiative Inc. (Accellera) under one or
  more contributor license agreements.  See the NOTICE file distributed
  with this work for additional information regarding copyright ownership.
  Accellera licenses this file to you under the Apache License, Version 2.0
  (the "License"); you may not use this file except in compliance with the
  License.  You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
  implied.  See the License for the specific language governing
  permissions and limitations under the License.

 *****************************************************************************/

#ifndef \
    __SYSTEMC_EXT_TLM_CORE_1_REQ_RSP_CHANNELS_REQ_RSP_CHANNELS_PUT_GET_IMP_HH__
#define \
    __SYSTEMC_EXT_TLM_CORE_1_REQ_RSP_CHANNELS_REQ_RSP_CHANNELS_PUT_GET_IMP_HH__

#include "../../interfaces/master_slave_ifs.hh"

namespace tlm
{

template <typename PUT_DATA, typename GET_DATA>
class tlm_put_get_imp : private virtual tlm_put_if<PUT_DATA>,
  private virtual tlm_get_peek_if<GET_DATA>
{
  public:
    tlm_put_get_imp(tlm_put_if<PUT_DATA> &p, tlm_get_peek_if<GET_DATA> &g) :
        put_fifo(p), get_fifo(g)
    {}

    // Put interface.
    void put(const PUT_DATA &t) { put_fifo.put(t); }
    bool nb_put(const PUT_DATA &t) { return put_fifo.nb_put(t); }
    bool
    nb_can_put(tlm_tag<PUT_DATA> *t=nullptr) const
    {
        return put_fifo.nb_can_put(t);
    }
    const sc_core::sc_event &
    ok_to_put(tlm_tag<PUT_DATA> *t=nullptr) const
    {
        return put_fifo.ok_to_put(t);
    }

    // Get interface.
    GET_DATA get(tlm_tag<GET_DATA> * =nullptr) { return get_fifo.get(); }
    bool nb_get(GET_DATA &t) { return get_fifo.nb_get(t); }
    bool
    nb_can_get(tlm_tag<GET_DATA> *t=nullptr) const
    {
        return get_fifo.nb_can_get(t);
    }

    virtual const sc_core::sc_event &
    ok_to_get(tlm_tag<GET_DATA> *t=nullptr) const
    {
        return get_fifo.ok_to_get(t);
    }

    // Peek interface.
    GET_DATA
    peek(tlm_tag<GET_DATA> * =nullptr) const
    {
        return get_fifo.peek();
    }
    bool nb_peek(GET_DATA &t) const { return get_fifo.nb_peek(t); }
    bool
    nb_can_peek(tlm_tag<GET_DATA> *t=nullptr) const
    {
        return get_fifo.nb_can_peek(t);
    }

    virtual const sc_core::sc_event &
    ok_to_peek(tlm_tag<GET_DATA> *t=nullptr) const
    {
        return get_fifo.ok_to_peek(t);
    }

  private:
    tlm_put_if<PUT_DATA> &put_fifo;
    tlm_get_peek_if<GET_DATA> &get_fifo;
};

template <typename REQ, typename RSP>
class tlm_master_imp : private tlm_put_get_imp<REQ, RSP>,
    public virtual tlm_master_if<REQ, RSP>
{
  public:
    tlm_master_imp(tlm_put_if<REQ> &req, tlm_get_peek_if<RSP> &rsp) :
        tlm_put_get_imp<REQ, RSP>(req, rsp)
    {}
};

template <typename REQ, typename RSP>
class tlm_slave_imp : private tlm_put_get_imp<RSP, REQ>,
    public virtual tlm_slave_if<REQ, RSP>
{
  public:
    tlm_slave_imp(tlm_get_peek_if<REQ> &req, tlm_put_if<RSP> &rsp) :
        tlm_put_get_imp<RSP, REQ>(rsp, req)
    {}
};

} // namespace tlm

#endif
/*__SYSTEMC_EXT_TLM_CORE_1_REQ_RSP_CHANNELS_REQ_RSP_CHANNELS_PUT_GET_IMP_HH__*/
