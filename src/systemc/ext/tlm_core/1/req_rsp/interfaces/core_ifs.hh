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

#ifndef __SYSTEMC_EXT_TLM_CORE_1_REQ_RSP_INTERFACES_CORE_IFS_HH__
#define __SYSTEMC_EXT_TLM_CORE_1_REQ_RSP_INTERFACES_CORE_IFS_HH__

#include "tag.hh"

namespace tlm
{

// Bidirectional blocking interfaces.
template <typename REQ, typename RSP>
class tlm_transport_if : public virtual sc_core::sc_interface
{
  public:
    virtual RSP transport(const REQ &) = 0;

    virtual void
    transport(const REQ &req, RSP &rsp)
    {
        rsp = transport(req);
    }
};

// Uni-directional blocking interfaces.
template <typename T>
class tlm_blocking_get_if : public virtual sc_core::sc_interface
{
  public:
    virtual T get(tlm_tag<T> *t=nullptr) = 0;
    virtual void get(T &t) { t = get(); }
};

template <typename T>
class tlm_blocking_put_if : public virtual sc_core::sc_interface
{
  public:
    virtual void put(const T &t) = 0;
};

// Uni-directional non blocking interfaces.

template <typename T>
class tlm_nonblocking_get_if : public virtual sc_core::sc_interface
{
  public:
    virtual bool nb_get(T &t) = 0;
    virtual bool nb_can_get(tlm_tag<T> *t=nullptr) const = 0;
    virtual const sc_core::sc_event &
        ok_to_get(tlm_tag<T> *t=nullptr) const = 0;
};

template <typename T>
class tlm_nonblocking_put_if : public virtual sc_core::sc_interface
{
  public:
    virtual bool nb_put(const T &t) = 0;
    virtual bool nb_can_put(tlm_tag<T> *t=nullptr) const = 0;
    virtual const sc_core::sc_event &
        ok_to_put(tlm_tag<T> *t=nullptr) const = 0;
};

// Combined uni-directional blocking and non blocking.
template <typename T>
class tlm_get_if : public virtual tlm_blocking_get_if<T>,
    public virtual tlm_nonblocking_get_if<T>
{};

template <typename T>
class tlm_put_if : public virtual tlm_blocking_put_if<T>,
    public virtual tlm_nonblocking_put_if<T>
{};

// Peek interfaces.
template <typename T>
class tlm_blocking_peek_if : public virtual sc_core::sc_interface
{
  public:
    virtual T peek(tlm_tag<T> *t=nullptr) const = 0;
    virtual void peek(T &t) const { t = peek(); }
};

template <typename T>
class tlm_nonblocking_peek_if : public virtual sc_core::sc_interface
{
  public:
    virtual bool nb_peek(T &t) const = 0;
    virtual bool nb_can_peek(tlm_tag<T> *t=nullptr) const = 0;
    virtual const sc_core::sc_event &
        ok_to_peek(tlm_tag<T> *t=nullptr) const = 0;
};

template <typename T>
class tlm_peek_if :
    public virtual tlm_blocking_peek_if<T>,
    public virtual tlm_nonblocking_peek_if<T>
{};

// Get_peek interfaces.
template <typename T>
class tlm_blocking_get_peek_if : public virtual tlm_blocking_get_if<T>,
    public virtual tlm_blocking_peek_if<T>
{};

template <typename T>
class tlm_nonblocking_get_peek_if : public virtual tlm_nonblocking_get_if<T>,
    public virtual tlm_nonblocking_peek_if<T>
{};

template <typename T>
class tlm_get_peek_if : public virtual tlm_get_if<T>,
    public virtual tlm_peek_if<T>, public virtual tlm_blocking_get_peek_if<T>,
    public virtual tlm_nonblocking_get_peek_if<T>
{};

} // namespace tlm

#endif /* __SYSTEMC_EXT_TLM_CORE_1_REQ_RSP_INTERFACES_CORE_IFS_HH__ */
