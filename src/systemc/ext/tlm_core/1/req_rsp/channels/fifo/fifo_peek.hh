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

#ifndef __TLM_CORE_1_REQ_RSP_CHANNELS_FIFO_FIFO_PEEK_HH__
#define __TLM_CORE_1_REQ_RSP_CHANNELS_FIFO_FIFO_PEEK_HH__

namespace tlm
{

template <typename T>
inline T
tlm_fifo<T>::peek(tlm_tag<T> *) const
{
    while (is_empty()) {
        // call free-standing sc_core::wait(),
        // since sc_prim_channel::wait(.) is not const
        sc_core::wait(m_data_written_event);
    }
    return buffer.read_data();
}

template <typename T>
inline bool
tlm_fifo<T>::nb_peek(T &t) const
{
    if (used() < 1) {
        return false;
    }

    t = buffer.peek_data(0);
    return true;
}

template <typename T>
inline bool
tlm_fifo<T>::nb_peek(T &t, int n) const
{
    if (n >= used() || n < -1) {
        return false;
    }

    if (n == -1) {
        n = used() - 1;
    }

    t = buffer.peek_data(n);
    return true;
}

template <typename T>
inline bool
tlm_fifo<T>::nb_can_peek(tlm_tag<T> *) const
{
    return !is_empty();
}

template <typename T>
inline bool
tlm_fifo<T>::nb_poke(const T &t, int n)
{
    if (n >= used() || n < 0) {
        return false;
    }

    buffer.poke_data(n) = t;
    return true;
}

} // namespace tlm

#endif /* __TLM_CORE_1_REQ_RSP_CHANNELS_FIFO_FIFO_PEEK_HH__ */
