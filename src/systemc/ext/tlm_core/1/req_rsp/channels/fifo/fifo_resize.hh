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

#ifndef __TLM_CORE_1_REQ_RSP_CHANNELS_FIFO_FIFO_RESIZE_HH__
#define __TLM_CORE_1_REQ_RSP_CHANNELS_FIFO_FIFO_RESIZE_HH__

// Resize interface.
namespace tlm
{

template <typename T>
inline void
tlm_fifo<T>::nb_expand(unsigned int n)
{
    if (m_size >= 0) {
        m_expand = true;
        m_size += n;
        request_update();
    }
}

template <typename T>
inline void
tlm_fifo<T>::nb_unbound(unsigned int n)
{
    m_expand = true;
    m_size = -n;

    if (buffer.size() < static_cast<int>(n)) {
        buffer.resize(n);
    }

    request_update();
}

template <typename T>
inline bool
tlm_fifo<T>::nb_reduce(unsigned int n)
{
    if (m_size < 0) {
        return false;
    }

    return nb_bound(size() - n);
}

template <typename T>
inline bool
tlm_fifo<T>::nb_bound(unsigned int new_size)
{
    bool ret = true;

    if (static_cast<int>(new_size) < used()) {
        new_size = used();
        ret = false;
    }

    m_size = new_size;
    return ret;
}

} // namespace tlm

#endif /* __TLM_CORE_1_REQ_RSP_CHANNELS_FIFO_FIFO_RESIZE_HH__ */
