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

#ifndef __TLM_CORE_1_REQ_RSP_CHANNELS_FIFO_FIFO_PUT_GET_HH__
#define __TLM_CORE_1_REQ_RSP_CHANNELS_FIFO_FIFO_PUT_GET_HH__

namespace tlm
{

// Get interface.
template <typename T>
inline T
tlm_fifo<T>::get(tlm_tag<T> *)
{
    while (is_empty()) {
        wait(m_data_written_event);
    }

    m_num_read++;
    request_update();

    return buffer.read();
}

// Non-blocking read.
template <typename T>
inline bool
tlm_fifo<T>::nb_get(T &val_)
{
    if (is_empty()) {
        return false;
    }

    m_num_read++;
    request_update();

    val_ = buffer.read();

    return true;
}

template <typename T>
inline bool
tlm_fifo<T>::nb_can_get(tlm_tag<T> *) const
{
    return !is_empty();
}


// Put interface.
template <typename T>
inline void
tlm_fifo<T>::put(const T &val_)
{
    while (is_full()) {
        wait(m_data_read_event);
    }

    if (buffer.is_full()) {
        buffer.resize(buffer.size() * 2);
    }

    m_num_written++;
    buffer.write(val_);

    request_update();
}

template <typename T>
inline bool
tlm_fifo<T>::nb_put(const T &val_)
{
    if (is_full()) {
        return false;
    }

    if (buffer.is_full()) {
        buffer.resize(buffer.size() * 2);
    }

    m_num_written++;
    buffer.write(val_);
    request_update();

    return true;
}

template <typename T>
inline bool
tlm_fifo<T>::nb_can_put(tlm_tag<T> *) const
{
    return !is_full();
}

} // namespace tlm

#endif /* __TLM_CORE_1_REQ_RSP_CHANNELS_FIFO_FIFO_PUT_GET_HH__ */
