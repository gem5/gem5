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

#ifndef __SYSTEMC_EXT_TLM_CORE_2_GENERIC_PAYLOAD_HELPERS_HH__
#define __SYSTEMC_EXT_TLM_CORE_2_GENERIC_PAYLOAD_HELPERS_HH__

namespace tlm
{

enum tlm_endianness { TLM_UNKNOWN_ENDIAN, TLM_LITTLE_ENDIAN, TLM_BIG_ENDIAN };

inline tlm_endianness
get_host_endianness()
{
    static tlm_endianness host_endianness = TLM_UNKNOWN_ENDIAN;

    if (host_endianness == TLM_UNKNOWN_ENDIAN) {
        unsigned int number = 1;
        unsigned char *p_msb_or_lsb = (unsigned char *)&number;
        host_endianness = (p_msb_or_lsb[0] == 0) ?
            TLM_BIG_ENDIAN : TLM_LITTLE_ENDIAN;
    }
    return host_endianness;
}

inline bool
host_has_little_endianness()
{
    static tlm_endianness host_endianness = TLM_UNKNOWN_ENDIAN;
    static bool host_little_endian = false;

    if (host_endianness == TLM_UNKNOWN_ENDIAN) {
        unsigned int number = 1;
        unsigned char *p_msb_or_lsb = (unsigned char *)&number;

        host_little_endian = (p_msb_or_lsb[0] == 0) ? false : true;
    }

    return host_little_endian;
}

inline bool
has_host_endianness(tlm_endianness endianness)
{
    if (host_has_little_endianness()) {
        return endianness == TLM_LITTLE_ENDIAN;
    } else {
        return endianness == TLM_BIG_ENDIAN;
    }
}

} // namespace tlm

#endif /* __SYSTEMC_EXT_TLM_CORE_2_GENERIC_PAYLOAD_HELPERS_HH__ */
