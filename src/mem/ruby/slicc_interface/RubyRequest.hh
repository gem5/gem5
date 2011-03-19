/*
 * Copyright (c) 2009 Mark D. Hill and David A. Wood
 * All rights reserved.
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

#ifndef __MEM_RUBY_SLICC_INTERFACE_RUBY_REQUEST_HH__
#define __MEM_RUBY_SLICC_INTERFACE_RUBY_REQUEST_HH__

#include <ostream>

#include "mem/packet.hh"
#include "mem/protocol/RubyAccessMode.hh"
#include "mem/protocol/CacheRequestType.hh"
#include "mem/protocol/Message.hh"
#include "mem/protocol/PrefetchBit.hh"
#include "mem/ruby/common/Address.hh"

typedef void* RubyPortHandle;
enum RubyRequestType {
  RubyRequestType_NULL,
  RubyRequestType_IFETCH,
  RubyRequestType_LD,
  RubyRequestType_ST,
  RubyRequestType_Load_Linked,
  RubyRequestType_Store_Conditional,
  RubyRequestType_RMW_Read,
  RubyRequestType_RMW_Write,
  RubyRequestType_Locked_RMW_Read,
  RubyRequestType_Locked_RMW_Write,
  RubyRequestType_NUM
};

class RubyRequest
{
  public:
    uint64_t paddr;
    uint8_t* data;
    int len;
    uint64_t pc;
    RubyRequestType type;
    RubyAccessMode access_mode;
    PacketPtr pkt;
    unsigned proc_id;

    RubyRequest() {}
    RubyRequest(uint64_t _paddr,
                uint8_t* _data,
                int _len,
                uint64_t _pc,
                RubyRequestType _type,
                RubyAccessMode _access_mode,
                PacketPtr _pkt,
                unsigned _proc_id = 100)
        : paddr(_paddr),
          data(_data),
          len(_len),
          pc(_pc),
          type(_type),
          access_mode(_access_mode),
          pkt(_pkt),
          proc_id(_proc_id)
    {}

    void print(std::ostream& out) const;
};

std::string RubyRequestType_to_string(const RubyRequestType& obj);
RubyRequestType string_to_RubyRequestType(std::string);
std::ostream& operator<<(std::ostream& out, const RubyRequestType& obj);
std::ostream& operator<<(std::ostream& out, const RubyRequest& obj);

#endif
