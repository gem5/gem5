/*
 * Copyright (c) 2009 Princeton University
 * Copyright (c) 2009 The Regents of the University of California
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
 *
 * Authors:  Hangsheng Wang (Orion 1.0, Princeton)
 *           Xinping Zhu (Orion 1.0, Princeton)
 *           Xuning Chen (Orion 1.0, Princeton)
 *           Bin Li (Orion 2.0, Princeton)
 *           Kambiz Samadi (Orion 2.0, UC San Diego)
 */

#ifndef __BUFFER_H__
#define __BUFFER_H__

#include "mem/ruby/network/orion/Type.hh"

class OrionConfig;
class TechParameter;

class SRAM;
class Register;

class Buffer
{
  public:
    enum BufferModel
    {
      NO_MODEL = 0,
      BUF_SRAM,
      BUF_REG
    };

  public:
    Buffer(
      const string& buffer_model_str_,
      bool is_fifo_,
      bool is_outdrv_,
      uint32_t num_entry_,
      uint32_t line_width_,
      uint32_t num_read_port_,
      uint32_t num_write_port_,
      const OrionConfig* orion_cfg_ptr_
    );
    ~Buffer();

  public:
    double get_dynamic_energy(bool is_read_, bool is_max_) const;
    double get_static_power() const;

    void print_all() const;

  private:
    void init();

  private:
    BufferModel m_buffer_model;
    uint32_t m_num_entry;
    uint32_t m_line_width;

    bool m_is_fifo;
    bool m_is_outdrv;
    uint32_t m_num_read_port;
    uint32_t m_num_write_port;
    const OrionConfig* m_orion_cfg_ptr;
    const TechParameter* m_tech_param_ptr;

    SRAM* m_sram_ptr;
    Register* m_reg_ptr;
};

#endif

