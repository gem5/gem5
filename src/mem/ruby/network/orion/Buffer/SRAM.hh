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

#ifndef __SRAM_H__
#define __SRAM_H__

#include "mem/ruby/network/orion/OrionConfig.hh"
#include "mem/ruby/network/orion/TechParameter.hh"
#include "mem/ruby/network/orion/Type.hh"

class OutdrvUnit;
class AmpUnit;
class BitlineUnit;
class MemUnit;
class PrechargeUnit;
class WordlineUnit;
class DecoderUnit;

class SRAM
{
  public:
    SRAM(
      uint32_t num_entry_,
      uint32_t line_width_,
      bool is_fifo_,
      bool is_outdrv_,
      uint32_t num_read_port_,
      uint32_t num_write_port_,
      uint32_t num_data_end_,
      const string& rowdec_model_str_,
      const string& wl_model_str_,
      const string& bl_pre_model_str_,
      const string& mem_model_str_,
      const string& bl_model_str_,
      const string& amp_model_str_,
      const string& outdrv_model_str_,
      const TechParameter* tech_param_ptr_
    );
    ~SRAM();

  public:
    uint32_t get_line_width() const { return m_line_width; }
    uint32_t get_num_data_end() const { return m_num_data_end; }
    uint32_t get_num_read_port() const { return m_num_read_port; }
    uint32_t get_num_write_port() const { return m_num_write_port; }
    uint32_t get_num_port() const { return (m_num_read_port+m_num_write_port); }
    bool get_is_outdrv() const { return m_is_outdrv; }
    uint32_t get_num_row() const { return m_num_entry; }
    uint32_t get_num_col() const { return m_line_width; }

    double calc_e_read(bool is_max_) const;
    double calc_e_write(bool is_max_) const;
    double calc_i_static() const;

  private:
    void init();

  private:
    uint32_t m_num_entry;
    uint32_t m_line_width;
    bool m_is_fifo;
    bool m_is_outdrv;
    string m_rowdec_model_str;
    string m_wl_model_str;
    string m_bl_pre_model_str;
    string m_mem_model_str;
    string m_bl_model_str;
    string m_amp_model_str;
    string m_outdrv_model_str;
    const TechParameter* m_tech_param_ptr;

    OutdrvUnit* m_outdrv_unit_ptr;
    AmpUnit* m_amp_unit_ptr;
    BitlineUnit* m_bl_unit_ptr;
    MemUnit* m_mem_unit_ptr;
    PrechargeUnit* m_bl_pre_unit_ptr;
    WordlineUnit* m_wl_unit_ptr;
    DecoderUnit* m_rowdec_unit_ptr;

    uint32_t m_num_read_port;
    uint32_t m_num_write_port;
    uint32_t m_num_data_end;

    uint32_t m_rowdec_width;
};

#endif

