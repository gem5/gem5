/*
 * Copyright (c) 2010 Massachusetts Institute of Technology
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
 * Authors: Chia-Hsin Owen Chen
 */

#ifndef __ORIONCONFIG_H__
#define __ORIONCONFIG_H__

#include <iostream>
#include <map>
#include <sstream>

#include "mem/ruby/network/orion/Type.hh"

class TechParameter;

class OrionConfig
{
  public:
    OrionConfig(const string& cfg_fn_);
    OrionConfig(const OrionConfig& orion_cfg_);
    ~OrionConfig();

  public:
    void set_num_in_port(uint32_t num_in_port_);
    void set_num_out_port(uint32_t num_out_port_);
    void set_num_vclass(uint32_t num_vclass_);
    void set_num_vchannel(uint32_t num_vchannel_);
    void set_in_buf_num_set(uint32_t in_buf_num_set_);
    void set_flit_width(uint32_t flit_width_);

    void read_file(const std::string& filename_);
    void print_config(std::ostream& out_);

  public:
    template<class T>
    T get(const std::string& key_) const;
    const TechParameter* get_tech_param_ptr() const { return m_tech_param_ptr; }
    uint32_t get_num_in_port() const { return m_num_in_port; }
    uint32_t get_num_out_port() const { return m_num_out_port; }
    uint32_t get_num_vclass() const { return m_num_vclass; }
    uint32_t get_num_vchannel() const { return m_num_vchannel; }
    uint32_t get_in_buf_num_set() const { return m_in_buf_num_set; }
    uint32_t get_flit_width() const { return m_flit_width; }

  private:
    std::map<std::string, std::string> m_params_map;

    TechParameter* m_tech_param_ptr;
    uint32_t m_num_in_port;
    uint32_t m_num_out_port;
    uint32_t m_num_vclass;
    uint32_t m_num_vchannel;
    uint32_t m_in_buf_num_set;
    uint32_t m_flit_width;

  protected:
    struct key_not_found
    {
      std::string m_key;
      key_not_found(const std::string& key_ = string()) : m_key(key_)
      {}
    };
    template<class T>
    static T string_as_T(const std::string& str_);
    template<class T>
    static std::string T_as_string(const T& t_);

  private:
    static std::string ms_param_name[];
};

template<class T>
T OrionConfig::get(const string& key_) const
{
    std::map<std::string, std::string>::const_iterator it;

    it = m_params_map.find(key_);
    if (it == m_params_map.end()) 
    {
        std::cerr << key_ << " NOT FOUND!" << std::endl;
        throw key_not_found(key_);
    }
    return string_as_T<T>(it->second);
}

    template<class T>
T OrionConfig::string_as_T(const string& str_)
{
    T ret;
    std::istringstream ist(str_);
    ist >> ret;
    return ret;
}

template<>
inline string OrionConfig::string_as_T<string>(const string& str_)
{
    return str_;
}

template<>
inline bool OrionConfig::string_as_T<bool>(const string& str_)
{
    bool ret;
    if (str_ == string("TRUE"))
    {
        ret = true;
    }
    else if (str_ == string("FALSE"))
    {
        ret = false;
    }
    else
    {
        std::cerr << "Invalid bool value: '" << str_ <<
          "'. Treated as FALSE." << std::endl;
        ret = false;
    }
    return ret;
}

template<class T>
string OrionConfig::T_as_string(const T& t)
{
    std::ostringstream ost;
    ost << t;
    return ost.str();
}

#endif

