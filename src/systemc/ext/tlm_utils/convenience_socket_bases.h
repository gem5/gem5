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
#ifndef __SYSTEMC_EXT_TLM_UTILS_CONVENIENCE_SOCKET_BASES_H__
#define __SYSTEMC_EXT_TLM_UTILS_CONVENIENCE_SOCKET_BASES_H__

namespace sc_core
{

class sc_object;

} // namespace sc_core

namespace tlm_utils
{

// Implementation-defined base class helper for convenience sockets.
class convenience_socket_base
{
  public:
    void display_warning(const char *msg) const;
    void display_error(const char *msg) const;

  protected:
    virtual ~convenience_socket_base() {}

  private:
    virtual const char *get_report_type() const = 0;
    virtual const sc_core::sc_object *get_socket() const = 0;
};

// Implementation-defined base class helper for simple sockets.
class simple_socket_base : public convenience_socket_base
{
    virtual const char *get_report_type() const;

  protected:
    void elaboration_check(const char *action) const;
};

// Implementation-defined base class helper for passthrough sockets.
class passthrough_socket_base : public convenience_socket_base
{
    virtual const char *get_report_type() const;
};

// Implementation-defined base class helper for multi sockets.
class multi_socket_base : public convenience_socket_base
{
    virtual const char *get_report_type() const;
};

// Implementation-defined base class for callback helpers.
class convenience_socket_cb_holder
{
  public:
    void display_warning(const char *msg) const;
    void display_error(const char *msg) const;

  protected:
    explicit convenience_socket_cb_holder(convenience_socket_base *owner) :
        m_owner(owner)
    {}

  private:
    convenience_socket_base *m_owner;
};

} // namespace tlm_utils

#endif /* __SYSTEMC_EXT_TLM_UTILS_CONVENIENCE_SOCKET_BASES_H__ */
