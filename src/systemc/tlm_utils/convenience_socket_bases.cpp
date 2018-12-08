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

#include "tlm_utils/convenience_socket_bases.h"

#include "sysc/kernel/sc_object.h"
#include "sysc/kernel/sc_simcontext.h"
#include "sysc/utils/sc_report.h"
#include <sstream>

namespace tlm_utils {

void
convenience_socket_base::display_warning(const char* text) const
{
  std::stringstream s;
  s << get_socket()->name() << ": " << text;
  SC_REPORT_WARNING(get_report_type(), s.str().c_str());
}

void
convenience_socket_base::display_error(const char* text) const
{
  std::stringstream s;
  s << get_socket()->name() << ": " << text;
  SC_REPORT_ERROR(get_report_type(), s.str().c_str());
}

//simple helpers for warnings an errors to shorten in code notation

void
convenience_socket_cb_holder::display_warning(const char* msg) const
{
  m_owner->display_warning(msg);
}

void
convenience_socket_cb_holder::display_error(const char* msg) const
{
  m_owner->display_error(msg);
}

const char*
simple_socket_base::get_report_type() const {
  return "/OSCI_TLM-2/simple_socket";
}

void
simple_socket_base::elaboration_check(const char* action) const
{
  if (sc_core::sc_get_curr_simcontext()->elaboration_done()) {
    std::stringstream s;
    s << " elaboration completed, " << action << " not allowed";
    display_error(s.str().c_str());
  }
}

const char*
passthrough_socket_base::get_report_type() const {
  return "/OSCI_TLM-2/passthrough_socket";
}

const char*
multi_socket_base::get_report_type() const {
  return "/OSCI_TLM-2/multi_socket";
}

} // namespace tlm_utils
