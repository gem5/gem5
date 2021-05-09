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

/*
 * Copyright 2018 Google, Inc.
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

#include <sstream>

#include "base/cprintf.hh"
#include "systemc/core/object.hh"
#include "systemc/ext/utils/messages.hh"
#include "systemc/ext/utils/sc_report_handler.hh"
#include "systemc/ext/utils/sc_vector.hh"

namespace sc_core
{

sc_vector_base::size_type sc_vector_base::size() const { return objs.size(); }

const std::vector<sc_object *> &
sc_vector_base::get_elements() const
{
    elements.clear();
    for (auto ptr: objs) {
        sc_object *obj_ptr = objectCast(ptr);
        if (obj_ptr)
            elements.push_back(obj_ptr);
    }
    return elements;
}

void
sc_vector_base::checkIndex(size_type index) const
{
    if (index >= size()) {
        std::ostringstream ss;
        gem5::ccprintf(ss, "%s[%d] >= size() = %d", name(), index, size());
        SC_REPORT_ERROR(sc_core::SC_ID_OUT_OF_BOUNDS_, ss.str().c_str());
        sc_abort();
    }
}

void
sc_vector_base::forceParent() const
{
    sc_gem5::pushParentObj(get_parent_object());
}

void
sc_vector_base::unforceParent() const
{
    sc_gem5::popParentObj();
}

void
sc_vector_base::reportEmpty(const char *kind_, bool empty_dest) const
{
    std::ostringstream ss;

    ss << "target `" << name() << "' " << "(" << kind_ << ") ";

    if (!size())
        ss << "not initialised yet";
    else if (empty_dest)
        ss << "empty range given";
    else
        ss << "empty destination range given";

    SC_REPORT_WARNING(SC_ID_VECTOR_BIND_EMPTY_, ss.str().c_str());
}

} // namespace sc_core
