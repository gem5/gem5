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
 * Copyright 2026 The Regents of The University of California
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

#include <algorithm>
#include <stdexcept>
#include <string>

#include "systemc.h"

using namespace sc_core;

struct Element : sc_object
{
    int value;

    Element(const char *name, int value = 0, bool fail = false)
        : sc_object(name), value(value)
    {
        if (fail) {
            throw std::runtime_error("element construction failed");
        }
    }
};

struct Owner : sc_module
{
    sc_vector<Element> elements;

    Owner(sc_module_name name) : sc_module(name), elements("elements") {}
};

struct Appender : sc_module
{
    Appender(sc_module_name name, Owner &owner, sc_vector<Element> &roots)
        : sc_module(name)
    {
        owner.elements.emplace_back(2);
        roots.emplace_back(3);

        bool threw = false;
        try {
            owner.elements.emplace_back(4, true);
        } catch (const std::runtime_error &) {
            threw = true;
        }
        sc_assert(threw);
        sc_assert(owner.elements.size() == 2);

        Element after_failure("after_failure");
        sc_assert(after_failure.get_parent_object() == this);
    }
};

int
sc_main(int, char *[])
{
    sc_vector<Element> roots("roots");
    Owner owner("owner");
    owner.elements.emplace_back(1);
    Appender appender("appender", owner, roots);

    sc_assert(owner.elements.size() == 2);
    const auto &children = owner.get_child_objects();
    for (size_t i = 0; i < owner.elements.size(); ++i) {
        const auto &element = owner.elements[i];
        sc_assert(element.get_parent_object() == &owner);
        sc_assert(element.value == int(i + 1));
        sc_assert(std::string(element.name()).find("owner.elements_") == 0);
        sc_assert(sc_find_object(element.name()) == &element);
        sc_assert(std::find(children.begin(), children.end(), &element) !=
                  children.end());
    }
    sc_assert(roots.size() == 1);
    sc_assert(roots[0].get_parent_object() == nullptr);
    sc_assert(roots[0].value == 3);
    const auto &top = sc_get_top_level_objects();
    sc_assert(std::find(top.begin(), top.end(), &roots[0]) != top.end());

    std::cout << "Program completed" << std::endl;
    return 0;
}
