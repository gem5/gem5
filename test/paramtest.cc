/*
 * Copyright (c) 2003 The Regents of The University of Michigan
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

//
// This file is not part of the regular simulator.  It is solely for
// testing the parameter code.  Edit the Makefile to add param_test.cc
// to the sources list, then use configs/test.ini as the configuration
// file.
//
#include "sim/sim_object.hh"
#include "mem/cache/cache.hh"

class ParamTest : public SimObject
{
  public:
    ParamTest(string name)
        : SimObject(name)
    {
    }

    virtual ~ParamTest() {}
};

enum Enum1Type { Enum0 };
enum Enum2Type { Enum10 };

BEGIN_DECLARE_SIM_OBJECT_PARAMS(ParamTest)

    Param<int> intparam;
    VectorParam<int> vecint;
    Param<string> stringparam;
    VectorParam<string> vecstring;
    Param<bool> boolparam;
    VectorParam<bool> vecbool;
    SimObjectParam<BaseMemory *> memobj;
    SimObjectVectorParam<BaseMemory *> vecmemobj;
    SimpleEnumParam<Enum1Type> enum1;
    MappedEnumParam<Enum2Type> enum2;
    SimpleEnumVectorParam<Enum1Type> vecenum1;
    MappedEnumVectorParam<Enum2Type> vecenum2;

END_DECLARE_SIM_OBJECT_PARAMS(ParamTest)

const char *enum1_strings[] =
{
    "zero", "one", "two", "three"
};

const EnumParamMap enum2_map[] =
{
    { "ten", 10 },
    { "twenty", 20 },
    { "thirty", 30 },
    { "fourty", 40 }
};

BEGIN_INIT_SIM_OBJECT_PARAMS(ParamTest)

    INIT_PARAM(intparam, "intparam"),
    INIT_PARAM(vecint, "vecint"),
    INIT_PARAM(stringparam, "stringparam"),
    INIT_PARAM(vecstring, "vecstring"),
    INIT_PARAM(boolparam, "boolparam"),
    INIT_PARAM(vecbool, "vecbool"),
    INIT_PARAM(memobj, "memobj"),
    INIT_PARAM(vecmemobj, "vecmemobj"),
    INIT_ENUM_PARAM(enum1, "enum1", enum1_strings),
    INIT_ENUM_PARAM(enum2, "enum2", enum2_map),
    INIT_ENUM_PARAM(vecenum1, "vecenum1", enum1_strings),
    INIT_ENUM_PARAM(vecenum2, "vecenum2", enum2_map)

END_INIT_SIM_OBJECT_PARAMS(ParamTest)


CREATE_SIM_OBJECT(ParamTest)
{
    return new ParamTest(getInstanceName());
}

REGISTER_SIM_OBJECT("ParamTest", ParamTest)
