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

#define SC_ENABLE_OBSERVERS
#define SC_INCLUDE_FX
#include "systemc.h"

#if SC_CPLUSPLUS < 201103L
# define override /* empty */
#endif // SC_CPLUSPLUS >= 201103L

template<typename Observed, typename Base>
class observer: public Base {
    virtual void construct(const Observed&) override {
        std::cout << "construct" << std::endl;
    }
    virtual void destruct(const Observed&) override {
        std::cout << "destruct" << std::endl;
    }
    virtual void read(const Observed&) override {
        std::cout << "read" << std::endl;
    }
    virtual void write(const Observed&) override {
        std::cout << "write" << std::endl;
    }
};

template<typename T, typename Observed, typename Base>
void observe(char const *name) {
    std::cout << std::endl;
    std::cout << name << std::endl;

    observer<Observed, Base> o;
    T value(&o);

    std::cout << "to_short  "; value.to_short();
    std::cout << "to_ushort "; value.to_ushort();
    std::cout << "to_int    "; value.to_int();
    std::cout << "to_uint   "; value.to_uint();
    std::cout << "to_long   "; value.to_long();
    std::cout << "to_ulong  "; value.to_ulong();
    std::cout << "to_int64  "; value.to_int64();
    std::cout << "to_uint64 "; value.to_uint64();
    std::cout << "to_float  "; value.to_float();
    std::cout << "to_double "; value.to_double();
}

int sc_main(int, char *[]) {
    observe<sc_fxval, sc_fxval, sc_dt::sc_fxval_observer>("sc_fxval");
    observe<sc_fxval_fast, sc_fxval_fast, sc_dt::sc_fxval_fast_observer>("sc_fxval_fast");
    observe<sc_fixed<1, 2>, sc_fxnum, sc_dt::sc_fxnum_observer>("sc_fixed");
    observe<sc_fixed_fast<3, 4>, sc_fxnum_fast, sc_dt::sc_fxnum_fast_observer>("sc_fixed_fast");
    observe<sc_ufixed<5, 6>, sc_fxnum, sc_dt::sc_fxnum_observer>("sc_ufixed");
    observe<sc_ufixed_fast<7, 8>, sc_fxnum_fast, sc_dt::sc_fxnum_fast_observer>("sc_ufixed_fast");

    return 0;
}
