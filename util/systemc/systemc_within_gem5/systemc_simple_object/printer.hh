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

#ifndef __SYSTEMC_SIMPLE_OBJECT_PRINTER_HH__
#define __SYSTEMC_SIMPLE_OBJECT_PRINTER_HH__

#include <string>

// This include brings in the gem5 statistics classes and DPRINTF mechanism
// which is not necessary, but shows that if they're gem5 aware, models can
// take advantage of these sorts of gem5 mechanisms.
#include "base/statistics.hh"
#include "base/trace.hh"

// Include the standard top level "systemc" header. For models which aren't
// aware of gem5, systemc/ext would be in their include path directly and the
// include wouldn't need the systemc/ext prefix.
#include "systemc/ext/systemc"

// This class is a garden variety sc_module, except that it uses DPRINTFN in
// one of its methods, and maintains a gem5 statistic.
class Printer : public sc_core::sc_module
{
  public:
    sc_core::sc_in<const char *> input;
    std::string prefix;

    SC_CTOR(Printer)
    {
        SC_THREAD(print);
    }

    void
    print()
    {
        int i = 0;
        while (true) {
            wait(input.value_changed_event());

            // DPRINTFN works as expected here because sc_objects have a name()
            // method which DPRINTFN relies on. Normally name() would come from
            // a SimObject whose members were in scope, but it doesn't have to.
            DPRINTFN("Word %d: %s%s\n", i++, prefix, input.read());

            // Manage the gem5 statistic like normal.
            numWords++;
        }
    }

    gem5::statistics::Scalar numWords;

    // Gem5 statistics should be set up during the "end_of_elabortion"
    // callback.
    void
    end_of_elaboration() override
    {
        numWords
            .name(std::string(name()) + ".numWords")
            .desc("number of words printed")
            ;
    }
};

#endif // __SYSTEMC_SIMPLE_OBJECT_PRINTER_HH__
