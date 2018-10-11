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
 *
 * Authors: Gabe Black
 */

#ifndef __SYSTEMC_SIMPLE_OBJECT_FEEDER_HH__
#define __SYSTEMC_SIMPLE_OBJECT_FEEDER_HH__

#include <string>
#include <vector>

#include "sim/eventq.hh"
#include "sim/sim_object.hh"
#include "systemc_simple_object/printer.hh"

// The "external" header interface for systemc is in systemc/ext and is what
// a model which doesn't know anything about gem5 would use. This includes a
// particular file from those headers, but alternatively we could include
// systemc/ext/systemc or systemc/ext/systemc.h.
#include "systemc/ext/channel/sc_buffer.hh"

// This implementation (mostly) just uses standard gem5 mechanisms.
class Gem5_FeederParams;

class Feeder : public SimObject
{
  public:
    Feeder(Gem5_FeederParams *params);

    void feed();

  private:
    Printer *printer;
    Tick delay;
    std::vector<std::string> strings;
    int index;

    // SystemC types can be used in gem5 classes, but that should be avoided
    // except to help interact with systemc objects/models.
    sc_core::sc_buffer<const char *> buf;

    EventWrapper<Feeder, &Feeder::feed> event;

    void startup() override;
};

#endif // __SYSTEMC_SIMPLE_OBJECT_PRINTER_HH__
