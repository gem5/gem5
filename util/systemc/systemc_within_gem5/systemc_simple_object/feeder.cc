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

#include "params/Gem5_Feeder.hh"
#include "sim/sim_exit.hh"
#include "systemc_simple_object/feeder.hh"

Feeder::Feeder(Gem5_FeederParams *params) :
    SimObject(params), printer(params->printer), delay(params->delay),
    strings(params->strings), index(0), event(this)
{
    // Bind the printer objects "input" port to our sc_buffer. This will let
    // us feed it values. If some other object was responsible for the
    // sc_buffer, we could interact with it directly and not have built it
    // ourselves.
    //
    // Alternatively Printer could define a normal c++ method which we could
    // call to feed it words without involving any systemc specific classes.
    printer->input(buf);
}

void
Feeder::startup()
{
    schedule(&event, curTick() + delay);
}

void
Feeder::feed()
{
    if (index >= strings.size())
        exitSimLoop("Printed all the words.");
    else
        buf.write(strings[index++].c_str());

    schedule(&event, curTick() + delay);
}

Feeder *
Gem5_FeederParams::create()
{
    return new Feeder(this);
}
