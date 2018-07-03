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

#include "systemc/core/scheduler.hh"

#include "base/fiber.hh"

namespace sc_gem5
{

Scheduler::Scheduler() : _numCycles(0), _current(nullptr) {}

void
Scheduler::initialize()
{
    update();

    while (!initList.empty())
        ready(initList.getNext());

    delta();
}

void
Scheduler::runCycles()
{
    while (!readyList.empty()) {
        evaluate();
        update();
        delta();
    }
}

void
Scheduler::yield()
{
    _current = readyList.getNext();
    if (!_current) {
        // There are no more processes, so return control to evaluate.
        Fiber::primaryFiber()->run();
    } else {
        _current->popListNode();
        // Switch to whatever Fiber is supposed to run this process. All
        // Fibers which aren't running should be parked at this line.
        _current->fiber()->run();
        // If the current process hasn't been started yet, start it. This
        // should always be true for methods, but may not be true for threads.
        if (_current && !_current->running())
            _current->run();
    }
}

void
Scheduler::evaluate()
{
    if (!readyList.empty())
        _numCycles++;

    do {
        yield();
    } while (!readyList.empty());
}

void
Scheduler::update()
{
}

void
Scheduler::delta()
{
}

Scheduler scheduler;

} // namespace sc_gem5
