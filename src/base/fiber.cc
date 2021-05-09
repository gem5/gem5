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

#include "base/fiber.hh"

#if HAVE_VALGRIND
#include <valgrind/valgrind.h>
#endif

// Mac OS requires _DARWIN_C_SOURCE if _POSIX_C_SOURCE is defined,
// otherwise it will mask the definition of MAP_ANONYMOUS.
// _POSIX_C_SOURCE is already defined by including <ucontext.h> in
// base/fiber.hh
#if defined(__APPLE__) && defined(__MACH__)
#define _DARWIN_C_SOURCE
#endif

#include <sys/mman.h>
#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <cstring>

#include "base/logging.hh"

namespace gem5
{

namespace
{

/*
 * The PrimaryFiber class is a special case that attaches to the currently
 * executing context. That makes handling the "primary" fiber, aka the one
 * which most of gem5 is running under, no different than other Fibers.
 */
class PrimaryFiber : public Fiber
{
  public:
    PrimaryFiber() : Fiber(nullptr, 0) { setStarted(); }
    void main() { panic("PrimaryFiber main executed.\n"); }
};

PrimaryFiber _primaryFiber;

// A pointer to whatever the currently executing Fiber is.
Fiber *_currentFiber = &_primaryFiber;

// A pointer to the Fiber which is currently being started/initialized.
Fiber *startingFiber = nullptr;

} // anonymous namespace

void
Fiber::entryTrampoline()
{
    startingFiber->start();
}

Fiber::Fiber(size_t stack_size) : Fiber(primaryFiber(), stack_size)
{}

Fiber::Fiber(Fiber *link, size_t stack_size) :
    link(link), stack(nullptr), stackSize(stack_size), guardPage(nullptr),
    guardPageSize(sysconf(_SC_PAGE_SIZE)), _started(false), _finished(false)
{
    if (stack_size) {
        guardPage = mmap(nullptr, guardPageSize + stack_size,
                         PROT_READ | PROT_WRITE,
                         MAP_ANONYMOUS | MAP_PRIVATE, -1, 0);
        if (guardPage == (void *)MAP_FAILED) {
            perror("mmap");
            fatal("Could not mmap %d byte fiber stack.\n", stack_size);
        }
        stack = (void *)((uint8_t *)guardPage + guardPageSize);
        if (mprotect(guardPage, guardPageSize, PROT_NONE)) {
            perror("mprotect");
            fatal("Could not forbid access to fiber stack guard page.");
        }
    }
#if HAVE_VALGRIND
    valgrindStackId = VALGRIND_STACK_REGISTER(
            stack, (uint8_t *)stack + stack_size);
#endif
}

Fiber::~Fiber()
{
    panic_if(stack && _currentFiber == this, "Fiber stack is in use.");
#if HAVE_VALGRIND
    VALGRIND_STACK_DEREGISTER(valgrindStackId);
#endif
    if (guardPage)
        munmap(guardPage, guardPageSize + stackSize);
}

void
Fiber::createContext()
{
    // Set up a context for the new fiber, starting it in the trampoline.
    getcontext(&ctx);
    ctx.uc_stack.ss_sp = stack;
    ctx.uc_stack.ss_size = stackSize;
    ctx.uc_link = nullptr;
    makecontext(&ctx, &entryTrampoline, 0);

    // Swap to the new context so it can enter its start() function. It
    // will then swap itself back out and return here.
    startingFiber = this;
    panic_if(!_currentFiber, "No active Fiber object.");
    swapcontext(&_currentFiber->ctx, &ctx);

    // The new context is now ready and about to call main().
}

void
Fiber::start()
{
    // Avoid a dangling pointer.
    startingFiber = nullptr;

    setStarted();

    if (_setjmp(jmp) == 0) {
        // Swap back to the parent context which is still considered "current",
        // now that we're ready to go.
        int ret = swapcontext(&ctx, &_currentFiber->ctx);
        panic_if(ret == -1, strerror(errno));
    }

    // Call main() when we're been reactivated for the first time.
    main();

    // main has returned, so this Fiber has finished. Switch to the "link"
    // Fiber.
    _finished = true;
    link->run();
}

void
Fiber::run()
{
    panic_if(_finished, "Fiber has already run to completion.");

    // If we're already running this fiber, we're done.
    if (_currentFiber == this)
        return;

    if (!_started)
        createContext();

    // Switch out of the current Fiber's context and this one's in.
    Fiber *prev = _currentFiber;
    Fiber *next = this;
    _currentFiber = next;
    if (_setjmp(prev->jmp) == 0)
        _longjmp(next->jmp, 1);
}

Fiber *Fiber::currentFiber() { return _currentFiber; }
Fiber *Fiber::primaryFiber() { return &_primaryFiber; }

} // namespace gem5
