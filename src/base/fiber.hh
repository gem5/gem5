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

#ifndef __BASE_FIBER_HH__
#define __BASE_FIBER_HH__

// ucontext functions (like getcontext, setcontext etc) have been marked
// as deprecated and are hence hidden in latest macOS releases.
// By defining _XOPEN_SOURCE we make them available at compilation time.
#if defined(__APPLE__) && defined(__MACH__)
#define _XOPEN_SOURCE 600
#include <ucontext.h>
#undef _XOPEN_SOURCE
#else
#include <ucontext.h>
#endif

// Avoid fortify source for longjmp to work between ucontext stacks.
#pragma push_macro("__USE_FORTIFY_LEVEL")
#undef __USE_FORTIFY_LEVEL
#include <setjmp.h>
#pragma pop_macro("__USE_FORTIFY_LEVEL")

#include <cstddef>
#include <cstdint>

#include "config/have_valgrind.hh"

namespace gem5
{

/**
 * This class represents a fiber, which is a light weight sort of thread which
 * is cooperatively scheduled and runs sequentially with other fibers, swapping
 * in and out of a single actual thread of execution.
 *
 * To define your own threads, create a subclass of Fiber and override its
 * main() function to do what you want your fiber to do. You can start it by
 * calling its run() method which will stop your execution and start the other
 * fiber in your place.
 *
 * If your main() function ends, that fiber will automatically switch to either
 * the primary fiber, or to a particular fiber you specified at construction
 * time, and your fiber is considered finished.
 */

class Fiber
{
  public:
    /**
     * @ingroup api_fiber
     */
    const static size_t DefaultStackSize = 0x50000;

    /**
     * @param Link points to another fiber which will start executing when this
     *     fiber's main function returns.
     * @param stack_size is the size of the stack available to this fiber.
     *
     * @ingroup api_fiber
     * @{
     */
    Fiber(size_t stack_size=DefaultStackSize);
    Fiber(Fiber *link, size_t stack_size=DefaultStackSize);
    /** @} */ // end of api_fiber

    /**
     * @ingroup api_fiber
     */
    virtual ~Fiber();

    /**
     * Start executing the fiber represented by this object. This function
     * will "return" when the current fiber is switched back to later on.
     *
     * @ingroup api_fiber
     */
    void run();

    /**
     * Returns whether the "main" function of this fiber has finished.
     *
     * @ingroup api_fiber
     */
    bool finished() const { return _finished; };

    /**
     * Returns whether the "main" function of this fiber has started.
     *
     * @ingroup api_fiber
     */
    bool started() const { return _started; };

    /**
     * Get a pointer to the current running Fiber.
     *
     * @ingroup api_fiber
     */
    static Fiber *currentFiber();

    /**
     * Get a pointer to the primary Fiber.
     * This Fiber represents the thread of execution started by the OS, and
     * which has a Fiber attached to it after the fact.
     *
     * @ingroup api_fiber
     */
    static Fiber *primaryFiber();

  protected:
    /**
     * This method is called when this fiber is first run. Override it to
     * give your fiber something to do. When main returns, the fiber will
     * mark itself as finished and switch to its link fiber.
     */
    virtual void main() = 0;

    void setStarted() { _started = true; }

  private:
    static void entryTrampoline();
    void start();

    ucontext_t ctx;
    // ucontext is slow in swapcontext. Here we use _setjmp/_longjmp to avoid
    // the additional signals for speed up.
    jmp_buf jmp;

    Fiber *link;

    // The stack for this context, or a nullptr if allocated elsewhere.
    void *stack;
    size_t stackSize;
    void *guardPage;
    size_t guardPageSize;
#if HAVE_VALGRIND
    unsigned valgrindStackId;
#endif

    bool _started;
    bool _finished;
    void createContext();
};

} // namespace gem5

#endif // __BASE_FIBER_HH__
