/*
 * Copyright (c) 2018 ARM Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
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

#ifndef __BASE_COROUTINE_HH__
#define __BASE_COROUTINE_HH__

#include <functional>
#include <stack>

#include "base/compiler.hh"
#include "base/fiber.hh"

GEM5_DEPRECATED_NAMESPACE(m5, gem5);
namespace gem5
{

/**
 * This template defines a Coroutine wrapper type with a Boost-like
 * interface. It is built on top of the gem5 fiber class.
 * The two template parameters (Arg and Ret) are the coroutine
 * argument and coroutine return types which are passed between
 * the coroutine and the caller via operator() and get() method.
 * This implementation doesn't support passing multiple values,
 * so a tuple must be used in that scenario.
 *
 * Most methods are templatized since it is relevant to distinguish
 * the cases where one or both of the template parameters are void
 */
template <typename Arg, typename Ret>
class Coroutine : public Fiber
{

    // This empty struct type is meant to replace coroutine channels
    // in case the channel should be void (Coroutine template parameters
    // are void. (See following ArgChannel, RetChannel typedef)
    struct Empty {};
    using ArgChannel = typename std::conditional_t<
        std::is_same_v<Arg, void>, Empty, std::stack<Arg>>;

    using RetChannel = typename std::conditional_t<
        std::is_same_v<Ret, void>, Empty, std::stack<Ret>>;

  public:
    /**
     * CallerType:
     * A reference to an object of this class will be passed
     * to the coroutine task. This is the way it is possible
     * for the coroutine to interface (e.g. switch back)
     * to the coroutine caller.
     */
    class CallerType
    {
        friend class Coroutine;
      protected:
        CallerType(Coroutine& _coro) : coro(_coro), callerFiber(nullptr) {}

      public:
        /**
         * operator() is the way we can jump outside the coroutine
         * and return a value to the caller.
         *
         * This method is generated only if the coroutine returns
         * a value (Ret != void)
         *
         * @ingroup api_coroutine
         */
        template <typename T = Ret>
        CallerType&
        operator()(typename std::enable_if_t<
                   !std::is_same_v<T, void>, T> param)
        {
            retChannel.push(param);
            callerFiber->run();
            return *this;
        }

        /**
         * operator() is the way we can jump outside the coroutine
         *
         * This method is generated only if the coroutine doesn't
         * return a value (Ret = void)
         *
         * @ingroup api_coroutine
         */
        template <typename T = Ret>
        typename std::enable_if_t<std::is_same_v<T, void>,
                                CallerType> &
        operator()()
        {
            callerFiber->run();
            return *this;
        }

        /**
         * get() is the way we can extrapolate arguments from the
         * coroutine caller.
         * The coroutine blocks, waiting for the value, unless it is already
         * available; otherwise caller execution is resumed,
         * and coroutine won't execute until a value is pushed
         * from the caller.
         *
         * @return arg coroutine argument
         *
         * @ingroup api_coroutine
         */
        template <typename T = Arg>
        typename std::enable_if_t<!std::is_same_v<T, void>, T>
        get()
        {
            auto& args_channel = coro.argsChannel;
            while (args_channel.empty()) {
                callerFiber->run();
            }

            auto ret = args_channel.top();
            args_channel.pop();
            return ret;
        }

      private:
        Coroutine& coro;
        Fiber* callerFiber;
        RetChannel retChannel;
    };

    /**
     * @ingroup api_coroutine
     * @{
     */
    Coroutine() = delete;
    Coroutine(const Coroutine& rhs) = delete;
    Coroutine& operator=(const Coroutine& rhs) = delete;
    /** @} */ // end of api_coroutine

    /**
     * Coroutine constructor.
     * The only way to construct a coroutine is to pass it the routine
     * it needs to run. The first argument of the function should be a
     * reference to the Coroutine<Arg,Ret>::caller_type which the
     * routine will use as a way for yielding to the caller.
     * The optional second boolean argument controls if the Coroutine
     * should be run on creation, which mimics Boost's Coroutine
     * semantics by default. This can be disabled as an optimization to
     * avoid unnecessary context switches on Coroutine creation.
     *
     * @param f task run by the coroutine
     * @param run_coroutine set to false to disable running the coroutine
     *                      immediately after it is created
     *
     * @ingroup api_coroutine
     */
    Coroutine(std::function<void(CallerType&)> f, bool run_coroutine = true)
      : Fiber(), task(f), caller(*this)
    {
        // When desired, run the Coroutine after it is created
        if (run_coroutine)
            this->call();
    }

    /**
     * @ingroup api_coroutine
     */
    virtual ~Coroutine() {}

  public:
    /** Coroutine interface */

    /**
     * operator() is the way we can jump inside the coroutine
     * and passing arguments.
     *
     * This method is generated only if the coroutine takes
     * arguments (Arg != void)
     *
     * @ingroup api_coroutine
     */
    template <typename T = Arg>
    Coroutine&
    operator()(typename std::enable_if_t<!std::is_same_v<T, void>, T> param)
    {
        argsChannel.push(param);
        this->call();
        return *this;
    }

    /**
     * operator() is the way we can jump inside the coroutine.
     *
     * This method is generated only if the coroutine takes
     * no arguments. (Arg = void)
     *
     * @ingroup api_coroutine
     */
    template <typename T = Arg>
    typename std::enable_if_t<std::is_same_v<T, void>, Coroutine> &
    operator()()
    {
        this->call();
        return *this;
    }

    /**
     * get() is the way we can extrapolate return values
     * (yielded) from the coroutine.
     * The caller blocks, waiting for the value, unless it is already
     * available; otherwise coroutine execution is resumed,
     * and caller won't execute until a value is yielded back
     * from the coroutine.
     *
     * @return ret yielded value
     *
     * @ingroup api_coroutine
     */
    template <typename T = Ret>
    typename std::enable_if_t<!std::is_same_v<T, void>, T>
    get()
    {
        auto& ret_channel = caller.retChannel;
        while (ret_channel.empty()) {
            this->call();
        }

        auto ret = ret_channel.top();
        ret_channel.pop();
        return ret;
    }

    /**
     * Check if coroutine is still running
     *
     * @ingroup api_coroutine
     */
    operator bool() const { return !this->finished(); }

  private:
    /**
     * Overriding base (Fiber) main.
     * This method will be automatically called by the Fiber
     * running engine and it is a simple wrapper for the task
     * that the coroutine is supposed to run.
     */
    void main() override { this->task(caller); }

    void
    call()
    {
        caller.callerFiber = currentFiber();
        run();
    }

  private:
    /** Arguments for the coroutine */
    ArgChannel argsChannel;

    /** Coroutine task */
    std::function<void(CallerType&)> task;

    /** Coroutine caller */
    CallerType caller;
};

} //namespace gem5

#endif // __BASE_COROUTINE_HH__
