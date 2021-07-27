/*
 * Copyright 2019 Google Inc.
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

#ifndef __SIM_GUEST_ABI_VARARGS_HH__
#define __SIM_GUEST_ABI_VARARGS_HH__

#include <memory>
#include <sstream>
#include <type_traits>

#include "sim/guest_abi/definition.hh"

namespace gem5
{

class ThreadContext;

GEM5_DEPRECATED_NAMESPACE(GuestABI, guest_abi);
namespace guest_abi
{

/*
 * These templates implement a variadic argument mechanism for guest ABI
 * functions. A function might be written like this:
 *
 * void
 * func(ThreadContext *tc, VarArgs<Addr, int> varargs)
 * {
 *     warn("Address = %#x, int = %d.",
 *          varargs.get<Addr>(), varargs.get<int>());
 * }
 *
 * where an object of type VarArgs<...> is its last argument. The types given
 * to the template specify what types the function might need to retrieve from
 * varargs. The varargs object will then have get<> methods for each of those
 * types.
 *
 * Note that each get<> will happen live. If you modify values through the
 * ThreadContext *tc and then run get<>(), you may alter one of your arguments.
 * If you're going to use tc to modify state, it would be a good idea to use
 * get<>() as soon as possible to avoid corrupting the functions arguments.
 */

// A recursive template which defines virtual functions to retrieve each of the
// requested types. This provides the ABI agnostic interface the function uses.
template <typename ...Types>
class VarArgsBase;

template <typename First, typename ...Types>
class VarArgsBase<First, Types...> : public VarArgsBase<Types...>
{
  public:
    virtual ~VarArgsBase() = default;

    // The virtual function takes a reference parameter so that the different
    // _getImpl methods can co-exist through overloading.
    virtual void _getImpl(First &) = 0;

    // Make sure base class _getImpl-es aren't hidden by this one.
    using VarArgsBase<Types...>::_getImpl;
};

// The base case of the recursion.
template <>
class VarArgsBase<>
{
  protected:
    // This just gives the "using" statement in the non base case something to
    // refer to.
    void _getImpl();
};


// A recursive template which defines the ABI specific implementation of the
// interface defined above.
//
// The types in Types are consumed one by one, and by
// the time we get down to the base case we'd have lost track of the complete
// set we need to know what interface to inherit. The Base parameter keeps
// track of that through the recursion.
template <typename ABI, typename Base, typename ...Types>
class VarArgsImpl;

template <typename ABI, typename Base, typename First, typename ...Types>
class VarArgsImpl<ABI, Base, First, Types...> :
    public VarArgsImpl<ABI, Base, Types...>
{
  protected:
    // Bring forward the base class constructor.
    using VarArgsImpl<ABI, Base, Types...>::VarArgsImpl;
    // Make sure base class _getImpl-es don't get hidden by ours.
    using VarArgsImpl<ABI, Base, Types...>::_getImpl;

    // Implement a version of _getImple, using the ABI specialized version of
    // the Argument class.
    void
    _getImpl(First &first) override
    {
        first = Argument<ABI, First>::get(this->tc, this->state);
    }
};

// The base case of the recursion, which inherits from the interface class.
template <typename ABI, typename Base>
class VarArgsImpl<ABI, Base> : public Base
{
  protected:
    // Declare state to pass to the Argument<>::get methods.
    ThreadContext *tc;
    typename ABI::State state;
    // Make sure base class _getImpl-es don't get hidden by ours.
    using Base::_getImpl;

    // Give the "using" statement in our subclass something to refer to.
    void _getImpl();

  public:
    VarArgsImpl(ThreadContext *_tc, const typename ABI::State &_state) :
        tc(_tc), state(_state)
    {}
};

// A wrapper which provides a nice interface to the virtual functions, and a
// hook for the Argument template mechanism.
template <typename ...Types>
class VarArgs
{
  private:
    // This points to the implementation which knows how to read arguments
    // based on the ABI being used.
    std::shared_ptr<VarArgsBase<Types...>> _ptr;

  public:
    VarArgs(VarArgsBase<Types...> *ptr) : _ptr(ptr) {}

    // This template is a friendlier wrapper around the virtual functions the
    // raw interface provides. This version lets you pick a type which it then
    // returns, instead of having to pre-declare a variable to pass in.
    template <typename Arg>
    Arg
    get()
    {
        Arg arg;
        _ptr->_getImpl(arg);
        return arg;
    }
};

template <typename T>
struct IsVarArgs : public std::false_type {};

template <typename ...Types>
struct IsVarArgs<VarArgs<Types...>> : public std::true_type {};

template <typename T>
constexpr bool IsVarArgsV = IsVarArgs<T>::value;

template <typename ...Types>
std::ostream &
operator << (std::ostream &os, const VarArgs<Types...> &va)
{
    os << "...";
    return os;
}

// The ABI independent hook which tells the guest_abi mechanism what to do with
// a VarArgs argument. It constructs the underlying implementation which knows
// about the ABI, and installs it in the VarArgs wrapper to give to the
// function.
template <typename ABI, typename ...Types>
struct Argument<ABI, VarArgs<Types...>>
{
    static VarArgs<Types...>
    get(ThreadContext *tc, typename ABI::State &state)
    {
        using Base = VarArgsBase<Types...>;
        using Impl = VarArgsImpl<ABI, Base, Types...>;
        return VarArgs<Types...>(new Impl(tc, state));
    }
};

} // namespace guest_abi
} // namespace gem5

#endif // __SIM_GUEST_ABI_VARARGS_HH__
