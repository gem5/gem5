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
 *
 * Authors: Gabe Black
 */

#ifndef __SIM_GUEST_ABI_HH__
#define __SIM_GUEST_ABI_HH__

#include <functional>
#include <memory>
#include <sstream>
#include <type_traits>

class ThreadContext;

namespace GuestABI
{

/*
 * To implement an ABI, a subclass needs to implement a system of
 * specializations of these two templates Result and Argument, and define a
 * "Position" type.
 *
 * The Position type carries information about, for instance, how many
 * integer registers have been consumed gathering earlier arguments. It
 * may contain multiple elements if there are multiple dimensions to track,
 * for instance the number of integer and floating point registers used so far.
 *
 * Result and Argument are class templates instead of function templates so
 * that they can be partially specialized if necessary. C++ doesn't let you
 * partially specialize function templates because that conflicts with
 * template resolution using the function's arguments. Since we already know
 * what type we want and we don't need argument based resolution, we can just
 * wrap the desired functionality in classes and sidestep the problem.
 *
 * Also note that these templates have an "Enabled" parameter to support
 * std::enable_if style conditional specializations.
 */

template <typename ABI, typename Ret, typename Enabled=void>
struct Result
{
  private:
    /*
     * Store result "ret" into the state accessible through tc.
     *
     * Note that the declaration below is only to document the expected
     * signature and is private so it won't be used by accident.
     * Specializations of this Result class should define their own version
     * of this method which actually does something and is public.
     */
    static void store(ThreadContext *tc, const Ret &ret);

    /*
     * Adjust the position of arguments based on the return type, if necessary.
     *
     * This method can be excluded if no adjustment is necessary.
     */
    static void allocate(ThreadContext *tc, typename ABI::Position &position);
};

/*
 * This partial specialization prevents having to special case 'void' when
 * working with return types.
 */
template <typename ABI>
struct Result<ABI, void>
{};

template <typename ABI, typename Arg, typename Enabled=void>
struct Argument
{
    /*
     * Retrieve an argument of type Arg from the state accessible through tc,
     * assuming the state represented by "position" has already been used.
     * Also update position to account for this argument as well.
     *
     * Like Result::store above, the declaration below is only to document
     * the expected method signature.
     */
    static Arg get(ThreadContext *tc, typename ABI::Position &position);
};


/*
 * This struct template provides a default allocate() method in case the
 * Result template doesn't provide one. This is the default in cases where the
 * return type doesn't affect how arguments are laid out.
 */
template <typename ABI, typename Ret, typename Enabled=void>
struct ResultAllocator
{
    static void
    allocate(ThreadContext *tc, typename ABI::Position &position)
    {}
};

/*
 * If the return type *does* affect how the arguments are laid out, the ABI
 * can implement an allocate() method for the various return types, and this
 * specialization will call into it.
 */
template <typename ABI, typename Ret>
struct ResultAllocator<ABI, Ret, decltype((void)&Result<ABI, Ret>::allocate)>
{
    static void
    allocate(ThreadContext *tc, typename ABI::Position &position)
    {
        Result<ABI, Ret>::allocate(tc, position);
    }
};


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
        first = Argument<ABI, First>::get(this->tc, this->position);
    }
};

// The base case of the recursion, which inherits from the interface class.
template <typename ABI, typename Base>
class VarArgsImpl<ABI, Base> : public Base
{
  protected:
    // Declare state to pass to the Argument<>::get methods.
    ThreadContext *tc;
    typename ABI::Position position;

    // Give the "using" statement in our subclass something to refer to.
    void _getImpl();

  public:
    VarArgsImpl(ThreadContext *_tc, const typename ABI::Position &_pos) :
        tc(_tc), position(_pos)
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

template <typename ...Types>
std::ostream &
operator << (std::ostream &os, const VarArgs<Types...> &va)
{
    os << "...";
    return os;
}

// The ABI independent hook which tells the GuestABI mechanism what to do with
// a VarArgs argument. It constructs the underlying implementation which knows
// about the ABI, and installs it in the VarArgs wrapper to give to the
// function.
template <typename ABI, typename ...Types>
struct Argument<ABI, VarArgs<Types...>>
{
    static VarArgs<Types...>
    get(ThreadContext *tc, typename ABI::Position &position)
    {
        using Base = VarArgsBase<Types...>;
        using Impl = VarArgsImpl<ABI, Base, Types...>;
        return VarArgs<Types...>(new Impl(tc, position));
    }
};


/*
 * These functions will likely be common among all ABIs and implement the
 * mechanism of gathering arguments, calling the target function, and then
 * storing the result. They might need to be overridden if, for instance,
 * the location of arguments need to be determined in a different order.
 * For example, there might be an ABI which gathers arguments starting
 * from the last in the list instead of the first. This is unlikely but
 * still possible to support by redefining these functions..
 */

// With no arguments to gather, call the target function and store the
// result.
template <typename ABI, typename Ret>
static typename std::enable_if<!std::is_void<Ret>::value, Ret>::type
callFrom(ThreadContext *tc, typename ABI::Position &position,
        std::function<Ret(ThreadContext *)> target)
{
    Ret ret = target(tc);
    Result<ABI, Ret>::store(tc, ret);
    return ret;
}

// With no arguments to gather and nothing to return, call the target function.
template <typename ABI>
static void
callFrom(ThreadContext *tc, typename ABI::Position &position,
        std::function<void(ThreadContext *)> target)
{
    target(tc);
}

// Recursively gather arguments for target from tc until we get to the base
// case above.
template <typename ABI, typename Ret, typename NextArg, typename ...Args>
static typename std::enable_if<!std::is_void<Ret>::value, Ret>::type
callFrom(ThreadContext *tc, typename ABI::Position &position,
        std::function<Ret(ThreadContext *, NextArg, Args...)> target)
{
    // Extract the next argument from the thread context.
    NextArg next = Argument<ABI, NextArg>::get(tc, position);

    // Build a partial function which adds the next argument to the call.
    std::function<Ret(ThreadContext *, Args...)> partial =
        [target,next](ThreadContext *_tc, Args... args) {
            return target(_tc, next, args...);
        };

    // Recursively handle any remaining arguments.
    return callFrom<ABI, Ret, Args...>(tc, position, partial);
}

// Recursively gather arguments for target from tc until we get to the base
// case above. This version is for functions that don't return anything.
template <typename ABI, typename NextArg, typename ...Args>
static void
callFrom(ThreadContext *tc, typename ABI::Position &position,
        std::function<void(ThreadContext *, NextArg, Args...)> target)
{
    // Extract the next argument from the thread context.
    NextArg next = Argument<ABI, NextArg>::get(tc, position);

    // Build a partial function which adds the next argument to the call.
    std::function<void(ThreadContext *, Args...)> partial =
        [target,next](ThreadContext *_tc, Args... args) {
            target(_tc, next, args...);
        };

    // Recursively handle any remaining arguments.
    callFrom<ABI, Args...>(tc, position, partial);
}



/*
 * These functions are like the ones above, except they print the arguments
 * a target function would be called with instead of actually calling it.
 */

// With no arguments to print, add the closing parenthesis and return.
template <typename ABI, typename Ret>
static void
dumpArgsFrom(int count, std::ostream &os, ThreadContext *tc,
             typename ABI::Position &position)
{
    os << ")";
}

// Recursively gather arguments for target from tc until we get to the base
// case above, and append those arguments to the string stream being
// constructed.
template <typename ABI, typename Ret, typename NextArg, typename ...Args>
static void
dumpArgsFrom(int count, std::ostream &os, ThreadContext *tc,
             typename ABI::Position &position)
{
    // Either open the parenthesis or add a comma, depending on where we are
    // in the argument list.
    os << (count ? ", " : "(");

    // Extract the next argument from the thread context.
    NextArg next = Argument<ABI, NextArg>::get(tc, position);

    // Add this argument to the list.
    os << next;

    // Recursively handle any remaining arguments.
    dumpArgsFrom<ABI, Ret, Args...>(count + 1, os, tc, position);
}

} // namespace GuestABI


// These functions wrap a simulator level function with the given signature.
// The wrapper takes one argument, a thread context to extract arguments from
// and write a result (if any) back to. For convenience, the wrapper also
// returns the result of the wrapped function.

template <typename ABI, typename Ret, typename ...Args>
Ret
invokeSimcall(ThreadContext *tc,
              std::function<Ret(ThreadContext *, Args...)> target)
{
    // Default construct a Position to track consumed resources. Built in
    // types will be zero initialized.
    auto position = typename ABI::Position();
    GuestABI::ResultAllocator<ABI, Ret>::allocate(tc, position);
    return GuestABI::callFrom<ABI, Ret, Args...>(tc, position, target);
}

template <typename ABI, typename Ret, typename ...Args>
Ret
invokeSimcall(ThreadContext *tc, Ret (*target)(ThreadContext *, Args...))
{
    return invokeSimcall<ABI>(
            tc, std::function<Ret(ThreadContext *, Args...)>(target));
}

template <typename ABI, typename ...Args>
void
invokeSimcall(ThreadContext *tc,
              std::function<void(ThreadContext *, Args...)> target)
{
    // Default construct a Position to track consumed resources. Built in
    // types will be zero initialized.
    auto position = typename ABI::Position();
    GuestABI::callFrom<ABI, Args...>(tc, position, target);
}

template <typename ABI, typename ...Args>
void
invokeSimcall(ThreadContext *tc, void (*target)(ThreadContext *, Args...))
{
    invokeSimcall<ABI>(
            tc, std::function<void(ThreadContext *, Args...)>(target));
}


// These functions also wrap a simulator level function. Instead of running the
// function, they return a string which shows what arguments the function would
// be invoked with if it were called from the given context.

template <typename ABI, typename Ret, typename ...Args>
std::string
dumpSimcall(std::string name, ThreadContext *tc,
            std::function<Ret(ThreadContext *, Args...)> target=
            std::function<Ret(ThreadContext *, Args...)>())
{
    auto position = typename ABI::Position();
    std::ostringstream ss;

    GuestABI::ResultAllocator<ABI, Ret>::allocate(tc, position);
    ss << name;
    GuestABI::dumpArgsFrom<ABI, Ret, Args...>(0, ss, tc, position);
    return ss.str();
}

template <typename ABI, typename Ret, typename ...Args>
std::string
dumpSimcall(std::string name, ThreadContext *tc,
            Ret (*target)(ThreadContext *, Args...))
{
    return dumpSimcall<ABI>(
            name, tc, std::function<Ret(ThreadContext *, Args...)>(target));
}

#endif // __SIM_GUEST_ABI_HH__
