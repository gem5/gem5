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

#ifndef __SIM_GUEST_ABI_DEFINITION_HH__
#define __SIM_GUEST_ABI_DEFINITION_HH__

class ThreadContext;

namespace GuestABI
{

/*
 * To implement an ABI, a subclass needs to implement a system of
 * specializations of these two templates Result and Argument, and define a
 * "State" type.
 *
 * The State type carries information about, for instance, how many
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
     * Store result "ret" into the state accessible through tc. Optionally
     * accept "state" in case it holds some signature wide information.
     *
     * Note that the declaration below is only to document the expected
     * signature and is private so it won't be used by accident.
     * Specializations of this Result class should define their own version
     * of this method which actually does something and is public.
     */
    static void store(ThreadContext *tc, const Ret &ret);
    static void store(ThreadContext *tc, const Ret &ret,
                      typename ABI::State &state);

    /*
     * Prepare for a result of type Ret. This might mean, for instance,
     * allocating an argument register for a result pointer.
     *
     * This method can be excluded if no preparation is necessary.
     */
    static void prepare(ThreadContext *tc, typename ABI::State &state);
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
     * assuming the state represented by "state" has already been used.
     * Also update state to account for this argument as well.
     *
     * Like Result::store above, the declaration below is only to document
     * the expected method signature.
     */
    static Arg get(ThreadContext *tc, typename ABI::State &state);

    /*
     * Prepare for an argument of type Arg. This might mean, for instance,
     * allocating an argument register for a result pointer.
     *
     * This method can be excluded if no preparation is necessary.
     */
    static void allocate(ThreadContext *tc, typename ABI::State &state);
};

} // namespace GuestABI

#endif // __SIM_GUEST_ABI_DEFINITION_HH__
