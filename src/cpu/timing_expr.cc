/*
 * Copyright (c) 2013-2014 ARM Limited
 * All rights reserved.
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

#include "cpu/timing_expr.hh"

#include "base/intmath.hh"

namespace gem5
{

TimingExprEvalContext::TimingExprEvalContext(const StaticInstPtr &inst_,
    ThreadContext *thread_, TimingExprLet *let_) :
    inst(inst_), thread(thread_), let(let_)
{
    /* Reserve space to hold the results of evaluating the
     *  let expressions */
    if (let) {
        unsigned int num_defns = let->defns.size();

        results.resize(num_defns, 0);
        resultAvailable.resize(num_defns, false);
    }
}

uint64_t
TimingExprSrcReg::eval(TimingExprEvalContext &context)
{
    return context.thread->getReg(context.inst->srcRegIdx(index));
}

uint64_t
TimingExprLet::eval(TimingExprEvalContext &context)
{
    TimingExprEvalContext new_context(context.inst, context.thread, this);

    return expr->eval(new_context);
}

uint64_t
TimingExprRef::eval(TimingExprEvalContext &context)
{
    /* Lookup the result, evaluating if necessary.  @todo, this
     *  should have more error checking */
    if (!context.resultAvailable[index]) {
        context.results[index] = context.let->defns[index]->eval(context);
        context.resultAvailable[index] = true;
    }

    return context.results[index];
}

uint64_t
TimingExprUn::eval(TimingExprEvalContext &context)
{
    uint64_t arg_value = arg->eval(context);
    uint64_t ret = 0;

    switch (op) {
      case enums::timingExprSizeInBits:
        if (arg_value == 0)
            ret = 0;
        else
            ret = ceilLog2(arg_value);
        break;
      case enums::timingExprNot:
        ret = arg_value != 0;
        break;
      case enums::timingExprInvert:
        ret = ~arg_value;
        break;
      case enums::timingExprSignExtend32To64:
        ret = static_cast<int64_t>(
            static_cast<int32_t>(arg_value));
        break;
      case enums::timingExprAbs:
        if (static_cast<int64_t>(arg_value) < 0)
            ret = -arg_value;
        else
            ret = arg_value;
        break;
      default:
        break;
    }

    return ret;
}

uint64_t
TimingExprBin::eval(TimingExprEvalContext &context)
{
    uint64_t left_value = left->eval(context);
    uint64_t right_value = right->eval(context);
    uint64_t ret = 0;

    switch (op) {
      case enums::timingExprAdd:
        ret = left_value + right_value;
        break;
      case enums::timingExprSub:
        ret = left_value - right_value;
        break;
      case enums::timingExprUMul:
        ret = left_value * right_value;
        break;
      case enums::timingExprUDiv:
        if (right_value != 0) {
            ret = left_value / right_value;
        }
        break;
      case enums::timingExprUCeilDiv:
        if (right_value != 0) {
            ret = (left_value + (right_value - 1)) / right_value;
        }
        break;
      case enums::timingExprSMul:
        ret = static_cast<int64_t>(left_value) *
            static_cast<int64_t>(right_value);
        break;
      case enums::timingExprSDiv:
        if (right_value != 0) {
            ret = static_cast<int64_t>(left_value) /
                static_cast<int64_t>(right_value);
        }
        break;
      case enums::timingExprEqual:
        ret = left_value == right_value;
        break;
      case enums::timingExprNotEqual:
        ret = left_value != right_value;
        break;
      case enums::timingExprULessThan:
        ret = left_value < right_value;
        break;
      case enums::timingExprUGreaterThan:
        ret = left_value > right_value;
        break;
      case enums::timingExprSLessThan:
        ret = static_cast<int64_t>(left_value) <
            static_cast<int64_t>(right_value);
        break;
      case enums::timingExprSGreaterThan:
        ret = static_cast<int64_t>(left_value) >
            static_cast<int64_t>(right_value);
        break;
      case enums::timingExprAnd:
        ret = (left_value != 0) && (right_value != 0);
        break;
      case enums::timingExprOr:
        ret = (left_value != 0) || (right_value != 0);
        break;
      default:
        break;
    }

    return ret;
}

uint64_t
TimingExprIf::eval(TimingExprEvalContext &context)
{
    uint64_t cond_value = cond->eval(context);

    if (cond_value != 0)
        return trueExpr->eval(context);
    else
        return falseExpr->eval(context);
}

} // namespace gem5
