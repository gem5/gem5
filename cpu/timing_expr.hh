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

/*
 * These classes define an expression language over uint64_t with only
 * a few operators.  This can be used to form expressions for the extra
 * delay required in variable execution time instructions.
 *
 * Expressions, in evaluation, will have access to the ThreadContext and
 * a StaticInst.
 */

#ifndef __CPU_TIMING_EXPR_HH__
#define __CPU_TIMING_EXPR_HH__

#include "cpu/static_inst.hh"
#include "cpu/thread_context.hh"
#include "enums/TimingExprOp.hh"
#include "params/TimingExpr.hh"
#include "params/TimingExprBin.hh"
#include "params/TimingExprIf.hh"
#include "params/TimingExprLet.hh"
#include "params/TimingExprLiteral.hh"
#include "params/TimingExprRef.hh"
#include "params/TimingExprSrcReg.hh"
#include "params/TimingExprUn.hh"
#include "sim/sim_object.hh"

namespace gem5
{

/** These classes are just the C++ counterparts for those in Expr.py and
 *  are, therefore, documented there */

class TimingExprLet;

/** Object to gather the visible context for evaluation */
class TimingExprEvalContext
{
  public:
    /** Special visible context */
    const StaticInstPtr &inst;
    ThreadContext *thread;

    /** Context visible as sub expressions.  results will hold the results
     *  of (lazily) evaluating let's expressions.  resultAvailable elements
     *  are true when a result has actually been evaluated */
    TimingExprLet *let;
    std::vector<uint64_t> results;
    std::vector<bool > resultAvailable;

    TimingExprEvalContext(const StaticInstPtr &inst_,
        ThreadContext *thread_, TimingExprLet *let_);
};

class TimingExpr : public SimObject
{
  public:
    TimingExpr(const TimingExprParams &params) :
        SimObject(params)
    { }

    virtual uint64_t eval(TimingExprEvalContext &context) = 0;
};

class TimingExprLiteral : public TimingExpr
{
  public:
    uint64_t value;

    TimingExprLiteral(const TimingExprLiteralParams &params) :
        TimingExpr(params),
        value(params.value)
    { }

    uint64_t eval(TimingExprEvalContext &context) { return value; }
};

class TimingExprSrcReg : public TimingExpr
{
  public:
    unsigned int index;

    TimingExprSrcReg(const TimingExprSrcRegParams &params) :
        TimingExpr(params),
        index(params.index)
    { }

    uint64_t eval(TimingExprEvalContext &context);
};

class TimingExprLet : public TimingExpr
{
  public:
    std::vector<TimingExpr *> defns;
    TimingExpr *expr;

    TimingExprLet(const TimingExprLetParams &params) :
        TimingExpr(params),
        defns(params.defns),
        expr(params.expr)
    { }

    uint64_t eval(TimingExprEvalContext &context);
};

class TimingExprRef : public TimingExpr
{
  public:
    unsigned int index;

    TimingExprRef(const TimingExprRefParams &params) :
        TimingExpr(params),
        index(params.index)
    { }

    uint64_t eval(TimingExprEvalContext &context);
};

class TimingExprUn : public TimingExpr
{
  public:
    enums::TimingExprOp op;
    TimingExpr *arg;

    TimingExprUn(const TimingExprUnParams &params) :
        TimingExpr(params),
        op(params.op),
        arg(params.arg)
    { }

    uint64_t eval(TimingExprEvalContext &context);
};

class TimingExprBin : public TimingExpr
{
  public:
    enums::TimingExprOp op;
    TimingExpr *left;
    TimingExpr *right;

    TimingExprBin(const TimingExprBinParams &params) :
        TimingExpr(params),
        op(params.op),
        left(params.left),
        right(params.right)
    { }

    uint64_t eval(TimingExprEvalContext &context);
};

class TimingExprIf : public TimingExpr
{
  public:
    TimingExpr *cond;
    TimingExpr *trueExpr;
    TimingExpr *falseExpr;

    TimingExprIf(const TimingExprIfParams &params) :
        TimingExpr(params),
        cond(params.cond),
        trueExpr(params.trueExpr),
        falseExpr(params.falseExpr)
    { }

    uint64_t eval(TimingExprEvalContext &context);
};

} // namespace gem5

#endif
