#ifndef __CPU_TIMING_EXPR_IF_HH__
#define __CPU_TIMING_EXPR_IF_HH__

#include "cpu/timing_expr.hh"
#include "params/TimingExprIf.hh"

namespace gem5
{

class TimingExprIf : public TimingExpr
{
  public:
    TimingExpr *cond;
    TimingExpr *trueExpr;
    TimingExpr *falseExpr;

    TimingExprIf(const TimingExprIfParams &params);

    uint64_t eval(TimingExprEvalContext &context) override;
};

} // namespace gem5

#endif
