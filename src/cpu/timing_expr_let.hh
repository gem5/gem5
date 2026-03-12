#ifndef __CPU_TIMING_EXPR_LET_HH__
#define __CPU_TIMING_EXPR_LET_HH__

#include "cpu/timing_expr.hh"
#include "params/TimingExprLet.hh"

namespace gem5
{

class TimingExprLet : public TimingExpr
{
  public:
    std::vector<TimingExpr *> defns;
    TimingExpr *expr;

    TimingExprLet(const TimingExprLetParams &params);

    uint64_t eval(TimingExprEvalContext &context) override;
};

} // namespace gem5

#endif
