#ifndef __CPU_TIMING_EXPR_UN_HH__
#define __CPU_TIMING_EXPR_UN_HH__

#include "cpu/timing_expr.hh"
#include "enums/TimingExprOp.hh"
#include "params/TimingExprUn.hh"

namespace gem5
{

class TimingExprUn : public TimingExpr
{
  public:
    enums::TimingExprOp op;
    TimingExpr *arg;

    TimingExprUn(const TimingExprUnParams &params);

    uint64_t eval(TimingExprEvalContext &context) override;
};

} // namespace gem5

#endif
