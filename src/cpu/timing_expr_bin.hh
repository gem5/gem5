#ifndef __CPU_TIMING_EXPR_BIN_HH__
#define __CPU_TIMING_EXPR_BIN_HH__

#include "cpu/timing_expr.hh"
#include "enums/TimingExprOp.hh"
#include "params/TimingExprBin.hh"

namespace gem5
{

class TimingExprBin : public TimingExpr
{
  public:
    enums::TimingExprOp op;
    TimingExpr *left;
    TimingExpr *right;

    TimingExprBin(const TimingExprBinParams &params);

    uint64_t eval(TimingExprEvalContext &context) override;
};

} // namespace gem5

#endif
