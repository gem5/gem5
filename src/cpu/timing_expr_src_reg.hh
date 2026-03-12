#ifndef __CPU_TIMING_EXPR_SRC_REG_HH__
#define __CPU_TIMING_EXPR_SRC_REG_HH__

#include "cpu/timing_expr.hh"
#include "params/TimingExprSrcReg.hh"

namespace gem5
{

class TimingExprSrcReg : public TimingExpr
{
  public:
    unsigned int index;

    TimingExprSrcReg(const TimingExprSrcRegParams &params);

    uint64_t eval(TimingExprEvalContext &context) override;
};

} // namespace gem5

#endif
