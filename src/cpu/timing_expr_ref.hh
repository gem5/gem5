#ifndef __CPU_TIMING_EXPR_REF_HH__
#define __CPU_TIMING_EXPR_REF_HH__

#include "cpu/timing_expr.hh"
#include "params/TimingExprRef.hh"

namespace gem5
{

class TimingExprRef : public TimingExpr
{
  public:
    unsigned int index;

    TimingExprRef(const TimingExprRefParams &params);

    uint64_t eval(TimingExprEvalContext &context) override;
};

} // namespace gem5

#endif
