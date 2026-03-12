#ifndef __CPU_TIMING_EXPR_LITERAL_HH__
#define __CPU_TIMING_EXPR_LITERAL_HH__

#include "cpu/timing_expr.hh"
#include "params/TimingExprLiteral.hh"

namespace gem5
{

class TimingExprLiteral : public TimingExpr
{
  public:
    uint64_t value;

    TimingExprLiteral(const TimingExprLiteralParams &params);

    uint64_t eval(TimingExprEvalContext &context) override { return value; }
};

} // namespace gem5

#endif
