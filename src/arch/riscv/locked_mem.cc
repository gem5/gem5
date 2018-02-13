#include "arch/riscv/locked_mem.hh"

#include <stack>

#include "base/types.hh"

namespace RiscvISA
{
    std::unordered_map<int, std::stack<Addr>> locked_addrs;
}
