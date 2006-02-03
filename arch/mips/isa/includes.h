////////////////////////////////////////////////////////////////////
//
// Output include file directives.
//

output header {{
#include <sstream>
#include <iostream>
#include <iomanip>

#include "cpu/static_inst.hh"
#include "traps.hh"
#include "mem/mem_req.hh"  // some constructors use MemReq flags
}};

output decoder {{
#include "base/cprintf.hh"
#include "base/loader/symtab.hh"
#include "cpu/exec_context.hh"  // for Jump::branchTarget()

#include <math.h>
#if defined(linux)
#include <fenv.h>
#endif
}};

output exec {{
#include <math.h>
#if defined(linux)
#include <fenv.h>
#endif

#ifdef FULL_SYSTEM
//#include "arch/alpha/pseudo_inst.hh"
#endif
#include "cpu/base.hh"
#include "cpu/exetrace.hh"
#include "sim/sim_exit.hh"
}};

