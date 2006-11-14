%module debug

%{
// include these files when compiling debug_wrap.cc
#include "sim/host.hh"
%}

%include "stdint.i"
%include "sim/host.hh"

%inline %{
extern void schedBreakCycle(Tick when);
%}

%wrapper %{
// fix up module name to reflect the fact that it's inside the m5 package
#undef SWIG_name
#define SWIG_name "m5.internal._debug"
%}
