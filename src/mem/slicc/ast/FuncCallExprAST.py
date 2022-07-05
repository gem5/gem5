# Copyright (c) 2020 ARM Limited
# All rights reserved.
#
# The license below extends only to copyright in the software and shall
# not be construed as granting a license to any other intellectual
# property including but not limited to intellectual property relating
# to a hardware implementation of the functionality of the software
# licensed hereunder.  You may use the software subject to the license
# terms below provided that you ensure that this notice is replicated
# unmodified and in its entirety in all distributions of the software,
# modified or unmodified, in source code or in binary form.
#
# Copyright (c) 1999-2008 Mark D. Hill and David A. Wood
# Copyright (c) 2009 The Hewlett-Packard Development Company
# Copyright (c) 2013 Advanced Micro Devices, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met: redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer;
# redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution;
# neither the name of the copyright holders nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

from slicc.ast.ExprAST import ExprAST
from slicc.symbols import Func, Type


class FuncCallExprAST(ExprAST):
    def __init__(self, slicc, proc_name, exprs):
        super().__init__(slicc)
        self.proc_name = proc_name
        self.exprs = exprs

    def __repr__(self):
        return "[FuncCallExpr: %s %s]" % (self.proc_name, self.exprs)

    # When calling generate for statements in a in_port, the reference to
    # the port must be provided as the in_port kwarg (see InPortDeclAST)
    def generate(self, code, **kwargs):
        machine = self.state_machine

        if self.proc_name == "DPRINTF":
            # Code for inserting the location of the DPRINTF()
            # statement in the .sm file in the statement it self.
            # 'self.exprs[0].location' represents the location.
            # 'format' represents the second argument of the
            # original DPRINTF() call. It is left unmodified.
            # str_list is used for concatenating the argument
            # list following the format specifier. A DPRINTF()
            # call may or may not contain any arguments following
            # the format specifier. These two cases need to be
            # handled differently. Hence the check whether or not
            # the str_list is empty.

            dflag = "%s" % (self.exprs[0].name)
            machine.addDebugFlag(dflag)
            format = "%s" % (self.exprs[1].inline())
            format_length = len(format)
            str_list = []

            for i in range(2, len(self.exprs)):
                str_list.append("%s" % self.exprs[i].inline())

            if len(str_list) == 0:
                code(
                    'DPRINTF($0, "$1: $2")',
                    dflag,
                    self.exprs[0].location,
                    format[2 : format_length - 2],
                )
            else:
                code(
                    'DPRINTF($0, "$1: $2", $3)',
                    dflag,
                    self.exprs[0].location,
                    format[2 : format_length - 2],
                    ", ".join(str_list),
                )

            return self.symtab.find("void", Type)

        if self.proc_name == "DPRINTFN":
            format = "%s" % (self.exprs[0].inline())
            format_length = len(format)
            str_list = []

            for i in range(1, len(self.exprs)):
                str_list.append("%s" % self.exprs[i].inline())

            if len(str_list) == 0:
                code(
                    'DPRINTFN("$0: $1")',
                    self.exprs[0].location,
                    format[2 : format_length - 2],
                )
            else:
                code(
                    'DPRINTFN("$0: $1", $2)',
                    self.exprs[0].location,
                    format[2 : format_length - 2],
                    ", ".join(str_list),
                )

            return self.symtab.find("void", Type)

        # hack for adding comments to profileTransition
        if self.proc_name == "APPEND_TRANSITION_COMMENT":
            # FIXME - check for number of parameters
            code("APPEND_TRANSITION_COMMENT($0)", self.exprs[0].inline())
            return self.symtab.find("void", Type)

        func_name_args = self.proc_name

        for expr in self.exprs:
            actual_type, param_code = expr.inline(True)
            func_name_args += "_" + str(actual_type.ident)

        # Look up the function in the symbol table
        func = self.symtab.find(func_name_args, Func)

        # Check the types and get the code for the parameters
        if func is None:
            self.error("Unrecognized function name: '%s'", func_name_args)

        cvec, type_vec = func.checkArguments(self.exprs)

        # OK, the semantics of "trigger" here is that, ports in the
        # machine have different priorities. We always check the first
        # port for doable transitions. If nothing/stalled, we pick one
        # from the next port.
        #
        # One thing we have to be careful as the SLICC protocol
        # writter is : If a port have two or more transitions can be
        # picked from in one cycle, they must be independent.
        # Otherwise, if transition A and B mean to be executed in
        # sequential, and A get stalled, transition B can be issued
        # erroneously. In practice, in most case, there is only one
        # transition should be executed in one cycle for a given
        # port. So as most of current protocols.

        if self.proc_name == "trigger":
            code(
                """
{
"""
            )
            if machine.TBEType != None and machine.EntryType != None:
                code(
                    """
    TransitionResult result = doTransition(${{cvec[0]}}, ${{cvec[2]}}, ${{cvec[3]}}, ${{cvec[1]}});
"""
                )
            elif machine.TBEType != None:
                code(
                    """
    TransitionResult result = doTransition(${{cvec[0]}}, ${{cvec[2]}}, ${{cvec[1]}});
"""
                )
            elif machine.EntryType != None:
                code(
                    """
    TransitionResult result = doTransition(${{cvec[0]}}, ${{cvec[2]}}, ${{cvec[1]}});
"""
                )
            else:
                code(
                    """
    TransitionResult result = doTransition(${{cvec[0]}}, ${{cvec[1]}});
"""
                )

            assert "in_port" in kwargs
            in_port = kwargs["in_port"]

            code(
                """
    if (result == TransitionResult_Valid) {
        counter++;
        continue; // Check the first port again
    } else if (result == TransitionResult_ResourceStall) {
"""
            )
            if "rsc_stall_handler" in in_port.pairs:
                stall_func_name = in_port.pairs["rsc_stall_handler"]
                code(
                    """
        if (${{stall_func_name}}()) {
            counter++;
            continue; // Check the first port again
        } else {
            scheduleEvent(Cycles(1));
            // Cannot do anything with this transition, go check next doable transition (mostly likely of next port)
        }
"""
                )
            else:
                code(
                    """
        scheduleEvent(Cycles(1));
        // Cannot do anything with this transition, go check next doable transition (mostly likely of next port)
"""
                )
            code(
                """
    } else if (result == TransitionResult_ProtocolStall) {
"""
            )
            if "prot_stall_handler" in in_port.pairs:
                stall_func_name = in_port.pairs["prot_stall_handler"]
                code(
                    """
        if (${{stall_func_name}}()) {
            counter++;
            continue; // Check the first port again
        } else {
            scheduleEvent(Cycles(1));
            // Cannot do anything with this transition, go check next doable transition (mostly likely of next port)
        }
"""
                )
            else:
                code(
                    """
        scheduleEvent(Cycles(1));
        // Cannot do anything with this transition, go check next doable transition (mostly likely of next port)
"""
                )
            code(
                """
    }

}
"""
            )
        elif self.proc_name == "error":
            code("$0", self.exprs[0].embedError(cvec[0]))
        elif self.proc_name == "assert":
            error = self.exprs[0].embedError('"assert failure"')
            code(
                """
#ifndef NDEBUG
if (!(${{cvec[0]}})) {
    $error
}
#endif
"""
            )

        elif self.proc_name == "set_cache_entry":
            code("set_cache_entry(m_cache_entry_ptr, %s);" % (cvec[0]))
        elif self.proc_name == "unset_cache_entry":
            code("unset_cache_entry(m_cache_entry_ptr);")
        elif self.proc_name == "set_tbe":
            code("set_tbe(m_tbe_ptr, %s);" % (cvec[0]))
        elif self.proc_name == "unset_tbe":
            code("unset_tbe(m_tbe_ptr);")
        elif self.proc_name == "stallPort":
            code("scheduleEvent(Cycles(1));")

        else:
            # Normal function
            if "external" not in func and not func.isInternalMachineFunc:
                self.error("Invalid function")

            params = ""
            first_param = True

            for (param_code, type) in zip(cvec, type_vec):
                if first_param:
                    params = str(param_code)
                    first_param = False
                else:
                    params += ", "
                    params += str(param_code)

            fix = code.nofix()
            code("(${{func.c_name}}($params))")
            code.fix(fix)

        return func.return_type
