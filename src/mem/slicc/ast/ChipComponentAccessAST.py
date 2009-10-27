# Copyright (c) 1999-2008 Mark D. Hill and David A. Wood
# Copyright (c) 2009 The Hewlett-Packard Development Company
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

import re

from slicc.ast.ExprAST import ExprAST
from slicc.symbols import Type

class ChipComponentAccessAST(ExprAST):
    def __init__(self, slicc, machine, mach_version, component):
        super(ChipComponentAccessAST, self).__init__(slicc)
        self.mach_var = machine
        self.comp_var = component
        self.mach_ver_expr = mach_version

    def __repr__(self):
        return "[ChipAccessExpr: %r]" % self.expr_vec

    def generate(self, code):
        void_type = self.symtab.find("void", Type)

        mname = self.mach_var.name
        cname = self.comp_var.name
        var = self.symtab.machine_components[mname][cname]

        vcode = str(var.code)

        if self.chip_ver_expr is not None:
            # replace self.chip with specified chip
            gcode = "g_system.getChip(%s)" % self.chip_ver_expr.inline()
            vcode = re.sub("m_chip", gcode, vcode)

        # replace default "m_version" with the version we really want
        gcode = "(%s)" % self.mach_ver_expr.inline()
        vcode = re.sub("m_version", gcode, vcode)

        return_type, gcode = self.generate_access(var)
        code("($vcode)$gcode")
        return return_type

class ChipMethodAccessAST(ChipComponentAccessAST):
    def __init__(self, slicc, chip_version, machine, mach_version, component,
                 proc_name, expr_vec):
        s = super(ChipMethodAccessAST, self)
        s.__init__(slicc, machine, mach_version, component)

        self.chip_ver_expr = chip_version
        self.expr_vec = expr_vec
        self.proc_name = proc_name

    def generate_access(self, var):
        # generate code
        paramTypes = []
        gcode = []
        for expr in self.expr_vec:
            t,c = expr.generate()
            paramTypes.append(t)
            gcode.append(c)

        methodId = var.type.methodId(self.proc_name, paramTypes)

        # Verify that this is a method of the object
        if not var.type.methodExist(methodId):
            self.error("%s: Type '%s' does not have a method '%s'" % \
                       ("Invalid method call", var.type, methodId))

        expected_size = len(var.type.methodParamType(methodId))
        if len(self.expr_vec) != expected_size:
            # Right number of parameters
            self.error("Wrong number of parameters for function name: " +\
                       "'%s', expected: %d, actual: %d",
                       self.proc_name, expected_size, len(self.expr_vec))

        for expr,expected,actual in zip(self.expr_vec,
                                        var.type.methodParamType(methodId),
                                        paramTypes):
            # Check the types of the parameter
            if actual != expected:
                expr.error("Type mismatch: expected: %s actual: %s",
                           expected, actual)

        # method call
        code = ".%s(%s)" % (self.proc_name, ', '.join(gcode))

        # Return the return type of the method
        return var.type.methodReturnType(methodId), code

class LocalChipMethodAST(ChipMethodAccessAST):
    # method call from local chip
    def __init__(self, slicc, machine, mach_version, component, proc_name,
                 expr_vec):
        s = super(LocalChipMethodAST, self)
        s.__init__(slicc, None, machine, mach_version, component, proc_name,
                  expr_vec)

class SpecifiedChipMethodAST(ChipMethodAccessAST):
    # method call from specified chip
    def __init__(self, slicc, chip_version, machine, mach_version, component,
                 proc_name, expr_vec):
        s = super(SpecifiedChipMethodAST, self)
        s.__init__(slicc, chip_version, machine, mach_version, component,
                   proc_name, expr_vec)

class ChipMemberAccessAST(ChipComponentAccessAST):
    # member access from specified chip
    def __init__(self, chip_version, machine, mach_version, component,
                 field_name):
        s = super(ChipMemberAccessAST, self)
        s.__init__(slicc, machine, mach_version, component)

        self.chip_ver_expr = chip_version
        self.field_name = field_name

    def generate_access(self, var):
        # Verify that this is a valid field name for this type
        if not var.type.dataMemberExist(self.field_name):
            self.error("Invalid object field: " +\
                       "Type '%s' does not have data member %s",
                       var.type, self.field_name)

        code += ").m_%s" % self.field_name

        return var.type.dataMemberType(self.field_name), code

class LocalChipMemberAST(ChipMemberAccessAST):
    # member access from local chip
    def __init__(self, slicc, machine, mach_version, component, field_name):
        s = super(LocalChipMemberAST, self)
        s.__init__(slicc, None, machine, mach_version, component,  field_name)

class SpecifiedChipMemberAST(ChipMemberAccessAST):
    # member access from specified chip
    def __init__(self, chip_version, machine, mach_version, component,
                 field_name):
        s = super(SpecifiedChipMemberAST, self)
        s.__init__(slicc, chip_version, machine, mach_version, component,
                   field_name)
