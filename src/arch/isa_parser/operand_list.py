# Copyright (c) 2014, 2016, 2018-2019 ARM Limited
# All rights reserved
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
# Copyright (c) 2003-2005 The Regents of The University of Michigan
# Copyright (c) 2013,2015 Advanced Micro Devices, Inc.
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

from .util import assignRE, commentRE, stringRE
from .util import error


class OperandList(object):
    """Find all the operands in the given code block.  Returns an operand
    descriptor list (instance of class OperandList)."""

    def __init__(self, parser, code):
        self.items = []
        self.bases = {}
        # delete strings and comments so we don't match on operands inside
        for regEx in (stringRE, commentRE):
            code = regEx.sub("", code)

        # search for operands
        for match in parser.operandsRE().finditer(code):
            op = match.groups()
            # regexp groups are operand full name, base, and extension
            (op_full, op_base, op_ext) = op
            # If is a elem operand, define or update the corresponding
            # vector operand
            isElem = False
            if op_base in parser.elemToVector:
                isElem = True
                elem_op = (op_base, op_ext)
                op_base = parser.elemToVector[op_base]
                op_ext = ""  # use the default one
            # if the token following the operand is an assignment, this is
            # a destination (LHS), else it's a source (RHS)
            is_dest = assignRE.match(code, match.end()) != None
            is_src = not is_dest

            # see if we've already seen this one
            op_desc = self.find_base(op_base)
            if op_desc:
                if op_ext and op_ext != "" and op_desc.ext != op_ext:
                    error(
                        "Inconsistent extensions for operand %s: %s - %s"
                        % (op_base, op_desc.ext, op_ext)
                    )
                op_desc.is_src = op_desc.is_src or is_src
                op_desc.is_dest = op_desc.is_dest or is_dest
                if isElem:
                    (elem_base, elem_ext) = elem_op
                    found = False
                    for ae in op_desc.active_elems:
                        (ae_base, ae_ext) = ae
                        if ae_base == elem_base:
                            if ae_ext != elem_ext:
                                error(
                                    "Inconsistent extensions for elem"
                                    " operand %s" % elem_base
                                )
                            else:
                                found = True
                    if not found:
                        op_desc.active_elems.append(elem_op)
            else:
                # new operand: create new descriptor
                op_desc = parser.operandNameMap[op_base](
                    parser, op_full, op_ext, is_src, is_dest
                )
                # if operand is a vector elem, add the corresponding vector
                # operand if not already done
                if isElem:
                    op_desc.elemExt = elem_op[1]
                    op_desc.active_elems = [elem_op]
                self.append(op_desc)

        self.sort()

        # enumerate source & dest register operands... used in building
        # constructor later
        regs = list(filter(lambda i: i.isReg(), self.items))
        mem = list(filter(lambda i: i.isMem(), self.items))
        srcs = list(filter(lambda r: r.is_src, regs))
        dests = list(filter(lambda r: r.is_dest, regs))

        for idx, reg in enumerate(srcs):
            reg.src_reg_idx = idx
        for idx, reg in enumerate(dests):
            reg.dest_reg_idx = idx

        self.numSrcRegs = len(srcs)
        self.numDestRegs = len(dests)

        if len(mem) > 1:
            error("Code block has more than one memory operand")

        self.memOperand = mem[0] if mem else None

        # now make a final pass to finalize op_desc fields that may depend
        # on the register enumeration
        for op_desc in self.items:
            op_desc.finalize()

    def __len__(self):
        return len(self.items)

    def __getitem__(self, index):
        return self.items[index]

    def append(self, op_desc):
        self.items.append(op_desc)
        self.bases[op_desc.base_name] = op_desc

    def find_base(self, base_name):
        # like self.bases[base_name], but returns None if not found
        # (rather than raising exception)
        return self.bases.get(base_name)

    # internal helper function for concat[Some]Attr{Strings|Lists}
    def __internalConcatAttrs(self, attr_name, filter, result):
        for op_desc in self.items:
            if filter(op_desc):
                result += getattr(op_desc, attr_name)
        return result

    # return a single string that is the concatenation of the (string)
    # values of the specified attribute for all operands
    def concatAttrStrings(self, attr_name):
        return self.__internalConcatAttrs(attr_name, lambda x: 1, "")

    # like concatAttrStrings, but only include the values for the operands
    # for which the provided filter function returns true
    def concatSomeAttrStrings(self, filter, attr_name):
        return self.__internalConcatAttrs(attr_name, filter, "")

    # return a single list that is the concatenation of the (list)
    # values of the specified attribute for all operands
    def concatAttrLists(self, attr_name):
        return self.__internalConcatAttrs(attr_name, lambda x: 1, [])

    # like concatAttrLists, but only include the values for the operands
    # for which the provided filter function returns true
    def concatSomeAttrLists(self, filter, attr_name):
        return self.__internalConcatAttrs(attr_name, filter, [])

    def sort(self):
        self.items.sort(key=lambda a: a.sort_pri)


class SubOperandList(OperandList):
    """Find all the operands in the given code block.  Returns an operand
    descriptor list (instance of class OperandList)."""

    def __init__(self, parser, code, requestor_list):
        self.items = []
        self.bases = {}
        # delete strings and comments so we don't match on operands inside
        for regEx in (stringRE, commentRE):
            code = regEx.sub("", code)

        # search for operands
        for match in parser.operandsRE().finditer(code):
            op = match.groups()
            # regexp groups are operand full name, base, and extension
            (op_full, op_base, op_ext) = op
            # If is a elem operand, define or update the corresponding
            # vector operand
            if op_base in parser.elemToVector:
                elem_op = op_base
                op_base = parser.elemToVector[elem_op]
            # find this op in the requestor list
            op_desc = requestor_list.find_base(op_base)
            if not op_desc:
                error(
                    "Found operand %s which is not in the requestor list!"
                    % op_base
                )
            else:
                # See if we've already found this operand
                op_desc = self.find_base(op_base)
                if not op_desc:
                    # if not, add a reference to it to this sub list
                    self.append(requestor_list.bases[op_base])

        self.sort()

        pcs = list(filter(lambda i: i.isPCState(), self.items))
        mem = list(filter(lambda i: i.isMem(), self.items))

        if len(mem) > 1:
            error("Code block has more than one memory operand")

        part = any(p.isPCPart() for p in pcs)
        whole = any(not p.isPCPart() for p in pcs)

        if part and whole:
            error("Mixed whole and partial PC state operands")

        self.memOperand = mem[0] if mem else None

        # Whether the whole PC needs to be read so parts of it can be accessed
        self.readPC = any(i.isPCPart() for i in self.items)
        # Whether the whole PC needs to be written after parts of it were
        # changed
        self.setPC = any(i.isPCPart() and i.is_dest for i in self.items)
        # Whether this instruction manipulates the whole PC or parts of it.
        # Mixing the two is a bad idea and flagged as an error.
        self.pcPart = None
        if part:
            self.pcPart = True
        if whole:
            self.pcPart = False
