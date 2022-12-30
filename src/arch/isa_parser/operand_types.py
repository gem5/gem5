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


def overrideInOperand(func):
    func.override_in_operand = True
    return func


overrideInOperand.overrides = dict()


class OperandDesc(object):
    def __init__(
        self, base_cls, dflt_ext, reg_spec, flags=None, sort_pri=None
    ):

        from .isa_parser import makeList

        # Canonical flag structure is a triple of lists, where each list
        # indicates the set of flags implied by this operand always, when
        # used as a source, and when used as a dest, respectively.
        # For simplicity this can be initialized using a variety of fairly
        # obvious shortcuts; we convert these to canonical form here.
        if not flags:
            # no flags specified (e.g., 'None')
            flags = ([], [], [])
        elif isinstance(flags, str):
            # a single flag: assumed to be unconditional
            flags = ([flags], [], [])
        elif isinstance(flags, list):
            # a list of flags: also assumed to be unconditional
            flags = (flags, [], [])
        elif isinstance(flags, tuple):
            # it's a tuple: it should be a triple,
            # but each item could be a single string or a list
            (uncond_flags, src_flags, dest_flags) = flags
            flags = (
                makeList(uncond_flags),
                makeList(src_flags),
                makeList(dest_flags),
            )

        attrs = {}
        # reg_spec is either just a string or a dictionary
        # (for elems of vector)
        if isinstance(reg_spec, tuple):
            (reg_spec, elem_spec) = reg_spec
            if isinstance(elem_spec, str):
                attrs["elem_spec"] = elem_spec
            else:
                assert isinstance(elem_spec, dict)
                attrs["elems"] = elem_spec

        for key in dir(self):
            val = getattr(self, key)
            # If this is a method, extract the function that implements it.
            if hasattr(val, "__func__"):
                val = val.__func__
            # If this should override something in the operand
            if getattr(val, "override_in_operand", False):
                attrs[key] = val

        attrs.update(
            {
                "base_cls": base_cls,
                "dflt_ext": dflt_ext,
                "reg_spec": reg_spec,
                "flags": flags,
                "sort_pri": sort_pri,
            }
        )
        self.attrs = attrs

    def setName(self, name):
        self.attrs["base_name"] = name


class Operand(object):
    """Base class for operand descriptors.  An instance of this class
    (or actually a class derived from this one) represents a specific
    operand for a code block (e.g, "Rc.sq" as a dest). Intermediate
    derived classes encapsulates the traits of a particular operand
    type (e.g., "32-bit integer register")."""

    src_reg_constructor = "\n\tsetSrcRegIdx(_numSrcRegs++, %s);"
    dst_reg_constructor = "\n\tsetDestRegIdx(_numDestRegs++, %s);"

    def regId(self):
        return f"{self.reg_class}[{self.reg_spec}]"

    def srcRegId(self):
        return self.regId()

    def destRegId(self):
        return self.regId()

    def __init__(self, parser, full_name, ext, is_src, is_dest):
        self.parser = parser
        self.full_name = full_name
        self.ext = ext
        self.is_src = is_src
        self.is_dest = is_dest
        # The 'effective extension' (eff_ext) is either the actual
        # extension, if one was explicitly provided, or the default.
        if ext:
            self.eff_ext = ext
        elif hasattr(self, "dflt_ext"):
            self.eff_ext = self.dflt_ext

        if hasattr(self, "eff_ext"):
            self.ctype = parser.operandTypeMap[self.eff_ext]

    # Finalize additional fields (primarily code fields).  This step
    # is done separately since some of these fields may depend on the
    # register index enumeration that hasn't been performed yet at the
    # time of __init__().
    def finalize(self):
        self.flags = self.getFlags()
        self.constructor = self.makeConstructor()
        self.op_decl = self.makeDecl()

        if self.is_src:
            self.op_rd = self.makeRead()
            self.op_src_decl = self.makeDecl()
        else:
            self.op_rd = ""
            self.op_src_decl = ""

        if self.is_dest:
            self.op_wb = self.makeWrite()
            self.op_dest_decl = self.makeDecl()
        else:
            self.op_wb = ""
            self.op_dest_decl = ""

    def isMem(self):
        return 0

    def isReg(self):
        return 0

    def isPCState(self):
        return 0

    def isPCPart(self):
        return self.isPCState() and self.reg_spec

    def getFlags(self):
        # note the empty slice '[:]' gives us a copy of self.flags[0]
        # instead of a reference to it
        my_flags = self.flags[0][:]
        if self.is_src:
            my_flags += self.flags[1]
        if self.is_dest:
            my_flags += self.flags[2]
        return my_flags

    def makeDecl(self):
        # Note that initializations in the declarations are solely
        # to avoid 'uninitialized variable' errors from the compiler.
        return self.ctype + " " + self.base_name + " = 0;\n"


class RegOperand(Operand):
    def isReg(self):
        return 1

    def makeConstructor(self):
        c_src = ""
        c_dest = ""

        if self.is_src:
            c_src = self.src_reg_constructor % self.srcRegId()

        if self.is_dest:
            c_dest = self.dst_reg_constructor % self.destRegId()
            c_dest += f"\n\t_numTypedDestRegs[{self.reg_class}.type()]++;"

        return c_src + c_dest


class RegValOperand(RegOperand):
    def makeRead(self):
        reg_val = f"xc->getRegOperand(this, {self.src_reg_idx})"

        if self.ctype == "float":
            reg_val = f"bitsToFloat32({reg_val})"
        elif self.ctype == "double":
            reg_val = f"bitsToFloat64({reg_val})"

        return f"{self.base_name} = {reg_val};\n"

    def makeWrite(self):
        reg_val = self.base_name

        if self.ctype == "float":
            reg_val = f"floatToBits32({reg_val})"
        elif self.ctype == "double":
            reg_val = f"floatToBits64({reg_val})"

        return f"""
        {{
            RegVal final_val = {reg_val};
            xc->setRegOperand(this, {self.dest_reg_idx}, final_val);
            if (traceData) {{
                traceData->setData({self.reg_class}, final_val);
            }}
        }}"""


class RegOperandDesc(OperandDesc):
    def __init__(self, reg_class, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.attrs["reg_class"] = reg_class


class IntRegOperandDesc(RegOperandDesc):
    def __init__(self, *args, **kwargs):
        super().__init__("intRegClass", RegValOperand, *args, **kwargs)


class FloatRegOperandDesc(RegOperandDesc):
    def __init__(self, *args, **kwargs):
        super().__init__("floatRegClass", RegValOperand, *args, **kwargs)


class CCRegOperandDesc(RegOperandDesc):
    def __init__(self, *args, **kwargs):
        super().__init__("ccRegClass", RegValOperand, *args, **kwargs)


class VecElemOperandDesc(RegOperandDesc):
    def __init__(self, *args, **kwargs):
        super().__init__("vecElemClass", RegValOperand, *args, **kwargs)


class VecRegOperand(RegOperand):
    reg_class = "vecRegClass"

    def __init__(self, parser, full_name, ext, is_src, is_dest):
        super().__init__(parser, full_name, ext, is_src, is_dest)
        self.elemExt = None

    def makeDeclElem(self, elem_op):
        (elem_name, elem_ext) = elem_op
        (elem_spec, dflt_elem_ext) = self.elems[elem_name]
        if elem_ext:
            ext = elem_ext
        else:
            ext = dflt_elem_ext
        ctype = self.parser.operandTypeMap[ext]
        return "\n\t%s %s = 0;" % (ctype, elem_name)

    def makeDecl(self):
        if not self.is_dest and self.is_src:
            c_decl = "\t/* Vars for %s*/" % (self.base_name)
            if hasattr(self, "active_elems"):
                if self.active_elems:
                    for elem in self.active_elems:
                        c_decl += self.makeDeclElem(elem)
            return c_decl + "\t/* End vars for %s */\n" % (self.base_name)
        else:
            return ""

    # Read destination register to write
    def makeReadWElem(self, elem_op):
        (elem_name, elem_ext) = elem_op
        (elem_spec, dflt_elem_ext) = self.elems[elem_name]
        if elem_ext:
            ext = elem_ext
        else:
            ext = dflt_elem_ext
        ctype = self.parser.operandTypeMap[ext]
        c_read = "\t\t%s& %s = %s[%s];\n" % (
            ctype,
            elem_name,
            self.base_name,
            elem_spec,
        )
        return c_read

    def makeReadW(self):
        tmp_name = f"tmp_d{self.dest_reg_idx}"
        c_readw = (
            f"\t\tauto &{tmp_name} = \n"
            f"\t\t    *({self.parser.namespace}::VecRegContainer *)\n"
            f"\t\t    xc->getWritableRegOperand(\n"
            f"\t\t        this, {self.dest_reg_idx});\n"
        )
        if self.elemExt:
            ext = f"{self.parser.operandTypeMap[self.elemExt]}"
            c_readw += f"\t\tauto {self.base_name} = {tmp_name}.as<{ext}>();\n"
        if self.ext:
            ext = f"{self.parser.operandTypeMap[self.ext]}"
            c_readw += f"\t\tauto {self.base_name} = {tmp_name}.as<{ext}>();\n"
        if hasattr(self, "active_elems"):
            if self.active_elems:
                for elem in self.active_elems:
                    c_readw += self.makeReadWElem(elem)
        return c_readw

    # Normal source operand read
    def makeReadElem(self, elem_op, name):
        (elem_name, elem_ext) = elem_op
        (elem_spec, dflt_elem_ext) = self.elems[elem_name]

        if elem_ext:
            ext = elem_ext
        else:
            ext = dflt_elem_ext
        ctype = self.parser.operandTypeMap[ext]
        c_read = "\t\t%s = %s[%s];\n" % (elem_name, name, elem_spec)
        return c_read

    def makeRead(self):
        name = self.base_name
        if self.is_dest and self.is_src:
            name += "_merger"

        tmp_name = f"tmp_s{self.src_reg_idx}"
        c_read = (
            f"\t\t{self.parser.namespace}::VecRegContainer "
            f"{tmp_name};\n"
            f"\t\txc->getRegOperand(this, {self.src_reg_idx},\n"
            f"\t\t    &{tmp_name});\n"
        )
        # If the parser has detected that elements are being access, create
        # the appropriate view
        if self.elemExt:
            ext = f"{self.parser.operandTypeMap[self.elemExt]}"
            c_read += f"\t\tauto {name} = {tmp_name}.as<{ext}>();\n"
        if self.ext:
            ext = f"{self.parser.operandTypeMap[self.ext]}"
            c_read += f"\t\tauto {name} = {tmp_name}.as<{ext}>();\n"
        if hasattr(self, "active_elems"):
            if self.active_elems:
                for elem in self.active_elems:
                    c_read += self.makeReadElem(elem, name)
        return c_read

    def makeWrite(self):
        return f"""
        if (traceData) {{
            traceData->setData({self.reg_class}, &tmp_d{self.dest_reg_idx});
        }}
        """

    def finalize(self):
        super().finalize()
        if self.is_dest:
            self.op_rd = self.makeReadW() + self.op_rd


class VecRegOperandDesc(RegOperandDesc):
    def __init__(self, *args, **kwargs):
        super().__init__("vecRegClass", VecRegOperand, *args, **kwargs)


class VecPredRegOperand(RegOperand):
    reg_class = "vecPredRegClass"

    def makeDecl(self):
        return ""

    def makeRead(self):
        tmp_name = f"tmp_s{self.src_reg_idx}"
        c_read = (
            f"\t\t{self.parser.namespace}::VecPredRegContainer \n"
            f"\t\t        {tmp_name};\n"
            f"xc->getRegOperand(this, {self.src_reg_idx}, "
            f"&{tmp_name});\n"
        )
        if self.ext:
            c_read += (
                f"\t\tauto {self.base_name} = {tmp_name}.as<"
                f"{self.parser.operandTypeMap[self.ext]}>();\n"
            )
        return c_read

    def makeReadW(self):
        tmp_name = f"tmp_d{self.dest_reg_idx}"
        c_readw = (
            f"\t\tauto &{tmp_name} = \n"
            f"\t\t    *({self.parser.namespace}::"
            f"VecPredRegContainer *)xc->getWritableRegOperand("
            f"this, {self.dest_reg_idx});\n"
        )
        if self.ext:
            c_readw += (
                f"\t\tauto {self.base_name} = {tmp_name}.as<"
                f"{self.parser.operandTypeMap[self.ext]}>();\n"
            )
        return c_readw

    def makeWrite(self):
        return f"""
        if (traceData) {{
            traceData->setData({self.reg_class}, &tmp_d{self.dest_reg_idx});
        }}
        """

    def finalize(self):
        super().finalize()
        if self.is_dest:
            self.op_rd = self.makeReadW() + self.op_rd


class VecPredRegOperandDesc(RegOperandDesc):
    def __init__(self, *args, **kwargs):
        super().__init__("vecPredRegClass", VecPredRegOperand, *args, **kwargs)


class ControlRegOperand(Operand):
    reg_class = "miscRegClass"

    def isReg(self):
        return 1

    def isControlReg(self):
        return 1

    def makeConstructor(self):
        c_src = ""
        c_dest = ""

        if self.is_src:
            c_src = self.src_reg_constructor % self.srcRegId()

        if self.is_dest:
            c_dest = self.dst_reg_constructor % self.destRegId()

        return c_src + c_dest

    def makeRead(self):
        bit_select = 0
        if self.ctype == "float" or self.ctype == "double":
            error("Attempt to read control register as FP")

        return (
            f"{self.base_name} = "
            f"xc->readMiscRegOperand(this, {self.src_reg_idx});\n"
        )

    def makeWrite(self):
        if self.ctype == "float" or self.ctype == "double":
            error("Attempt to write control register as FP")
        wb = (
            f"xc->setMiscRegOperand(this, "
            f"{self.dest_reg_idx}, {self.base_name});\n"
        )
        wb += f"""
        if (traceData) {{
            traceData->setData({self.reg_class}, {self.base_name});
        }}
        """

        return wb


class ControlRegOperandDesc(RegOperandDesc):
    def __init__(self, *args, **kwargs):
        super().__init__("miscRegClass", ControlRegOperand, *args, **kwargs)


class MemOperand(Operand):
    def isMem(self):
        return 1

    def makeConstructor(self):
        return ""

    def makeDecl(self):
        # Declare memory data variable.
        return f"{self.ctype} {self.base_name} = {{}};\n"

    def makeRead(self):
        return ""

    def makeWrite(self):
        return ""


class MemOperandDesc(OperandDesc):
    def __init__(self, *args, **kwargs):
        super().__init__(MemOperand, *args, **kwargs)


class PCStateOperand(Operand):
    def __init__(self, parser, *args, **kwargs):
        super().__init__(parser, *args, **kwargs)
        self.parser = parser

    def makeConstructor(self):
        return ""

    def makeRead(self):
        if self.reg_spec:
            # A component of the PC state.
            return (
                f"{self.base_name} = "
                f"__parserAutoPCState.{self.reg_spec}();\n"
            )
        else:
            # The whole PC state itself.
            return (
                f"{self.base_name} = "
                f"xc->pcState().as<{self.parser.namespace}::PCState>();\n"
            )

    def makeWrite(self):
        if self.reg_spec:
            # A component of the PC state.
            return "__parserAutoPCState.%s(%s);\n" % (
                self.reg_spec,
                self.base_name,
            )
        else:
            # The whole PC state itself.
            return f"xc->pcState({self.base_name});\n"

    def makeDecl(self):
        ctype = f"{self.parser.namespace}::PCState"
        if self.isPCPart():
            ctype = self.ctype
        # Note that initializations in the declarations are solely
        # to avoid 'uninitialized variable' errors from the compiler.
        return "%s %s = 0;\n" % (ctype, self.base_name)

    def isPCState(self):
        return 1


class PCStateOperandDesc(OperandDesc):
    def __init__(self, *args, **kwargs):
        super().__init__(PCStateOperand, *args, **kwargs)
