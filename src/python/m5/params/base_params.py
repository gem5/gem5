# Copyright (c) 2012-2014, 2017-2019, 2021, 2024-2025 Arm Limited
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
# Copyright (c) 2004-2006 The Regents of The University of Michigan
# Copyright (c) 2010-2011 Advanced Micro Devices, Inc.
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

#####################################################################
#
# Parameter description classes
#
# The _params dictionary in each class maps parameter names to either
# a Param or a VectorParam object.  These objects contain the
# parameter description string, the parameter type, and the default
# value (if any).  The convert() method on these objects is used to
# force whatever value is assigned to the parameter to the appropriate
# type.
#
# Note that the default values are loaded into the class's attribute
# space when the parameter dictionary is initialized (in
# MetaSimObject._new_param()); after that point they aren't used.
#
#####################################################################

import math
import pprint

from .. import proxy
from ..util import warn
from .null_params import (
    NULL,
    NullOpt,
    isNullOpt,
    isNullPointer,
)


def isSimObject(*args, **kwargs):
    from .. import SimObject

    return SimObject.isSimObject(*args, **kwargs)


def isSimObjectSequence(*args, **kwargs):
    from .. import SimObject

    return SimObject.isSimObjectSequence(*args, **kwargs)


def isSimObjectClass(*args, **kwargs):
    from .. import SimObject

    return SimObject.isSimObjectClass(*args, **kwargs)


allParams = {}


class MetaParamValue(type):
    def __new__(mcls, name, bases, dct):
        cls = super().__new__(mcls, name, bases, dct)
        if name in allParams:
            warn(
                "%s already exists in allParams. This may be caused by the "
                "Python 2.7 compatibility layer." % (name,)
            )
        allParams[name] = cls
        return cls


# Dummy base class to identify types that are legitimate for SimObject
# parameters.
class ParamValue(metaclass=MetaParamValue):
    cmd_line_settable = False

    # Generate the code needed as a prerequisite for declaring a C++
    # object of this type.  Typically generates one or more #include
    # statements.  Used when declaring parameters of this type.
    @classmethod
    def cxx_predecls(cls, code):
        pass

    @classmethod
    def pybind_predecls(cls, code):
        cls.cxx_predecls(code)

    # default for printing to .ini file is regular string conversion.
    # will be overridden in some cases
    def ini_str(self):
        return str(self)

    # default for printing to .json file is regular string conversion.
    # will be overridden in some cases, mostly to use native Python
    # types where there are similar JSON types
    def config_value(self):
        return str(self)

    # Prerequisites for .ini parsing with cxx_ini_parse
    @classmethod
    def cxx_ini_predecls(cls, code):
        pass

    # parse a .ini file entry for this param from string expression
    # src into lvalue dest (of the param's C++ type)
    @classmethod
    def cxx_ini_parse(cls, code, src, dest, ret):
        code(f"// Unhandled param type: {cls.__name__}")
        code(f"{ret} false;")

    # allows us to blithely call unproxy() on things without checking
    # if they're really proxies or not
    def unproxy(self, base):
        return self

    # Produce a human readable version of the stored value
    def pretty_print(self, value):
        return str(value)


# Regular parameter description.
class ParamDesc:
    def __init__(self, *args, **kwargs):
        if args:
            if len(args) == 1:
                self.desc = args[0]
            elif len(args) == 2:
                self.default = args[0]
                self.desc = args[1]
            else:
                raise TypeError("too many arguments")

        if "desc" in kwargs:
            assert not hasattr(self, "desc")
            self.desc = kwargs["desc"]
            del kwargs["desc"]

        if "default" in kwargs:
            assert not hasattr(self, "default")
            self.default = kwargs["default"]
            del kwargs["default"]

        if kwargs:
            raise TypeError(f"extra unknown kwargs {kwargs}")

        if not hasattr(self, "desc"):
            raise TypeError("desc attribute missing")


class SingleTypeParamDesc(ParamDesc):
    """
    ParamDesc with a single type. This applies for example
    to an Int Param, or a Vector<Int> Param, and not to
    parameters with multiple data types, like a Dict Param,
    which has more than one type (the type of the key and the
    type of the value).
    """

    def __init__(self, ptype_str, ptype, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.ptype_str = ptype_str
        # remember ptype only if it is provided
        if ptype != None:
            self.ptype = ptype

    def __getattr__(self, attr):
        if attr == "ptype":
            from .. import SimObject

            ptype = SimObject.allClasses[self.ptype_str]
            assert isSimObjectClass(ptype)
            self.ptype = ptype
            return ptype

        raise AttributeError(
            f"'{type(self).__name__}' object has no attribute '{attr}'"
        )

    @property
    def ptypes(self):
        return [self.ptype]

    def example_str(self):
        if hasattr(self.ptype, "ex_str"):
            return self.ptype.ex_str
        else:
            return self.ptype_str

    # Is the param available to be exposed on the command line
    def isCmdLineSettable(self):
        if hasattr(self.ptype, "cmd_line_settable"):
            return self.ptype.cmd_line_settable
        else:
            return False

    def convert(self, value):
        if isinstance(value, proxy.BaseProxy):
            value.set_param_desc(self)
            return value
        if "ptype" not in self.__dict__ and isNullPointer(value):
            # deferred evaluation of SimObject; continue to defer if
            # we're just assigning a null pointer
            return value
        if isinstance(value, self.ptype):
            return value
        if isNullPointer(value) and isSimObjectClass(self.ptype):
            return value
        return self.ptype(value)

    def pretty_print(self, value):
        if isinstance(value, proxy.BaseProxy):
            return str(value)
        if isNullPointer(value):
            return NULL
        return self.ptype(value).pretty_print(value)

    def cxx_predecls(self, code):
        code("#include <cstddef>", add_once=True)
        self.ptype.cxx_predecls(code)

    def pybind_predecls(self, code):
        self.ptype.pybind_predecls(code)

    def cxx_decl(self, code):
        code("${{self.ptype.cxx_type}} ${{self.name}};")


# Vector-valued parameter description.  Just like ParamDesc, except
# that the value is a vector (list) of the specified type instead of a
# single value.


class VectorParamValue(list, metaclass=MetaParamValue):
    def __setattr__(self, attr, value):
        raise AttributeError(
            f"Not allowed to set {attr} on '{type(self).__name__}'"
        )

    def config_value(self):
        return [v.config_value() for v in self]

    def ini_str(self):
        return " ".join([v.ini_str() for v in self])

    def getValue(self):
        return [v.getValue() for v in self]

    def unproxy(self, base):
        if len(self) == 1 and isinstance(self[0], proxy.BaseProxy):
            # The value is a proxy (e.g. Parent.any, Parent.all or
            # Parent.x) therefore try resolve it
            return self[0].unproxy(base)
        else:
            return [v.unproxy(base) for v in self]


class DictParamValue(dict, metaclass=MetaParamValue):
    def __setattr__(self, attr, value):
        raise AttributeError(
            f"Not allowed to set {attr} on '{type(self).__name__}'"
        )

    def config_value(self):
        return {
            key.config_value(): value.config_value()
            for key, value in self.items()
        }

    def ini_str(self):
        # This is flattening the dictionary as a sequence of key, value pairs
        return " ".join([elem.ini_str() for kv in self.items() for elem in kv])

    def getValue(self):
        return {
            key.getValue(): value.getValue() for key, value in self.items()
        }

    def unproxy(self, base):
        return {
            key.unproxy(base): value.unproxy(base)
            for key, value in self.items()
        }


class SimObjectVector(VectorParamValue):
    # support clone operation
    def __call__(self, **kwargs):
        return SimObjectVector([v(**kwargs) for v in self])

    def clear_parent(self, old_parent):
        for v in self:
            v.clear_parent(old_parent)

    def set_parent(self, parent, name):
        if len(self) == 1:
            self[0].set_parent(parent, name)
        else:
            width = int(math.ceil(math.log(len(self)) / math.log(10)))
            for i, v in enumerate(self):
                v.set_parent(parent, "%s%0*d" % (name, width, i))

    def has_parent(self):
        return any([e.has_parent() for e in self if not isNullPointer(e)])

    # return 'cpu0 cpu1' etc. for print_ini()
    def get_name(self):
        return " ".join([v._name for v in self])

    # By iterating through the constituent members of the vector here
    # we can nicely handle iterating over all a SimObject's children
    # without having to provide lots of special functions on
    # SimObjectVector directly.
    def descendants(self):
        for v in self:
            yield from v.descendants()

    def get_config_as_dict(self):
        a = []
        for v in self:
            a.append(v.get_config_as_dict())
        return a

    # If we are replacing an item in the vector, make sure to set the
    # parent reference of the new SimObject to be the same as the parent
    # of the SimObject being replaced. Useful to have if we created
    # a SimObjectVector of temporary objects that will be modified later in
    # configuration scripts.
    def __setitem__(self, key, value):
        val = self[key]
        if value.has_parent():
            warn(
                f"SimObject {value.get_name()} already has a parent"
                + " that is being overwritten by a SimObjectVector"
            )
        value.set_parent(val.get_parent(), val._name)
        super().__setitem__(key, value)

    # Enumerate the params of each member of the SimObject vector. Creates
    # strings that will allow indexing into the vector by the python code and
    # allow it to be specified on the command line.
    def enumerateParams(self, flags_dict={}, cmd_line_str="", access_str=""):
        if hasattr(self, "_paramEnumed"):
            print(f"Cycle detected enumerating params at {cmd_line_str}?!")
        else:
            x = 0
            for vals in self:
                # Each entry in the SimObjectVector should be an
                # instance of a SimObject
                flags_dict = vals.enumerateParams(
                    flags_dict,
                    cmd_line_str + "%d." % x,
                    access_str + "[%d]." % x,
                )
                x = x + 1

        return flags_dict


class VectorParamDesc(SingleTypeParamDesc):
    # Convert assigned value to appropriate type.  If the RHS is not a
    # list or tuple, it generates a single-element list.
    def convert(self, value):
        if isinstance(value, (list, tuple)):
            # list: coerce each element into new list
            tmp_list = [SingleTypeParamDesc.convert(self, v) for v in value]
        elif isinstance(value, str):
            # If input is a csv string
            tmp_list = [
                SingleTypeParamDesc.convert(self, v)
                for v in value.strip("[").strip("]").split(",")
            ]
        else:
            # singleton: coerce to a single-element list
            tmp_list = [SingleTypeParamDesc.convert(self, value)]

        if isSimObjectSequence(tmp_list):
            return SimObjectVector(tmp_list)
        else:
            return VectorParamValue(tmp_list)

    # Produce a human readable example string that describes
    # how to set this vector parameter in the absence of a default
    # value.
    def example_str(self):
        s = super().example_str()
        help_str = "[" + s + "," + s + ", ...]"
        return help_str

    # Produce a human readable representation of the value of this vector param.
    def pretty_print(self, value):
        if isinstance(value, (list, tuple)):
            tmp_list = [
                SingleTypeParamDesc.pretty_print(self, v) for v in value
            ]
        elif isinstance(value, str):
            tmp_list = [
                SingleTypeParamDesc.pretty_print(self, v)
                for v in value.split(",")
            ]
        else:
            tmp_list = [SingleTypeParamDesc.pretty_print(self, value)]

        return tmp_list

    # This is a helper function for the new config system
    def __call__(self, value):
        if isinstance(value, (list, tuple)):
            # list: coerce each element into new list
            tmp_list = [SingleTypeParamDesc.convert(self, v) for v in value]
        elif isinstance(value, str):
            # If input is a csv string
            tmp_list = [
                SingleTypeParamDesc.convert(self, v)
                for v in value.strip("[").strip("]").split(",")
            ]
        else:
            # singleton: coerce to a single-element list
            tmp_list = [SingleTypeParamDesc.convert(self, value)]

        return VectorParamValue(tmp_list)

    def cxx_predecls(self, code):
        code("#include <vector>", add_once=True)
        self.ptype.cxx_predecls(code)

    def pybind_predecls(self, code):
        code("#include <vector>", add_once=True)
        self.ptype.pybind_predecls(code)

    def cxx_decl(self, code):
        code("std::vector< ${{self.ptype.cxx_type}} > ${{self.name}};")


class OptionalParamDesc(SingleTypeParamDesc):
    def __init__(self, ptype_str, ptype, *args, **kwargs):
        if len(args) != 1:
            raise TypeError(
                "OptionalParamDesc takes exactly one argument: description."
            )
        if ptype_str not in allParams:
            raise TypeError(
                "OptionalParam only supports primitive parameter types."
            )
        kwargs["default"] = NullOpt
        super().__init__(ptype_str, ptype, *args, **kwargs)

    def convert(self, value):
        if isNullOpt(value):
            return value
        else:
            return SingleTypeParamDesc.convert(self, value)

    def cxx_predecls(self, code):
        code("#include <optional>", add_once=True)
        self.ptype.cxx_predecls(code)

    def pybind_predecls(self, code):
        code("#include <optional>", add_once=True)
        self.ptype.pybind_predecls(code)

    def cxx_decl(self, code):
        code("std::optional< ${{self.ptype.cxx_type}} > ${{self.name}};")


class DictParamDesc(ParamDesc):
    def __init__(
        self,
        key_ptype_str,
        key_ptype,
        val_ptype_str,
        val_ptype,
        *args,
        **kwargs,
    ):
        if isSimObjectClass(key_ptype) or isSimObjectClass(val_ptype):
            raise TypeError(
                f"Can't use a SimObject in '{type(self).__name__}'"
            )

        super().__init__(*args, **kwargs)
        self.key_desc = SingleTypeParamDesc(
            key_ptype_str, key_ptype, desc=self.desc
        )
        self.val_desc = SingleTypeParamDesc(
            val_ptype_str, val_ptype, desc=self.desc
        )

    @property
    def ptypes(self):
        return [self.key_desc.ptype, self.val_desc.ptype]

    # Convert assigned value to appropriate type.  If the RHS is not a
    # list or tuple, it generates a single-element list.
    def convert(self, value):
        if not isinstance(value, dict):
            raise TypeError("Should be a dict")

        tmp_dict = {
            self.key_desc.convert(key): self.val_desc.convert(val)
            for key, val in value.items()
        }

        return DictParamValue(tmp_dict)

    # Produce a human readable example string that describes
    # how to set this vector parameter in the absence of a default
    # value.
    def example_str(self):
        k = self.key_desc.example_str()
        v = self.val_desc.example_str()
        help_str = "{" + k + ":" + v + ", ...}"
        return help_str

    # Is the param available to be exposed on the command line
    def isCmdLineSettable(self):
        return getattr(
            self.key_desc.ptype, "cmd_line_settable", False
        ) and getattr(self.val_desc.ptype, "cmd_line_settable", False)

    # Produce a human readable representation of the value of this vector param.
    def pretty_print(self, value):
        if not isinstance(value, dict):
            raise TypeError("Should be a dict")

        tmp_dict = {
            self.key_desc.pretty_print(key): self.val_desc.pretty_print(val)
            for key, val in value.items()
        }

        return pprint.pformat(tmp_dict)

    # This is a helper function for the new config system
    def __call__(self, value):
        return self.convert(value)

    def cxx_predecls(self, code):
        code("#include <unordered_map>", add_once=True)
        self.key_desc.ptype.cxx_predecls(code)
        self.val_desc.ptype.cxx_predecls(code)

    def pybind_predecls(self, code):
        code("#include <unordered_map>", add_once=True)
        self.key_desc.ptype.pybind_predecls(code)
        self.val_desc.ptype.pybind_predecls(code)

    def cxx_decl(self, code):
        ktype = self.key_desc.ptype.cxx_type
        vtype = self.val_desc.ptype.cxx_type
        code("std::unordered_map<${{ktype}}, ${{vtype}}> ${{self.name}};")


class ParamFactory:
    def __init__(self, param_desc_class, ptype_str=None):
        self.param_desc_class = param_desc_class
        self.ptype_str = ptype_str

    def __getattr__(self, attr):
        if self.ptype_str:
            attr = self.ptype_str + "." + attr
        return ParamFactory(self.param_desc_class, attr)

    # E.g., Param.Int(5, "number of widgets")
    def __call__(self, *args, **kwargs):
        ptype = None
        try:
            ptype = allParams[self.ptype_str]
        except KeyError:
            # if name isn't defined yet, assume it's a SimObject, and
            # try to resolve it later
            pass
        return self.param_desc_class(self.ptype_str, ptype, *args, **kwargs)


class DictParamFactory:
    """
    Factory class whose purpose is to store the (descriptor
    class+key_type+value_type), and to generate the descriptor object
    once arguments are passed (via the __call__). Last item in the
    chain of factory classes

    DictParamKeyFactory -> DictParamValueFactory -> DictParamFactory
    """

    def __init__(self, param_desc_class, key_ptype_str, val_ptype_str):
        self.param_desc_class = param_desc_class
        self.key_ptype_str = key_ptype_str
        self.val_ptype_str = val_ptype_str

    # E.g., DictParam.Int.String({5: "example string"}, "map of widgets")
    def __call__(self, *args, **kwargs):
        key_ptype = None
        val_ptype = None
        try:
            key_ptype = allParams[self.key_ptype_str]
            val_ptype = allParams[self.val_ptype_str]
        except KeyError:
            # if name isn't defined yet, assume it's a SimObject, and
            # try to resolve it later
            pass
        return self.param_desc_class(
            self.key_ptype_str,
            key_ptype,
            self.val_ptype_str,
            val_ptype,
            *args,
            **kwargs,
        )


class DictParamValueFactory:
    """
    Factory class whose purpose is to store the (descriptor
    class+key_type), and to generate a new factory object (DictParamFactory)
    once a value type is given (via the __getattr__)
    """

    def __init__(self, param_desc_class, key_ptype_str):
        self.param_desc_class = param_desc_class
        self.key_ptype_str = key_ptype_str

    def __getattr__(self, val_ptype_str):
        return DictParamFactory(
            self.param_desc_class, self.key_ptype_str, val_ptype_str
        )


class DictParamKeyFactory:
    """
    Factory class whose purpose is to store the (descriptor class) and to
    generate a new factory object DictParamValueFactory once a key type is
    given (via the __getattr__)
    """

    def __init__(self, param_desc_class):
        self.param_desc_class = param_desc_class

    def __getattr__(self, key_ptype_str):
        return DictParamValueFactory(self.param_desc_class, key_ptype_str)


Param = ParamFactory(SingleTypeParamDesc)
VectorParam = ParamFactory(VectorParamDesc)
OptionalParam = ParamFactory(OptionalParamDesc)
DictParam = DictParamKeyFactory(DictParamDesc)
