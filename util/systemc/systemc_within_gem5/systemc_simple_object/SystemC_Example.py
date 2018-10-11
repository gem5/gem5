# Copyright 2018 Google, Inc.
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
#
# Authors: Gabe Black

from m5.params import *
from m5.SimObject import SimObject

from SystemC import SystemC_ScModule

# This class is a subclass of sc_module, and all the special magic which makes
# that work is handled in the base classes.
class SystemC_Printer(SystemC_ScModule):
    type = 'SystemC_Printer'
    cxx_class = 'Printer'
    cxx_header = 'systemc_simple_object/printer.hh'
    # This parameter will be available in the SystemC_PrinterParams::create
    # function and can be passed to the c++ object's constructor, used to set
    # one of its member variables, as a parameter to one of its methods, etc.
    prefix = Param.String('', 'Prefix for each word')

# This is a standard gem5 SimObject class with no special accomodation for the
# fact that one of its parameters is a systemc object.
class Gem5_Feeder(SimObject):
    type = 'Gem5_Feeder'
    cxx_class = 'Feeder'
    cxx_header = 'systemc_simple_object/feeder.hh'
    # This parameter will be a pointer to an instance of the class above.
    printer = Param.SystemC_Printer('Printer for our words.')
    delay = Param.Latency('1ns', 'Time to wait between each word.')
    strings = VectorParam.String([], 'Words to print.')
