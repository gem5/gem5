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


import argparse
import sys

import m5
from m5.objects import (
    Gem5_Feeder,
    Root,
    SystemC_Kernel,
    SystemC_Printer,
)

# pylint:disable=unused-variable

parser = argparse.ArgumentParser()
parser.add_argument("--word", action="append", default=[])
parser.add_argument("--delay", default="1ns")
parser.add_argument("--prefix", default="")

args = parser.parse_args()

printer = SystemC_Printer()
printer.prefix = args.prefix

feeder = Gem5_Feeder()
feeder.printer = printer
feeder.delay = args.delay
feeder.strings = args.word

kernel = SystemC_Kernel(feeder=feeder)
root = Root(full_system=True, systemc_kernel=kernel)

m5.instantiate(None)

cause = m5.simulate(m5.MaxTick).getCause()
print(cause)
