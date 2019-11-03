# Copyright (c) 2007 The Regents of The University of Michigan
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
#
# Authors: Gabe Black

from __future__ import print_function

from micro_asm import MicroAssembler, Combinational_Macroop, Rom_Macroop, Rom

class Bah(object):
    def __init__(self):
        self.mnemonic = "bah"

class Bah_Tweaked(object):
    def __init__(self):
        self.mnemonic = "bah_tweaked"

class Hoop(object):
    def __init__(self, first_param, second_param):
        self.mnemonic = "hoop_%s_%s" % (first_param, second_param)
    def __str__(self):
        return "%s" % self.mnemonic

class Dah(object):
    def __init__(self):
        self.mnemonic = "dah"

microops = {
    "bah": Bah,
    "hoop": Hoop,
    "dah": Dah
}

class TestMacroop(Combinational_Macroop):
    def tweak(self):
        microops["bah"] = Bah_Tweaked
    def untweak(self):
        microops["bah"] = Bah
    def print_debug(self, message):
        print(message)

    def __init__(self, name):
        super(TestMacroop, self).__init__(name)
        self.directives = {
            "tweak": self.tweak,
            "untweak": self.untweak,
            "print": self.print_debug
        }

assembler = MicroAssembler(TestMacroop, microops, Rom('main ROM'), Rom_Macroop)

testAssembly = '''
# Single line comment

def rom {
    goo: bah
    extern la: hoop 4*8, "a"
}; /* multiline comment on one line */

/* multi line comment across lines
   to make sure they work */

def macroop squishy {
    .tweak
    bah
    .untweak
    .print "In the midst"
    bah
    dah # single line comment after something
    .tweak
};

#Extending the rom...
def rom
{
    #Here's more stuff for the rom
    bah
};

def macroop squashy {
    bah
};

def macroop jumper (bar);
'''
assembler.assemble(testAssembly)
