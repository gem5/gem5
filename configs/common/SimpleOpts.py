# -*- coding: utf-8 -*-
# Copyright (c) 2015 Jason Power
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
# Authors: Jason Power

""" Options wrapper for simple gem5 configuration scripts

This module wraps the optparse class so that we can register options
from each class instead of only from the configuration script.

"""

# Module-level variable to track if we've called the parse_args function yet
called_parse_args = False

# For fatal
import m5

# import the options parser
from optparse import OptionParser

# add the options we want to be able to control from the command line
parser = OptionParser()

def add_option(*args, **kwargs):
    """Call "add_option" to the global options parser
    """

    if (parser.has_option(args[0]) or
            (len(args) > 1 and parser.has_option(args[1])) ):
        m5.fatal("Duplicate option: %s" % str(args))

    if called_parse_args:
        m5.fatal("Can't add an option after calling SimpleOpts.parse_args")

    parser.add_option(*args, **kwargs)

def parse_args():
    global called_parse_args
    called_parse_args = True

    return parser.parse_args()

def set_usage(*args, **kwargs):
    parser.set_usage(*args, **kwargs)

def print_help(*args, **kwargs):
    parser.print_help(*args, **kwargs)

