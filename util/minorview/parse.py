# Copyright (c) 2013 ARM Limited
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


def list_parser(names):
    """Parse a list of elements, some of which might be one-level sublists
    within parentheses, into a a list of lists of those elements.  For
    example: list_parser('(a,b),c') -> [['a', 'b'], 'c']"""
    elems = re.split(",", names)
    ret = []
    accum = []
    for elem in elems:
        if re.search(r"^\((.*)\)$", elem):
            accum.append(re.sub(r"^\((.*)\)", "\\1", elem))
            ret.append(accum)
            accum = []
        elif re.search(r"^\(", elem):
            accum.append(re.sub(r"^\(", "", elem))
        elif re.search(r"\)$", elem):
            accum.append(re.sub(r"\)$", "", elem))
            ret.append(accum)
            accum = []
        elif len(accum) != 0:
            accum.append(elem)
        else:
            ret.append([elem])

    if len(accum) > 0:
        print("Non matching brackets in", names)

    return ret


def map2(f, ls):
    """map to a depth of 2.  That is, given a list of lists, apply
    f to those innermost elements"""
    return [list(map(f, l)) for l in ls]


def remove_trailing_ws(line):
    return re.sub(r"\s*$", "", line)


def remove_leading_and_trailing_ws(line):
    return re.sub(r"\s*$", "", re.sub(r"^\s*", "", line))


def parse_pairs_list(pairString):
    """parse a string like 'name=value name2=value2' into a
    list of pairs of ('name', 'value') ..."""
    ret = []
    pairs = re.finditer(r'(\w+)(=("[^"]*"|[^\s]*))?', pairString)
    for pair in pairs:
        name, rest, value = pair.groups()
        if value is not None:
            value = re.sub('^"(.*)"$', "\\1", value)
            ret.append((name, value))
        else:
            ret.append((name, ""))
    return ret


def parse_indexed_list(string):
    """parse a string of the form "(index,value),(index,value)..."
    into a list of index, value pairs"""

    ret = []
    pairs = list_parser(string)
    for pair in pairs:
        if len(pair) == 2:
            index, value = pair
            ret.append((int(index), value))

    return ret


def parse_pairs(pairString):
    """parse a string like 'name=value name2=value2' into a
    dictionary of {'name': 'value', 'name2': 'value2'}"""
    return dict(parse_pairs_list(pairString))
