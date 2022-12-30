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

import gtk

# All the miscellaneous colours used in the interface
unknownColour = gtk.gdk.color_parse("magenta")
blockedColour = gtk.gdk.color_parse("grey")
bubbleColour = gtk.gdk.color_parse("bisque")
emptySlotColour = gtk.gdk.color_parse("grey90")
reservedSlotColour = gtk.gdk.color_parse("cyan")
errorColour = gtk.gdk.color_parse("blue")
backgroundColour = gtk.gdk.color_parse("white")
faultColour = gtk.gdk.color_parse("dark cyan")
readColour = gtk.gdk.color_parse("red")
writeColour = gtk.gdk.color_parse("white")

black = gtk.gdk.color_parse("black")


def name_to_colour(name):
    """Convert a colour name to a GdkColor"""
    try:
        ret = gtk.gdk.color_parse(name)
    except:
        ret = unknownColour
    return ret


number_colour_code = list(
    map(
        name_to_colour,
        [
            "black",
            "brown",
            "red",
            "orange",
            "yellow",
            "green",
            "blue",
            "violet",
            "grey",
            "white",
        ],
    )
)


def number_to_colour(num):
    """Convert the last decimal digit of an integer into a resistor
    stripe colour"""
    return number_colour_code[num % 10]
