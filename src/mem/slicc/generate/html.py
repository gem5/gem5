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

from m5.util.code_formatter import code_formatter

def createSymbol(symbol, title):
    code = code_formatter()
    code('''
<HTML><BODY><BIG>
$title: ${{formatShorthand(symbol.short)}} - ${{symbol.desc}}
</BIG></BODY></HTML>
''')
    return code

def formatShorthand(short):
    munged_shorthand = ""
    mode_is_normal = True

    # -- Walk over the string, processing superscript directives
    gen = enumerate(short)
    for i,c in gen:
        if c == '!':
            # -- Reached logical end of shorthand name
            break
        elif c == '_':
            munged_shorthand += " "
        elif c == '^':
            # -- Process super/subscript formatting
            mode_is_normal = not mode_is_normal
            if mode_is_normal:
                # -- Back to normal mode
                munged_shorthand += "</SUP>"
            else:
                # -- Going to superscript mode
                munged_shorthand += "<SUP>"
        elif c == '\\':
            # -- Process Symbol character set
            if i + 1 < len(short):
                # -- Proceed to next char. Yes I know that changing
                # the loop var is ugly!
                i,c = gen.next()
                munged_shorthand += "<B><FONT size=+1>"
                munged_shorthand += c
                munged_shorthand += "</FONT></B>"
            else:
                # -- FIXME: Add line number info later
                panic("Encountered a `\\` without anything following it!")
        else:
            # -- Pass on un-munged
            munged_shorthand += c

    # -- Do any other munging
    if not mode_is_normal:
        # -- Back to normal mode
        munged_shorthand += "</SUP>"

    return munged_shorthand

