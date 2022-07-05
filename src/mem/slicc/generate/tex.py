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

from code_formatter import code_formatter


class tex_formatter(code_formatter):
    braced = "<>"
    double_braced = "<<>>"


def printTexTable(sm, code):
    tex = tex_formatter()
    tex(
        r"""
%& latex
\documentclass[12pt]{article}
\usepackage{graphics}
\begin{document}
\begin{tabular}{|l||$<<"l" * len(sm.events)>>|} \hline
"""
    )

    for event in sm.events:
        code(r" & \rotatebox{90}{$<<event.short>>}")
    tex(r"\\ \hline \hline")

    for state in sm.states:
        state_str = state.short
        for event in sm.events:
            state_str += " & "
            trans = sm.get_transition(state, event)
            if trans:
                actions = trans.getActionShorthands()
                # FIXME: should compare index, not the string
                if trans.getNextStateShorthand() != state.short:
                    nextState = trans.getNextStateShorthand()
                else:
                    nextState = ""
                state_str += actions
                if nextState and actions:
                    state_str += "/"
                state_str += nextState
        tex(r"$0 \\", state_str)
    tex(
        r"""
\hline
\end{tabular}
\end{document}
"""
    )

    code.append(tex)
