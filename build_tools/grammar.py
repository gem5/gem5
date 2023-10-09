# Copyright (c) 2006-2011 Nathan Binkert <nate@binkert.org>
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
import os

import ply.lex
import ply.yacc


class ParseError(Exception):
    def __init__(self, message, token=None):
        Exception.__init__(self, message)
        self.token = token


class Grammar(object):
    def setupLexerFactory(self, **kwargs):
        if "module" in kwargs:
            raise AttributeError("module is an illegal attribute")
        self.lex_kwargs = kwargs

    def setupParserFactory(self, **kwargs):
        if "module" in kwargs:
            raise AttributeError("module is an illegal attribute")

        if "output" in kwargs:
            dir, tab = os.path.split(output)
            if not tab.endswith(".py"):
                raise AttributeError("The output file must end with .py")
            kwargs["outputdir"] = dir
            kwargs["tabmodule"] = tab[:-3]

        self.yacc_kwargs = kwargs

    def __getattr__(self, attr):
        if attr == "lexers":
            self.lexers = []
            return self.lexers

        if attr == "lex_kwargs":
            self.setupLexerFactory()
            return self.lex_kwargs

        if attr == "yacc_kwargs":
            self.setupParserFactory()
            return self.yacc_kwargs

        if attr == "lex":
            self.lex = ply.lex.lex(module=self, **self.lex_kwargs)
            return self.lex

        if attr == "yacc":
            self.yacc = ply.yacc.yacc(module=self, **self.yacc_kwargs)
            return self.yacc

        if attr == "current_lexer":
            if not self.lexers:
                return None
            return self.lexers[-1][0]

        if attr == "current_source":
            if not self.lexers:
                return "<none>"
            return self.lexers[-1][1]

        if attr == "current_line":
            if not self.lexers:
                return -1
            return self.current_lexer.lineno

        raise AttributeError(
            "'%s' object has no attribute '%s'" % (type(self), attr)
        )

    def parse_string(self, data, source="<string>", debug=None, tracking=0):
        if not isinstance(data, str):
            raise AttributeError(
                "argument must be a string, was '%s'" % type(f)
            )

        lexer = self.lex.clone()
        lexer.input(data)
        self.lexers.append((lexer, source))

        lrtab = ply.yacc.LRTable()
        lrtab.lr_productions = self.yacc.productions
        lrtab.lr_action = self.yacc.action
        lrtab.lr_goto = self.yacc.goto

        parser = ply.yacc.LRParser(lrtab, self.yacc.errorfunc)
        result = parser.parse(lexer=lexer, debug=debug, tracking=tracking)
        self.lexers.pop()
        return result

    def parse_file(self, f, **kwargs):
        if isinstance(f, str):
            source = f
            f = open(f, "r")
        elif isinstance(f, file):
            source = f.name
        else:
            raise AttributeError(
                "argument must be either a string or file, was '%s'" % type(f)
            )

        return self.parse_string(f.read(), source, **kwargs)

    def p_error(self, t):
        if t:
            msg = "Syntax error at %s:%d:%d\n>>%s<<" % (
                self.current_source,
                t.lineno,
                t.lexpos + 1,
                t.value,
            )
        else:
            msg = "Syntax error at end of %s" % (self.current_source,)
        raise ParseError(msg, t)

    def t_error(self, t):
        msg = "Illegal character %s @ %d:%d" % (
            repr(t.value[0]),
            t.lineno,
            t.lexpos,
        )
        raise ParseError(msg, t)
