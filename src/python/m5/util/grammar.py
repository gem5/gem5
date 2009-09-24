# Copyright (c) 2006-2009 Nathan Binkert <nate@binkert.org>
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

from ply import lex, yacc

class TokenError(lex.LexError):
    def __init__(self, msg, t):
        super(TokenError, self).__init__(msg)
        self.token = t

class ParseError(yacc.YaccError):
    def __init__(self, message, token=None):
        super(ParseError, self).__init__(message)
        self.token = token

class Tokenizer(object):
    def __init__(self, lexer, data):
        if isinstance(data, basestring):
            indata = [ data ]
        elif isinstance(data, file):
            indata = data.xreadlines()
        else:
            indata = data

        def _input():
            for i,line in enumerate(indata):
                lexer.lineno = i + 1
                lexer.input(line)
                while True:
                    tok = lexer.token()
                    if not tok:
                        break
                    yield tok
        self.input = _input()
        self.lexer = lexer

    def next(self):
        return self.input.next()

    def __iter__(self):
        return self

    def token(self):
        try:
            return self.next()
        except StopIteration:
            return None

    def __getattr__(self, attr):
        return getattr(self.lexer, attr)

class Grammar(object):
    def __init__(self, output=None, debug=False):
        self.yacc_args = {}
        self.yacc_args['debug'] = debug

        if output:
            import os

            dir,tab = os.path.split(output)
            if not tab.endswith('.py'):
                raise AttributeError, 'The output file must end with .py'
            self.yacc_args['outputdir'] = dir
            self.yacc_args['tabmodule'] = tab[:-3]

    def t_error(self, t):
        raise lex.LexError("Illegal character %s @ %d:%d" % \
              (`t.value[0]`, t.lineno, t.lexpos), `t.value[0]`)

    def p_error(self, t):
        if t:
            msg = "Syntax error at %d:%d\n>>%s<<" % \
                  (t.lineno, t.lexpos + 1, t.value)
        else:
            msg = "Syntax error at end of input"
        raise ParseError(msg, t)

    def __getattr__(self, attr):
        if attr == 'parser':
            import ply.yacc
            parser = ply.yacc.yacc(module=self, **self.yacc_args)
            self.parser = parser
            return parser

        if attr == 'lexer':
            import ply.lex
            lexer = ply.lex.lex(module=self)
            self.lexer = lexer
            return lexer

        raise AttributeError, "'%s' object has no attribute '%s'" % \
              (self.__class__.__name__, attr)

    def parse(self, stmt, **kwargs):
        self.lexer.lineno = 1
        result = self.parser.parse(lexer=Tokenizer(self.lexer, stmt), **kwargs)
        self.parser.restart()

        return result

