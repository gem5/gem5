# Copyright (c) 2022 Arm Limited
# All rights reserved.
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

try:
    import builtins
except ImportError:
    # Python 2 fallback
    import __builtin__ as builtins
import inspect
import os
import re


class lookup:
    def __init__(self, formatter, frame, *args, **kwargs):
        self.frame = frame
        self.formatter = formatter
        self.dict = self.formatter._dict
        self.args = args
        self.kwargs = kwargs
        self.locals = {}

    def __setitem__(self, item, val):
        self.locals[item] = val

    def __getitem__(self, item):
        if item in self.locals:
            return self.locals[item]

        if item in self.kwargs:
            return self.kwargs[item]

        if item == "__file__":
            return self.frame.f_code.co_filename

        if item == "__line__":
            return self.frame.f_lineno

        if self.formatter.locals and item in self.frame.f_locals:
            return self.frame.f_locals[item]

        if item in self.dict:
            return self.dict[item]

        if self.formatter.globals and item in self.frame.f_globals:
            return self.frame.f_globals[item]

        if item in builtins.__dict__:
            return builtins.__dict__[item]

        try:
            item = int(item)
            return self.args[item]
        except ValueError:
            pass
        raise IndexError("Could not find '%s'" % item)


class code_formatter_meta(type):
    pattern = r"""
    (?:
      %(delim)s(?P<escaped>%(delim)s)              | # escaped delimiter
      ^(?P<indent>[ ]*)%(delim)s(?P<lone>%(ident)s)$ | # lone identifier
      %(delim)s(?P<ident>%(ident)s)                | # identifier
      %(delim)s%(lb)s(?P<b_ident>%(ident)s)%(rb)s  | # braced identifier
      %(delim)s(?P<pos>%(pos)s)                    | # positional parameter
      %(delim)s%(lb)s(?P<b_pos>%(pos)s)%(rb)s      | # braced positional
      %(delim)s%(ldb)s(?P<eval>.*?)%(rdb)s         | # double braced expression
      %(delim)s(?P<invalid>)                       # ill-formed delimiter exprs
    )
    """

    def __init__(cls, name, bases, dct):
        super().__init__(name, bases, dct)
        if "pattern" in dct:
            pat = cls.pattern
        else:
            # tuple expansion to ensure strings are proper length
            lb, rb = cls.braced
            lb1, lb2, rb2, rb1 = cls.double_braced
            pat = code_formatter_meta.pattern % {
                "delim": re.escape(cls.delim),
                "ident": cls.ident,
                "pos": cls.pos,
                "lb": re.escape(lb),
                "rb": re.escape(rb),
                "ldb": re.escape(lb1 + lb2),
                "rdb": re.escape(rb2 + rb1),
            }
        cls.pattern = re.compile(pat, re.VERBOSE | re.DOTALL | re.MULTILINE)


class code_formatter(metaclass=code_formatter_meta):
    delim = r"$"
    ident = r"[_A-z]\w*"
    pos = r"[0-9]+"
    braced = r"{}"
    double_braced = r"{{}}"

    globals = True
    locals = True
    fix_newlines = True

    def __init__(self, *args, **kwargs):
        self._data = []
        self._dict = {}
        self._indent_level = 0
        self._indent_spaces = 4
        self.globals = kwargs.pop("globals", type(self).globals)
        self.locals = kwargs.pop("locals", type(self).locals)
        self._fix_newlines = kwargs.pop(
            "fix_newlines",
            type(self).fix_newlines,
        )

        if args:
            self.__call__(args)

    def indent(self, count=1):
        self._indent_level += self._indent_spaces * count

    def dedent(self, count=1):
        assert self._indent_level >= (self._indent_spaces * count)
        self._indent_level -= self._indent_spaces * count

    def fix(self, status):
        previous = self._fix_newlines
        self._fix_newlines = status
        return previous

    def nofix(self):
        previous = self._fix_newlines
        self._fix_newlines = False
        return previous

    def clear():
        self._data = []

    def write(self, *args):
        f = open(os.path.join(*args), "w")
        name, extension = os.path.splitext(f.name)

        # Add a comment to inform which file generated the generated file
        # to make it easier to backtrack and modify generated code
        frame = inspect.currentframe().f_back
        if re.match(r"^\.(cc|hh|c|h)$", extension) is not None:
            f.write(
                f"""/**
 * DO NOT EDIT THIS FILE!
 * File automatically generated by
 *   {frame.f_code.co_filename}:{frame.f_lineno}
 */

""",
            )
        elif re.match(r"^\.py$", extension) is not None:
            f.write(
                f"""#
# DO NOT EDIT THIS FILE!
# File automatically generated by
#   {frame.f_code.co_filename}:{frame.f_lineno}
#

""",
            )
        elif re.match(r"^\.html$", extension) is not None:
            f.write(
                f"""<!--
 DO NOT EDIT THIS FILE!
 File automatically generated by
   {frame.f_code.co_filename}:{frame.f_lineno}
-->

""",
            )

        for data in self._data:
            f.write(data)
        f.close()

    def __str__(self):
        data = "".join(self._data)
        self._data = [data]
        return data

    def __getitem__(self, item):
        return self._dict[item]

    def __setitem__(self, item, value):
        self._dict[item] = value

    def __delitem__(self, item):
        del self._dict[item]

    def __contains__(self, item):
        return item in self._dict

    def __iadd__(self, data):
        self.append(data)

    def append(self, data):
        if isinstance(data, code_formatter):
            self._data.extend(data._data)
        else:
            self._append(str(data))

    def _append(self, data):
        if not self._fix_newlines:
            self._data.append(data)
            return

        initial_newline = not self._data or self._data[-1] == "\n"
        for line in data.splitlines():
            if line:
                if self._indent_level:
                    self._data.append(" " * self._indent_level)
                self._data.append(line)

            if line or not initial_newline:
                self._data.append("\n")

            initial_newline = False

    def __call__(self, *args, **kwargs):
        if not args:
            self._data.append("\n")
            return

        format = args[0]
        args = args[1:]

        frame = inspect.currentframe().f_back

        l = lookup(self, frame, *args, **kwargs)

        def convert(match):
            ident = match.group("lone")
            # check for a lone identifier
            if ident:
                indent = match.group("indent")  # must be spaces
                lone = f"{l[ident]}"

                def indent_lines(gen):
                    for line in gen:
                        yield indent
                        yield line

                return "".join(indent_lines(lone.splitlines(True)))

            # check for an identifier, braced or not
            ident = match.group("ident") or match.group("b_ident")
            if ident is not None:
                return f"{l[ident]}"

            # check for a positional parameter, braced or not
            pos = match.group("pos") or match.group("b_pos")
            if pos is not None:
                pos = int(pos)
                if pos > len(args):
                    raise ValueError(
                        "Positional parameter #%d not found in pattern" % pos,
                        code_formatter.pattern,
                    )
                return f"{args[int(pos)]}"

            # check for a double braced expression
            eval_expr = match.group("eval")
            if eval_expr is not None:
                result = eval(eval_expr, {}, l)
                return f"{result}"

            # check for an escaped delimiter
            if match.group("escaped") is not None:
                return "$"

            # At this point, we have to match invalid
            if match.group("invalid") is None:
                # didn't match invalid!
                raise ValueError(
                    "Unrecognized named group in pattern",
                    code_formatter.pattern,
                )

            i = match.start("invalid")
            if i == 0:
                colno = 1
                lineno = 1
            else:
                lines = format[:i].splitlines(True)
                colno = i - sum(len(z) for z in lines)
                lineno = len(lines)

                raise ValueError(
                    "Invalid format string: line %d, col %d" % (lineno, colno),
                )

        d = code_formatter.pattern.sub(convert, format)
        self._append(d)


__all__ = ["code_formatter"]

if __name__ == "__main__":
    from .code_formatter import code_formatter

    f = code_formatter()

    class Foo(dict):
        def __init__(self, **kwargs):
            self.update(kwargs)

        def __getattr__(self, attr):
            return self[attr]

    x = "this is a test"
    l = [[Foo(x=[Foo(y=9)])]]

    y = code_formatter()
    y(
        """
{
    this_is_a_test();
}
""",
    )
    f("    $y")
    f(
        """$__file__:$__line__
{""",
    )
    f("${{', '.join(str(x) for x in range(4))}}")
    f("${x}")
    f("$x")
    f.indent()
    for i in range(5):
        f("$x")
        f("$i")
        f("$0", "zero")
        f("$1 $0", "zero", "one")
        f("${0}", "he went")
        f("${0}asdf", "he went")
    f.dedent()

    f(
        """
    ${{l[0][0]["x"][0].y}}
}
""",
        1,
        9,
    )

    print(f, end=" ")
