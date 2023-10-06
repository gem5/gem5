#!/usr/bin/env python3
#
# Copyright (c) 2014, 2016 ARM Limited
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
# Copyright (c) 2006 The Regents of The University of Michigan
# Copyright (c) 2007,2011 The Hewlett-Packard Development Company
# Copyright (c) 2016 Advanced Micro Devices, Inc.
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
import inspect
import os
import re
import sys
from abc import ABCMeta
from abc import abstractmethod
from difflib import SequenceMatcher

from . import sort_includes
from . import style
from .file_types import lang_type
from .region import *


def safefix(fix_func):
    """Decorator for the fix functions of the Verifier class.
    This function wraps the fix function and creates a backup file
    just in case there is an error.
    """

    def safefix_wrapper(*args, **kwargs):
        # Check to be sure that this is decorating a function we expect:
        # a class method with filename as the first argument (after self)
        assert os.path.exists(args[1])
        self = args[0]
        assert is_verifier(self.__class__)
        filename = args[1]

        # Now, Let's make a backup file.
        from shutil import copyfile

        backup_name = filename + ".bak"
        copyfile(filename, backup_name)

        # Try to apply the fix. If it fails, then we revert the file
        # Either way, we need to clean up our backup file
        try:
            fix_func(*args, **kwargs)
        except Exception as e:
            # Restore the original file to the backup file
            self.ui.write("Error! Restoring the original file.\n")
            copyfile(backup_name, filename)
            raise
        finally:
            # Clean up the backup file
            os.remove(backup_name)

    return safefix_wrapper


def _modified_regions(old, new):
    try:
        m = SequenceMatcher(a=old, b=new, autojunk=False)
    except TypeError:
        # autojunk was introduced in Python 2.7. We need a fallback
        # mechanism to support old Python versions.
        m = SequenceMatcher(a=old, b=new)
    regions = Regions()
    for tag, i1, i2, j1, j2 in m.get_opcodes():
        if tag != "equal":
            regions.extend(Region(i1, i2))
    return regions


class Verifier(object, metaclass=ABCMeta):
    """Base class for style verifiers

    Verifiers check for style violations and optionally fix such
    violations. Implementations should either inherit from this class
    (Verifier) if they need to work on entire files or LineVerifier if
    they operate on a line-by-line basis.

    Subclasses must define these class attributes:
      languages = set of strings identifying applicable languages
      test_name = long descriptive name of test, will be used in
                  messages such as "error in <foo>" or "invalid <foo>"
      opt_name = short name used to generate command-line options to
                 control the test (--fix-<foo>, --ignore-<foo>, etc.)

    """

    def __init__(self, ui, opts, base=None):
        self.ui = ui
        self.base = base

        # opt_name must be defined as a class attribute of derived classes.
        # Check test-specific opts first as these have precedence.
        self.opt_fix = opts.get("fix_" + self.opt_name, False)
        self.opt_ignore = opts.get("ignore_" + self.opt_name, False)
        self.opt_skip = opts.get("skip_" + self.opt_name, False)
        # If no test-specific opts were set, then set based on "-all" opts.
        if not (self.opt_fix or self.opt_ignore or self.opt_skip):
            self.opt_fix = opts.get("fix_all", False)
            self.opt_ignore = opts.get("ignore_all", False)
            self.opt_skip = opts.get("skip_all", False)

    def normalize_filename(self, name):
        abs_name = os.path.abspath(name)
        if self.base is None:
            return abs_name

        abs_base = os.path.abspath(self.base)
        return os.path.relpath(abs_name, start=abs_base)

    def open(self, filename, mode):
        try:
            f = open(filename, mode)
        except OSError as msg:
            print(f"could not open file {filename}: {msg}")
            return None

        return f

    def skip(self, filename):
        # We never want to handle symlinks, so always skip them: If the
        # location pointed to is a directory, skip it. If the location is a
        # file inside the gem5 directory, it will be checked as a file, so
        # symlink can be skipped. If the location is a file outside gem5, we
        # don't want to check it anyway.
        if os.path.islink(filename):
            return True
        return lang_type(filename) not in self.languages

    def apply(self, filename, regions=all_regions):
        """Possibly apply to specified regions of file 'filename'.

        Verifier is skipped if --skip-<test> option was provided or if
        file is not of an applicable type.  Otherwise file is checked
        and error messages printed.  Errors are fixed or ignored if
        the corresponding --fix-<test> or --ignore-<test> options were
        provided.  If neither, the user is prompted for an action.

        Returns True to abort, False otherwise.
        """
        if not (self.opt_skip or self.skip(filename)):
            errors = self.check(filename, regions)
            if errors and not self.opt_ignore:
                if self.opt_fix:
                    self.fix(filename, regions)
                else:
                    result = self.ui.prompt(
                        "(a)bort, (i)gnore, or (f)ix?",
                        "aif",
                        "a",
                    )
                    if result == "f":
                        self.fix(filename, regions)
                    elif result == "a":
                        return True  # abort

        return False

    @abstractmethod
    def check(self, filename, regions=all_regions, fobj=None, silent=False):
        """Check specified regions of file 'filename'.

        Given that it is possible that the current contents of the file
        differ from the file as 'staged to commit', for those cases, and
        maybe others, the argument fobj should be a file object open and reset
        with the contents matching what the file would look like after the
        commit. This is needed keep the messages using 'filename' meaningful.

        The argument silent is useful to prevent output when we run check in
        the staged file vs the actual file to detect if the user forgot
        staging fixes to the commit. This way, we prevent reporting errors
        twice in stderr.

        Line-by-line checks can simply provide a check_line() method
        that returns True if the line is OK and False if it has an
        error.  Verifiers that need a multi-line view (like
        SortedIncludes) must override this entire function.

        Returns a count of errors (0 if none), though actual non-zero
        count value is not currently used anywhere.
        """
        pass

    @abstractmethod
    def fix(self, filename, regions=all_regions):
        """Fix specified regions of file 'filename'.

        Line-by-line fixes can simply provide a fix_line() method that
        returns the fixed line. Verifiers that need a multi-line view
        (like SortedIncludes) must override this entire function.
        """
        pass


class LineVerifier(Verifier):
    def check(self, filename, regions=all_regions, fobj=None, silent=False):
        close = False
        if fobj is None:
            fobj = self.open(filename, "rb")
            close = True

        lang = lang_type(filename)
        assert lang in self.languages

        errors = 0
        for num, line in enumerate(fobj):
            if num not in regions:
                continue
            s_line = line.decode("utf-8").rstrip("\n")
            if not self.check_line(s_line, language=lang):
                if not silent:
                    self.ui.write(
                        "invalid %s in %s:%d\n"
                        % (self.test_name, filename, num + 1),
                    )
                    if self.ui.verbose:
                        self.ui.write(f">>{s_line[:-1]}<<\n")
                errors += 1
        if close:
            fobj.close()
        return errors

    @safefix
    def fix(self, filename, regions=all_regions):
        f = self.open(filename, "r+")

        lang = lang_type(filename)
        assert lang in self.languages

        lines = list(f)

        f.seek(0)
        f.truncate()

        for i, line in enumerate(lines):
            line = line.rstrip("\n")
            if i in regions:
                line = self.fix_line(line, language=lang)

            f.write(line)
            f.write("\n")
        f.close()
        self.current_language = None

    @abstractmethod
    def check_line(self, line, **kwargs):
        pass

    @abstractmethod
    def fix_line(self, line, **kwargs):
        pass


class Whitespace(LineVerifier):
    """Check whitespace.

    Specifically:
    - No tabs used for indent
    - No trailing whitespace
    """

    languages = set(
        ("C", "C++", "swig", "python", "asm", "isa", "scons", "make", "dts"),
    )
    trail_only = set(("make", "dts"))

    test_name = "whitespace"
    opt_name = "white"

    _lead = re.compile(r"^([ \t]+)")
    _trail = re.compile(r"([ \t]+)$")

    def skip_lead(self, language):
        return language in Whitespace.trail_only

    def check_line(self, line, language):
        if not self.skip_lead(language):
            match = Whitespace._lead.search(line)
            if match and match.group(1).find("\t") != -1:
                return False

        match = Whitespace._trail.search(line)
        if match:
            return False

        return True

    def fix_line(self, line, language):
        if not self.skip_lead(language) and Whitespace._lead.search(line):
            newline = ""
            for i, c in enumerate(line):
                if c == " ":
                    newline += " "
                elif c == "\t":
                    newline += " " * (
                        style.tabsize - len(newline) % style.tabsize
                    )
                else:
                    newline += line[i:]
                    break

            line = newline

        return line.rstrip()


class SortedIncludes(Verifier):
    """Check for proper sorting of include statements"""

    languages = sort_includes.default_languages
    test_name = "include file order"
    opt_name = "include"

    def __init__(self, *args, **kwargs):
        super(SortedIncludes, self).__init__(*args, **kwargs)
        self.sort_includes = sort_includes.SortIncludes()

    def check(self, filename, regions=all_regions, fobj=None, silent=False):
        close = False
        if fobj is None:
            fobj = self.open(filename, "rb")
            close = True
        norm_fname = self.normalize_filename(filename)

        old = [l.decode("utf-8").rstrip("\n") for l in fobj]
        if close:
            fobj.close()

        if len(old) == 0:
            return 0

        language = lang_type(filename, old[0])
        new = list(self.sort_includes(old, norm_fname, language))

        modified = _modified_regions(old, new) & regions

        if modified:
            if not silent:
                self.ui.write(
                    "invalid sorting of includes in %s. Note: If "
                    "there is more than one empty line under the "
                    "#include region, please reduce it to one.\n" % (filename),
                )
                if self.ui.verbose:
                    for start, end in modified.regions:
                        self.ui.write("bad region [%d, %d)\n" % (start, end))
            return 1

        return 0

    @safefix
    def fix(self, filename, regions=all_regions):
        f = self.open(filename, "r+")
        norm_fname = self.normalize_filename(filename)

        old = f.readlines()
        lines = [l.rstrip("\n") for l in old]
        language = lang_type(filename, lines[0])
        sort_lines = list(self.sort_includes(lines, norm_fname, language))
        new = "".join(line + "\n" for line in sort_lines)

        f.seek(0)
        f.truncate()

        for i, line in enumerate(sort_lines):
            f.write(line)
            f.write("\n")
        f.close()


class ControlSpace(LineVerifier):
    """Check for exactly one space after if/while/for"""

    languages = set(("C", "C++"))
    test_name = "spacing after if/while/for"
    opt_name = "control"

    _any_control = re.compile(r"\b(if|while|for)([ \t]*)\(")

    def check_line(self, line, **kwargs):
        match = ControlSpace._any_control.search(line)
        return not (match and match.group(2) != " ")

    def fix_line(self, line, **kwargs):
        new_line = ControlSpace._any_control.sub(r"\1 (", line)
        return new_line


class LineLength(LineVerifier):
    languages = set(("C", "C++", "swig", "python", "asm", "isa", "scons"))
    test_name = "line length"
    opt_name = "length"

    def check_line(self, line, language, **kwargs):
        # Ignore line length check for include pragmas of C/C++.
        if language in {"C", "C++"}:
            if line.startswith("#include"):
                return True
        return style.normalized_len(line) <= 79

    def fix(self, filename, regions=all_regions, **kwargs):
        self.ui.write("Warning: cannot automatically fix overly long lines.\n")

    def fix_line(self, line):
        pass


class ControlCharacters(LineVerifier):
    languages = set(("C", "C++", "swig", "python", "asm", "isa", "scons"))
    test_name = "control character"
    opt_name = "ascii"

    invalid = "".join(
        [chr(i) for i in range(0, 0x20) if chr(i) not in ("\n", "\t")],
    )

    def check_line(self, line, **kwargs):
        return self.fix_line(line) == line

    def fix_line(self, line, **kwargs):
        return "".join(c for c in line if c not in ControlCharacters.invalid)


class BoolCompare(LineVerifier):
    languages = set(("C", "C++", "python"))
    test_name = "boolean comparison"
    opt_name = "boolcomp"

    regex = re.compile(r"\s*==\s*([Tt]rue|[Ff]alse)\b")

    def check_line(self, line, **kwargs):
        return self.regex.search(line) == None

    def fix_line(self, line, **kwargs):
        match = self.regex.search(line)
        if match:
            if match.group(1) in ("true", "True"):
                line = self.regex.sub("", line)
            else:
                self.ui.write(
                    "Warning: cannot automatically fix "
                    "comparisons with false/False.\n",
                )
        return line


class StructureBraces(LineVerifier):
    """Check if the opening braces of structures are not on the same line of
    the structure name. This includes classes, structs, enums and unions.

    This verifier matches lines starting in optional indent, followed by
    an optional typedef and the structure's keyword, followed by any
    character until the first opening brace is seen. Any extra characters
    after the opening brace are saved for a recursive check, if needed.

    This fixes, for example:
        1) "struct A {"
        2) "enum{"
        3) "    class B { // This is a class"
        4) "union { struct C {"
    to:
        1) "struct A\n{"
        2) "enum\n{"
        3) "    class B\n    {\n        // This is a class"
        4) "union\n{\n        struct C\n        {"

    @todo Make this work for multi-line structure declarations. e.g.,

        class MultiLineClass
          : public BaseClass {
    """

    languages = set(("C", "C++"))
    test_name = "structure opening brace position"
    opt_name = "structurebrace"

    # Matches the indentation of the line
    regex_indentation = "(?P<indentation>\s*)"
    # Matches an optional "typedef" before the keyword
    regex_typedef = "(?P<typedef>(typedef\s+)?)"
    # Matches the structure's keyword
    regex_keyword = "(?P<keyword>class|struct|enum|union)"
    # A negative lookahead to avoid incorrect matches with variable's names
    # e.g., "classifications = {" should not be fixed here.
    regex_avoid = "(?![^\{\s])"
    # Matches anything after the keyword and before the opening brace.
    # e.g., structure name, base type, type of inheritance, etc
    regex_name = "(?P<name>[^\{]*)"
    # Matches anything after the opening brace, which should be
    # parsed recursively
    regex_extra = "(?P<extra>.*)$"
    regex = re.compile(
        r"^"
        + regex_indentation
        + regex_typedef
        + regex_keyword
        + regex_avoid
        + regex_name
        + "\{"
        + regex_extra,
    )

    def check_line(self, line, **kwargs):
        return (self.regex.search(line) == None) or (
            line.count("{") == line.count("};")
        )

    def fix_line(self, line, **kwargs):
        match = self.regex.search(line)

        if match:
            # Move the opening brace to the next line
            match_indentation = match.group("indentation")
            match_typedef = match.group("typedef")
            match_keyword = match.group("keyword")
            match_name = match.group("name").rstrip()
            match_extra = match.group("extra").lstrip()
            line = (
                match_indentation
                + match_typedef
                + match_keyword
                + match_name
                + "\n"
                + match_indentation
                + "{"
            )

            # The opening brace should be alone in its own line, so move any
            # extra contents to the next line
            if match_extra != "":
                # Check if the extra line obeys the opening brace rule
                # (in case there are nested declarations)
                line_extra = match_indentation + "    " + match_extra
                if not self.check_line(line_extra):
                    line_extra = self.fix_line(line_extra)
                line += "\n" + line_extra

        return line


def is_verifier(cls):
    """Determine if a class is a Verifier that can be instantiated"""

    return (
        inspect.isclass(cls)
        and issubclass(cls, Verifier)
        and not inspect.isabstract(cls)
    )


# list of all verifier classes
all_verifiers = [
    v for n, v in inspect.getmembers(sys.modules[__name__], is_verifier)
]
