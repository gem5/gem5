# Copyright (c) 2017-2020 ARM Limited
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
# Copyright (c) 2007 The Regents of The University of Michigan
# Copyright (c) 2010 The Hewlett-Packard Development Company
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

import m5

import _m5.stats
from m5.objects import Root
from m5.params import isNullPointer
from .gem5stats import JsonOutputVistor
from m5.util import attrdict, fatal

# Stat exports
from _m5.stats import schedStatEvent as schedEvent
from _m5.stats import periodicStatDump

outputList = []

# Dictionary of stat visitor factories populated by the _url_factory
# visitor.
factories = {}

# List of all factories. Contains tuples of (factory, schemes,
# enabled).
all_factories = []


def _url_factory(schemes, enable=True):
    """Wrap a plain Python function with URL parsing helpers

    Wrap a plain Python function f(fn, **kwargs) to expect a URL that
    has been split using urlparse.urlsplit. First positional argument
    is assumed to be a filename, this is created as the concatenation
    of the netloc (~hostname) and path in the parsed URL. Keyword
    arguments are derived from the query values in the URL.

    Arguments:
        schemes: A list of URL schemes to use for this function.

    Keyword arguments:
        enable: Enable/disable this factory. Typically used when the
                presence of a function depends on some runtime property.

    For example:
        wrapped_f(urlparse.urlsplit("text://stats.txt?desc=False")) ->
        f("stats.txt", desc=False)

    """

    from functools import wraps

    def decorator(func):
        @wraps(func)
        def wrapper(url):
            try:
                from urllib.parse import parse_qs
            except ImportError:
                # Python 2 fallback
                from urlparse import parse_qs
            from ast import literal_eval

            qs = parse_qs(url.query, keep_blank_values=True)

            # parse_qs returns a list of values for each parameter. Only
            # use the last value since kwargs don't allow multiple values
            # per parameter. Use literal_eval to transform string param
            # values into proper Python types.
            def parse_value(key, values):
                if len(values) == 0 or (len(values) == 1 and not values[0]):
                    fatal(f"{url.geturl()}: '{key}' doesn't have a value.")
                elif len(values) > 1:
                    fatal(f"{url.geturl()}: '{key}' has multiple values.")
                else:
                    try:
                        return key, literal_eval(values[0])
                    except ValueError:
                        fatal(
                            f"{url.geturl()}: {values[0]} isn't a valid Python literal"
                        )

            kwargs = dict([parse_value(k, v) for k, v in qs.items()])

            try:
                return func(f"{url.netloc}{url.path}", **kwargs)
            except TypeError:
                fatal("Illegal stat visitor parameter specified")

        all_factories.append((wrapper, schemes, enable))
        for scheme in schemes:
            assert scheme not in factories
            factories[scheme] = wrapper if enable else None
        return wrapper

    return decorator


@_url_factory([None, "", "text", "file"])
def _textFactory(fn, desc=True, spaces=True):
    """Output stats in text format.

    Text stat files contain one stat per line with an optional
    description. The description is enabled by default, but can be
    disabled by setting the desc parameter to False.

    Parameters:
      * desc (bool): Output stat descriptions (default: True)
      * spaces (bool): Output alignment spaces (default: True)

    Example:
      text://stats.txt?desc=False;spaces=False

    """

    return _m5.stats.initText(fn, desc, spaces)


@_url_factory(["h5"], enable=hasattr(_m5.stats, "initHDF5"))
def _hdf5Factory(fn, chunking=10, desc=True, formulas=True):
    """Output stats in HDF5 format.

    The HDF5 file format is a structured binary file format. It has
    the multiple benefits over traditional text stat files:

      * Efficient storage of time series (multiple stat dumps)
      * Fast lookup of stats
      * Plenty of existing tooling (e.g., Python libraries and graphical
        viewers)
      * File format can be used to store frame buffers together with
        normal stats.

    There are some drawbacks compared to the default text format:
      * Large startup cost (single stat dump larger than text equivalent)
      * Stat dumps are slower than text


    Known limitations:
      * Distributions and histograms currently unsupported.
      * No support for forking.


    Parameters:
      * chunking (unsigned): Number of time steps to pre-allocate (default: 10)
      * desc (bool): Output stat descriptions (default: True)
      * formulas (bool): Output derived stats (default: True)

    Example:
      h5://stats.h5?desc=False;chunking=100;formulas=False

    """

    return _m5.stats.initHDF5(fn, chunking, desc, formulas)


@_url_factory(["json"])
def _jsonFactory(fn):
    """Output stats in JSON format.

    Example:
      json://stats.json

    """

    return JsonOutputVistor(fn)


def addStatVisitor(url):
    """Add a stat visitor specified using a URL string

    Stat visitors are specified using URLs on the following format:
    format://path[?param=value[;param=value]]

    The available formats are listed in the factories list. Factories
    are called with the path as the first positional parameter and the
    parameters are keyword arguments. Parameter values must be valid
    Python literals.

    """

    try:
        from urllib.parse import urlsplit
    except ImportError:
        # Python 2 fallback
        from urlparse import urlsplit

    parsed = urlsplit(url)

    try:
        factory = factories[parsed.scheme]
    except KeyError:
        fatal(f"Illegal stat file type '{parsed.scheme}' specified.")

    if factory is None:
        fatal(f"Stat type '{parsed.scheme}' disabled at compile time")

    outputList.append(factory(parsed))


def printStatVisitorTypes():
    """List available stat visitors and their documentation"""

    import inspect

    def print_doc(doc):
        for line in doc.splitlines():
            print(f"| {line}")
        print()

    enabled_visitors = [x for x in all_factories if x[2]]
    for factory, schemes, _ in enabled_visitors:
        print(f"{', '.join(filter(lambda x: x is not None, schemes))}:")

        # Try to extract the factory doc string
        print_doc(inspect.getdoc(factory))


def initSimStats():
    _m5.stats.initSimStats()
    _m5.stats.registerPythonStatsHandlers()


def _visit_groups(visitor, root=None):
    if root is None:
        root = Root.getInstance()
    for group in root.getStatGroups().values():
        visitor(group)
        _visit_groups(visitor, root=group)


def _visit_stats(visitor, root=None):
    def for_each_stat(g):
        for stat in g.getStats():
            visitor(g, stat)

    _visit_groups(for_each_stat, root=root)


def _bindStatHierarchy(root):
    def _bind_obj(name, obj):
        if isNullPointer(obj):
            return
        if m5.SimObject.isSimObjectVector(obj):
            if len(obj) == 1:
                _bind_obj(name, obj[0])
            else:
                for idx, obj in enumerate(obj):
                    _bind_obj(f"{name}{idx}", obj)
        else:
            # We need this check because not all obj.getCCObject() is an
            # instance of Stat::Group. For example, sc_core::sc_module, the C++
            # class of SystemC_ScModule, is not a subclass of Stat::Group. So
            # it will cause a type error if obj is a SystemC_ScModule when
            # calling addStatGroup().
            if isinstance(obj.getCCObject(), _m5.stats.Group):
                parent = root
                while parent:
                    if hasattr(parent, "addStatGroup"):
                        parent.addStatGroup(name, obj.getCCObject())
                        break
                    parent = parent.get_parent()

            _bindStatHierarchy(obj)

    for name, obj in root._children.items():
        _bind_obj(name, obj)


names = []
stats_dict = {}
stats_list = []


def enable():
    """Enable the statistics package.  Before the statistics package is
    enabled, all statistics must be created and initialized and once
    the package is enabled, no more statistics can be created."""

    def check_stat(group, stat):
        if not stat.check() or not stat.baseCheck():
            fatal(
                "statistic '%s' (%d) was not properly initialized "
                "by a regStats() function\n",
                stat.name,
                stat.id,
            )

        if not (stat.flags & flags.display):
            stat.name = "__Stat%06d" % stat.id

    # Legacy stat
    global stats_list
    stats_list = list(_m5.stats.statsList())

    for stat in stats_list:
        check_stat(None, stat)

    stats_list.sort(key=lambda s: s.name.split("."))
    for stat in stats_list:
        stats_dict[stat.name] = stat
        stat.enable()

    # New stats
    _visit_stats(check_stat)
    _visit_stats(lambda g, s: s.enable())

    _m5.stats.enable()


def prepare():
    """Prepare all stats for data access.  This must be done before
    dumping and serialization."""

    # Legacy stats
    for stat in stats_list:
        stat.prepare()

    # New stats
    _visit_stats(lambda g, s: s.prepare())


def _dump_to_visitor(visitor, roots=None):
    # New stats
    def dump_group(group):
        for stat in group.getStats():
            stat.visit(visitor)
        for n, g in group.getStatGroups().items():
            visitor.beginGroup(n)
            dump_group(g)
            visitor.endGroup()

    if roots:
        # New stats from selected subroots.
        for root in roots:
            for p in root.path_list():
                visitor.beginGroup(p)
            dump_group(root)
            for p in reversed(root.path_list()):
                visitor.endGroup()
    else:
        # New stats starting from root.
        dump_group(Root.getInstance())

        # Legacy stats
        for stat in stats_list:
            stat.visit(visitor)


lastDump = 0
# List[SimObject].
global_dump_roots = []


def dump(roots=None):
    """Dump all statistics data to the registered outputs"""

    all_roots = []
    if roots is not None:
        all_roots.extend(roots)
    global global_dump_roots
    all_roots.extend(global_dump_roots)

    now = m5.curTick()
    global lastDump
    assert lastDump <= now
    new_dump = lastDump != now
    lastDump = now

    # Don't allow multiple global stat dumps in the same tick. It's
    # still possible to dump a multiple sub-trees.
    if not new_dump and not all_roots:
        return

    # Only prepare stats the first time we dump them in the same tick.
    if new_dump:
        _m5.stats.processDumpQueue()
        # Notify new-style stats group that we are about to dump stats.
        sim_root = Root.getInstance()
        if sim_root:
            sim_root.preDumpStats()
        prepare()

    for output in outputList:
        if isinstance(output, JsonOutputVistor):
            if not all_roots:
                output.dump(Root.getInstance())
            else:
                output.dump(all_roots)
        else:
            if output.valid():
                output.begin()
                _dump_to_visitor(output, roots=all_roots)
                output.end()


def reset():
    """Reset all statistics to the base state"""

    # call reset stats on all SimObjects
    root = Root.getInstance()
    if root:
        root.resetStats()

    # call any other registered legacy stats reset callbacks
    for stat in stats_list:
        stat.reset()

    _m5.stats.processResetQueue()


flags = attrdict(
    {
        "none": 0x0000,
        "init": 0x0001,
        "display": 0x0002,
        "total": 0x0010,
        "pdf": 0x0020,
        "cdf": 0x0040,
        "dist": 0x0080,
        "nozero": 0x0100,
        "nonan": 0x0200,
    }
)
