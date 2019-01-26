# Copyright (c) 2017 ARM Limited
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
#
# Authors: Nathan Binkert
#          Andreas Sandberg

import m5

import _m5.stats
from m5.objects import Root
from m5.util import attrdict, fatal

# Stat exports
from _m5.stats import schedStatEvent as schedEvent
from _m5.stats import periodicStatDump

outputList = []

def _url_factory(func):
    """Wrap a plain Python function with URL parsing helpers

    Wrap a plain Python function f(fn, **kwargs) to expect a URL that
    has been split using urlparse.urlsplit. First positional argument
    is assumed to be a filename, this is created as the concatenation
    of the netloc (~hostname) and path in the parsed URL. Keyword
    arguments are derived from the query values in the URL.

    For example:
        wrapped_f(urlparse.urlsplit("text://stats.txt?desc=False")) ->
        f("stats.txt", desc=False)

    """

    from functools import wraps

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
                fatal("%s: '%s' doesn't have a value." % (url.geturl(), key))
            elif len(values) > 1:
                fatal("%s: '%s' has multiple values." % (url.geturl(), key))
            else:
                try:
                    return key, literal_eval(values[0])
                except ValueError:
                    fatal("%s: %s isn't a valid Python literal" \
                          % (url.geturl(), values[0]))

        kwargs = dict([ parse_value(k, v) for k, v in qs.items() ])

        try:
            return func("%s%s" % (url.netloc, url.path), **kwargs)
        except TypeError:
            fatal("Illegal stat visitor parameter specified")

    return wrapper

@_url_factory
def _textFactory(fn, desc=True):
    """Output stats in text format.

    Text stat files contain one stat per line with an optional
    description. The description is enabled by default, but can be
    disabled by setting the desc parameter to False.

    Example: text://stats.txt?desc=False

    """

    return _m5.stats.initText(fn, desc)

factories = {
    # Default to the text factory if we're given a naked path
    "" : _textFactory,
    "file" : _textFactory,
    "text" : _textFactory,
}

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
        fatal("Illegal stat file type specified.")

    outputList.append(factory(parsed))

def initSimStats():
    _m5.stats.initSimStats()
    _m5.stats.registerPythonStatsHandlers()

names = []
stats_dict = {}
stats_list = []
def enable():
    '''Enable the statistics package.  Before the statistics package is
    enabled, all statistics must be created and initialized and once
    the package is enabled, no more statistics can be created.'''

    global stats_list
    stats_list = list(_m5.stats.statsList())

    for stat in stats_list:
        if not stat.check() or not stat.baseCheck():
            fatal("statistic '%s' (%d) was not properly initialized " \
                  "by a regStats() function\n", stat.name, stat.id)

        if not (stat.flags & flags.display):
            stat.name = "__Stat%06d" % stat.id

    stats_list.sort(key=lambda s: s.name.split('.'))
    for stat in stats_list:
        stats_dict[stat.name] = stat
        stat.enable()

    _m5.stats.enable();

def prepare():
    '''Prepare all stats for data access.  This must be done before
    dumping and serialization.'''

    for stat in stats_list:
        stat.prepare()

lastDump = 0
def dump():
    '''Dump all statistics data to the registered outputs'''

    curTick = m5.curTick()

    global lastDump
    assert lastDump <= curTick
    if lastDump == curTick:
        return
    lastDump = curTick

    _m5.stats.processDumpQueue()

    prepare()

    for output in outputList:
        if output.valid():
            output.begin()
            for stat in stats_list:
                stat.visit(output)
            output.end()

def reset():
    '''Reset all statistics to the base state'''

    # call reset stats on all SimObjects
    root = Root.getInstance()
    if root:
        for obj in root.descendants(): obj.resetStats()

    # call any other registered stats reset callbacks
    for stat in stats_list:
        stat.reset()

    _m5.stats.processResetQueue()

flags = attrdict({
    'none'    : 0x0000,
    'init'    : 0x0001,
    'display' : 0x0002,
    'total'   : 0x0010,
    'pdf'     : 0x0020,
    'cdf'     : 0x0040,
    'dist'    : 0x0080,
    'nozero'  : 0x0100,
    'nonan'   : 0x0200,
})
