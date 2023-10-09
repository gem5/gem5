# Copyright (c) 2020 ARM Limited
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
# Copyright (c) 2017 Mark D. Hill and David A. Wood
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
# Authors: Sean Wilson
"""
Helper classes for writing tests with this test library.
"""
import difflib
import errno
import os
import re
import shutil
import stat
import subprocess
import tempfile
import threading
import time
from collections import namedtuple
from collections.abc import MutableSet


class TimedWaitPID(object):
    """Utility to monkey-patch os.waitpid() with os.wait4().

    This allows process usage time to be obtained directly from the OS
    when used with APIs, such as `subprocess`, which use os.waitpid to
    join child processes.

    The resource usage data from os.wait4() is stored in a functor and
    can be obtained using the get_time_for_pid() method.

    To avoid unbounded memory usage, the time record is deleted after
    it is read.

    """

    TimeRecord = namedtuple("_TimeRecord", "user_time system_time")

    class Wrapper(object):
        def __init__(self):
            self._time_for_pid = {}
            self._access_lock = threading.Lock()

        def __call__(self, pid, options):
            pid, status, resource_usage = os.wait4(pid, options)
            with self._access_lock:
                self._time_for_pid[pid] = TimedWaitPID.TimeRecord(
                    resource_usage.ru_utime, resource_usage.ru_stime
                )
            return (pid, status)

        def has_time_for_pid(self, pid):
            with self._access_lock:
                return pid in self._time_for_pid

        def get_time_for_pid(self, pid):
            with self._access_lock:
                if pid not in self._time_for_pid:
                    raise Exception("No resource usage for pid {}".format(pid))
                time_for_pid = self._time_for_pid[pid]
                del self._time_for_pid[pid]
                return time_for_pid

    _wrapper = None
    _wrapper_lock = threading.Lock()
    _original_os_waitpid = None

    @staticmethod
    def install():
        with TimedWaitPID._wrapper_lock:
            if TimedWaitPID._wrapper is None:
                TimedWaitPID._wrapper = TimedWaitPID.Wrapper()
            if TimedWaitPID._original_os_waitpid is None:
                TimedWaitPID._original_os_waitpid = os.waitpid
                os.waitpid = TimedWaitPID._wrapper

    @staticmethod
    def restore():
        with TimedWaitPID._wrapper_lock:
            if TimedWaitPID._original_os_waitpid is not None:
                os.waitpid = TimedWaitPID._original_os_waitpid
                TimedWaitPID._original_os_waitpid = None

    @staticmethod
    def has_time_for_pid(pid):
        with TimedWaitPID._wrapper_lock:
            return TimedWaitPID._wrapper.has_time_for_pid(pid)

    @staticmethod
    def get_time_for_pid(pid):
        with TimedWaitPID._wrapper_lock:
            return TimedWaitPID._wrapper.get_time_for_pid(pid)


# Patch os.waitpid()
TimedWaitPID.install()

# TODO Tear out duplicate logic from the sandbox IOManager
def log_call(logger, command, time, *popenargs, **kwargs):
    """
    Calls the given process and automatically logs the command and output.

    If stdout or stderr are provided output will also be piped into those
    streams as well.

    :params stdout: Iterable of items to write to as we read from the
        subprocess.

    :params stderr: Iterable of items to write to as we read from the
        subprocess.
    """
    if isinstance(command, str):
        cmdstr = command
    else:
        try:
            command = list(map(str, command))
            cmdstr = " ".join(command)
        except TypeError as e:
            logger.trace(
                "Argument <command> must be an iterable of string-convertible types"
            )
            raise e

    logger_callback = logger.trace
    logger.trace("Logging call to command: %s" % cmdstr)

    stdout_redirect = kwargs.get("stdout", tuple())
    stderr_redirect = kwargs.get("stderr", tuple())

    if hasattr(stdout_redirect, "write"):
        stdout_redirect = (stdout_redirect,)
    if hasattr(stderr_redirect, "write"):
        stderr_redirect = (stderr_redirect,)

    kwargs["stdout"] = subprocess.PIPE
    kwargs["stderr"] = subprocess.PIPE
    p = subprocess.Popen(command, *popenargs, **kwargs)

    def log_output(log_callback, pipe, redirects=tuple()):
        # Read iteractively, don't allow input to fill the pipe.
        for line in iter(pipe.readline, b""):
            line = line.decode("utf-8")
            for r in redirects:
                r.write(line)
            log_callback(line.rstrip())

    stdout_thread = threading.Thread(
        target=log_output, args=(logger_callback, p.stdout, stdout_redirect)
    )
    stdout_thread.setDaemon(True)
    stderr_thread = threading.Thread(
        target=log_output, args=(logger_callback, p.stderr, stderr_redirect)
    )
    stderr_thread.setDaemon(True)

    stdout_thread.start()
    stderr_thread.start()

    retval = p.wait()
    stdout_thread.join()
    stderr_thread.join()

    if time is not None and TimedWaitPID.has_time_for_pid(p.pid):
        resource_usage = TimedWaitPID.get_time_for_pid(p.pid)
        time["user_time"] = resource_usage.user_time
        time["system_time"] = resource_usage.system_time

    # Return the return exit code of the process.
    if retval != 0:
        raise subprocess.CalledProcessError(retval, cmdstr)


# lru_cache stuff (Introduced in python 3.2+)
# Renamed and modified to cacheresult
class _HashedSeq(list):
    """
    This class guarantees that hash() will be called no more than once per
    element. This is important because the cacheresult() will hash the key
    multiple times on a cache miss.

    .. note:: From cpython 3.7
    """

    __slots__ = "hashvalue"

    def __init__(self, tup, hash=hash):
        self[:] = tup
        self.hashvalue = hash(tup)

    def __hash__(self):
        return self.hashvalue


def _make_key(
    args,
    kwds,
    typed,
    kwd_mark=(object(),),
    fasttypes={int, str, frozenset, type(None)},
    tuple=tuple,
    type=type,
    len=len,
):
    """
    Make a cache key from optionally typed positional and keyword arguments.
    The key is constructed in a way that is flat as possible rather than as
    a nested structure that would take more memory.  If there is only a single
    argument and its data type is known to cache its hash value, then that
    argument is returned without a wrapper. This saves space and improves
    lookup speed.

    .. note:: From cpython 3.7
    """
    key = args
    if kwds:
        key += kwd_mark
        for item in kwds.items():
            key += item
    if typed:
        key += tuple(type(v) for v in args)
        if kwds:
            key += tuple(type(v) for v in kwds.values())
    elif len(key) == 1 and type(key[0]) in fasttypes:
        return key[0]
    return _HashedSeq(key)


def cacheresult(function, typed=False):
    """
    :param typed: If typed is True, arguments of different types will be
        cached separately. I.e. f(3.0) and f(3) will be treated as distinct
        calls with distinct results.

    .. note:: From cpython 3.7
    """
    sentinel = object()  # unique object used to signal cache misses
    cache = {}

    def wrapper(*args, **kwds):
        # Simple caching without ordering or size limit
        key = _make_key(args, kwds, typed)
        result = cache.get(key, sentinel)
        if result is not sentinel:
            return result
        result = function(*args, **kwds)
        cache[key] = result
        return result

    return wrapper


class OrderedSet(MutableSet):
    """
    Maintain ordering of insertion in items to the set with quick iteration.

    http://code.activestate.com/recipes/576694/
    """

    def __init__(self, iterable=None):
        self.end = end = []
        end += [None, end, end]  # sentinel node for doubly linked list
        self.map = {}  # key --> [key, prev, next]
        if iterable is not None:
            self |= iterable

    def __len__(self):
        return len(self.map)

    def __contains__(self, key):
        return key in self.map

    def add(self, key):
        if key not in self.map:
            end = self.end
            curr = end[1]
            curr[2] = end[1] = self.map[key] = [key, curr, end]

    def update(self, keys):
        for key in keys:
            self.add(key)

    def discard(self, key):
        if key in self.map:
            key, prev, next = self.map.pop(key)
            prev[2] = next
            next[1] = prev

    def __iter__(self):
        end = self.end
        curr = end[2]
        while curr is not end:
            yield curr[0]
            curr = curr[2]

    def __reversed__(self):
        end = self.end
        curr = end[1]
        while curr is not end:
            yield curr[0]
            curr = curr[1]

    def pop(self, last=True):
        if not self:
            raise KeyError("set is empty")
        key = self.end[1][0] if last else self.end[2][0]
        self.discard(key)
        return key

    def __repr__(self):
        if not self:
            return "%s()" % (self.__class__.__name__,)
        return "%s(%r)" % (self.__class__.__name__, list(self))

    def __eq__(self, other):
        if isinstance(other, OrderedSet):
            return len(self) == len(other) and list(self) == list(other)
        return set(self) == set(other)


def absdirpath(path):
    """
    Return the directory component of the absolute path of the given path.
    """
    return os.path.dirname(os.path.abspath(path))


joinpath = os.path.join


def mkdir_p(path):
    """
    Same thing as mkdir -p

    https://stackoverflow.com/a/600612
    """
    try:
        os.makedirs(path)
    except OSError as exc:  # Python >2.5
        if exc.errno == errno.EEXIST and os.path.isdir(path):
            pass
        else:
            raise


class FrozenSetException(Exception):
    """Signals one tried to set a value in a 'frozen' object."""

    pass


class AttrDict(object):
    """Object which exposes its own internal dictionary through attributes."""

    def __init__(self, dict_={}):
        self.update(dict_)

    def __getattr__(self, attr):
        dict_ = self.__dict__
        if attr in dict_:
            return dict_[attr]
        raise AttributeError("Could not find %s attribute" % attr)

    def __setattr__(self, attr, val):
        self.__dict__[attr] = val

    def __iter__(self):
        return iter(self.__dict__)

    def __getitem__(self, item):
        return self.__dict__[item]

    def update(self, items):
        self.__dict__.update(items)


class FrozenAttrDict(AttrDict):
    """An AttrDict whose attributes cannot be modified directly."""

    __initialized = False

    def __init__(self, dict_={}):
        super(FrozenAttrDict, self).__init__(dict_)
        self.__initialized = True

    def __setattr__(self, attr, val):
        if self.__initialized:
            raise FrozenSetException(
                "Cannot modify an attribute in a FozenAttrDict"
            )
        else:
            super(FrozenAttrDict, self).__setattr__(attr, val)

    def update(self, items):
        if self.__initialized:
            raise FrozenSetException(
                "Cannot modify an attribute in a FozenAttrDict"
            )
        else:
            super(FrozenAttrDict, self).update(items)


class InstanceCollector(object):
    """
    A class used to simplify collecting of Classes.

    >> instance_list = collector.create()
    >> # Create a bunch of classes which call collector.collect(self)
    >> # instance_list contains all instances created since
    >> # collector.create was called
    >> collector.remove(instance_list)
    """

    def __init__(self):
        self.collectors = []

    def create(self):
        collection = []
        self.collectors.append(collection)
        return collection

    def remove(self, collector):
        self.collectors.remove(collector)

    def collect(self, instance):
        for col in self.collectors:
            col.append(instance)


def append_dictlist(dict_, key, value):
    """
    Append the `value` to a list associated with `key` in `dict_`.
    If `key` doesn't exist, create a new list in the `dict_` with value in it.
    """
    list_ = dict_.get(key, [])
    list_.append(value)
    dict_[key] = list_


def _filter_file(fname, filters):
    with open(fname, "r") as file_:
        for line in file_:
            for regex in filters:
                if re.match(regex, line):
                    break
            else:
                yield line


def _copy_file_keep_perms(source, target):
    """Copy a file keeping the original permisions of the target."""
    st = os.stat(target)
    shutil.copy2(source, target)
    os.chown(target, st[stat.ST_UID], st[stat.ST_GID])


def _filter_file_inplace(fname, dir, filters):
    """
    Filter the given file writing filtered lines out to a temporary file, then
    copy that tempfile back into the original file.
    """
    (_, tfname) = tempfile.mkstemp(dir=dir, text=True)
    with open(tfname, "w") as tempfile_:
        for line in _filter_file(fname, filters):
            tempfile_.write(line)

    # Now filtered output is into tempfile_
    _copy_file_keep_perms(tfname, fname)


def diff_out_file(ref_file, out_file, logger, ignore_regexes=tuple()):
    """Diff two files returning the diff as a string."""

    if not os.path.exists(ref_file):
        raise OSError("%s doesn't exist in reference directory" % ref_file)
    if not os.path.exists(out_file):
        raise OSError("%s doesn't exist in output directory" % out_file)

    _filter_file_inplace(out_file, os.path.dirname(out_file), ignore_regexes)
    _filter_file_inplace(ref_file, os.path.dirname(out_file), ignore_regexes)

    # try :
    (_, tfname) = tempfile.mkstemp(dir=os.path.dirname(out_file), text=True)
    with open(tfname, "r+") as tempfile_:
        try:
            log_call(
                logger,
                ["diff", out_file, ref_file],
                time=None,
                stdout=tempfile_,
            )
        except OSError:
            # Likely signals that diff does not exist on this system. fallback
            # to difflib
            with open(out_file, "r") as outf, open(ref_file, "r") as reff:
                diff = difflib.unified_diff(
                    iter(reff.readline, ""),
                    iter(outf.readline, ""),
                    fromfile=ref_file,
                    tofile=out_file,
                )
                return "".join(diff)
        except subprocess.CalledProcessError:
            tempfile_.seek(0)
            return "".join(tempfile_.readlines())
        else:
            return None


class Timer:
    def __init__(self):
        self.restart()

    def restart(self):
        self._start = self.timestamp()
        self._stop = None

    def stop(self):
        self._stop = self.timestamp()
        return self._stop - self._start

    def runtime(self):
        return self._stop - self._start

    def active_time(self):
        return self.timestamp() - self._start

    @staticmethod
    def timestamp():
        return time.time()
