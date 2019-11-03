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

'''
This module supplies the global `test_log` object which all testing
results and messages are reported through.
'''
import wrappers


class LogLevel():
    Fatal = 0
    Error = 1
    Warn  = 2
    Info  = 3
    Debug = 4
    Trace = 5


class RecordTypeCounterMetaclass(type):
    '''
    Record type metaclass.

    Adds a static integer value in addition to typeinfo so identifiers
    are common across processes, networks and module reloads.
    '''
    counter = 0
    def __init__(cls, name, bases, dct):
        cls.type_id = RecordTypeCounterMetaclass.counter
        RecordTypeCounterMetaclass.counter += 1


class Record(object):
    '''
    A generic object that is passed to the :class:`Log` and its handlers.

    ..note: Although not statically enforced, all items in the record should be
        be pickleable. This enables logging accross multiple processes.
    '''
    __metaclass__ = RecordTypeCounterMetaclass

    def __init__(self, **data):
        self.data = data

    def __getitem__(self, item):
        if item not in self.data:
            raise KeyError('%s not in record %s' %\
                    (item, self.__class__.__name__))
        return self.data[item]

    def __str__(self):
        return str(self.data)


class StatusRecord(Record):
    def __init__(self, obj, status):
        Record.__init__(self, metadata=obj.metadata, status=status)
class ResultRecord(Record):
    def __init__(self, obj, result):
        Record.__init__(self, metadata=obj.metadata, result=result)
#TODO Refactor this shit... Not ideal. Should just specify attributes.
class TestStatus(StatusRecord):
    pass
class SuiteStatus(StatusRecord):
    pass
class LibraryStatus(StatusRecord):
    pass
class TestResult(ResultRecord):
    pass
class SuiteResult(ResultRecord):
    pass
class LibraryResult(ResultRecord):
    pass
# Test Output Types
class TestStderr(Record):
    pass
class TestStdout(Record):
    pass
# Message (Raw String) Types
class TestMessage(Record):
    pass
class LibraryMessage(Record):
    pass


class Log(object):
    def __init__(self):
        self.handlers = []
        self._opened = False # TODO Guards to methods
        self._closed = False # TODO Guards to methods

    def finish_init(self):
        self._opened = True

    def close(self):
        self._closed = True
        for handler in self.handlers:
            handler.close()

    def log(self, record):
        if not self._opened:
            self.finish_init()
        if self._closed:
            raise Exception('The log has been closed'
                ' and is no longer available.')

        map(lambda handler:handler.prehandle(), self.handlers)
        for handler in self.handlers:
            handler.handle(record)
            handler.posthandle()

    def add_handler(self, handler):
        if self._opened:
            raise Exception('Unable to add a handler once the log is open.')
        self.handlers.append(handler)

    def close_handler(self, handler):
        handler.close()
        self.handlers.remove(handler)


class Handler(object):
    '''
    Empty implementation of the interface available to handlers which
    is expected by the :class:`Log`.
    '''
    def __init__(self):
        pass

    def handle(self, record):
        pass

    def close(self):
        pass

    def prehandle(self):
        pass

    def posthandle(self):
        pass


class LogWrapper(object):
    _result_typemap = {
        wrappers.LoadedLibrary.__name__: LibraryResult,
        wrappers.LoadedSuite.__name__: SuiteResult,
        wrappers.LoadedTest.__name__: TestResult,
    }
    _status_typemap = {
        wrappers.LoadedLibrary.__name__: LibraryStatus,
        wrappers.LoadedSuite.__name__: SuiteStatus,
        wrappers.LoadedTest.__name__: TestStatus,
    }
    def __init__(self, log):
        self.log_obj = log

    def log(self, *args, **kwargs):
        self.log_obj.log(*args, **kwargs)

    # Library Logging Methods
    # TODO Replace these methods in a test/create a wrapper?
    # That way they still can log like this it's just hidden that they
    # capture the current test.
    def message(self, message, level=LogLevel.Info, bold=False, **metadata):
        self.log_obj.log(LibraryMessage(message=message, level=level,
                bold=bold, **metadata))

    def error(self, message):
        self.message(message, LogLevel.Error)

    def warn(self, message):
        self.message(message, LogLevel.Warn)

    def info(self, message):
        self.message(message, LogLevel.Info)

    def debug(self, message):
        self.message(message, LogLevel.Debug)

    def trace(self, message):
        self.message(message, LogLevel.Trace)

    # Ongoing Test Logging Methods
    def status_update(self, obj, status):
        self.log_obj.log(
                self._status_typemap[obj.__class__.__name__](obj, status))

    def result_update(self, obj, result):
        self.log_obj.log(
                self._result_typemap[obj.__class__.__name__](obj, result))

    def test_message(self, test, message, level):
        self.log_obj.log(TestMessage(message=message, level=level,
                test_uid=test.uid, suite_uid=test.parent_suite.uid))

    # NOTE If performance starts to drag on logging stdout/err
    # replace metadata with just test and suite uid tags.
    def test_stdout(self, test, suite, buf):
        self.log_obj.log(TestStdout(buffer=buf, metadata=test.metadata))

    def test_stderr(self, test, suite, buf):
        self.log_obj.log(TestStderr(buffer=buf, metadata=test.metadata))

    def close(self):
        self.log_obj.close()

class TestLogWrapper(object):
    def __init__(self, log, test, suite):
        self.log_obj = log
        self.test = test

    def test_message(self, message, level):
        self.log_obj.test_message(test=self.test,
                message=message, level=level)

    def error(self, message):
        self.test_message(message, LogLevel.Error)

    def warn(self, message):
        self.test_message(message, LogLevel.Warn)

    def info(self, message):
        self.test_message(message, LogLevel.Info)

    def debug(self, message):
        self.test_message(message, LogLevel.Debug)

    def trace(self, message):
        self.test_message(message, LogLevel.Trace)

test_log = LogWrapper(Log())
