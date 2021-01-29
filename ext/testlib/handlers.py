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
Handlers for the testlib Log.


'''
import multiprocessing
import os
import sys
import threading
import time
import traceback

import testlib.helper as helper
import testlib.log as log
import testlib.result as result
import testlib.state as state
import testlib.terminal as terminal

from queue import Queue, Empty
from testlib.configuration import constants


class _TestStreamManager(object):
    def __init__(self):
        self._writers = {}

    def open_writer(self, test_result):
        if test_result in self._writers:
            raise ValueError('Cannot have multiple writters on a single test.')
        self._writers[test_result] = _TestStreams(test_result.stdout,
                test_result.stderr)

    def get_writer(self, test_result):
        if test_result not in self._writers:
            self.open_writer(test_result)
        return self._writers[test_result]

    def close_writer(self, test_result):
        if test_result in self._writers:
            writer = self._writers.pop(test_result)
            writer.close()

    def close(self):
        for writer in self._writers.values():
            writer.close()
        self._writers.clear()

class _TestStreams(object):
    def __init__(self, stdout, stderr):
        helper.mkdir_p(os.path.dirname(stdout))
        helper.mkdir_p(os.path.dirname(stderr))
        self.stdout = open(stdout, 'w')
        self.stderr = open(stderr, 'w')

    def close(self):
        self.stdout.close()
        self.stderr.close()

class ResultHandler(object):
    '''
    Log handler which listens for test results and output saving data as
    it is reported.

    When the handler is closed it writes out test results in the python pickle
    format.
    '''
    def __init__(self, schedule, directory):
        '''
        :param schedule: The entire schedule as a :class:`LoadedLibrary`
            object.

        :param directory: Directory to save test stdout/stderr and aggregate
            results to.
        '''
        self.directory = directory
        self.internal_results = result.InternalLibraryResults(schedule,
                directory)
        self.test_stream_manager = _TestStreamManager()
        self._closed = False

        self.mapping = {
            log.LibraryStatus.type_id: self.handle_library_status,

            log.SuiteResult.type_id: self.handle_suite_result,
            log.TestResult.type_id: self.handle_test_result,

            log.TestStderr.type_id: self.handle_stderr,
            log.TestStdout.type_id: self.handle_stdout,
        }

    def handle(self, record):
        if not self._closed:
            self.mapping.get(record.type_id, lambda _:None)(record)

    def handle_library_status(self, record):
        if record['status'] in (state.Status.Complete, state.Status.Avoided):
            self.test_stream_manager.close()

    def handle_suite_result(self, record):
        suite_result = self.internal_results.get_suite_result(
                    record['metadata'].uid)
        suite_result.result = record['result']

    def handle_test_result(self, record):
        test_result = self._get_test_result(record)
        test_result.result = record['result']

    def handle_stderr(self, record):
        self.test_stream_manager.get_writer(
            self._get_test_result(record)
        ).stderr.write(record['buffer'])

    def handle_stdout(self, record):
        self.test_stream_manager.get_writer(
            self._get_test_result(record)
        ).stdout.write(record['buffer'])

    def _get_test_result(self, test_record):
        return self.internal_results.get_test_result(
                    test_record['metadata'].uid,
                    test_record['metadata'].suite_uid)

    def _save(self):
        #FIXME Hardcoded path name
        result.InternalSavedResults.save(
            self.internal_results,
            os.path.join(self.directory, constants.pickle_filename))
        result.JUnitSavedResults.save(
            self.internal_results,
            os.path.join(self.directory, constants.xml_filename))

    def close(self):
        if self._closed:
            return
        self._closed = True
        self._save()

    def unsuccessful(self):
        '''
        Performs an or reduce on all of the results.
        Returns true if at least one test is unsuccessful, false when all tests
        pass
        '''
        for suite_result in self.internal_results:
            if suite_result.unsuccessful:
                return True
        # If all are successful, then this wasn't "unsuccessful"
        return False


#TODO Change from a handler to an internal post processor so it can be used
# to reprint results
class SummaryHandler(object):
    '''
    A log handler which listens to the log for test results
    and reports the aggregate results when closed.
    '''
    color = terminal.get_termcap()
    reset = color.Normal
    colormap = {
            state.Result.Errored: color.Red,
            state.Result.Failed: color.Red,
            state.Result.Passed: color.Green,
            state.Result.Skipped: color.Cyan,
    }

    def __init__(self):
        self.mapping = {
            log.TestResult.type_id: self.handle_testresult,
            log.LibraryStatus.type_id: self.handle_library_status,
        }
        self._timer = helper.Timer()
        self.results = []

    def handle_library_status(self, record):
        if record['status'] == state.Status.Building:
            self._timer.restart()

    def handle_testresult(self, record):
        result = record['result'].value
        if result in (state.Result.Skipped, state.Result.Failed,
                state.Result.Passed, state.Result.Errored):
            self.results.append(result)

    def handle(self, record):
        self.mapping.get(record.type_id, lambda _:None)(record)

    def close(self):
        print(self._display_summary())

    def _display_summary(self):
        most_severe_outcome = None
        outcome_fmt = ' {count} {outcome}'
        strings = []

        outcome_count = [0] * len(state.Result.enums)
        for result in self.results:
            outcome_count[result] += 1

        # Iterate over enums so they are in order of severity
        for outcome in state.Result.enums:
            outcome = getattr(state.Result, outcome)
            count  = outcome_count[outcome]
            if count:
                strings.append(outcome_fmt.format(count=count,
                        outcome=state.Result.enums[outcome]))
                most_severe_outcome = outcome
        string = ','.join(strings)
        if most_severe_outcome is None:
            string = ' No testing done'
            most_severe_outcome = state.Result.Passed
        else:
            string = ' Results:' + string + ' in {:.2} seconds '.format(
                    self._timer.active_time())
        string += ' '
        return terminal.insert_separator(
                string,
                color=self.colormap[most_severe_outcome] + self.color.Bold)

class TerminalHandler(object):
    color = terminal.get_termcap()
    verbosity_mapping = {
        log.LogLevel.Warn: color.Yellow,
        log.LogLevel.Error: color.Red,
    }
    default = color.Normal

    def __init__(self, verbosity=log.LogLevel.Info, machine_only=False):
        self.stream = verbosity >= log.LogLevel.Trace
        self.verbosity = verbosity
        self.machine_only = machine_only
        self.mapping = {
            log.TestResult.type_id: self.handle_testresult,
            log.SuiteStatus.type_id: self.handle_suitestatus,
            log.TestStatus.type_id: self.handle_teststatus,
            log.TestStderr.type_id: self.handle_stderr,
            log.TestStdout.type_id: self.handle_stdout,
            log.TestMessage.type_id: self.handle_testmessage,
            log.LibraryMessage.type_id: self.handle_librarymessage,
        }

    def _display_outcome(self, name, outcome, reason=None):
        print(self.color.Bold
                 + SummaryHandler.colormap[outcome]
                 + name
                 + ' '
                 + state.Result.enums[outcome]
                 + SummaryHandler.reset)

        if reason is not None:
            log.test_log.info('')
            log.test_log.info('Reason:')
            log.test_log.info(reason)
            log.test_log.info(terminal.separator('-'))

    def handle_teststatus(self, record):
        if record['status'] == state.Status.Running:
            log.test_log.debug('Starting Test Case: %s' %\
                    record['metadata'].name)

    def handle_testresult(self, record):
        self._display_outcome(
            'Test: %s'  % record['metadata'].name,
            record['result'].value)

    def handle_suitestatus(self, record):
        if record['status'] == state.Status.Running:
              log.test_log.debug('Starting Test Suite: %s ' %\
                    record['metadata'].name)

    def handle_stderr(self, record):
        if self.stream:
            print(record.data['buffer'], file=sys.stderr, end='')

    def handle_stdout(self, record):
        if self.stream:
            print(record.data['buffer'], file=sys.stdout, end='')

    def handle_testmessage(self, record):
        if self.stream:
            print(self._colorize(record['message'], record['level']))

    def handle_librarymessage(self, record):
        if not self.machine_only or record.data.get('machine_readable', False):
            print(self._colorize(record['message'], record['level'],
                    record['bold']))

    def _colorize(self, message, level, bold=False):
        return '%s%s%s%s' % (
                self.color.Bold if bold else '',
                self.verbosity_mapping.get(level, ''),
                message,
                self.default)

    def handle(self, record):
        if record.data.get('level', self.verbosity) > self.verbosity:
            return
        self.mapping.get(record.type_id, lambda _:None)(record)

    def close(self):
        pass

class MultiprocessingHandlerWrapper(object):
    '''
    A handler class which forwards log records to subhandlers, enabling
    logging across multiprocessing python processes.

    The 'parent' side of the handler should execute either
    :func:`async_process` or :func:`process` to forward
    log records to subhandlers.
    '''
    def __init__(self, *subhandlers):
        # Create thread to spin handing recipt of messages
        # Create queue to push onto
        self.queue = multiprocessing.Queue()
        self.queue.cancel_join_thread()
        self._shutdown = threading.Event()

        # subhandlers should be accessed with the _handler_lock
        self._handler_lock = threading.Lock()
        self._subhandlers = subhandlers

    def add_handler(self, handler):
        self._handler_lock.acquire()
        self._subhandlers = (handler, ) + self._subhandlers
        self._handler_lock.release()

    def _with_handlers(self, callback):
        exception = None
        self._handler_lock.acquire()
        for handler in self._subhandlers:
            # Prevent deadlock when using this handler by delaying
            # exception raise until we get a chance to unlock.
            try:
                callback(handler)
            except Exception as e:
                exception = e
                break
        self._handler_lock.release()

        if exception is not None:
            raise exception

    def async_process(self):
        self.thread = threading.Thread(target=self.process)
        self.thread.daemon = True
        self.thread.start()

    def process(self):
        while not self._shutdown.is_set():
            try:
                item = self.queue.get(timeout=0.1)
                self._handle(item)
            except (KeyboardInterrupt, SystemExit):
                raise
            except EOFError:
                return
            except Empty:
                continue

    def _drain(self):
        while True:
            try:
                item = self.queue.get(block=False)
                self._handle(item)
            except (KeyboardInterrupt, SystemExit):
                raise
            except EOFError:
                return
            except Empty:
                return

    def _handle(self, record):
        self._with_handlers(lambda handler: handler.handle(record))

    def handle(self, record):
        self.queue.put(record)

    def _close(self):
        if hasattr(self, 'thread'):
            self.thread.join()
        _wrap(self._drain)
        self._with_handlers(lambda handler: _wrap(handler.close))

        # NOTE Python2 has an known bug which causes IOErrors to be raised
        # if this shutdown doesn't go cleanly on both ends.
        # This sleep adds some time for the sender threads on this process to
        # finish pickling the object and complete shutdown after the queue is
        # closed.
        time.sleep(.2)
        self.queue.close()
        time.sleep(.2)

    def close(self):
        if not self._shutdown.is_set():
            self._shutdown.set()
            self._close()


def _wrap(callback, *args, **kwargs):
    try:
        callback(*args, **kwargs)
    except:
        traceback.print_exc()
