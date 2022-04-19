# Copyright (c) 2020-2021 ARM Limited
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

'''
Global configuration module which exposes two types of configuration
variables:

1. config
2. constants (Also attached to the config variable as an attribute)

The main motivation for this module is to have a centralized location for
defaults and configuration by command line and files for the test framework.

A secondary goal is to reduce programming errors by providing common constant
strings and values as python attributes to simplify detection of typos.
A simple typo in a string can take a lot of debugging to uncover the issue,
attribute errors are easier to notice and most autocompletion systems detect
them.

The config variable is initialzed by calling :func:`initialize_config`.
Before this point only ``constants`` will be availaible. This is to ensure
that library function writers never accidentally get stale config attributes.

Program arguments/flag arguments are available from the config as attributes.
If an attribute was not set by the command line or the optional config file,
then it will fallback to the `_defaults` value, if still the value is not
found an AttributeError will be raised.

:func define_defaults:
    Provided by the config if the attribute is not found in the config or
    commandline. For instance, if we are using the list command fixtures might
    not be able to count on the build_dir being provided since we aren't going
    to build anything.

:var constants:
    Values not directly exposed by the config, but are attached to the object
    for centralized access. I.E. you can reach them with
    :code:`config.constants.attribute`. These should be used for setting
    common string names used across the test framework.
    :code:`_defaults.build_dir = None` Once this module has been imported
    constants should not be modified and their base attributes are frozen.
'''
import abc
import argparse
import copy
import os
import re

from pickle import HIGHEST_PROTOCOL as highest_pickle_protocol

from testlib.helper import absdirpath, AttrDict, FrozenAttrDict

class UninitialzedAttributeException(Exception):
    '''
    Signals that an attribute in the config file was not initialized.
    '''
    pass

class UninitializedConfigException(Exception):
    '''
    Signals that the config was not initialized before trying to access an
    attribute.
    '''
    pass

class TagRegex(object):
    def __init__(self, include, regex):
        self.include = include
        self.regex = re.compile(regex)

    def __str__(self):
        type_ = 'Include' if self.include else 'Remove'
        return '%10s: %s' % (type_, self.regex.pattern)

class _Config(object):
    _initialized = False

    __shared_dict = {}

    constants = AttrDict()
    _defaults = AttrDict()
    _config = {}

    _cli_args = {}
    _post_processors = {}

    def __init__(self):
        # This object will act as if it were a singleton.
        self.__dict__ = self.__shared_dict

    def _init(self, parser):
        self._parse_commandline_args(parser)
        self._run_post_processors()
        self._initialized = True

    def _add_post_processor(self, attr, post_processor):
        '''
        :param attr: Attribute to pass to and recieve from the
        :func:`post_processor`.

        :param post_processor: A callback functions called in a chain to
            perform additional setup for a config argument. Should return a
            tuple containing the new value for the config attr.
        '''
        if attr not in self._post_processors:
            self._post_processors[attr] = []
        self._post_processors[attr].append(post_processor)

    def _set(self, name, value):
        self._config[name] = value

    def _parse_commandline_args(self, parser):
        args = parser.parse_args()

        self._config_file_args = {}

        for attr in dir(args):
            # Ignore non-argument attributes.
            if not attr.startswith('_'):
                self._config_file_args[attr] = getattr(args, attr)
        self._config.update(self._config_file_args)

    def _run_post_processors(self):
        for attr, callbacks in self._post_processors.items():
            newval = self._lookup_val(attr)
            for callback in callbacks:
                newval = callback(newval)
            if newval is not None:
                newval = newval[0]
            self._set(attr, newval)


    def _lookup_val(self, attr):
        '''
        Get the attribute from the config or fallback to defaults.

        :returns: If the value is not stored return None. Otherwise a tuple
            containing the value.
        '''
        if attr in self._config:
            return (self._config[attr],)
        elif hasattr(self._defaults, attr):
            return (getattr(self._defaults, attr),)

    def __getattr__(self, attr):
        if attr in dir(super(_Config, self)):
            return getattr(super(_Config, self), attr)
        elif not self._initialized:
            raise UninitializedConfigException(
                'Cannot directly access elements from the config before it is'
                ' initialized')
        else:
            val = self._lookup_val(attr)
            if val is not None:
                return val[0]
            else:
                raise UninitialzedAttributeException(
                    '%s was not initialzed in the config.' % attr)

    def get_tags(self):
        d = {typ: set(self.__getattr__(typ))
            for typ in self.constants.supported_tags}
        if any(map(lambda vals: bool(vals), d.values())):
            return d
        else:
            return {}

def define_defaults(defaults):
    '''
    Defaults are provided by the config if the attribute is not found in the
    config or commandline. For instance, if we are using the list command
    fixtures might not be able to count on the build_dir being provided since
    we aren't going to build anything.
    '''
    defaults.base_dir = os.path.abspath(os.path.join(absdirpath(__file__),
                                                      os.pardir,
                                                      os.pardir))
    defaults.result_path = os.path.join(os.getcwd(), 'testing-results')
    defaults.resource_url = 'http://dist.gem5.org/dist/develop'
    defaults.resource_path = os.path.abspath(os.path.join(defaults.base_dir,
                                            'tests',
                                            'gem5',
                                            'resources'))

def define_constants(constants):
    '''
    'constants' are values not directly exposed by the config, but are attached
    to the object for centralized access. These should be used for setting
    common string names used across the test framework. A simple typo in
    a string can take a lot of debugging to uncover the issue, attribute errors
    are easier to notice and most autocompletion systems detect them.
    '''
    constants.system_out_name = 'system-out'
    constants.system_err_name = 'system-err'

    constants.isa_tag_type = 'isa'
    constants.x86_tag = 'X86'
    constants.gcn3_x86_tag = 'GCN3_X86'
    constants.vega_x86_tag = 'VEGA_X86'
    constants.sparc_tag = 'SPARC'
    constants.riscv_tag = 'RISCV'
    constants.arm_tag = 'ARM'
    constants.mips_tag = 'MIPS'
    constants.power_tag = 'POWER'
    constants.null_tag = 'NULL'
    constants.all_compiled_tag = 'ALL'

    constants.variant_tag_type = 'variant'
    constants.opt_tag = 'opt'
    constants.debug_tag = 'debug'
    constants.fast_tag = 'fast'

    constants.length_tag_type = 'length'
    constants.quick_tag = 'quick'
    constants.long_tag = 'long'
    constants.very_long_tag = 'very-long'

    constants.host_isa_tag_type = 'host'
    constants.host_x86_64_tag = 'x86_64'
    constants.host_arm_tag = 'aarch64'

    constants.kvm_tag = 'kvm'

    constants.supported_tags = {
        constants.isa_tag_type : (
            constants.x86_tag,
            constants.gcn3_x86_tag,
            constants.vega_x86_tag,
            constants.sparc_tag,
            constants.riscv_tag,
            constants.arm_tag,
            constants.mips_tag,
            constants.power_tag,
            constants.null_tag,
            constants.all_compiled_tag,
            ),
        constants.variant_tag_type: (
            constants.opt_tag,
            constants.debug_tag,
            constants.fast_tag,
        ),
        constants.length_tag_type: (
            constants.quick_tag,
            constants.long_tag,
            constants.very_long_tag,
        ),
        constants.host_isa_tag_type: (
            constants.host_x86_64_tag,
            constants.host_arm_tag,
        ),
    }

    # Binding target ISA with host ISA. This is useful for the
    # case where host ISA and target ISA need to coincide
    constants.target_host = {
        constants.arm_tag   : (constants.host_arm_tag,),
        constants.x86_tag   : (constants.host_x86_64_tag,),
        constants.gcn3_x86_tag : (constants.host_x86_64_tag,),
        constants.vega_x86_tag : (constants.host_x86_64_tag,),
        constants.sparc_tag : (constants.host_x86_64_tag,),
        constants.riscv_tag : (constants.host_x86_64_tag,),
        constants.mips_tag  : (constants.host_x86_64_tag,),
        constants.power_tag : (constants.host_x86_64_tag,),
        constants.null_tag  : (None,),
        constants.all_compiled_tag: (None,),
    }

    constants.supported_isas = constants.supported_tags['isa']
    constants.supported_variants = constants.supported_tags['variant']
    constants.supported_lengths = constants.supported_tags['length']
    constants.supported_hosts = constants.supported_tags['host']

    constants.tempdir_fixture_name = 'tempdir'
    constants.gem5_simulation_stderr = 'simerr'
    constants.gem5_simulation_stdout = 'simout'
    constants.gem5_simulation_stats = 'stats.txt'
    constants.gem5_simulation_config_ini = 'config.ini'
    constants.gem5_simulation_config_json = 'config.json'
    constants.gem5_returncode_fixture_name = 'gem5-returncode'
    constants.gem5_binary_fixture_name = 'gem5'
    constants.xml_filename = 'results.xml'
    constants.pickle_filename = 'results.pickle'
    constants.pickle_protocol = highest_pickle_protocol

    # The root directory which all test names will be based off of.
    constants.testing_base = absdirpath(os.path.join(absdirpath(__file__),
                                                     os.pardir))

def define_post_processors(config):
    '''
    post_processors are used to do final configuration of variables. This is
    useful if there is a dynamically set default, or some function that needs
    to be applied after parsing in order to set a configration value.

    Post processors must accept a single argument that will either be a tuple
    containing the already set config value or ``None`` if the config value
    has not been set to anything. They must return the modified value in the
    same format.
    '''

    def set_default_build_dir(build_dir):
        '''
        Post-processor to set the default build_dir based on the base_dir.

        .. seealso :func:`~_Config._add_post_processor`
        '''
        if not build_dir or build_dir[0] is None:
            base_dir = config._lookup_val('base_dir')[0]
            build_dir = (os.path.join(base_dir, 'build'),)
        return build_dir

    def fix_verbosity_hack(verbose):
        return (verbose[0].val,)

    def threads_as_int(threads):
        if threads is not None:
            return (int(threads[0]),)

    def test_threads_as_int(test_threads):
        if test_threads is not None:
            return (int(test_threads[0]),)

    def default_isa(isa):
        if not isa[0]:
            return [constants.supported_tags[constants.isa_tag_type]]
        else:
            return isa

    def default_variant(variant):
        if not variant[0]:
            # Default variant is only opt. No need to run tests with multiple
            # different compilation targets
            return [[constants.opt_tag]]
        else:
            return variant

    def default_length(length):
        if not length[0]:
            return [[constants.quick_tag]]
        else:
            return length

    def default_host(host):
        if not host[0]:
            try:
                import platform
                host_machine = platform.machine()
                if host_machine not in constants.supported_hosts:
                    raise ValueError("Invalid host machine")
                return [[host_machine]]
            except:
                return [[constants.host_x86_64_tag]]
        else:
            return host

    def compile_tag_regex(positional_tags):
        if not positional_tags:
            return positional_tags
        else:
            new_positional_tags_list = []
            positional_tags = positional_tags[0]

            for flag, regex in positional_tags:
                if flag == 'exclude_tags':
                    tag_regex = TagRegex(False, regex)
                elif flag  == 'include_tags':
                    tag_regex = TagRegex(True, regex)
                else:
                    raise ValueError('Unsupported flag.')
                new_positional_tags_list.append(tag_regex)

            return (new_positional_tags_list,)

    config._add_post_processor('build_dir', set_default_build_dir)
    config._add_post_processor('verbose', fix_verbosity_hack)
    config._add_post_processor('isa', default_isa)
    config._add_post_processor('variant', default_variant)
    config._add_post_processor('length', default_length)
    config._add_post_processor('host', default_host)
    config._add_post_processor('threads', threads_as_int)
    config._add_post_processor('test_threads', test_threads_as_int)
    config._add_post_processor(StorePositionalTagsAction.position_kword,
                               compile_tag_regex)
class Argument(object):
    '''
    Class represents a cli argument/flag for a argparse parser.

    :attr name: The long name of this object that will be stored in the arg
        output by the final parser.
    '''
    def __init__(self, *flags, **kwargs):
        self.flags = flags
        self.kwargs = kwargs

        if len(flags) == 0:
            raise ValueError("Need at least one argument.")
        elif 'dest' in kwargs:
            self.name = kwargs['dest']
        elif len(flags) > 1 or flags[0].startswith('-'):
            for flag in flags:
                if not flag.startswith('-'):
                    raise ValueError("invalid option string %s: must start"
                    "with a character '-'" % flag)

                if flag.startswith('--'):
                    if not hasattr(self, 'name'):
                        self.name = flag.lstrip('-')

        if not hasattr(self, 'name'):
            self.name = flags[0].lstrip('-')
        self.name = self.name.replace('-', '_')

    def add_to(self, parser):
        '''Add this argument to the given parser.'''
        parser.add_argument(*self.flags, **self.kwargs)

    def copy(self):
        '''Copy this argument so you might modify any of its kwargs.'''
        return copy.deepcopy(self)


class _StickyInt:
    '''
    A class that is used to cheat the verbosity count incrementer by
    pretending to be an int. This makes the int stay on the heap and eat other
    real numbers when they are added to it.

    We use this so we can allow the verbose flag to be provided before or after
    the subcommand. This likely has no utility outside of this use case.
    '''
    def __init__(self, val=0):
        self.val = val
        self.type = int
    def __add__(self, other):
        self.val += other
        return self

common_args = NotImplemented

class StorePositionAction(argparse.Action):
    '''Base class for classes wishing to create namespaces where
    arguments are stored in the order provided via the command line.
    '''
    position_kword = 'positional'

    def __call__(self, parser, namespace, values, option_string=None):
        if not self.position_kword in namespace:
            setattr(namespace, self.position_kword, [])
        previous = getattr(namespace, self.position_kword)
        previous.append((self.dest, values))
        setattr(namespace, self.position_kword, previous)

class StorePositionalTagsAction(StorePositionAction):
    position_kword = 'tag_filters'

def define_common_args(config):
    '''
    Common args are arguments which are likely to be simular between different
    subcommands, so they are available to all by placing their definitions
    here.
    '''
    global common_args

    parse_comma_separated_string = lambda st: st.split(',')

    # A list of common arguments/flags used across cli parsers.
    common_args = [
        Argument(
            'directories',
            nargs='*',
            default=[os.getcwd()],
            help='Space separated list of directories to start searching '
                 'for tests in'),
        Argument(
            '--exclude-tags',
            action=StorePositionalTagsAction,
            help='A tag comparison used to select tests.'),
        Argument(
            '--include-tags',
            action=StorePositionalTagsAction,
            help='A tag comparison used to select tests.'),
        Argument(
            '--isa',
            action='extend',
            default=[],
            type=parse_comma_separated_string,
            help="Only tests that are valid with one of these ISAs. "
                 "Comma separated."),
        Argument(
            '--variant',
            action='extend',
            default=[],
            type=parse_comma_separated_string,
            help="Only tests that are valid with one of these binary variants"
                 "(e.g., opt, debug). Comma separated."),
        Argument(
            '--length',
            action='extend',
            default=[],
            type=parse_comma_separated_string,
            help="Only tests that are one of these lengths. Comma separated."),
        Argument(
            '--host',
            action='append',
            default=[],
            help="Only tests that are meant to runnable on the selected host"),
        Argument(
            '--uid',
            action='store',
            default=None,
            help='UID of a specific test item to run.'),
        Argument(
            '--build-dir',
            action='store',
            help='Build directory for SCons'),
        Argument(
            '--base-dir',
            action='store',
            default=config._defaults.base_dir,
            help='Directory to change to in order to exec scons.'),
        Argument(
            '-j', '--threads',
            action='store',
            default=1,
            help='Number of threads to run SCons with.'),
        Argument(
            '-t', '--test-threads',
            action='store',
            default=1,
            help='Number of threads to spawn to run concurrent tests with.'),
        Argument(
            '-v',
            action='count',
            dest='verbose',
            default=_StickyInt(),
            help='Increase verbosity'),
        Argument(
            '--config-path',
            action='store',
            default=os.getcwd(),
            help='Path to read a testing.ini config in'
        ),
        Argument(
            '--skip-build',
            action='store_true',
            default=False,
            help='Skip the building component of SCons targets.'
        ),
        Argument(
            '--result-path',
            action='store',
            help='The path to store results in.'
        ),
        Argument(
            '--bin-path',
            action='store',
            default=config._defaults.resource_path,
            help='Path where resources are stored (downloaded if not present)'
        ),
        Argument(
            '--resource-url',
            action='store',
            default=config._defaults.resource_url,
            help='The URL where the resources reside.'
        ),

    ]

    # NOTE: There is a limitation which arises due to this format. If you have
    # multiple arguments with the same name only the final one in the list
    # will be saved.
    #
    # e.g. if you have a -v argument which increments verbosity level and
    # a separate --verbose flag which 'store's verbosity level. the final
    # one in the list will be saved.
    common_args = AttrDict({arg.name:arg for arg in common_args})

class ArgParser(object, metaclass=abc.ABCMeta):
    class ExtendAction(argparse.Action):
        def __call__(self, parser, namespace, values, option_string=None):
            items = getattr(namespace, self.dest, [])
            items.extend(values)
            setattr(namespace, self.dest, items)

    def __init__(self, parser):
        # Copy public methods of the parser.
        for attr in dir(parser):
            if not attr.startswith('_'):
                setattr(self, attr, getattr(parser, attr))
        self.parser = parser
        self.parser.register('action', 'extend', ArgParser.ExtendAction)
        self.add_argument = self.parser.add_argument

        # Argument will be added to all parsers and subparsers.
        common_args.verbose.add_to(parser)


class CommandParser(ArgParser):
    '''
    Main parser which parses command strings and uses those to direct to
    a subparser.
    '''
    def __init__(self):
        parser = argparse.ArgumentParser()
        super(CommandParser, self).__init__(parser)
        self.subparser = self.add_subparsers(dest='command')


class RunParser(ArgParser):
    '''
    Parser for the \'run\' command.
    '''
    def __init__(self, subparser):
        parser = subparser.add_parser(
            'run',
            help='''Run Tests.'''
        )

        super(RunParser, self).__init__(parser)

        common_args.uid.add_to(parser)
        common_args.skip_build.add_to(parser)
        common_args.directories.add_to(parser)
        common_args.build_dir.add_to(parser)
        common_args.base_dir.add_to(parser)
        common_args.bin_path.add_to(parser)
        common_args.threads.add_to(parser)
        common_args.test_threads.add_to(parser)
        common_args.isa.add_to(parser)
        common_args.variant.add_to(parser)
        common_args.length.add_to(parser)
        common_args.host.add_to(parser)
        common_args.include_tags.add_to(parser)
        common_args.exclude_tags.add_to(parser)


class ListParser(ArgParser):
    '''
    Parser for the \'list\' command.
    '''
    def __init__(self, subparser):
        parser = subparser.add_parser(
            'list',
            help='''List and query test metadata.'''
        )
        super(ListParser, self).__init__(parser)

        Argument(
            '--suites',
            action='store_true',
            default=False,
            help='List all test suites.'
        ).add_to(parser)
        Argument(
            '--tests',
            action='store_true',
            default=False,
            help='List all test cases.'
        ).add_to(parser)
        Argument(
            '--fixtures',
            action='store_true',
            default=False,
            help='List all fixtures.'
        ).add_to(parser)
        Argument(
            '--all-tags',
            action='store_true',
            default=False,
            help='List all tags.'
        ).add_to(parser)
        Argument(
            '-q',
            dest='quiet',
            action='store_true',
            default=False,
            help='Quiet output (machine readable).'
        ).add_to(parser)

        common_args.directories.add_to(parser)
        common_args.bin_path.add_to(parser)
        common_args.isa.add_to(parser)
        common_args.variant.add_to(parser)
        common_args.length.add_to(parser)
        common_args.host.add_to(parser)
        common_args.include_tags.add_to(parser)
        common_args.exclude_tags.add_to(parser)


class RerunParser(ArgParser):
    def __init__(self, subparser):
        parser = subparser.add_parser(
            'rerun',
            help='''Rerun failed tests.'''
        )
        super(RerunParser, self).__init__(parser)

        common_args.skip_build.add_to(parser)
        common_args.directories.add_to(parser)
        common_args.build_dir.add_to(parser)
        common_args.base_dir.add_to(parser)
        common_args.bin_path.add_to(parser)
        common_args.threads.add_to(parser)
        common_args.test_threads.add_to(parser)
        common_args.isa.add_to(parser)
        common_args.variant.add_to(parser)
        common_args.length.add_to(parser)
        common_args.host.add_to(parser)

config = _Config()
define_constants(config.constants)

# Constants are directly exposed and available once this module is created.
# All constants MUST be defined before this point.
config.constants = FrozenAttrDict(config.constants.__dict__)
constants = config.constants

'''
This config object is the singleton config object available throughout the
framework.
'''
def initialize_config():
    '''
    Parse the commandline arguments and setup the config varibles.
    '''
    global config

    # Setup constants and defaults
    define_defaults(config._defaults)
    define_post_processors(config)
    define_common_args(config)

    # Setup parser and subcommands
    baseparser = CommandParser()
    runparser = RunParser(baseparser.subparser)
    listparser = ListParser(baseparser.subparser)
    rerunparser = RerunParser(baseparser.subparser)

    # Initialize the config by parsing args and running callbacks.
    config._init(baseparser)
