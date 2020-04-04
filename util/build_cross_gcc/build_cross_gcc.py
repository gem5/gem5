#! /usr/bin/env python
# Copyright 2020 Google, Inc.
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

import abc
import argparse
import glob
import multiprocessing
import os
import os.path
import pickle
import shutil
import six
import subprocess
import textwrap

SETTINGS_FILE = '.build_cross_gcc.settings'
LOG_FILE = 'build_cross_gcc.log'

all_settings = {}
all_steps = {}

description_paragraphs = [
        '''
        This script helps automate building a gcc based cross compiler.
        The process is broken down into a series of steps which can be
        executed one at a time or in arbtitrary sequences. It's assumed that
        you've already downloaded the following sources into the current
        directory:''',
        '',
        '''1. binutils''',
        '''2. gcc''',
        '''3. glibc''',
        '''4. linux kernel''',
        '''5. gdb''',
        '',
        '''
        The entire process can be configured with a series of settings
        which are stored in a config file called {settings_file}. These
        settings can generally also be set from the command line, and at run
        time using step 0 of the process. Many will set themselves to
        reasonable defaults if no value was loaded from a previous
        configuration or a saved settings file.''',
        '',
        '''
        Prebaked config options can be loaded in from an external file to
        make it easier to build particular cross compilers without having to
        mess with a lot of options.'''
        '',
        '''
        When settings are listed, any setting which has a value which has
        failed validation or which hasn't been set and doesn't have a
        reasonable default will be marked with a X in the far left hand
        column. Settings will generally refuse to be set to invalid values,
        unless they were like that by default and the user refused to correct
        them.''',
        '',
        '''This script is based on the excellent how-to here:''',
        '''https://preshing.com/20141119/how-to-build-a-gcc-cross-compiler/''',
        '',
        '''
        Please view that webpage for a detailed explanation of what this
        script does.'''
        ]

def help_text_wrapper(text):
    width = shutil.get_terminal_size().columns
    text = textwrap.dedent(text)
    text = text.strip()
    return textwrap.fill(text, width=width)

description = '\n'.join(list(map(help_text_wrapper, description_paragraphs)))

argparser = argparse.ArgumentParser(
        formatter_class=argparse.RawDescriptionHelpFormatter,
        description=description)


#
# Some helper utilities.
#

def confirm(prompt):
    while True:
        yn = input('{} (N/y): '.format(prompt))
        if yn == '':
            yn = 'n'
        if yn.lower() in ('y', 'Yes'):
            return True
        elif yn.lower() in ('n', 'No'):
            return False


def setup_build_dir(subdir):
    build_dir_base = BuildDirBase.setting()
    target = Target.setting()
    if not (build_dir_base.valid and target.valid):
        return False
    target_build_dir = os.path.join(build_dir_base.get(), target.get())
    build_dir = os.path.join(target_build_dir, 'build-{}'.format(subdir))
    if not os.path.isdir(build_dir):
        os.makedirs(build_dir)
    return build_dir

def run_commands(working_dir, *cmds):
    with open(LOG_FILE, 'a') as log:
        print('In working directory {:s} (log in {:s}):'.format(
            working_dir, LOG_FILE))
        for cmd in cmds:
            print(textwrap.fill(cmd, initial_indent='  ',
                                subsequent_indent='    ',
                                width=shutil.get_terminal_size().columns))
            print('', file=log)
            print(cmd, file=log)
            print('', file=log)
            if subprocess.call(cmd, shell=True, cwd=working_dir,
                               stdout=log, stderr=subprocess.STDOUT) != 0:
                return False
        return True


#
# Settings.
#

class MetaSetting(type):
    def __new__(mcls, name, bases, d):
        cls = super(MetaSetting, mcls).__new__(mcls, name, bases, d)
        key = d.get('key', None)
        if key is not None:
            assert('default' in d)
            instance = cls()
            instance.value = None
            instance.valid = False
            all_settings[key] = instance
        return cls

@six.add_metaclass(MetaSetting)
@six.add_metaclass(abc.ABCMeta)
class Setting(object):
    key = None

    @abc.abstractmethod
    def set(self, value):
        'Validate and set the setting to "value", and return if successful.'
        self.value = value
        self.valid = True
        return True

    def set_default(self):
        'Set this setting to its default value, and return if successful.'
        return self.set(self.default)

    def set_arg(self, value):
        'Set this setting to value if not None, and return if successful.'
        if value:
            return self.set(value)
        else:
            # Nothing happened, so nothing failed.
            return True

    def get(self):
        'Return the value of this setting.'
        return self.value

    @abc.abstractmethod
    def describe(self):
        'Return a string describing this setting.'
        return ''

    @abc.abstractmethod
    def add_to_argparser(self, argparser):
        'Add command line options associated with this setting.'

    @abc.abstractmethod
    def set_from_args(self, args):
        'Set this setting from the command line arguments, if requested.'
        return True

    @classmethod
    def setting(cls):
        s = all_settings[cls.key]
        if not s.valid:
            print('"{}" is not valid.'.format(s.key))
        return s

class DirectorySetting(Setting):
    def set(self, value):
        if not os.path.exists(value):
            print('Path "{:s}" does not exist.'.format(value))
        elif not os.path.isdir(value):
            print('Path "{:s}" is not a directory.'.format(value))
        else:
            self.value = value
            self.valid = True
        return self.valid

    def set_default(self):
        if not self.set(self.default):
            if not os.path.exists(self.default):
                if confirm('Create?'):
                    try:
                        os.mkdirs(value)
                        assert(self.set(self.default))
                    except:
                        print('Failed to make directory')
                        self.valid = False
                        return False
                else:
                    self.value = self.default
                    self.valid = False
                    return False

class Prefix(DirectorySetting):
    default = os.path.join(os.environ['HOME'], 'cross')
    key = 'PREFIX'

    def describe(self):
        return 'Path prefix to install to.'

    def add_to_argparser(self, parser):
        parser.add_argument('--prefix', help=self.describe())

    def set_from_args(self, args):
        return self.set_arg(args.prefix)

class BuildDirBase(DirectorySetting):
    default = os.getcwd()
    key = 'BUILD_DIR_BASE'

    def describe(self):
        return 'Path prefix for build directory(ies).'

    def add_to_argparser(self, parser):
        parser.add_argument('--build-dir-base', help=self.describe())

    def set_from_args(self, args):
        return self.set_arg(args.build_dir_base)

class Target(Setting):
    key = 'TARGET'
    default = None

    def set_default(self):
        self.value = '(not set)'
        self.valid = False
        return False

    def describe(self):
        return 'Tuple for the target architecture.'

    def add_to_argparser(self, parser):
        parser.add_argument('--target', help=self.describe())

    def set_from_args(self, args):
        return self.set_arg(args.target)

class LinuxArch(Setting):
    key = 'LINUX_ARCH'
    default = None

    def set_default(self):
        self.value = '(not set)'
        self.valid = False
        return False

    def describe(self):
        return 'The arch directory for Linux headers.'

    def add_to_argparser(self, parser):
        parser.add_argument('--linux-arch', help=self.describe())

    def set_from_args(self, args):
        return self.set_arg(args.linux_arch)

class SourceDirSetting(Setting):
    def set(self, value):
        if os.path.isdir(value):
            self.value = value
            self.valid = True
        return self.valid

    def set_default(self):
        matches = list(filter(os.path.isdir, glob.glob(self.pattern)))
        if len(matches) == 0:
            self.valid = False
            return False
        if len(matches) > 1:
            while True:
                print()
                print('Multple options for "{:s}":'.format(self.key))
                choices = list(enumerate(matches))
                for number, value in choices:
                    print('{:>5}: {:s}'.format(number, value))
                choice = input('Which one? ')
                try:
                    choice = choices[int(choice)][1]
                except:
                    print('Don\'t know what to do with "{:s}".'.format(choice))
                    continue
                return self.set(choice)
        return self.set(matches[0])

    def describe(self):
        return 'Directory with the extracted {} source.'.format(self.project)

class BinutilsSourceDir(SourceDirSetting):
    key = 'BINUTILS_SRC_DIR'
    default = None
    pattern = 'binutils-*'
    project = 'binutils'

    def add_to_argparser(self, parser):
        parser.add_argument('--binutils-src', help=self.describe())

    def set_from_args(self, args):
        return self.set_arg(args.binutils_src)

class GccSourceDir(SourceDirSetting):
    key = 'GCC_SRC_DIR'
    default = None
    pattern = 'gcc-*'
    project = 'gcc'

    def add_to_argparser(self, parser):
        parser.add_argument('--gcc-src', help=self.describe())

    def set_from_args(self, args):
        return self.set_arg(args.gcc_src)

class GlibcSourceDir(SourceDirSetting):
    key = 'GLIBC_SRC_DIR'
    default = None
    pattern = 'glibc-*'
    project = 'glibc'

    def add_to_argparser(self, parser):
        parser.add_argument('--glibc-src', help=self.describe())

    def set_from_args(self, args):
        return self.set_arg(args.glibc_src)

class LinuxSourceDir(SourceDirSetting):
    key = 'LINUX_SRC_DIR'
    default = None
    pattern = 'linux-*'
    project = 'linux'

    def add_to_argparser(self, parser):
        parser.add_argument('--linux-src', help=self.describe())

    def set_from_args(self, args):
        return self.set_arg(args.linux_src)

class GdbSourceDir(SourceDirSetting):
    key = 'GDB_SRC_DIR'
    default = None
    pattern = 'gdb-*'
    project = 'gdb'

    def add_to_argparser(self, parser):
        parser.add_argument('--gdb-src', help=self.describe())

    def set_from_args(self, args):
        return self.set_arg(args.gdb_src)

class Parallelism(Setting):
    key = 'J'
    default = None

    def set(self, value):
        try:
            value = int(value)
        except:
            print('Can\'t convert "{:s}" into an integer.'.format(value))
        if value < 0:
            print('Parallelism can\'t be negative.')
            return False
        self.value = value
        self.valid = True
        return self.valid

    def set_default(self):
        self.set(multiprocessing.cpu_count())

    def describe(self):
        return 'The level of parellism to request from "make".'

    def add_to_argparser(self, parser):
        parser.add_argument('-j', help=self.describe())

    def set_from_args(self, args):
        return self.set_arg(args.j)



#
# Steps of the build process.
#

class MetaStep(type):
    def __new__(mcls, name, bases, d):
        cls = super(MetaStep, mcls).__new__(mcls, name, bases, d)
        number = d.get('number', None)
        if number is not None:
            all_steps[number] = cls()
        return cls

@six.add_metaclass(MetaStep)
@six.add_metaclass(abc.ABCMeta)
class Step(object):
    'Steps to set up a cross compiling gcc.'
    number = None

    @abc.abstractmethod
    def run(self):
        'Execute this step.'
        pass

    @abc.abstractmethod
    def describe(self):
        'Return a string describing this step.'
        return ''


class Configure(Step):
    number = 0

    def describe(self):
        return 'Adjust settings.'

    def get_setting(self):
        settings = list(enumerate(all_settings.items()))
        all_keys = list(all_settings.keys())
        max_key_length = max([len(key) for key in all_keys])
        while True:
            for number, (key, setting) in settings:
                print('{}{:>4}: {:{key_len}s} - {:s}'.format(
                    ' ' if setting.valid else 'X',
                    number, key, setting.describe(), key_len=max_key_length))
                print('      {}'.format(setting.value))
            print()
            key = input('Value to modify, or "done": ')
            if key == "done":
                save_settings()
                return None
            if key not in all_keys:
                try:
                    key = settings[int(key)][1][0]
                except:
                    print('Don\'t know what to do with "{:s}."'.format(key))
                    continue
            return all_settings[key]

    def run(self):
        while True:
            setting = self.get_setting()
            if not setting:
                return True

            new_value = input('New value ({:s}): '.format(setting.get()))
            if new_value:
                setting.set(new_value)
                save_settings()

        print_settings()
        return True

class BuildBinutils(Step):
    number = 1

    def describe(self):
        return 'Build binutils.'

    def run(self):
        prefix = Prefix.setting()
        target = Target.setting()
        j = Parallelism.setting()
        source_dir = BinutilsSourceDir.setting()
        build_dir = setup_build_dir('binutils')

        if not all((prefix, target, j, source_dir, build_dir)):
            return False

        prefix = prefix.get()
        target = target.get()
        j = j.get()
        build_dir = os.path.abspath(build_dir)
        source_dir = os.path.abspath(source_dir.get())

        return run_commands(build_dir,
                '{configure} --prefix={prefix} --target={target} '
                '--disable-multilib'.format(
                    configure=os.path.join(source_dir, 'configure'),
                    prefix=prefix, target=target),
                'make -j{j}'.format(j=j),
                'make install'
                )

class InstallLinuxHeaders(Step):
    number = 2

    def describe(self):
        return 'Install Linux headers.'

    def run(self):
        source_dir = LinuxSourceDir.setting()
        linux_arch = LinuxArch.setting()
        prefix = Prefix.setting()
        target = Target.setting()

        if not all((source_dir, linux_arch, prefix, target)):
            return False

        source_dir = os.path.abspath(source_dir.get())
        linux_arch = linux_arch.get()
        prefix = os.path.abspath(prefix.get())
        target = target.get()

        hdr_path = os.path.join(prefix, target)

        return run_commands(source_dir,
                'make ARCH={arch} INSTALL_HDR_PATH={hdr_path} '
                'headers_install'.format(arch=linux_arch, hdr_path=hdr_path))

class Compilers(Step):
    number = 3

    def describe(self):
        return 'Build C and C++ compilers.'

    def run(self):
        prefix = Prefix.setting()
        target = Target.setting()
        j = Parallelism.setting()
        source_dir = GccSourceDir.setting()
        build_dir = setup_build_dir('gcc')

        if not all((prefix, target, j, source_dir, build_dir)):
            return False

        prefix = prefix.get()
        target = target.get()
        j = j.get()
        build_dir = os.path.abspath(build_dir)
        source_dir = os.path.abspath(source_dir.get())

        return run_commands(build_dir,
                '{configure} --prefix={prefix} --target={target} '
                '--enable-languages=c,c++ --disable-multilib'.format(
                    configure=os.path.join(source_dir, 'configure'),
                    prefix=prefix, target=target),
                'make -j{j} all-gcc LIMITS_H_TEST=true'.format(j=j),
                'make install-gcc'
                )

class CHeaders(Step):
    number = 4

    def describe(self):
        return 'Standard C library headers and startup files.'

    def run(self):
        prefix = Prefix.setting()
        target = Target.setting()
        j = Parallelism.setting()
        source_dir = GlibcSourceDir.setting()
        build_dir = setup_build_dir('glibc')

        if not all((prefix, target, j, source_dir, build_dir)):
            return False

        prefix = prefix.get()
        target = target.get()
        j = j.get()
        source_dir = os.path.abspath(source_dir.get())
        build_dir = os.path.abspath(build_dir)

        return run_commands(build_dir,
                '{configure} --prefix={prefix} --build=$MACHTYPE '
                '--host={host} --target={target} --with-headers={hdr_path} '
                '--disable-multilib libc_cv_forced_unwind=yes'.format(
                    configure=os.path.join(source_dir, 'configure'),
                    prefix=os.path.join(prefix, target),
                    host=target, target=target,
                    hdr_path=os.path.join(prefix, target, 'include')),
                'make install-bootstrap-headers=yes install-headers',
                'make -j{j} csu/subdir_lib'.format(j=j),
                'install csu/crt1.o csu/crti.o csu/crtn.o {lib_path}'.format(
                    lib_path=os.path.join(prefix, target, 'lib')),
                '{target}-gcc -nostdlib -nostartfiles -shared -x c /dev/null '
                '-o {libc_so}'.format(target=target,
                    libc_so=os.path.join(prefix, target, 'lib', 'libc.so')),
                'touch {stubs_h}'.format(stubs_h=os.path.join(
                    prefix, target, 'include', 'gnu', 'stubs.h'))
                )

class CompilerSupportLib(Step):
    number = 5

    def describe(self):
        return 'Build the compiler support library.'

    def run(self):
        j = Parallelism.setting()
        build_dir = setup_build_dir('gcc')

        if not all((j, build_dir)):
            return False

        j = j.get()
        build_dir = os.path.abspath(build_dir)

        return run_commands(build_dir,
            'make -j{j} all-target-libgcc'.format(j=j),
            'make install-target-libgcc'
            )

class StandardCLib(Step):
    number = 6

    def describe(self):
        return 'Install the standard C library.'

    def run(self):
        j = Parallelism.setting()
        build_dir = setup_build_dir('glibc')

        if not all((j, build_dir)):
            return False

        j = j.get()
        build_dir = os.path.abspath(build_dir)

        return run_commands(build_dir,
                'make -j{j}'.format(j=j),
                'make install',
                )

class BuildGdb(Step):
    number = 7

    def describe(self):
        return 'Build GDB.'

    def run(self):
        prefix = Prefix.setting()
        target = Target.setting()
        j = Parallelism.setting()
        source_dir = GdbSourceDir.setting()
        build_dir = setup_build_dir('gdb')

        if not all((prefix, target, j, source_dir, build_dir)):
            return False

        prefix = prefix.get()
        target = target.get()
        j = j.get()
        source_dir = os.path.abspath(source_dir.get())
        build_dir = os.path.abspath(build_dir)

        return run_commands(build_dir,
                '{configure} --prefix={prefix} --target={target} '
                '$MACHTYPE'.format(prefix=prefix, target=target,
                    configure=os.path.join(source_dir, 'configure')),
                'make -j{j}'.format(j=j),
                'make install'
                )

class StandardCxxLib(Step):
    number = 8

    def describe(self):
        return 'Install the standard C++ library.'

    def run(self):
        j = Parallelism.setting()
        build_dir = setup_build_dir('gcc')

        if not all((j, build_dir)):
            return False

        j = j.get()
        build_dir = os.path.abspath(build_dir)

        return run_commands(build_dir,
                'make -j{j}'.format(j=j),
                'make install'
                )


#
# The engine that makes it all go.
#

def get_steps():
    while True:
        print()
        print('Steps:')
        for _, step in sorted(all_steps.items()):
            print('{:>5} {:s}'.format(
                '{:d}:'.format(step.number), step.describe()))
        print()
        steps = input('Comma separated list of steps, or '
                      '"exit", or "all" (all): ')
        if not steps:
            steps = 'all'
        if steps == 'exit':
            return []
        if steps == 'all':
            keys = list([str(key) for key in all_steps.keys()])
            steps = ','.join(keys)
        try:
            return list([all_steps[int(i)] for i in steps.split(",")])
        except:
            print('Don\'t know what to do with "{:s}"'.format(steps))

def print_settings():
    print()
    print('Settings:')
    for setting in all_settings.values():
        print('{}    {} = {}'.format(
            ' ' if setting.valid else 'X', setting.key, setting.value))

def save_settings():
    settings = {}
    for setting in all_settings.values():
        if setting.valid:
            settings[setting.key] = setting.get()
    with open(SETTINGS_FILE, 'wb') as settings_file:
        pickle.dump(settings, settings_file)

def load_settings():
    if os.path.exists(SETTINGS_FILE):
        with open(SETTINGS_FILE, 'rb') as settings_file:
            settings = pickle.load(settings_file)
    else:
        settings = {}

    for setting in all_settings.values():
        if setting.key in settings:
            setting.set(settings[setting.key])

def load_settings_file(path):
    with open(path, 'r') as settings:
        for line in settings.readlines():
            if not line:
                continue
            try:
                key, val = line.split('=')
            except:
                print('Malformated line "{}" in settings file "{}".'.format(
                    line, path))
                return False
            key = key.strip()
            val = val.strip()
            if key not in all_settings:
                print('Unknown setting "{}" found in settings '
                      'file "{}".'.format(key, path))
                return False
            setting = all_settings[key]
            if not setting.set(val):
                print('Failed to set "{}" to "{}" from '
                      'settings file "{}".'.format(key, val, path))
                return False
    return True



argparser.add_argument('--settings-file',
        help='A file with name=value settings to load.')

def main():
    # Install command line options for each setting.
    for setting in all_settings.values():
        setting.add_to_argparser(argparser)

    args = argparser.parse_args()

    # Load settings from the last time we ran. Lowest priority.
    load_settings()

    # If requested, read in a settings file. Medium priority.
    if args.settings_file:
        if not load_settings_file(args.settings_file):
            return

    # Set settings based on command line options. Highest priority.
    for setting in all_settings.values():
        setting.set_from_args(args)

    # If a setting is still not valid, try setting it to its default.
    for setting in all_settings.values():
        if not setting.valid:
            setting.set_default()

    # Print out the resulting settings.
    print_settings()

    while True:
        steps = get_steps()
        if not steps:
            return
        for step in steps:
            print()
            print('Step {:d}: {:s}'.format(step.number, step.describe()))
            print()
            if not step.run():
                print()
                print('Step failed, aborting.')
                break

if __name__ == "__main__":
    main()
