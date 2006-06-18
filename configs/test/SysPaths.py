import os, sys
from os.path import isdir, join as joinpath
from os import environ as env

systemdir = None
bindir = None
diskdir = None
scriptdir = None

def load_defaults():
    global systemdir, bindir, diskdir, scriptdir
    if not systemdir:
        try:
                path = env['M5_PATH'].split(':')
        except KeyError:
                path = [ '/dist/m5/system', '/n/poolfs/z/dist/m5/system' ]

        for systemdir in path:
            if os.path.isdir(systemdir):
                break
        else:
            raise ImportError, "Can't find a path to system files."

    if not bindir:
        bindir = joinpath(systemdir, 'binaries')
    if not diskdir:
        diskdir = joinpath(systemdir, 'disks')
    if not scriptdir:
        scriptdir = joinpath(systemdir, 'boot')

def disk(file):
    load_defaults()
    return joinpath(diskdir, file)

def binary(file):
    load_defaults()
    return joinpath(bindir, file)

def script(file):
    load_defaults()
    return joinpath(scriptdir, file)

