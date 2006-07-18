import os, sys
from os.path import isdir, join as joinpath
from os import environ as env

def disk(file):
    system()
    return joinpath(disk.dir, file)

def binary(file):
    system()
    return joinpath(binary.dir, file)

def script(file):
    system()
    return joinpath(script.dir, file)

def system():
    if not system.dir:
        try:
                path = env['M5_PATH'].split(':')
        except KeyError:
                path = [ '/dist/m5/system', '/n/poolfs/z/dist/m5/system' ]

        for system.dir in path:
            if os.path.isdir(system.dir):
                break
        else:
            raise ImportError, "Can't find a path to system files."

    if not binary.dir:
        binary.dir = joinpath(system.dir, 'binaries')
    if not disk.dir:
        disk.dir = joinpath(system.dir, 'disks')
    if not script.dir:
        script.dir = joinpath(system.dir, 'boot')

system.dir = None
binary.dir = None
disk.dir = None
script.dir = None
