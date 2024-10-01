#!/usr/bin/python3
#
# Copyright 2020 Google, Inc.
#
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
# gem5img.py
# Script for managing a gem5 disk image.
#

import os
import re
import string
from argparse import ArgumentParser
from os import environ as env
from subprocess import (
    PIPE,
    STDOUT,
    CalledProcessError,
    Popen,
)
from sys import (
    argv,
    exit,
)

# Some constants.
MaxLBACylinders = 16383
MaxLBAHeads = 16
MaxLBASectors = 63
MaxLBABlocks = MaxLBACylinders * MaxLBAHeads * MaxLBASectors

BlockSize = 512
MiB = 1024 * 1024

# Setup PATH to look in the sbins.
env["PATH"] += ":/sbin:/usr/sbin"

# Whether to print debug output.
debug = False


# Figure out cylinders, heads and sectors from a size in blocks.
def chsFromSize(sizeInBlocks):
    if sizeInBlocks >= MaxLBABlocks:
        sizeInMiBs = (sizeInBlocks * BlockSize) / MiB
        print("%d MiB is too big for LBA, truncating file." % sizeInMiBs)
        return (MaxLBACylinders, MaxLBAHeads, MaxLBASectors)

    sectors = sizeInBlocks
    if sizeInBlocks > 63:
        sectors = 63

    headSize = sizeInBlocks / sectors
    heads = 16
    if headSize < 16:
        heads = sizeInBlocks

    cylinders = sizeInBlocks / (sectors * heads)

    return (cylinders, heads, sectors)


# Figure out if we should use sudo.
def needSudo():
    if not hasattr(needSudo, "notRoot"):
        needSudo.notRoot = os.geteuid() != 0
        if needSudo.notRoot:
            print("You are not root. Using sudo.")
    return needSudo.notRoot


# Run an external command.
def runCommand(command, inputVal=""):
    print("%>", " ".join(command))
    proc = Popen(command, stdin=PIPE)
    proc.communicate(inputVal.encode())
    return proc.returncode


# Run an external command and capture its output. This is intended to be
# used with non-interactive commands where the output is for internal use.
def getOutput(command, inputVal=""):
    global debug
    if debug:
        print("%>", " ".join(command))
    proc = Popen(command, stderr=STDOUT, stdin=PIPE, stdout=PIPE)
    (out, err) = proc.communicate(inputVal)
    return (out.decode(), proc.returncode)


# Run a command as root, using sudo if necessary.
def runPriv(command, inputVal=""):
    realCommand = command
    if needSudo():
        realCommand = [findProg("sudo")] + command
    return runCommand(realCommand, inputVal)


def privOutput(command, inputVal=""):
    realCommand = command
    if needSudo():
        realCommand = [findProg("sudo")] + command
    return getOutput(realCommand, inputVal)


# Find the path to a program.
def findProg(program, cleanupDev=None):
    (out, returncode) = getOutput(["which", program])
    if returncode != 0:
        if cleanupDev:
            cleanupDev.destroy()
        exit(f"Unable to find program {program}, check your PATH variable.")
    return out.strip()


class LoopbackDevice:
    def __init__(self, devFile=None):
        self.devFile = devFile

    def __str__(self):
        return str(self.devFile)

    def setup(self, fileName, offset=False):
        assert not self.devFile
        (out, returncode) = privOutput([findProg("losetup"), "-f"])
        if returncode != 0:
            print(out)
            return returncode
        self.devFile = out.strip()
        command = [findProg("losetup"), self.devFile, fileName]
        if offset:
            off = findPartOffset(self.devFile, fileName, 0)
            command = command[:1] + ["-o", "%d" % off] + command[1:]
        return runPriv(command)

    def destroy(self):
        assert self.devFile
        returncode = runPriv([findProg("losetup"), "-d", self.devFile])
        self.devFile = None
        return returncode


def findPartOffset(devFile, fileName, partition):
    # Attach a loopback device to the file so we can use sfdisk on it.
    dev = LoopbackDevice()
    dev.setup(fileName)
    # Dump the partition information.
    command = [findProg("sfdisk"), "-d", dev.devFile]
    (out, returncode) = privOutput(command)
    if returncode != 0:
        print(out)
        exit(returncode)

    # Parse each line of the sfdisk output looking for the first
    # partition description.
    SFDISK_PARTITION_INFO_RE = re.compile(
        r"^\s*"  # Start of line
        r"(?P<name>\S+)"  # Name
        r"\s*:\s*"  # Separator
        r"start=\s*(?P<start>\d+),\s*"  # Partition start record
        r"size=\s*(?P<size>\d+),\s*"  # Partition size record
        r"type=(?P<type>.+?),*?"  # Partition type record
        r".*"  # anything else, e.g., name field
        r"\s*$"  # End of line
    )
    lines = out.splitlines()
    for line in lines:
        match = SFDISK_PARTITION_INFO_RE.match(line)
        if match:
            sectors = int(match.group("start"))
            break
    else:
        # No partition description was found
        print("No partition description was found in sfdisk output:")
        print("\n".join(f"  {line.rstrip()}" for line in lines))
        print("Could not determine size of first partition.")
        exit(1)

    # Free the loopback device and return an answer.
    dev.destroy()
    return sectors * BlockSize


def mountPointToDev(mountPoint):
    (mountTable, returncode) = getOutput([findProg("mount")])
    if returncode != 0:
        print(mountTable)
        exit(returncode)
    mountTable = mountTable.splitlines()
    for line in mountTable:
        chunks = line.split()
        try:
            if os.path.samefile(chunks[2], mountPoint):
                return LoopbackDevice(chunks[0])
        except OSError:
            continue
    return None


# Commands for the gem5img.py script
commands = {}
commandOrder = []


class Command:
    def addArgument(self, *args, **kargs):
        self.parser.add_argument(*args, **kargs)

    def __init__(self, name, description, posArgs):
        self.name = name
        self.description = description
        self.func = None
        self.posArgs = posArgs
        commands[self.name] = self
        commandOrder.append(self.name)
        usage = "%(prog)s [options]"
        posUsage = ""
        for posArg in posArgs:
            (argName, argDesc) = posArg
            usage += f" {argName}"
            posUsage += "\n  %s: %s" % posArg
        usage += posUsage
        self.parser = ArgumentParser(usage=usage, description=description)
        self.addArgument(
            "-d",
            "--debug",
            dest="debug",
            action="store_true",
            help="Verbose output.",
        )
        self.addArgument("pos", nargs="*")

    def parseArgs(self, argv):
        self.options = self.parser.parse_args(argv[2:])
        self.args = self.options.pos
        if len(self.args) != len(self.posArgs):
            self.parser.error("Incorrect number of arguments")
        global debug
        if self.options.debug:
            debug = True

    def runCom(self):
        if not self.func:
            exit(f"Unimplemented command {self.name}!")
        self.func(self.options, self.args)


# A command which prepares an image with an partition table and an empty file
# system.
initCom = Command(
    "init",
    "Create an image with an empty file system.",
    [("file", "Name of the image file."), ("mb", "Size of the file in MiB.")],
)
initCom.addArgument(
    "-t",
    "--type",
    dest="fstype",
    action="store",
    default="ext2",
    help="Type of file system to use. Appended to mkfs.",
)

# A command to mount the first partition in the image.
mountCom = Command(
    "mount",
    "Mount the first partition in the disk image.",
    [
        ("file", "Name of the image file."),
        ("mount point", "Where to mount the image."),
    ],
)


def mountComFunc(options, args):
    (path, mountPoint) = args
    if not os.path.isdir(mountPoint):
        print(f"Mount point {mountPoint} is not a directory.")

    dev = LoopbackDevice()
    if dev.setup(path, offset=True) != 0:
        exit(1)

    if runPriv([findProg("mount"), str(dev), mountPoint]) != 0:
        dev.destroy()
        exit(1)


mountCom.func = mountComFunc

# A command to unmount the first partition in the image.
umountCom = Command(
    "umount",
    "Unmount the disk image mounted at mount_point.",
    [("mount_point", "What mount point to unmount.")],
)


def umountComFunc(options, args):
    (mountPoint,) = args
    if not os.path.isdir(mountPoint):
        print(f"Mount point {mountPoint} is not a directory.")
        exit(1)

    dev = mountPointToDev(mountPoint)
    if not dev:
        print(f"Unable to find mount information for {mountPoint}.")

    # Unmount the loopback device.
    if runPriv([findProg("umount"), mountPoint]) != 0:
        exit(1)

    # Destroy the loopback device.
    dev.destroy()


umountCom.func = umountComFunc


# A command to create an empty file to hold the image.
newCom = Command(
    "new",
    'File creation part of "init".',
    [("file", "Name of the image file."), ("mb", "Size of the file in MiB.")],
)


def newImage(file, mb):
    (cylinders, heads, sectors) = chsFromSize((mb * MiB) / BlockSize)
    size = cylinders * heads * sectors * BlockSize

    # We lseek to the end of the file and only write one byte there. This
    # leaves a "hole" which many file systems are smart enough not to actually
    # store to disk and which is defined to read as zero.
    fd = os.open(file, os.O_WRONLY | os.O_CREAT)
    os.lseek(fd, size - 1, os.SEEK_SET)
    os.write(fd, b"\0")


def newComFunc(options, args):
    (file, mb) = args
    mb = int(mb)
    newImage(file, mb)


newCom.func = newComFunc

# A command to partition the image file like a raw disk device.
partitionCom = Command(
    "partition",
    'Partition part of "init".',
    [("file", "Name of the image file.")],
)


def partition(dev, cylinders, heads, sectors):
    # Use sfdisk to partition the device
    # The specified options are intended to work with both new and old
    # versions of sfdisk (see https://askubuntu.com/a/819614)
    comStr = ";"
    return runPriv(
        [findProg("sfdisk"), "--no-reread", "-u", "S", "-L", str(dev)],
        inputVal=comStr,
    )


def partitionComFunc(options, args):
    (path,) = args

    dev = LoopbackDevice()
    if dev.setup(path) != 0:
        exit(1)

    # Figure out the dimensions of the file.
    size = os.path.getsize(path)
    if partition(dev, *chsFromSize(size / BlockSize)) != 0:
        dev.destroy()
        exit(1)

    dev.destroy()


partitionCom.func = partitionComFunc

# A command to format the first partition in the image.
formatCom = Command(
    "format",
    'Formatting part of "init".',
    [("file", "Name of the image file.")],
)
formatCom.addArgument(
    "-t",
    "--type",
    dest="fstype",
    action="store",
    default="ext2",
    help="Type of file system to use. Appended to mkfs.",
)


def formatImage(dev, fsType):
    return runPriv([findProg(f"mkfs.{fsType}", dev), str(dev)])


def formatComFunc(options, args):
    (path,) = args

    dev = LoopbackDevice()
    if dev.setup(path, offset=True) != 0:
        exit(1)

    # Format the device.
    if formatImage(dev, options.fstype) != 0:
        dev.destroy()
        exit(1)

    dev.destroy()


formatCom.func = formatComFunc


def initComFunc(options, args):
    (path, mb) = args
    mb = int(mb)
    newImage(path, mb)
    dev = LoopbackDevice()
    if dev.setup(path) != 0:
        exit(1)
    size = os.path.getsize(path)
    if partition(dev, *chsFromSize((mb * MiB) / BlockSize)) != 0:
        dev.destroy()
        exit(1)
    dev.destroy()
    if dev.setup(path, offset=True) != 0:
        exit(1)
    if formatImage(dev, options.fstype) != 0:
        dev.destroy()
        exit(1)
    dev.destroy()


initCom.func = initComFunc


# Figure out what command was requested and execute it.
if len(argv) < 2 or argv[1] not in commands:
    print("Usage: %s [command] <command arguments>")
    print("where [command] is one of ")
    for name in commandOrder:
        command = commands[name]
        print(f"    {command.name}: {command.description}")
    print("Watch for orphaned loopback devices and delete them with")
    print("losetup -d. Mounted images will belong to root, so you may need")
    print("to use sudo to modify their contents.")
    exit(1)

command = commands[argv[1]]
command.parseArgs(argv)
command.runCom()
