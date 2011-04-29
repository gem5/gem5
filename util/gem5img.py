#!/usr/bin/python
#
# gem5img.py
# Script for managing a gem5 disk image.
#

from optparse import OptionParser
import os
from os import environ as env
import string
from subprocess import CalledProcessError, Popen, PIPE, STDOUT
from sys import exit, argv


# Some constants.
MaxLBACylinders = 16383
MaxLBAHeads = 16
MaxLBASectors = 63
MaxLBABlocks = MaxLBACylinders * MaxLBAHeads * MaxLBASectors

BlockSize = 512
MB = 1024 * 1024

# Setup PATH to look in the sbins.
env['PATH'] += ':/sbin:/usr/sbin'

# Whether to print debug output.
debug = False

# Figure out cylinders, heads and sectors from a size in blocks.
def chsFromSize(sizeInBlocks):
    if sizeInBlocks >= MaxLBABlocks:
        sizeInMBs = (sizeInBlocks * BlockSize) / MB
        print '%d MB is too big for LBA, truncating file.' % sizeInMBs
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
    if not hasattr(needSudo, 'notRoot'):
        needSudo.notRoot = (os.geteuid() != 0)
        if needSudo.notRoot:
            print 'You are not root. Using sudo.'
    return needSudo.notRoot

# Run an external command.
def runCommand(command, inputVal=''):
    print "%>", ' '.join(command)
    proc = Popen(command, stdin=PIPE)
    proc.communicate(inputVal)
    return proc.returncode

# Run an external command and capture its output. This is intended to be
# used with non-interactive commands where the output is for internal use.
def getOutput(command, inputVal=''):
    global debug
    if debug:
        print "%>", ' '.join(command)
    proc = Popen(command, stderr=STDOUT,
                 stdin=PIPE, stdout=PIPE)
    (out, err) = proc.communicate(inputVal)
    return (out, proc.returncode)

# Run a command as root, using sudo if necessary.
def runPriv(command, inputVal=''):
    realCommand = command
    if needSudo():
        realCommand = [findProg('sudo')] + command
    return runCommand(realCommand, inputVal)

def privOutput(command, inputVal=''):
    realCommand = command
    if needSudo():
        realCommand = [findProg('sudo')] + command
    return getOutput(realCommand, inputVal)

# Find the path to a program.
def findProg(program, cleanupDev=None):
    (out, returncode) = getOutput(['which', program])
    if returncode != 0:
        if cleanupDev:
            cleanupDev.destroy()
        exit("Unable to find program %s, check your PATH variable." % program)
    return string.strip(out)

class LoopbackDevice(object):
    def __init__(self, devFile=None):
        self.devFile = devFile
    def __str__(self):
        return str(self.devFile)

    def setup(self, fileName, offset=False):
        assert not self.devFile
        (out, returncode) = privOutput([findProg('losetup'), '-f'])
        if returncode != 0:
            print out
            return returncode
        self.devFile = string.strip(out)
        command = [findProg('losetup'), self.devFile, fileName]
        if offset:
            off = findPartOffset(self.devFile, fileName, 0)
            command = command[:1] + \
                      ["-o", "%d" % off] + \
                      command[1:]
        return runPriv(command)

    def destroy(self):
        assert self.devFile
        returncode = runPriv([findProg('losetup'), '-d', self.devFile])
        self.devFile = None
        return returncode

def findPartOffset(devFile, fileName, partition):
    # Attach a loopback device to the file so we can use sfdisk on it.
    dev = LoopbackDevice()
    dev.setup(fileName)
    # Dump the partition information.
    command = [findProg('sfdisk'), '-d', dev.devFile]
    (out, returncode) = privOutput(command)
    if returncode != 0:
        print out
        exit(returncode)
    lines = out.splitlines()
    # Make sure the first few lines of the output look like what we expect.
    assert(lines[0][0] == '#')
    assert(lines[1] == 'unit: sectors')
    assert(lines[2] == '')
    # This line has information about the first partition.
    chunks = lines[3].split()
    # The fourth chunk is the offset of the partition in sectors followed by
    # a comma. We drop the comma and convert that to an integer.
    sectors = string.atoi(chunks[3][:-1])
    # Free the loopback device and return an answer.
    dev.destroy()
    return sectors * BlockSize

def mountPointToDev(mountPoint):
    (mountTable, returncode) = getOutput([findProg('mount')])
    if returncode != 0:
        print mountTable
        exit(returncode)
    mountTable = mountTable.splitlines()
    for line in mountTable:
        chunks = line.split()
        if os.path.samefile(chunks[2], mountPoint):
            return LoopbackDevice(chunks[0])
    return None


# Commands for the gem5img.py script
commands = {}
commandOrder = []

class Command(object):
    def addOption(self, *args, **kargs):
        self.parser.add_option(*args, **kargs)

    def __init__(self, name, description, posArgs):
        self.name = name
        self.description = description
        self.func = None
        self.posArgs = posArgs
        commands[self.name] = self
        commandOrder.append(self.name)
        usage = 'usage: %prog [options]'
        posUsage = ''
        for posArg in posArgs:
            (argName, argDesc) = posArg
            usage += ' %s' % argName
            posUsage += '\n  %s: %s' % posArg
        usage += posUsage
        self.parser = OptionParser(usage=usage, description=description)
        self.addOption('-d', '--debug', dest='debug', action='store_true',
                       help='Verbose output.')

    def parseArgs(self, argv):
        (self.options, self.args) = self.parser.parse_args(argv[2:])
        if len(self.args) != len(self.posArgs):
            self.parser.error('Incorrect number of arguments')
        global debug
        if self.options.debug:
            debug = True

    def runCom(self):
        if not self.func:
            exit('Unimplemented command %s!' % self.name)
        self.func(self.options, self.args)


# A command which prepares an image with an partition table and an empty file
# system.
initCom = Command('init', 'Create an image with an empty file system.',
                  [('file', 'Name of the image file.'),
                   ('mb', 'Size of the file in MB.')])
initCom.addOption('-t', '--type', dest='fstype', action='store',
                  default='ext2',
                  help='Type of file system to use. Appended to mkfs.')

# A command to mount the first partition in the image.
mountCom = Command('mount', 'Mount the first partition in the disk image.',
                   [('file', 'Name of the image file.'),
                    ('mount point', 'Where to mount the image.')])

def mountComFunc(options, args):
    (path, mountPoint) = args
    if not os.path.isdir(mountPoint):
        print "Mount point %s is not a directory." % mountPoint

    dev = LoopbackDevice()
    if dev.setup(path, offset=True) != 0:
        exit(1)

    if runPriv([findProg('mount'), str(dev), mountPoint]) != 0:
        dev.destroy()
        exit(1)

mountCom.func = mountComFunc

# A command to unmount the first partition in the image.
umountCom = Command('umount', 'Unmount the first partition in the disk image.',
                    [('mount point', 'What mount point to unmount.')])

def umountComFunc(options, args):
    (mountPoint,) = args
    if not os.path.isdir(mountPoint):
        print "Mount point %s is not a directory." % mountPoint
        exit(1)

    dev = mountPointToDev(mountPoint)
    if not dev:
        print "Unable to find mount information for %s." % mountPoint

    # Unmount the loopback device.
    if runPriv([findProg('umount'), mountPoint]) != 0:
        exit(1)

    # Destroy the loopback device.
    dev.destroy()

umountCom.func = umountComFunc


# A command to create an empty file to hold the image.
newCom = Command('new', 'File creation part of "init".',
                 [('file', 'Name of the image file.'),
                  ('mb', 'Size of the file in MB.')])

def newImage(file, mb):
    (cylinders, heads, sectors) = chsFromSize((mb * MB) / BlockSize)
    size = cylinders * heads * sectors * BlockSize

    # We lseek to the end of the file and only write one byte there. This
    # leaves a "hole" which many file systems are smart enough not to actually
    # store to disk and which is defined to read as zero.
    fd = os.open(file, os.O_WRONLY | os.O_CREAT)
    os.lseek(fd, size - 1, os.SEEK_SET)
    os.write(fd, '\0')

def newComFunc(options, args):
    (file, mb) = args
    mb = string.atoi(mb)
    newImage(file, mb)


newCom.func = newComFunc

# A command to partition the image file like a raw disk device.
partitionCom = Command('partition', 'Partition part of "init".',
                       [('file', 'Name of the image file.')])

def partition(dev, cylinders, heads, sectors):
    # Use fdisk to partition the device
    comStr = '0,\n;\n;\n;\n'
    return runPriv([findProg('sfdisk'), '--no-reread', '-D', \
                   '-C', "%d" % cylinders, \
                   '-H', "%d" % heads, \
                   '-S', "%d" % sectors, \
                   str(dev)], inputVal=comStr)

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
formatCom = Command('format', 'Formatting part of "init".',
                    [('file', 'Name of the image file.')])
formatCom.addOption('-t', '--type', dest='fstype', action='store',
                    default='ext2',
                    help='Type of file system to use. Appended to mkfs.')

def formatImage(dev, fsType):
    return runPriv([findProg('mkfs.%s' % fsType, dev), str(dev)])

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
    mb = string.atoi(mb)
    newImage(path, mb)
    dev = LoopbackDevice()
    if dev.setup(path) != 0:
        exit(1)
    size = os.path.getsize(path)
    if partition(dev, *chsFromSize((mb * MB) / BlockSize)) != 0:
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
    print 'Usage: %s [command] <command arguments>'
    print 'where [command] is one of '
    for name in commandOrder:
        command = commands[name]
        print '    %s: %s' % (command.name, command.description)
    print 'Watch for orphaned loopback devices and delete them with'
    print 'losetup -d. Mounted images will belong to root, so you may need'
    print 'to use sudo to modify their contents.'
    exit(1)

command = commands[argv[1]]
command.parseArgs(argv)
command.runCom()
