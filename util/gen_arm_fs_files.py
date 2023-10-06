#!/usr/bin/env python3
# Copyright (c) 2020 ARM Limited
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
# Copyright (c) 2017-2018 Metempsy Technology Consulting
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
import os
import sys
from argparse import ArgumentDefaultsHelpFormatter
from argparse import ArgumentParser
from glob import glob
from platform import machine
from subprocess import call

from distutils import spawn


def run_cmd(explanation, working_dir, cmd, stdout=None):
    print(f"Running phase '{explanation}'")
    sys.stdout.flush()

    # some of the commands need $PWD to be properly set
    env = os.environ.copy()
    env["PWD"] = working_dir

    return_code = call(cmd, cwd=working_dir, stdout=stdout, env=env)

    if return_code == 0:
        return

    print(
        "Error running phase %s. Returncode: %d" % (explanation, return_code),
    )
    sys.exit(1)


def linux_clone():
    kernel_vexpress_gem5_dir = os.path.join(
        args.dest_dir,
        "linux-kernel-vexpress_gem5",
    )

    run_cmd(
        "clone linux kernel for VExpress_GEM5_V1 platform",
        args.dest_dir,
        [
            "git",
            "clone",
            "https://gem5.googlesource.com/arm/linux",
            kernel_vexpress_gem5_dir,
        ],
    )


def linux64():
    kernel_vexpress_gem5_dir = os.path.join(
        args.dest_dir,
        "linux-kernel-vexpress_gem5",
    )

    linux_bin = os.path.join(binaries_dir, "vmlinux.vexpress_gem5_v1_64")

    with open(revisions_dir + "/linux", "w+") as rev_file:
        run_cmd(
            "write revision of linux-kernel-vexpress_gem5 repo",
            kernel_vexpress_gem5_dir,
            ["git", "rev-parse", "--short", "HEAD"],
            rev_file,
        )

    run_cmd(
        "configure kernel for arm64",
        kernel_vexpress_gem5_dir,
        [
            "make",
            "ARCH=arm64",
            "CROSS_COMPILE=aarch64-linux-gnu-",
            "gem5_defconfig",
            make_jobs_str,
        ],
    )
    run_cmd(
        "compile kernel for arm64",
        kernel_vexpress_gem5_dir,
        [
            "make",
            "ARCH=arm64",
            "CROSS_COMPILE=aarch64-linux-gnu-",
            make_jobs_str,
        ],
    )
    run_cmd(
        "copy arm64 vmlinux",
        kernel_vexpress_gem5_dir,
        ["cp", "vmlinux", linux_bin],
    )
    run_cmd(
        "cleanup arm64 kernel compilation",
        kernel_vexpress_gem5_dir,
        ["make", "distclean"],
    )


def linux32():
    kernel_vexpress_gem5_dir = os.path.join(
        args.dest_dir,
        "linux-kernel-vexpress_gem5",
    )

    linux_bin = os.path.join(binaries_dir, "vmlinux.vexpress_gem5_v1")

    run_cmd(
        "configure kernel for arm",
        kernel_vexpress_gem5_dir,
        [
            "make",
            "ARCH=arm",
            "CROSS_COMPILE=arm-linux-gnueabihf-",
            "gem5_defconfig",
        ],
    )
    run_cmd(
        "compile kernel for arm",
        kernel_vexpress_gem5_dir,
        [
            "make",
            "ARCH=arm",
            "CROSS_COMPILE=arm-linux-gnueabihf-",
            make_jobs_str,
        ],
    )
    run_cmd(
        "copy arm vmlinux",
        kernel_vexpress_gem5_dir,
        ["cp", "vmlinux", linux_bin],
    )


def linux():
    """
    Checkout and build linux kernel for VExpress_GEM5_V1 (arm and arm64)
    """
    linux_clone()
    linux64()
    linux32()


def dtbs():
    """
    Build DTBs for VExpress_GEM5_V1
    """
    dt_dir = gem5_dir + "/system/arm/dt"
    run_cmd(
        "compile DTBs for VExpress_GEM5_V1 platform",
        dt_dir,
        ["make", make_jobs_str],
    )
    run_cmd(
        "copy DTBs",
        dt_dir,
        ["cp"] + glob(dt_dir + "/*dtb") + [binaries_dir],
    )


def bootloaders():
    """
    Build bootloaders arm64/arm
    """

    bootloader_arm64_dir = gem5_dir + "/system/arm/bootloader/arm64"
    run_cmd("compile arm64 bootloader", bootloader_arm64_dir, ["make"])
    run_cmd(
        "copy arm64 bootloader",
        bootloader_arm64_dir,
        ["cp", "boot.arm64", "boot_emm.arm64", "boot_v2.arm64", binaries_dir],
    )

    bootloader_arm_dir = gem5_dir + "/system/arm/bootloader/arm"
    run_cmd("compile arm bootloader", bootloader_arm_dir, ["make"])
    run_cmd(
        "copy arm bootloaders",
        bootloader_arm_dir,
        ["cp", "boot.arm", "boot_emm.arm", binaries_dir],
    )


def m5():
    """
    Build m5 binaries
    """
    m5_dir = gem5_dir + "/util/m5"
    run_cmd("compile arm64 m5", m5_dir, ["make", "-f", "Makefile.aarch64"])
    run_cmd(
        "copy arm64 m5",
        m5_dir,
        ["cp", "m5", binaries_dir + "/m5.aarch64"],
    )
    run_cmd(
        "clean arm64 m5",
        m5_dir,
        ["make", "clean", "-f", "Makefile.aarch64"],
    )
    run_cmd("compile arm m5", m5_dir, ["make", "-f", "Makefile.arm"])
    run_cmd("copy arm m5", m5_dir, ["cp", "m5", binaries_dir + "/m5.aarch32"])


def xen():
    """
    Build Xen for aarch64
    """
    xen_dir = os.path.join(args.dest_dir, "xen")
    bootwrapper_dir = os.path.join(args.dest_dir, "bootwrapper")
    linux_cmdline = "console=hvc0 root=/dev/vda rw mem=1G"
    xen_cmdline = (
        "dtuart=/uart@1c090000 console=dtuart no-bootscrub "
        + "dom0_mem=1G loglvl=all guest_loglvl=all"
    )

    run_cmd(
        "clone Xen",
        args.dest_dir,
        ["git", "clone", "git://xenbits.xen.org/xen.git", xen_dir],
    )

    run_cmd(
        "clone boot-wrapper-aarch64",
        args.dest_dir,
        [
            "git",
            "clone",
            "git://git.kernel.org/pub/"
            + "scm/linux/kernel/git/mark/boot-wrapper-aarch64.git",
            bootwrapper_dir,
        ],
    )

    # Need to compile arm64 Linux
    linux_dir = os.path.join(args.dest_dir, "linux-kernel-vexpress_gem5")
    linux_bin = os.path.join(linux_dir, "arch", "arm64", "boot", "Image")
    if not os.path.exists(linux_bin):
        linux_clone()
        linux64()

    # Need to compile DTBs
    dtb_bin = os.path.join(binaries_dir, "armv8_gem5_v2_1cpu.dtb")
    if not os.path.exists(dtb_bin):
        dtbs()

    # Building Xen
    run_cmd(
        "building xen for aarch64",
        xen_dir,
        [
            "make",
            "dist-xen",
            "XEN_TARGET_ARCH=arm64",
            "CROSS_COMPILE=aarch64-linux-gnu-",
            "CONFIG_EARLY_PRINTK=vexpress",
            make_jobs_str,
        ],
    )

    # Building boot-wrapper-aarch64
    run_cmd(
        "autoreconf boot-wrapper-aarch64",
        bootwrapper_dir,
        ["autoreconf", "-i"],
    )
    run_cmd(
        "configure boot-wrapper-aarch64",
        bootwrapper_dir,
        [
            "./configure",
            "--host=aarch64-linux-gnu",
            f"--with-kernel-dir={linux_dir}",
            f"--with-dtb={dtb_bin}",
            f"--with-cmdline='{linux_cmdline}'",
            f"--with-xen-cmdline='{xen_cmdline}'",
            f"--with-xen={os.path.join(xen_dir, 'xen', 'xen')}",
            "--enable-psci",
            "--enable-gicv3",
        ],
    )
    run_cmd("build boot-wrapper-aarch64", bootwrapper_dir, ["make"])

    # Copying the final binary
    run_cmd(
        "copy xen binary",
        bootwrapper_dir,
        ["cp", "xen-system.axf", binaries_dir],
    )

    with open(os.path.join(revisions_dir, "xen"), "w+") as rev_file:
        run_cmd(
            "write revision of xen repo",
            xen_dir,
            ["git", "rev-parse", "--short", "HEAD"],
            rev_file,
        )


script_dir = os.path.dirname(os.path.abspath(sys.argv[0]))
gem5_dir = os.path.dirname(script_dir)

all_binaries = {
    "linux": linux,
    "dtbs": dtbs,
    "bootloaders": bootloaders,
    "m5": m5,
    "xen": xen,
}

parser = ArgumentParser(formatter_class=ArgumentDefaultsHelpFormatter)

parser.add_argument(
    "--gem5-dir",
    default=gem5_dir,
    metavar="GEM5_DIR",
    help="gem5 root directory to be used for bootloader and "
    "VExpress_GEM5_V1 DTB sources. The default value is the gem5 root "
    "directory of the executed script",
)
parser.add_argument(
    "--dest-dir",
    default="/tmp",
    metavar="DEST_DIR",
    help="Directory to use for checking out the different kernel "
    "repositories. Generated files will be copied to "
    "DEST_DIR/binaries (which must not exist)",
)
parser.add_argument(
    "-j",
    "--make-jobs",
    type=int,
    default=1,
    metavar="MAKE_JOBS",
    help="Number of jobs to use with the 'make' commands.",
)
parser.add_argument(
    "-b",
    "--fs-binaries",
    action="append",
    choices=list(all_binaries.keys()),
    default=[],
    help="List of FS files to be generated. Defaulting to all",
)

args = parser.parse_args()

if not os.path.isdir(args.dest_dir):
    print(f"Error: {args.dest_dir} is not a directory.")
    sys.exit(1)

if not os.path.isdir(args.gem5_dir):
    print(f"Error: {args.gem5_dir} is not a directory.")
    sys.exit(1)

if machine() != "x86_64":
    print("Error: This script should run in a x86_64 machine")
    sys.exit(1)

binaries_dir = args.dest_dir + "/binaries"

if os.path.exists(binaries_dir):
    print(f"Error: {binaries_dir} already exists.")
    sys.exit(1)

revisions_dir = args.dest_dir + "/revisions"

if os.path.exists(revisions_dir):
    print(f"Error: {revisions_dir} already exists.")
    sys.exit(1)

os.mkdir(binaries_dir)
os.mkdir(revisions_dir)

make_jobs_str = "-j" + str(args.make_jobs)

rev_file = open(revisions_dir + "/gem5", "w+")
run_cmd(
    "write revision of gem5 repo",
    gem5_dir,
    ["git", "rev-parse", "--short", "HEAD"],
    rev_file,
)
rev_file.close()

binaries = args.fs_binaries if args.fs_binaries else list(all_binaries.keys())
for fs_binary in binaries:
    all_binaries[fs_binary]()

print(f"Done! All the generated files can be found in {binaries_dir}")

sys.exit(0)
