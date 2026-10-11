#!/usr/bin/env python3
# Copyright (c) 2023 The Regents of the University of California
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

"""Package a stopped, unregistered builder using existing libvirt privileges.

The builder disk is preserved. Libvirt flattens a new volume before the user's
image tools sanitize that copy. No host sudo, pool relocation, or live-disk copy.
"""

import argparse
import hashlib
import json
import re
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path


def run(*args):
    return subprocess.check_output(args, text=True).strip()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("domain")
    parser.add_argument("pool")
    parser.add_argument("version")
    parser.add_argument("output", type=Path)
    parser.add_argument("public_key", type=Path)
    args = parser.parse_args()
    if not re.fullmatch(r"[0-9]+(?:\.[0-9]+)+", args.version):
        parser.error("version must be dotted numbers, e.g. 2026.9.8.1")
    virsh = ["virsh", "-c", "qemu:///system"]
    if run(*virsh, "domstate", args.domain) != "shut off":
        parser.error("drain and shut down the builder before packaging")
    args.output.mkdir(mode=0o700, parents=True, exist_ok=False)
    xml = run(*virsh, "dumpxml", args.domain)
    (args.output / "builder.xml").write_text(xml)
    source = ET.fromstring(xml).find("./devices/disk[@device='disk']/source")
    source_path = source.get("file")
    volume_name = f"gem5-runner-{args.version}-flat.img"
    pool_xml = ET.fromstring(run(*virsh, "pool-dumpxml", args.pool))
    flat = Path(pool_xml.findtext("./target/path")) / volume_name
    import os

    volume = ET.Element("volume")
    ET.SubElement(volume, "name").text = volume_name
    ET.SubElement(volume, "capacity", unit="GiB").text = "128"
    target = ET.SubElement(volume, "target")
    ET.SubElement(target, "format", type="qcow2")
    permissions = ET.SubElement(target, "permissions")
    ET.SubElement(permissions, "owner").text = str(os.getuid())
    ET.SubElement(permissions, "group").text = str(os.getgid())
    ET.SubElement(permissions, "mode").text = "0600"
    volume_xml = args.output / "flat-volume.xml"
    volume_xml.write_text(ET.tostring(volume, encoding="unicode"))
    run(*virsh, "vol-create-from", args.pool, str(volume_xml), source_path)
    info = json.loads(run("qemu-img", "info", "--output=json", str(flat)))
    if info.get("backing-filename"):
        raise RuntimeError("libvirt clone was not flattened; refusing package")
    # Check before sanitizing: a production VM is never an image builder.
    names = run("virt-ls", "-a", str(flat), "/home/vagrant").splitlines()
    if any(name in names for name in (".runner", ".credentials", "_work")):
        raise RuntimeError("builder contains a registration or work directory")
    if (
        "gem5-runner.env"
        in run("virt-ls", "-a", str(flat), "/etc").splitlines()
    ):
        raise RuntimeError("builder contains runner credentials")
    run(
        "virt-sysprep",
        "-a",
        str(flat),
        "--operations",
        "defaults",
        "--ssh-inject",
        f"vagrant:file:{args.public_key.resolve()}",
        "--hostname",
        "gem5-runner-base",
        "--firstboot-command",
        "ssh-keygen -A; systemctl restart ssh",
    )
    image = args.output / "box.img"
    run("qemu-img", "convert", "-O", "qcow2", "-c", str(flat), str(image))
    (args.output / "metadata.json").write_text(
        json.dumps(
            {"provider": "libvirt", "format": "qcow2", "virtual_size": 128}
        )
    )
    (args.output / "Vagrantfile").write_text(
        'Vagrant.configure("2") { |config| config.ssh.username = "vagrant" }\n'
    )
    box = args.output / "gem5-runner.box"
    run(
        "tar",
        "-cf",
        str(box),
        "-C",
        str(args.output),
        "box.img",
        "metadata.json",
        "Vagrantfile",
    )
    digest = hashlib.file_digest(box.open("rb"), "sha256").hexdigest()
    (args.output / "SHA256SUMS").write_text(f"{digest}  gem5-runner.box\n")
    metadata = {
        "name": "gem5/runner-ubuntu2204",
        "versions": [
            {
                "version": args.version,
                "providers": [
                    {
                        "name": "libvirt",
                        "url": str(box.resolve()),
                        "checksum_type": "sha256",
                        "checksum": digest,
                    }
                ],
            }
        ],
    }
    manifest = args.output / "box-catalog.json"
    manifest.write_text(json.dumps(metadata, indent=2) + "\n")
    print(
        f"Packaged {box}. Original builder disk and flat copy are preserved."
    )
    print(f"Add the version with: vagrant box add {manifest}")


if __name__ == "__main__":
    main()
