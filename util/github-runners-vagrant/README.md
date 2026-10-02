# Vagrant GitHub Actions runners

Each VM runs one ephemeral GitHub registration at a time. The VM persists
between jobs. `action-run.sh` is supervised by the guest's
`gem5-runner.service`; logs go to the guest journal. This does not make the VM
disposable or establish a security boundary between jobs.

All `sudo` in the provisioning scripts is **inside the guest**. On Loupe, run
host commands as `vagrant` using the existing `sudo -u vagrant` access. These
tools use the installed Vagrant, libvirt, Python, qemu-img and libguestfs tools;
they do not require new host packages, host sysctls, or host administrator access.

## Configuration and operation

Set `NUM_RUNNERS`, `PERSONAL_ACCESS_TOKEN`, `GITHUB_ORG`, and `HOSTNAME` in
`Vagrantfile`. Keep the real configuration private. On Loupe the authoritative
copy is `/runners/Vagrantfile`. The credential design is unchanged in scope:
registration uses the existing PAT. The guest service reads it from the
root-only `/etc/gem5-runner.env`; the controller removes it from the environment
before starting the runner. Never publish this file or package it in a box.

If `gem5-resource-cache` exists beside `Vagrantfile`, it is shared at
`/gem5-resource-cache` using the existing 9p configuration. Installation records
the actual mount tag in a guest systemd mount unit, loads its virtio transport
early, and orders the mount before the runner,
so ordinary guest reboots do not depend on Vagrant remounting the cache over SSH.

Optional `runner-settings.json` overrides individual machines. For example:

```json
{
  "loupe-1": {
    "box": "gem5/runner-ubuntu2204",
    "box_version": "2026.9.8.1",
    "prebuilt": true,
    "storage_pool": "loupe-ssd",
    "disk_cache": "none",
    "cpus": 4,
    "memory": 32768
  }
}
```

A prebuilt box skips package provisioning; scripts and service configuration are
still installed. Changes to a VM's box require a replacement VM. Changing a
JSON field does not migrate an existing disk or update its guest kernel.

Run Vagrant from the deployment directory with the correct `VAGRANT_HOME`.
Provisioning refuses to overwrite an active controller. For a new VM:

```sh
cd /runners
VAGRANT_HOME=/runners vagrant up loupe-1 --provider=libvirt
```

Inside a guest, inspect the service and its explicit state:

```sh
cat ~/runner-state/status
sudo journalctl -u gem5-runner.service --since '1 hour ago'
sudo systemctl status gem5-runner.service
```

Health gates check Docker access **as the runner user**, free bytes and inodes,
container execution, bidirectional workspace bind mounts, and the shared cache.
After three failed health attempts, failures persist `~/runner-state/quarantine` and stop registration. After fixing
the reported issue, run `./runner-health.sh`, remove that marker, and start the
service. Do not repeatedly restart a quarantined runner without fixing it.

To drain:

```sh
mkdir -p ~/runner-state
touch ~/runner-state/drain
# Wait until runner-state/status says "drained" and the service is inactive.
```

An already idle listener can take **one final job** before draining. The marker
is checked at registration boundaries; it intentionally does not cancel an idle
listener based on a potentially stale GitHub busy flag. After draining, perform
maintenance, remove the marker and start the service. `systemctl stop` itself is
not a safe drain: it can terminate a running job.

## Cleanup and image retention

After a job, the controller deletes `_work`, removes the dedicated VM's
containers and unused volumes/networks, and limits Docker builder cache to
4 GB. It retains `ghcr.io/gem5/*` images and the small health image, subject to a
32 GiB conservative image-size cap and 20 GiB free-space floor. Shared layers
are counted repeatedly, so the cap errs toward freeing space. Oldest-created
images are evicted first when a limit is exceeded. Other images are removed.
This deliberately avoids an age filter that would repeatedly delete an old but
frequently pulled image. Recent diagnostic logs are retained for seven days.

The environment settings `RUNNER_IMAGE_CACHE_GIB`, `RUNNER_MIN_FREE_GIB`, and
`HEALTH_IMAGE` can override these defaults. Job containers continue pulling their
configured tags through GitHub Actions, so retaining a layer does not freeze
`:latest` permanently. Retained images share the existing VM trust model.

## Migrating an existing legacy loop

Copy the new scripts, unit file, and `migrate-runner.py` into a guest staging
directory such as `~/.runner-update`. Then, as the guest `vagrant` user:

```sh
cd ~/.runner-update
nohup python3 migrate-runner.py > migration.log 2>&1 < /dev/null &
```

The migration first saves the old scripts and private recovery arguments in
`~/runner-backups/<UTC timestamp>/`. It pauses only the outer legacy shell,
leaving its listener and job running. After all its live children and runner
workers exit, it replaces the shell, cleans the completed job, checks health,
and installs the service. Ordinary termination signals resume the old shell
if replacement has not begun. Do not SIGKILL the migration during a drain; if
that happens, inspect its log and resume the recorded legacy PID with SIGCONT
only after confirming its command still matches the backed-up controller.

An unexpected controller count, changed process, or installation failure stops
the migration rather than guessing. A failed installation after replacement
leaves the runner offline for inspection; its backup remains available.

## SSD canaries and versioned images

Keep the existing default pool. A separate directory pool on Loupe's existing
`/nobackup` SSD permits a canary without moving the fleet or `/nobackup/docker`.
Create the directory and pool as `vagrant` using libvirt's existing access:

```sh
mkdir -m 0711 /nobackup/loupe-ssd
virsh -c qemu:///system pool-define-as loupe-ssd dir --target /nobackup/loupe-ssd
virsh -c qemu:///system pool-start loupe-ssd
virsh -c qemu:///system pool-autostart loupe-ssd
```

Do not rerun creation over an existing pool. Use per-VM `storage_pool` and
`disk_cache: "none"` overrides for new canaries, and confirm the resulting XML.
A fast root disk can improve local extraction and Docker I/O; the shared 9p
resource cache still resides on the original disk. Measure real job preparation
and execution times before increasing fleet size or moving more VMs.

Build boxes in a **separate directory and Vagrant home**. Vagrant merges the
Vagrantfile in `VAGRANT_HOME` with the project's file; pointing a builder at
Loupe's `/runners` home also loads the fleet definition. A private home can reuse
the existing `gems` and `boxes` through symlinks, without copying
`/runners/Vagrantfile`. Give it a local `plugins.json` copied from the existing
registry with only `vagrant-libvirt` and `vagrant-reload` enabled. Loupe also has
an unrelated `reload` plugin which loads Rails and breaks `vagrant box add`;
exclude that entry from the private registry. The production registry need not
change. A separate SSD-backed `boxes` directory can avoid copying new boxes
through the busy fleet disk. Copy this directory's scripts and `Vagrantfile-image`
into the builder directory, naming that file `Vagrantfile` there.

The image recipe updates kernel dependencies, installs Docker and runner
2.337.0 with a SHA-256 check, reboots, prewarms the common Ubuntu gem5 image,
and runs a container health check. It records package and Docker inventories
inside the guest. It never registers a runner or receives a PAT. Test real
container compilation and cleanup before packaging.

After shutting down the builder, package a **new version**:

```sh
python3 package-image.py BUILDER_DOMAIN loupe-ssd 2026.9.8.1 \
  /nobackup/loupe-boxes/2026.9.8.1 /path/to/vagrant.pub
VAGRANT_HOME=/path/to/private-vagrant-home vagrant box add \
  /nobackup/loupe-boxes/2026.9.8.1/box-catalog.json
```

Loupe's `/boot/vmlinuz-*` files are root-only. For libguestfs, extract a
user-owned copy of the matching **already installed** kernel package and point
supermin at it and the matching, readable host modules. For example, as vagrant:

```sh
mkdir -p /var/lib/vagrant/loupe-libguestfs-kernel
cd /var/lib/vagrant/loupe-libguestfs-kernel
apt-get download linux-image-6.8.0-139-generic=6.8.0-139.139
dpkg-deb -x linux-image-6.8.0-139-generic_6.8.0-139.139_amd64.deb extracted
chmod u+r extracted/boot/vmlinuz-6.8.0-139-generic
export SUPERMIN_KERNEL="$PWD/extracted/boot/vmlinuz-6.8.0-139-generic"
export SUPERMIN_MODULES=/lib/modules/6.8.0-139-generic
```

Refresh the version/path together if the installed host kernel changes. This
extracts a package into a private directory; it does not install a host package
or change `/boot` permissions.

The packaging command requires Python 3.11+, installed on Loupe. It refuses a
running VM or an image containing runner credentials/workspace. It uses libvirt
to flatten a separate volume, sanitizes that copy, replaces the builder's SSH
authorization with the bootstrap public key, and regenerates host keys at first
boot. The original builder disk is preserved. The catalog pins the version and
archive checksum. Boot a fresh VM from that catalog, verify its new machine ID,
SSH identity, Docker and mount behavior, and test an ordinary guest reboot before
allowing jobs. Keep the prior box and cold VM disk until the canary is accepted.

## Backups and rollback

Before editing Loupe, keep a mode-0700 backup directory with a mode-0600 archive
of deployment scripts, Vagrant state (including keys), runner settings, and
plugin metadata. Save each domain XML and pool XML separately. These backups
contain credentials; never attach them to issues or PRs. Do not copy live qcow2
files as a purported consistent VM backup. Preserve a stopped original disk or
use a new canary, leaving existing disks untouched.

For a service rollback, first drain it. Save current settings, disable the new
service inside the guest, restore the backed-up scripts, and launch the legacy
script with its private `legacy-arguments.json` using a subprocess argument
array (never paste credentials into logs). The controller backup is per guest;
restoring `/runners` alone does not replace scripts already copied into guests.

For an image rollback, drain and shut down the canary, retain its disk for
inspection, and restore the previous version/settings or preserved cold domain.
Do not delete the old box or disk before verifying the replacement. Deleting the
new SSD pool is not necessary for rollback and must never delete an active VM's
storage.

## Validation

On Linux, `python3 test_runner.py -v` tests failure gates, cleanup bounds,
credential environment removal, duplicate-loop protection, and migration drain
recovery without making Docker or GitHub writes. Also run ShellCheck on the
shell scripts and `ruby -c` on both Vagrantfiles. Real Docker, mount, reboot,
packaging, and fresh-box tests are still required for a deployment.
