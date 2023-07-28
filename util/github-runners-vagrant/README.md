# Setting up a Github Actions Runner with Vagrant

This directory provides a way to setup a Github Actions runner using Vagrant to host the runner in a Virtual machine.

This tutorial has been written with the assumption of running on a machine with Ubuntu 22.04.
Setting up a runner on a different OS may require some changes.

Before anything else, copy this directory, "util/github-runners-vagrant", to the root of the location on your host system you wish to setup the VMs from.
The CWD is assumed to be this directory.

## Install Dependencies

```sh
sudo apt install vagrant
sudo apt-get build-dep vagrant ruby-libvirt
sudo apt-get install qemu libvirt-daemon-system libvirt-clients ebtables dnsmasq-base libxslt-dev libxml2-dev libvirt-dev zlib1g-dev ruby-dev

# Note: The vagrant-libvirt APT package does not work as intended. We must
# remove it from the system otherwise errors will occur (we will install it
# later using the vagrant plugin command).
sudo apt purge vagrant-libvirt
```

## Set up the Vagrantfiles for the GitHub repository

First, generate a Personal Access Token, which you can create [here](https://github.com/settings/tokens)
Make sure to set admin permissions on this token, then replace instances of `<PERSONAL ACCESS TOKEN>` in the Vagrantfiles ("Vagrantfile-builder" and "Vagrant-runner") with your token.

Next, replace instances of `<GITHUB REPO>` with your GitHub account name and the repository name, separated by a forward slash.
For example, if your GitHub account name is `example` and your repository name is `example-repo`, you would replace `<GITHUB REPO>` with `example/example-repo`.

## Install Vagrant Plugins

Once everything is set properly, set the `VAGRANT_HOME` environment variable to the directory in which the Vagrant files and other scripts are stored (i.e., the CWD).
For example:

```sh
export VAGRANT_HOME=`pwd`
```

After this, install the relevant vagrant plugins:

``` sh
vagrant plugin install dotenv
vagrant plugin install vagrant-libvirt
vagrant plugin install vagrant-reload
```

## The "builder" and "runner" VMs

The number of CPUs and the memory size differs between the "Vagrantfile-builder" and "Vagrantfile-runner".

In our work we have two types of machines "runners" and "builders".
Runners are single core machines with 8GB of memory, and builders are 4 core machines with 16GB of memory.
The latter is used for building gem5 binaries while the former is used for running instances of gem5.
You can expect each machine to take up approximately 60GB of disk space though VMs will consume the disk space they require.

The "Vagrantfile-builder" file is set to create a runner machine and the "Vagrantfile-builder" file is set to create a builder machine.

Specifying which Vagrantfile to use is done by setting the `VAGRANT_VAGRANTFILE` environment variable.

## Creating the virtual machine

Each VM on your host system must have a unique name.
Give the VM to be created a unique name by setting the `<VM NAME>` variables in the Vagrantfile you wish to utilize.

Then run:

```sh
VAGRANT_VAGRANTFILE=<VAGRANTFILE> vagrant up --provider=libvirt
```

This should automatically create your machine, as well as configure and start up a Github Actions runner.
You can check the status of the runner here: https://github.com/<account>/<repo>/settings/actions/runners

If the runner ever shows as offline, you can rerun the `vagrant up --provider=libvirt` command to make sure everything is working properly.

If you wish to create more than one runner you must edit the `<VM NAME>` in the Vagrant file.

## Helper scripts

The "vm_manager" script can be used to set up multiple builder and runner VMs.
To use this script simply modify the `NUM_RUNNERS`, `NUM_BUILDERS`, `RUNNER_PREFIX`, and `BUILDER_PREFIX` variables to the desired values.
Then run the script with:

```sh
./vm_manager.sh
```

This script will create any VMs that don't already exist and ensure those that do exists are running.

If you wish to destroy all the VMs you can run:

```sh
./vm_manager.sh destroy
```

**Note:** This script assumes "VAGRANT_HOME" is set to the CWD.

## Improving stability

Occasionally GitHub runner services, or VMs, go down. This is often silent and
usually only noticable from going to the GitHub repo page "settings" -> "actions" -> "runners" and observing the status.
When the VMs or the service stop working they need restarted.
To do so you can sun `./vm_manager.sh`. This will cycle through the VMs and execute a `vagrant up` command.
This does one of three things depending on the state of the VM:

1. If the VM is down this will bring the VM back online and start the GitHub runner service.
2. If the VM is up but the GitHub runner service is down, this will start the GitHub runner service.
3. If the VM is up and the GitHub runner service is running (i.e., everything is fine) then this does nothing.

Given there is no harm in running this command frequently, we recommend setting up a cron job to automatically execute `./vm_manager.sh` every few hours.

## Troubleshooting

### The default libvirt disk image storage pool is on the wrong drive

By default libvirt will store disk images in "/var/lib/libvirt/images".
This is not ideal as it is on a small root partition.
A solution to this is to change the default storage location.
To do so, do the following:

```sh
virsh pool-list --all # Confirm here a "default" pool exist. We'll modify this.
virsh pool-dumpxml default >default-pool.xml # We take a dump of the default then removed it.
virsh pool-destroy default
virsh pool-undefine default
vim default-pool.xml # Change the image path to the desired path
virsh pool-define default-pool.xml # From here we re-add the default.
virsh pool-start default
virsh pool-autostart default
```
