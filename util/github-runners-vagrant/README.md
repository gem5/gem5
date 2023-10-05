# Setting up a Github Actions Runner with Vagrant

This directory provides a way to setup Github Actions runners using Vagrant to host them in Virtual machines.

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

## Creating the virtual machines

The Vagrantfile in this directory defines the VMs that can be built and used to create a GitHub Actions runner.
This standard VM has 4-cores, 16GB of RAM, and 60GB of disk space.
This is sufficient to both compile gem5 and run most simulations.

At the top of the Vagrantfile, there are a few variables that must be set prior to creating the VMs.

* `NUM_RUNNERS`: The number of runners to create.
* `PERSONAL_ACCESS_TOKEN`: The GitHub personal access token to use.
You can generate a Personal Access Token [here](https://github.com/settings/tokens)
Make sure to set admin permissions on this token.
* `GITHUB_ORG`: The GitHub organization to add the runners to.
E.g., if the URL to your organization is https://github.com/orgs/gem5, then the variable should be set to "gem5".
* `HOSTNAME` : The hostname of the VM to be created (note, this will be appended with a number to create a unique hostname for each VM).
E.g., if set to `my-machine` and the number of runners set to `2`, two VMs will be created.
One called `my-machine-1` and the other `my-machine-2`.

When set simply run:

```sh
vagrant up --provider=libvirt
```

This should automatically create your machines then configure and start up a Github Actions runner in each.
You can check the status of the runner here: https://github.com/organizations/{GITHUB_ORG}/settings/actions/runners

If the runner ever shows as offline, you can rerun the `vagrant up --provider=libvirt` command to make sure everything is working properly.

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
