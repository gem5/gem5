# Developing gem5 in a container

The default configuration provides the tools to build and debug the checked-out
source. It does not install a separate gem5 executable. Build a workspace binary
with, for example, `scons build/ALL/gem5.opt -j2`.

The **gem5 Demonstration Container** configuration additionally installs
`gem5-release`, currently built from v25.1.0.1. This is a release executable for
teaching and examples; it does not include changes made in the workspace. It uses
`gem5.fast` to avoid distributing debug information for the release binary.
Use the workspace's `gem5.opt` or `gem5.debug` when debugging simulator changes.
Choose the configuration in the Codespaces creation options or VS Code's
**Dev Containers: Reopen in Container** command.

To build the images locally, run the following from `util/dockerfiles`:

```sh
docker buildx bake devcontainer
docker buildx bake devcontainer-demo
```

The demo build accepts `GEM5_VERSION` and `BUILD_JOBS` build arguments. The default
of two build jobs limits memory pressure, including when building under QEMU:

```sh
docker buildx bake devcontainer-demo \
    --set devcontainer-demo.args.GEM5_VERSION=25.1.0.1 \
    --set devcontainer-demo.args.BUILD_JOBS=2
```

Both configurations retain the Ubuntu 24.04 environment used by most gem5 CI
jobs. Formatting is provided by the repository's pinned pre-commit hooks.

## Guest workloads and disk images

Choose **gem5 Workload Development Container** for QEMU, `qemu-img`, and the
AArch64, RISC-V, and x86-64 cross-compilers. It also enables Docker-in-Docker for
nested build environments. The default and demo configurations omit these tools
and the nested Docker daemon. Building gem5 itself does not require a guest
cross-compiler. Installing QEMU does not grant access to host KVM; accelerated
guests still require an accessible `/dev/kvm`.

Build this image with `docker buildx bake devcontainer-workloads` from
`util/dockerfiles`.

## Image build dependencies

Bake connects all three images directly to the Ubuntu 24.04 base target. Local
base-Dockerfile changes therefore apply without first publishing a base image.
The demo target exports intermediate builder layers with `mode=max`; its
registry cache can be substantially larger than the final image. Other targets
retain the smaller default cache mode.

A local single-platform build can be loaded into Docker for testing:

```sh
docker buildx bake devcontainer --load \
    --set '*.platform=linux/arm64'
```

Use `linux/amd64` on an x86 host. Both supported host architectures can simulate
guest architectures other than their own; RISC-V is not an image host platform.

## Python tooling and hooks

Setup installs `requirements.txt` into the workspace's ignored `.venv` and
initializes the pinned pre-commit hook environments, including clang-format.
The terminal and Python extension use this virtual environment. Distro Python
modules remain available for tools such as SCons and pydot; development-tool
pins are installed in the virtual environment rather than system Python.

After changing branches or requirements, rerun `.devcontainer/on-create.sh` to
refresh the tools and hooks. Setup is safe to repeat.

## Codespaces prebuilds and caches

Configure a Codespaces prebuild for `develop` in the repository's **Settings >
Codespaces > Prebuild configuration**. Use the default devcontainer and an
update frequency appropriate to usage. This is a repository setting and is not
enabled merely by merging these files.

`updateContentCommand` installs requirements and prepares hook environments on
prebuild updates. `postCreateCommand` installs the hooks in each user's checkout.
No simulator build is performed automatically; use the build tasks for the ISA
you need. If source builds are later added to prebuilds, put incremental work in
`updateContentCommand` and measure the compute and storage cost first.

Caches live in the ignored `.devcontainer-cache` directory in the workspace.
This preserves them across both Codespaces rebuilds and local container rebuilds
without relying on a writable parent directory or a particular repository name.
`GEM5_RESOURCE_DIR` selects its `resources` subdirectory; explicit resource paths
in a configuration or test still take precedence. Compiler calls use ccache with
a 2 GiB limit. Use `ccache --show-stats` to inspect it. Resource downloads and
pre-commit environments have no automatic size cap; remove unused entries when
space is needed. The workspace `.venv` is refreshed during content setup.

The existing 8-CPU, 16-GB-memory and 32-GB-storage requirements remain unchanged.
Large full-system resources or multiple build variants may need more storage;
select build parallelism for the available memory rather than blindly using
all CPUs.

## Editor tasks and debugging

Run **gem5: build** to choose a build configuration and job count. It builds
`gem5.opt` and generates a matching compilation database. **gem5: generate
compiler commands** generates just the database. Both tasks update an ignored
symlink used by C++ IntelliSense, so it follows the selected build configuration.

**gem5: list quick suites** lists the ALL-ISA quick TestLib suites. Copy a suite
UID into **gem5: run selected quick suite** to run it, including required builds.
This task does not skip builds or run every quick suite automatically.

Build the desired configuration, open a gem5 Python configuration script, then
choose **gem5: debug current configuration script** and the same build
configuration. GDB launches the workspace binary with the open script. Add
script-specific arguments to the launch configuration when needed. This debugs
simulator C++; it does not attach a Python debugger to embedded Python code.

## Container user and workspace ownership

All configurations run development commands as `gem5`, with a writable home and
passwordless sudo for additional packages. VS Code can map this user's UID/GID
when opening a local Linux bind mount. Setup discovers the actual checkout
location and adds an exact Git trust entry only when ownership requires it;
forks and renamed directories do not need `/workspaces/gem5`.
