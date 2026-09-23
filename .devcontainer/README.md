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
