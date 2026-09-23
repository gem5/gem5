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
