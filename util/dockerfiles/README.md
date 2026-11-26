# The gem5 Dockerfiles

This directory contains the Dockerfiles used to build the gem5 Docker images.
The Docker images are used to run gem5 in a containerized environment.

Ubuntu 26.04 is the default environment for development and CI. The
`ubuntu-26.04_all-dependencies` image includes optional dependencies, while
`ubuntu-26.04_min-dependencies` checks that gem5 builds with a minimal package
set. The minimal image does not define the minimum supported dependency
versions; those continue to follow the oldest supported Ubuntu LTS release.

The Ubuntu 22.04 and 24.04 all-dependencies images remain available for distro
compatibility checks. Clang 14 through 16 also retain Ubuntu 24.04 bases because
Ubuntu 26.04 does not package those compilers. The GPU application image uses
Ubuntu 24.04 for its ROCm 7.0 repositories.

## The Docker Registry

We use the Github Container Registry to host the gem5 Docker images.
The images are available at the [ghcr.io/gem5] URI.

### Pulling the Docker Images

You can pull the gem5 Docker images using the following command:

```sh
# Example: Pulling the gem5 Ubuntu 26.04 image with all dependencies
docker pull ghcr.io/gem5/ubuntu-26.04_all-dependencies:latest
```

## Building the Docker Images

The gem5 Dockerfiles are available in this directory.
All the currently supported Docker images, stored in the registry, are built using these Dockerfiles.

### Docker buildx

The Dockerfiles are built using the Docker buildx feature. The buildx feature is used to build multi-platform images which allow for a single Docker image container variants of the environment for a given set of hardware platforms (most notable ISAs). The buildx feature is available in Docker 19.03 and later versions.

In order to build the multi-platforms images, you need to create a buildx builder.
Our Docker images are built for the following platforms: `linux/amd64`, `linux/arm64`, and `linux/riscv64` (note: `linux/riscv64` is not often used as the RISC-V architecture is not yet fully supported and worth targeting.).

To use the buildx feature, you need to have the QEMU system emulators installed on your system for the target platforms.
If you're working on a system with APT you can install the QEMU system emulators using the following command:

```sh
sudo apt install -y qemu-system-arm64 qemu-system-x86 qemu-system-riscv64
```

For other systems, you can refer to the QEMU documentation to install the system emulators.

To create a buildx builder with the QEMU emulators you can use the following command:

```sh
docker buildx create --name mybuilder --bootstrap --use --platform linux/aarch64,linux/amd64,linux/riscv64
```

This creates a buildx builder named `mybuilder` with the specified platforms then and sets it as the active builder.

In this setup we store the buildx configurations in the "docker-bake.hcl" file.
This specifies how our docker images are built and what platforms they are built for.
It is worth consulting these files and noting the "targets" and "groups".
These can be passed to the buildx command to build that target image or group of images.

For example, the following will build the "ubuntu-26.04_all-dependencies" image:

```sh
docker buildx bake ubuntu-26-04_all-dependencies
```

And the following will build all the gcc-compiler images:

```sh
docker buildx bake gcc-compiler
```

If no target is specified all the images will be built.

```sh
docker buildx bake
```

## Pushing the Docker Images

To push the Docker images to the Github Container Registry, you can use the following command:

```sh
docker buildx bake <target/group> --push
```

However, you need to authenticate with the Github Container Registry, creating a token with write access to the gem5 GitHub Docker registry.

### Authenticating with the Github Container Registry

To push  images, you need to authenticate with the Github Container Registry. You can authenticate using a Github Personal Access Token (PAT). The PAT can be generated from the Github settings. The PAT should have the `write:packages` scope to read the Github Container registry images.

When you have the PAT, you can authenticate using the following command:

```sh
echo $GITHUB_PAT | docker login ghcr.io -u $GITHUB_USERNAME --password-stdin
```

## gem5 Docker Tags

As is standard with Docker images, latest image created for each Dockerfile is tagged as `latest`.

Images compatible with a gem5 release series are tagged with that series. For
example, `ghcr.io/gem5/ubuntu-26.04_all-dependencies:v26-0` is an image for gem5
v26.0. Use the matching release tag when reproducing an older build; the
`latest` tag follows the current development environment.
