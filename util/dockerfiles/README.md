# gem5 Dockerfiles

This directory contains Dockerfiles used to create images used in the gem5 project.

## Building

The "docker-bake.json" file defines the building of each image currently used in the gem5 project.

To build all the images run:

```sh
docker buildx bake
```

To build a specific image run:

```sh
docker buildx bake <image-name>
```

E.g., for build of the gpu-fs image, run:

```sh
docker buildx bake gpu-fs
```

This images are stored on the GitHub Container Registry, at ghcr.io/gem5.
To push the images to the registry, run:

```sh
docker buildx bake --push
```

Or to push a specific image, run:

```sh
docker buildx bake --push <image-name>
```

**Note**: Pushing to the container registry requires specicial permissions.
Please contact the gem5 maintainers for more information.
