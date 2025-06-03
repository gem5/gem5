## rocm-build Dockerfile
The Dockerfile in this directory is used to build applications to be run with GPU full system.
Applications targeting AMD's ROCm GPU framework can be built using this docker (e.g., HIP, HSA, OpenCL, etc.).
The current major ROCm version targeted is 6.4.
This version matches the disk image provided in gem5-resources.

The purpose of this docker image is to allow building applications without requiring ROCm to be installed on the host machine.
If you have ROCm installed locally and the version matches the disk image of the simulated system, this docker is not required and you may build on the host normally.
This docker is also not the disk image used to simulated GPU full system applications (i.e., not an input to gem5 itself).

### Building the docker image
```sh
docker build -t <image_name> .
```

For example:

```sh
docker build -t rocm64-build .
```

### Building an application
Building an application requires that docker run in a directory which has access to all files needed to build.
The simplest example would be `square` in the `gem5-resources` repository.
Square provides a Makefile and has only one input file:

```sh
cd gem5-resources/src/gpu/square
docker run --rm -u $UID:$GID -v $PWD:$PWD -w $PWD rocm64-build make -f Makefile.default
```

More complex applications, such as applications requiring m5ops, applications with multiple build steps, or paths with symlinks require more complex --volume command line options.
See the docker documentation to figure out how to set the volumes to build your application.
