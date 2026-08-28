# GPU

These tests execute the public
`configs/example/gem5_library/x86-mi200-gpu.py` and
`configs/example/gem5_library/x86-mi355x-gpu.py` full-system examples without
requiring a ROCm installation on the host. The two Daily tests boot the GPUFS
image and check that ROCm identifies the simulated GPUs as `gfx90a` and
`gfx950`. The quick pull-request test uses the MI355X example to restore a
checkpoint and verify one `gfx950` GPU kernel result.

All three tests use an Atomic CPU, `ViperBoard`, and the Viper cache hierarchy.
They configure 8 GiB of system memory, 16 GiB of GPU memory, and four compute
units. Four compute units preserve one complete SQC and scalar-cache group
while avoiding the cost of the larger default topologies. The `MI210` and
`MI355X` components supply the driver setup directly; the tests do not use a
GPU-architecture compatibility override.

The examples obtain version `1.0.0` of
`x86-ubuntu-24.04-gpu-img` and `x86-linux-kernel-6.8.0-gpu`. The image is
extracted sparsely to reduce allocated storage, and its AMDGPU driver was
built for the paired kernel.

The shared `configs/example/gem5_library/x86_gpu.py` helper also supports
local `--image` and `--kernel` paths, KVM or Atomic CPUs, a reduced compute-unit
count, additional kernel arguments, and local or resource checkpoints. The
MI300X example uses the same helper even though it is not part of these tests.

## MI355X smoke checkpoint

The public `x86-mi355x-gpu-checkpoint.py` example supports the two
checkpoint-generation stages used by the
`x86-mi355x-gpu-fs-smoke-checkpoint` resource. It uses the same system builder
as the tested MI355X example and defaults to the checkpoint's Atomic CPU and
four-CU topology. First, boot the GPUFS image and stop after the initialized
HIP loader requests its checkpoint:

```bash
build/ALL/gem5.opt \
    configs/example/gem5_library/x86-mi355x-gpu-checkpoint.py \
    --mode=create-loader \
    --checkpoint-output=m5out/mi355x-loader \
    --gpu-application-binary=/path/to/hip-checkpoint-runner.py
```

Then restore that checkpoint, supply the `gfx950` code object through
`m5 readfile`, complete one warm-up dispatch, and save the final checkpoint:

```bash
build/ALL/gem5.opt \
    configs/example/gem5_library/x86-mi355x-gpu-checkpoint.py \
    --mode=create-kernel \
    --checkpoint-directory=m5out/mi355x-loader \
    --checkpoint-output=m5out/mi355x-smoke \
    --gpu-kernel-binary=/path/to/gpu-checkpoint-smoke
```

The final checkpoint contains the initialized HIP loader, the loaded
`gfx950` module, the resolved `_Z9incrementPi` function, its argument
state, and one completed warm-up dispatch. Restoration launches and verifies
a second dispatch. The checkpoint is therefore specific to that smoke kernel
and to the four-CU MI355X topology above. It cannot be used as a generic
post-boot MI355X checkpoint.

The checkpoint also contains the disk's copy-on-write state rather than the
complete base image. Restoration still requires the exact GPUFS disk and
Linux kernel resources used during creation. A local artifact can be checked
with:

```bash
build/ALL/gem5.opt \
    configs/example/gem5_library/x86-mi355x-gpu.py \
    --cpu-type=atomic \
    --num-cus=4 \
    --checkpoint-directory=m5out/mi355x-smoke \
    --kernel-arg=init=/home/gem5/run_gem5_app.sh
```

After validation, publish the final checkpoint as
`x86-mi355x-gpu-fs-smoke-checkpoint`, version `1.0.0`. Publish the matching
`gfx950` code-object source and build recipe as
`x86-mi355x-gpu-fs-smoke`. The quick TestLib suite obtains the final
checkpoint and passes only after the resumed loader prints
`GPU checkpoint restore test passed` to the guest serial output.
