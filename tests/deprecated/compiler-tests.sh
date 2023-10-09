#!/bin/bash

# This script will run all our supported compilers (see the "images" set)
# against gem5. The images for the latests supported gcc and clang compiler
# versions are run against all built targets. The remainder are evaluated
# against a random shuffling of built targets.

dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
gem5_root="${dir}/.."
build_dir="${gem5_root}/build"

# The per-container Docker memory limit.
docker_mem_limit="18g"

# All Docker images in the gem5 testing GCR which we want to compile with.
images=("gcc-version-12"
        "gcc-version-11"
        "gcc-version-10"
        "gcc-version-9"
        "gcc-version-8"
        "clang-version-14"
        "clang-version-13"
        "clang-version-12"
        "clang-version-11"
        "clang-version-10"
        "clang-version-9"
        "clang-version-8"
        "clang-version-7"
        # The following checks our support for Ubuntu 18.04, 20.04, and 22.04.
        "ubuntu-18.04_all-dependencies"
        "ubuntu-20.04_all-dependencies"
        "ubuntu-22.04_all-dependencies"
        # Here we test the minimum dependency scenario.
        "ubuntu-22.04_min-dependencies"
       )

# A subset of the above list: these images will build against every target,
# ignoring builds_per_compiler.
comprehensive=("gcc-version-12"
               "clang-version-14")

# All build targets in build_opt/ which we want to build using each image.
builds=("ALL"
        "ARM"
        "ARM_MESI_Three_Level"
        "ARM_MESI_Three_Level_HTM"
        "ARM_MOESI_hammer"
        "Garnet_standalone"
        "GCN3_X86"
        "MIPS"
        "NULL"
        "NULL_MESI_Two_Level"
        "NULL_MOESI_CMP_directory"
        "NULL_MOESI_CMP_token"
        "NULL_MOESI_hammer"
        "POWER"
        "RISCV"
        "SPARC"
        "GCN3_X86"
        "VEGA_X86"
        "X86"
        "X86_MI_example"
        "X86_MOESI_AMD_Base")

# The optimizations to use for each build target.
opts=(".opt"
      ".fast")

# The number of build targets to randomly pull from the build target list for
# each compiler. To perform a full comprehensive test which covers every
# possible pair of compiler and build target, set builds_per_compiler equal to
# the expression ${#builds[@]}.
builds_per_compiler=1

# Base URL of the gem5 testing images.
base_url="ghcr.io/gem5"

# Arguments passed into scons on every build target test.
if [ $# -eq 0 ];then
    # If none is sepcified by the user we pass "-j1" (compile on one thread).
    # If `build_args` is left as an empty string, this script will fail.
    build_args="-j1"
else
    build_args="$@"
fi

# Testing directory variables
mkdir -p "${build_dir}" # Create the build directory if it doesn't exist.
test_dir_final="${gem5_root}/compile-test-out"
test_dir="${gem5_root}/.compile-test-out"
exits="${test_dir}/exit-codes.csv"

# Create the testing output directory and files
rm -rf "${test_dir_final}"
rm -rf "${test_dir}"
mkdir "${test_dir}"
touch "${exits}"
echo "compiler,build_target,exit_code" >> "${exits}"

exit_code=0 # We return a non-zero exit code if any of the compilations fail.

for compiler in ${images[@]}; do
    echo "Starting build tests with '${compiler}'..."
    # Generate a randomized list of build targets
    build_permutation=($(shuf -i 0-$((${#builds[@]} - 1)) ))

    builds_count=$builds_per_compiler
    if [[ " ${comprehensive[@]} " =~ " $compiler " ]]; then
        echo "'$compiler' was found in the comprehensive tests. All ISAs will be built."
        builds_count=${#builds[@]}
    fi

    # Slice the first $builds_count entries of the permutation to get our
    # targets for this test
    build_indices=(${build_permutation[@]:0:$builds_count})

    repo_name="${base_url}/${compiler}:latest"

    # Grab compiler image
    docker pull $repo_name >/dev/null

    mkdir "${test_dir}/${compiler}"

    for build_index in ${build_indices[@]}; do
        for build_opt in ${opts[@]}; do
            build="${builds[$build_index]}"
            build_out="build/$build/gem5$build_opt"
            build_stdout="${test_dir}/${compiler}/${build}${build_opt}.stdout.txt"
            build_stderr="${test_dir}/${compiler}/${build}${build_opt}.stderr.txt"

            # Clean the build
            rm -rf "${build_dir}"

            touch "${build_stdout}"
            touch "${build_stderr}"

            echo "  * Building target '${build}${build_opt}' with '${compiler}'..."

            # Build with container
            {
                docker run --rm -v "${gem5_root}":"/gem5" -u $UID:$GID \
                    -w /gem5 --memory="${docker_mem_limit}" $repo_name \
                    /usr/bin/env python3 /usr/bin/scons --ignore-style \
                    "${build_out}" "${build_args}"
            }>"${build_stdout}" 2>"${build_stderr}"
            result=$?

            echo "${compiler},${build}/gem5${build_opt},${result}" >>"${exits}"

            if [ ${result} -ne 0 ]; then
                exit_code=1
                echo "  ! Failed with exit code ${result}."
            else
                echo "    Done."
            fi
        done
    done
done

mv "${test_dir}" "${test_dir_final}"

exit ${exit_code}
