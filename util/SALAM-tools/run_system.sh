#!/bin/bash
# Copyright (c) 2025 Akanksha Chaudhari, Matt Sinclair
# (University of Wisconsin-Madison)
# All rights reserved.
#
# This file contains modifications and/or code derived from:
# gem5-SALAM: https://github.com/TeCSAR-UNCC/gem5-SALAM
#
# The license below extends only to copyright in the software and shall
# not be construed as granting a license to any other intellectual
# property including but not limited to intellectual property relating
# to a hardware implementation of the functionality of the software
# licensed hereunder.  You may use the software subject to the license
# terms below provided that you ensure that this notice is replicated
# unmodified and in its entirety in all distributions of the software,
# modified or unmodified, in source code or in binary form.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met: redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer;
# redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution;
# neither the name of the copyright holders nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

BENCH=""
BENCH_PATH=""
CONFIG_NAME=""
FLAGS=""
BUILD=True
DEBUG=False
PRINT_TO_FILE=False
VALGRIND=False

while [[ $# -gt 0 ]]; do
  case $1 in
    --bench)
      if [[ -z "${2:-}" || "$2" == -* ]]; then
        echo "Missing value for $1"
        exit 1
      fi
      BENCH="$2"
      shift # past argument
      shift # past value
      ;;
    --bench-path)
      if [[ -z "${2:-}" || "$2" == -* ]]; then
        echo "Missing value for $1"
        exit 1
      fi
      BENCH_PATH="$2"
      shift # past argument
      shift # past value
      ;;
    --config-name)
      if [[ -z "${2:-}" || "$2" == -* ]]; then
        echo "Missing value for $1"
        exit 1
      fi
      CONFIG_NAME="$2"
      shift # past argument
      shift # past value
      ;;
    -f|--flags)
      if [[ -z "${2:-}" || "$2" == -* ]]; then
        echo "Missing value for $1"
        exit 1
      fi
      FLAGS="$2"
      shift # past argument
      shift # past value
      ;;
    -d|--debug)
      DEBUG=True
      shift # past argument
      ;;
    -b|--build)
      BUILD=True
      shift # past argument
      ;;
    --no-build)
      BUILD=False
      shift # past argument
      ;;
    -p|--print)
      PRINT_TO_FILE=True
      shift # past argument
      ;;
    -v|--valgrind)
      VALGRIND=True
      shift # past argument
      ;;
    -*)
      echo "Unknown option $1"
      exit 1
      ;;
    *)
      shift # past argument
      ;;
  esac
done

if [ "$BENCH" == "" ]; then
	echo "BENCH env var is not set, exiting"
	exit 1
fi

if [ "$M5_PATH" == "" ]; then
	echo "M5_PATH env var is not set, exiting"
	exit 1
fi

if [ "$ACC_BENCH_PATH" == "" ]; then
	echo "ACC_BENCH_PATH env var is not set, exiting"
	exit 1
fi

if [ "$CONFIG_NAME" == "" ]; then
	CONFIG_NAME="config.yml"
fi

if [ "$BENCH_PATH" == "" ]; then
	BENCH_PATH=$BENCH
fi

if [ ${DEBUG} == True ]; then
	BINARY="gdb --args ${M5_PATH}/build/ARM/gem5.debug"
elif [ ${VALGRIND} == True ]; then
	BINARY="valgrind --leak-check=yes --suppressions=util/valgrind-suppressions --suppressions=util/salam.supp --track-origins=yes --error-limit=no --leak-check=full --show-leak-kinds=definite,possible --show-reachable=no --log-file=$BENCH.log  ${M5_PATH}/build/ARM/gem5.debug" #--gen-suppressions=all
else
	BINARY="${M5_PATH}/build/ARM/gem5.opt"
fi

KERNEL=$ACC_BENCH_PATH/"$BENCH_PATH"/sw/main.elf

SYS_OPTS="--mem-size=16GB \
          --mem-type=DDR4_2400_8x8 \
          --kernel=$KERNEL \
          --machine-type=VExpress_GEM5_V1 \
          --dtb-file=none --bare-metal \
          --cpu-type=DerivO3CPU"

CACHE_OPTS="--caches --l2cache"

OUTDIR=BM_ARM_OUT/$BENCH_PATH/

DEBUG_FLAGS=""

if [ "${FLAGS}"  != "" ]; then
	DEBUG_FLAGS+="--debug-flags="
	DEBUG_FLAGS+=$FLAGS
fi

RUN_SCRIPT="$BINARY $DEBUG_FLAGS --outdir=$OUTDIR \
			$M5_PATH/configs/SALAM/fs.py $SYS_OPTS \
			--accpath=$ACC_BENCH_PATH/$BENCH_PATH \
			--accbench=$BENCH \
			--acccfg=$CONFIG_NAME $CACHE_OPTS"

if (! "$M5_PATH"/util/SALAM-tools/SALAM-Configurator/systembuilder.py --sys-name "$BENCH" --bench-path "$BENCH_PATH" --config-name $CONFIG_NAME) then
	echo "Configurator failed"
	exit 1
fi

if [ $BUILD == True ]; then
  echo "Building Bench"
  if ! make all -C "$ACC_BENCH_PATH/$BENCH_PATH"; then
    echo "Benchmark build failed"
    exit 1
  fi
fi

if [ ${PRINT_TO_FILE} == True ]; then
	mkdir -p "$OUTDIR"
	$RUN_SCRIPT > "${OUTDIR}"/debug-trace.txt
else
	$RUN_SCRIPT
fi
