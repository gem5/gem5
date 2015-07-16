#! /bin/bash

#
# Copyright (c) 2015 ARM Limited
# All rights reserved
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
#
# Authors: Gabor Dozsa


# This is a wrapper script to run a multi gem5 simulations.
# See the usage_func() below for hints on how to use it. Also,
# there are some examples in the util/multi directory (e.g.
# see util/multi/test-2nodes-AArch64.sh)
#
#
# Allocated hosts/cores are assumed to be listed in the LSB_MCPU_HOSTS
# environment variable (which is what LSF does by default).
# E.g. LSB_MCPU_HOSTS=\"hname1 2 hname2 4\" means we have altogether 6 slots
# allocated to launch the gem5 processes, 2 of them are on host hname1
# and 4 of them are on host hname2.
# If LSB_MCPU_HOSTS environment variable is not defined then we launch all
# processes on the localhost.
#
# Each gem5 process are passed in a unique rank ID [0..N-1] via the kernel
# boot params. The total number of gem5 processes is also passed in.
# These values can be used in the boot script to configure the MAC/IP
# addresses - among other things (see util/multi/bootscript.rcS).
#
# Each gem5 process will create an m5out.$GEM5_RANK directory for
# the usual output files. Furthermore, there will be a separate log file
# for each ssh session (we use ssh to start gem5 processes) and one for
# the server. These are called log.$GEM5_RANK and log.server.
#


# print help
usage_func ()
{
    echo "Usage:$0 [-debug] [-n nnodes] [-s server] [-p port] gem5_exe gem5_args"
    echo "     -debug   : debug mode (start gem5 in gdb)"
    echo "     nnodes   : number of gem5 processes"
    echo "     server   : message server executable"
    echo "     port     : message server listen port"
    echo "     gem5_exe : gem5 executable (full path required)"
    echo "     gem5_args: usual gem5 arguments ( m5 options, config script options)"
    echo "Note: if no LSF slots allocation is found all proceses are launched on the localhost."
}


# Process (optional) command line options

while true
do
    case "x$1" in
        x-n|x-nodes)
            NNODES=$2
            shift 2
            ;;
        x-s|x-server)
            TCP_SERVER=$2
            shift 2
            ;;
        x-p|x-port)
            SERVER_PORT=$2
            shift 2
            ;;
        x-debug)
            GEM5_DEBUG="-debug"
            shift 1
            ;;
        *)
            break
            ;;
    esac
done

# The remaining command line args must be the usual gem5 command
(($# < 2)) && { usage_func; exit -1; }
GEM5_EXE=$1
shift
GEM5_ARGS="$*"

# Default values to use (in case they are not defined as command line options)
DEFAULT_TCP_SERVER=$(dirname $0)/../../util/multi/tcp_server
DEFAULT_SERVER_PORT=2200

[ -z "$TCP_SERVER" ]  && TCP_SERVER=$DEFAULT_TCP_SERVER
[ -z "$SERVER_PORT" ] && SERVER_PORT=$DEFAULT_SERVER_PORT
[ -z "$NNODES" ]      && NNODES=2


#  Check if all the executables we need exist
[ -x "$TCP_SERVER" ] || { echo "Executable ${TCP_SERVER} not found"; exit 1; }
[ -x "$GEM5_EXE" ]   || { echo "Executable ${GEM5_EXE} not found"; exit 1; }


declare -a SSH_PIDS
declare -a HOSTS
declare -a NCORES

# Find out which cluster hosts/slots are allocated or
# use localhost if there is no LSF allocation.
# We assume that allocated slots are listed in the LSB_MCPU_HOSTS
# environment variable in the form:
# host1 nslots1 host2 nslots2 ...
# (This is what LSF does by default.)
NH=0
[ "x$LSB_MCPU_HOSTS" != "x" ] || LSB_MCPU_HOSTS="localhost $NNODES"
host=""
for hc in $LSB_MCPU_HOSTS
do
    if [ "x$host" == "x" ]
    then
        host=$hc
        HOSTS+=($hc)
    else
        NCORES+=($hc)
        ((NH+=hc))
        host=""
    fi
done
((NNODES==NH)) || { echo "(E) Number of cluster slots ($NH) and gem5 instances ($N) differ"; exit -1; }
#echo "hosts: ${HOSTS[@]}"
#echo "hosts: ${NCORES[@]}"
#echo ${#HOSTS[@]}


# function to clean up and abort if something goes wrong
abort_func ()
{
    echo
    echo "KILLED $(date)"
    # (try to) kill all gem5 processes on all hosts
    bname=$(basename $GEM5_EXE)
    killall -q $bname
    for h in ${HOSTS[@]}
    do
        ssh $h killall -q $bname
    done
    sleep 3
    # kill the message server and the watchdog
    [ "x$SERVER_PID" != "x" ] && kill $SERVER_PID 2>/dev/null
    [ "x$WATCHDOG_PID" != "x" ] && kill $WATCHDOG_PID 2>/dev/null
    exit -1
}


# We need a watchdog to trigger full clean up if a gem5 process dies
watchdog_func ()
{
    while true
    do
        sleep 30
        ((NDEAD=0))
        for p in ${SSH_PIDS[*]}
        do
            kill -0 $p 2>/dev/null || ((NDEAD+=1))
        done
        kill -0 $SERVER_PID || ((NDEAD+=1))
        if ((NDEAD>0))
        then
            # we may be in the middle of an orderly termination,
            # give it some time to complete before reporting abort
            sleep 60
            echo -n "(I) (some) gem5 process(es) exited"
            abort_func
        fi
    done
}

# This function launches the gem5 processes. We use it only to allow launching
# gem5 processes under gdb control (in the foreground) for debugging
start_func ()
{
    local N=$1
    local HOST=$2
    local ENV_ARGS=$3
    shift 3
    if [ "x$GEM5_DEBUG" != "x" ]
    then
        gdb --args "$@"
    else
        ssh $HOST $ENV_ARGS "$@" &>log.$N & 
    fi
}


# Trigger full clean up in case we are being killed by external signal
trap 'abort_func' INT TERM

# env args to be passed explicitly to gem5 processes started via ssh
ENV_ARGS="LD_LIBRARY_PATH=$LD_LIBRARY_PATH M5_PATH=$M5_PATH"

# launch the mesage server and check if it has started okay
$TCP_SERVER $GEM5_DEBUG $NNODES $SERVER_PORT &>log.server &
SERVER_PID=$!
sleep 2
kill -0 $SERVER_PID || { echo "Failed to start message server"; exit -1; }

# Now launch all the gem5 processes with ssh.
echo "START $(date)"
n=0
for ((i=0; i < ${#HOSTS[@]}; i++))
do
    h=${HOSTS[$i]}
    for ((j=0; j < ${NCORES[i]}; j++))
    do
        echo "starting gem5 on $h ..."
        start_func $n $h "$ENV_ARGS" $GEM5_EXE -d $(pwd)/m5out.$n $GEM5_ARGS \
        --multi                                                              \
        --multi-rank=$n                                                      \
        --multi-server-name=${HOSTS[0]}                                      \
        --multi-server-port=$SERVER_PORT                                     \
        --testsys-toplevel-LinuxArmSystem.boot_osflags="\"GEM5_RANK=$n GEM5_SIZE=$NNODES\""
        SSH_PIDS[$n]=$!
        ((n+=1))
    done
done

[ "x$GEM5_DEBUG" == "x" ] || {  kill $SERVER_PID; echo "DEBUG exit"; exit -1; }

# start watchdog to trigger complete abort (after a grace period) if any
# gem5 process dies
watchdog_func &
WATCHDOG_PID=$!

# wait for exit statuses
((NFAIL=0))
for p in ${SSH_PIDS[*]}
do
    wait $p || ((NFAIL+=1))
done
wait $SERVER_PID || ((NFAIL+=1))

# all done, let's terminate the watchdog
kill $WATCHDOG_PID 2>/dev/null

if ((NFAIL==0))
then
    echo "EXIT $(date)"
else
    echo "ABORT $(date)"
fi
