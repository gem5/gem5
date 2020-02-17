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
# Copyright (c) 2015 University of Illinois Urbana Champaign
# All rights reserved
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


# This is a wrapper script to run a dist gem5 simulations.
# See the usage_func() below for hints on how to use it. Also,
# there are some examples in the util/dist directory (e.g.
# see util/dist/test-2nodes-AArch64.sh)
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
# addresses - among other things (see util/dist/bootscript.rcS).
#
# Each gem5 process will create an m5out.$GEM5_RANK directory for
# the usual output files. Furthermore, there will be a separate log file
# for each ssh session (we use ssh to start gem5 processes) and one for
# the server. These are called log.$GEM5_RANK and log.switch.
#


# print help
usage_func ()
{
    echo "Usage:$0 [-debug] [-n nnodes] [-r rundir] [-c ckptdir] [-p port] [-sw switch]  [--sw-args sw_args] [-fs fullsystem]  [--fs-args fs_args] [--cf-args conf_args] [--m5-args m5_args] -x gem5_exe "
    echo "     -debug    : debug mode (start gem5 in gdb)"
    echo "     nnodes    : number of gem5 processes"
    echo "     rundir    : run simulation under this path. If not specified, current dir will be used"
    echo "     ckptdir   : dump/restore checkpoints to/from this path. If not specified, current dir will be used"

    echo "     fullsystem: fullsystem config file"
    echo "     fs_args   : fullsystem config specific argument list: arg1 arg2 ..."
    echo "     port      : switch listen port"
    echo "     switch    : switch config file"
    echo "     sw_args   : switch config specific argument list: arg1 arg2 ..."
    echo "     conf_args : common (for both fullsystem and switch) config argument list: arg1 arg2 ..."
    echo "     gem5_exe  : gem5 executable (full path required)"
    echo "     m5_args   : common m5 argument list (e.g. debug flags): arg1 arg2 ..."
    echo "Note: if no LSF slots allocation is found all proceses are launched on the localhost."
}

# Process (optional) command line options
FS_ARGS=" "
SW_ARGS=" "
CF_ARGS=" "
M5_ARGS=" "
while (($# > 0))
do
    case "x$1" in
        x-debug)
            GEM5_DEBUG="-debug"
            shift 1
            ;;
        x-n|x-nodes)
            NNODES=$2
            shift 2
            ;;
        x-r|x-rundir)
            RUN_DIR=$2
            shift 2
            ;;
        x-c|x-ckptdir)
            CKPT_DIR=$2
            shift 2
            ;;
        x-p|x-port)
            SW_PORT=$2
            shift 2
            ;;
        x-s|x-switch)
            SW_CONFIG=$2
            shift 2
            ;;
	x--sw-args)
	    CUR_ARGS="SW_ARGS"
	    shift 1
	    ;;
        x-f|x-fullsystem)
            FS_CONFIG=$2
            shift 2
            ;;
	x--fs-args)
	    CUR_ARGS="FS_ARGS"
	    shift 1
	    ;;
	x--cf-args)
	    CUR_ARGS="CF_ARGS"
	    shift 1
	    ;;
	x--m5-args)
	    CUR_ARGS="M5_ARGS"
	    shift 1
	    ;;
	x-x)
	    GEM5_EXE=$2
	    shift 2
	    ;;
	x-*)
	    [ -n "$CUR_ARGS" ] || { echo "Unexpected arg: $1"; usage_func; exit -1; }
	    case "x$2" in
		x-*|x)
		    eval $CUR_ARGS=\"${!CUR_ARGS} $1\"
		    shift 1
		    ;;
		*)
		    eval $CUR_ARGS=\"${!CUR_ARGS} $1 $2\"
		    shift 2
		    ;;
	    esac
	    ;;
        *)
            echo "Unknown arg: $1"
	    usage_func
	    exit 1
            ;;
    esac
done

# Default values to use (in case they are not defined as command line options)
DEFAULT_FS_CONFIG=$M5_PATH/configs/example/fs.py
DEFAULT_SW_CONFIG=$M5_PATH/configs/dist/sw.py
DEFAULT_SW_PORT=2200

[ -z "$FS_CONFIG" ] && FS_CONFIG=$DEFAULT_FS_CONFIG
[ -z "$SW_CONFIG" ] && SW_CONFIG=$DEFAULT_SW_CONFIG
[ -z "$SW_PORT" ] && SW_PORT=$DEFAULT_SW_PORT
[ -z "$NNODES" ] && NNODES=2
[ -z "$RUN_DIR" ] && RUN_DIR=$(pwd)
[ -z "$CKPT_DIR" ] && CKPT_DIR=$(pwd)

#  Check if all the executables we need exist
[ -f "$FS_CONFIG" ] || { echo "FS config ${FS_CONFIG} not found"; exit 1; }
[ -f "$SW_CONFIG" ] || { echo "Switch config ${SW_CONFIG} not found"; exit 1; }
[ -x "$GEM5_EXE" ]   || { echo "Executable ${GEM5_EXE} not found"; exit 1; }
# make sure that RUN_DIR exists
mkdir -p $RUN_DIR > /dev/null 2>&1

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
[ "x$LSB_MCPU_HOSTS" != "x" ] || LSB_MCPU_HOSTS="127.0.0.1 $NNODES"
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

# function to clean up and abort if something goes wrong
abort_func ()
{
    echo
    echo "KILLED $(date)"
    # Try to Kill the server first. That should trigger an exit for all connected
    # gem5 processes.
    [ "x$SW_PID" != "x" ] && kill $SW_PID 2>/dev/null
    sleep 20
    # (try to) kill gem5 processes - just in case something went wrong with the
    # server triggered exit
    bname=$(basename $GEM5_EXE)
    killall -q -s SIGKILL $bname
    for h in ${HOSTS[@]}
    do
	ssh $h killall -q -s SIGKILL $bname
    done
    sleep 5
    # kill the watchdog
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
        kill -0 $SW_PID || ((NDEAD+=1))
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

# This function launches the gem5 processes. The only purpose is to enable
# launching gem5 processes under gdb control for debugging
start_func ()
{
      local N=$1
      local HOST=$2
      local ENV_ARGS=$3
      shift 3
      if [ "x$GEM5_DEBUG" != "x" ]
      then
	      echo "DEBUG starting terminal..."
	      MY_ARGS="$@"
	      xterm -e "gdb --args $MY_ARGS" &
      else
        ssh $HOST $ENV_ARGS "$@" &> $RUN_DIR/log.$N &
      fi
}

# block till the gem5 process starts
connected ()
{
    FILE=$1
    STRING=$2
    echo -n "waiting for $3 to start "
    while : ;
    do
        kill -0 $4 || { echo "Failed to start $3"; exit -1; }
        [[ -f "$FILE" ]] &&                                                   \
        grep -q "$STRING" "$FILE" &&                                          \
        echo -e "\nnode #$3 started" &&                                       \
        break

        sleep 2
        echo -n "."
    done
}

# Trigger full clean up in case we are being killed by external signal
trap 'abort_func' INT TERM

# env args to be passed explicitly to gem5 processes started via ssh
ENV_ARGS="LD_LIBRARY_PATH=$LD_LIBRARY_PATH M5_PATH=$M5_PATH"

#cleanup log files before starting gem5 processes
rm $RUN_DIR/log.switch > /dev/null 2>&1

# make sure that CKPT_DIR exists
mkdir -p $CKPT_DIR/m5out.switch > /dev/null 2>&1
# launch switch gem5
SW_HOST=${HOSTS[0]}
echo "launch switch gem5 process on $SW_HOST ..."
start_func "switch" $SW_HOST "$ENV_ARGS" $GEM5_EXE -d $RUN_DIR/m5out.switch   \
          $M5_ARGS                                                            \
          $SW_CONFIG                                                          \
          $SW_ARGS                                                            \
          $CF_ARGS                                                            \
          --checkpoint-dir=$CKPT_DIR/m5out.switch                             \
          --is-switch                                                         \
          --dist-size=$NNODES                                                 \
          --dist-server-port=$SW_PORT
SW_PID=$!

# block here till switch process starts
connected $RUN_DIR/log.switch "tcp_iface listening on port" "switch" $SW_PID
LINE=$(grep -r "tcp_iface listening on port" $RUN_DIR/log.switch)

IFS=' ' read -ra ADDR <<< "$LINE"
# actual port that switch is listening on may be different
# from what we specified if the port was busy
SW_PORT=${ADDR[5]}

# Now launch all the gem5 processes with ssh.
echo "START $(date)"
n=0
for ((i=0; i < ${#HOSTS[@]}; i++))
do
    h=${HOSTS[$i]}
    for ((j=0; j < ${NCORES[i]}; j++))
    do
        #cleanup log files before starting gem5 processes
        rm $RUN_DIR/log.$n > /dev/null 2>&1
        # make sure that CKPT_DIR exists
        mkdir -p $CKPT_DIR/m5out.$n > /dev/null 2>&1
	    echo "starting gem5 on $h ..."
	    start_func $n $h "$ENV_ARGS" $GEM5_EXE -d $RUN_DIR/m5out.$n       \
                       $M5_ARGS                                               \
                       $FS_CONFIG                                             \
                       $FS_ARGS                                               \
                       $CF_ARGS                                               \
                       --checkpoint-dir=$CKPT_DIR/m5out.$n                    \
	               --dist                                                 \
	               --dist-rank=$n                                         \
	               --dist-size=$NNODES                                    \
                       --dist-server-name=${HOSTS[0]}                         \
                       --dist-server-port=$SW_PORT
	    SSH_PIDS[$n]=$!
	((n+=1))
    done
done

# Wait here if it is a debug session
[ "x$GEM5_DEBUG" == "x" ] || {  echo "DEBUG session"; wait $SW_PID; exit -1; }

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
wait $SW_PID || ((NFAIL+=1))

# all done, let's terminate the watchdog
kill $WATCHDOG_PID 2>/dev/null

if ((NFAIL==0))
then
    echo "EXIT $(date)"
else
    echo "ABORT $(date)"
fi
