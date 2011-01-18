#!/bin/bash
#
# makeblankimage.sh
# Make a blank M5 disk image
#

while getopts "m" OPT
do
        case "$OPT" in
                m)  MOUNT_IT=1
        esac
done

DEBUG=0

if [ $DEBUG -ne 0 ]; then
    set -x -e
    OUTPUT=""
else
    OUTPUT="> /dev/null 2>&1"
fi

abort() {
    echo $@
    exec /bin/false
}

find_prog() {
    PROG_PATH=`which $1`
    if [ $? -ne 0 ]; then
        abort "Unable to find program $1, check your PATH variable"
    fi
    echo $PROG_PATH
}

run_priv() {
    if [ "$HAVE_SUDO" = "y" ]; then
        eval $SUDO $@ $OUTPUT
    else
        eval $@ $OUTPUT
    fi

    if [ $? -ne 0 ]; then
        abort "Failed to run $@ as root"
    fi
}

usage() {
    abort "Usage: $0 [root-path to copy] [extra ownership commands ...]"
}

# Setup PATH to look in the sbins
export PATH=$PATH:/sbin:/usr/sbin

# Get all of the programs needed, or exit
DD=`find_prog dd`
SFDISK=`find_prog sfdisk`
LOSETUP=`find_prog losetup`
SUDO=`find_prog sudo`
MKE2FS=`find_prog mke2fs`
MKDIR=`find_prog mkdir`
MOUNT=`find_prog mount`
UMOUNT=`find_prog umount`
WHOAMI=`find_prog whoami`
CP=`find_prog cp`
CHOWN=`find_prog chown`

# Prompt for the root password, if needed
CUR_USER=`$WHOAMI`

if [ $# -ge 1 ]; then
    if [ ! $MOUNT_IT ]; then
        ROOT_PATH=$1

        if [ ! -d $ROOT_PATH ]; then
                usage
        fi
   else
        ROOT_PATH=""
   fi
else
    ROOT_PATH=""
fi

if [ ! "$CUR_USER" = "root" ]; then
    echo -n "Do you have sudo access? [y/n] "
    read HAVE_SUDO

    if [ ! "$HAVE_SUDO" = "y" ]; then
        abort "You must have sudo access or run this script as root"
    fi
fi

echo -n "How large do you want this disk image (in MB): "
read USER_SIZE_MB

# size in bytes = SIZE_MB * 1024 * 1024
# size in blocks = SIZE_BYTE / 512
let BLK_SIZE=$USER_SIZE_MB*1024*2

let MAX_LBA=16383*16*63

if [ $BLK_SIZE -ge $MAX_LBA ]; then
    CYLS=16383
    HEADS=16
    SECTORS=63
else
    # Set Sectors
    if [ $BLK_SIZE -ge 63 ]; then
        SECTORS=63
    else
        SECTORS=$BLK_SIZE
    fi

    # Set Heads
    let HEAD_SIZE=$BLK_SIZE/$SECTORS

    if [ $HEAD_SIZE -ge 16 ]; then
        HEADS=16
    else
        HEADS=$BLK_SIZE
    fi

    # Set Cylinders
    let SEC_HEAD=$SECTORS*$HEADS
    let CYLS=$BLK_SIZE/$SEC_HEAD
fi

# Recalculate number of sectors
let BLK_SIZE=$CYLS*$HEADS*$SECTORS

# Get the name of the file and directory to build in
echo -n "What directory would you like to build the image in? "
read IMAGE_DIR

if [ ! -d $IMAGE_DIR ]; then
    abort "The directory $IMAGE_DIR does not exist"
fi

echo -n "What would you like to name the image? "
read IMAGE_NAME

IMAGE_FILE=$IMAGE_DIR/$IMAGE_NAME

# DD the blank image
echo
echo "dd'ing the blank image (this make take a while)..."
eval $DD if=/dev/zero of=$IMAGE_FILE bs=512 count=$BLK_SIZE $OUTPUT
if [ $? -ne 0 ]; then
    abort "Unable to create the blank image $IMAGE_NAME in $IMAGE_DIR"
fi

# losetup the image with no offset to do the fdisk
echo
echo "Binding the image and partitioning..."
run_priv $LOSETUP /dev/loop0 $IMAGE_FILE
if [ $? -ne 0 ]; then
    abort "losetup to /dev/loop0 failed, make sure nothing is setup on loop0 (check by typing 'mount') "
fi

# fdisk the image
run_priv $SFDISK --no-reread -D -C $CYLS -H $HEADS -S $SECTORS /dev/loop0 <<EOF
0,
;
;
;
EOF

# Un-losetup the image
run_priv $LOSETUP -d /dev/loop0

# Mount the image with an offset and make the filesystem
echo
echo "Remounting image and formatting..."
let BASE_OFFSET=63*512

run_priv $LOSETUP -o $BASE_OFFSET /dev/loop0 $IMAGE_FILE

run_priv $MKE2FS /dev/loop0

# If a root path was specified then copy the root path into the image
if [ ! -z "$ROOT_PATH" ]; then
    echo "Copying root from $ROOT_PATH to image file"

    run_priv $MKDIR -p /tmp/mnt

    run_priv $MOUNT /dev/loop0 /tmp/mnt

    run_priv $CP -a $ROOT_PATH/* /tmp/mnt

    run_priv $CHOWN -R root.root /tmp/mnt

    # run extra permissions while disk is mounted
    TOPDIR=`pwd`
    cd /tmp/mnt
    i=2
    while [ $i -le $# ]; do
        run_priv ${!i}
        let i=i+1
    done
    cd $TOPDIR

    run_priv $UMOUNT /tmp/mnt
fi

run_priv $LOSETUP -d /dev/loop0


if [ $MOUNT_IT -eq 1 ]; then
        run_priv mount -o loop,offset=$BASE_OFFSET $IMAGE_FILE /tmp/mnt
else
        echo
        echo "Disk image creation complete."
        echo "To mount the image, run the following commands:"
        echo "# $MOUNT -o loop,offset=$BASE_OFFSET $IMAGE_FILE /mount/point"
        echo
        echo "And to unmount the image, run:"
        echo "# $UMOUNT /mount/point"
fi;
