#!/bin/csh -f

set OS=`uname -s`
set HOST_ARCH=`uname -m`

switch ($OS)
	case Linux:
		set OS_PART=linux
		breaksw
	case SunOS:
		set OS_PART=sol8-64
		breaksw
	case OSF1:
		set OS_PART=tru64-gcc
		breaksw
	default:
                set OS_PART=`echo $OS | sed 's/ /-/g'`
endsw

switch ($HOST_ARCH)
	case i586:
		set ARCH=x86
		breaksw
	case i686:
		set ARCH=x86
		breaksw
	case x86_64:
		set ARCH=amd64
		breaksw
	case sun4u:
		set ARCH=v9
		breaksw
	default:
                set ARCH=`echo $HOST_ARCH | sed 's/ /-/g'`
endsw

echo $ARCH-$OS_PART

