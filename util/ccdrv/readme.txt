This driver will read the address you point it to [count] times and 
print the results to the systemlog.

To build the driver (Linux 2.6.X only) execute:
make -C /path/to/linux-2.6.X/ SUBDIRS=$PWD modules


Insmodding the kernel module without options will print
the device addresses of eth0 and eth1 if they exist.

Insmodding the kernel module with the options:
dataAddr=0xXXXXXXXXX and count=XXXXX 

will read a long at addr dataAddr count times and return.

Between runs you need to rmmod the module from the kernel.


