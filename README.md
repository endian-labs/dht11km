dht11km
=======

DHT11/22 Linux kernel driver for Raspberry Pi based on code from Nigel Morton.

How to build the driver on a host
---------------------------------

Assuming that you have the RPI kernel with source on your host, issue the following command (replace the KDIR with your kernel path):

$ make ARCH=arm CROSS_COMPILE=arm-linux-gnueabi- KDIR=~/Documents/pi-linux

Copy the driver to the target
-----------------------------

Change IP address to match your RPI.

scp dht11km.ko pi@192.168.3.179:


Insert the driver for DHT22 verbose output
------------------------------------------

$ sudo insmod dht11km.ko gpio_pin=4 format=4

If you use DHT11 use format=3.

Create a dht device
-------------------

$ sudo mknod /dev/dht11 c 80 0

Test the device
---------------

$ cat /dev/dht11 
Temperature: 20.9C
Humidity: 41.0
Result:OK

