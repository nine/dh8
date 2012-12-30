dh8
===

Control software for the [Diavite DH-8](http://diavite.com/index.php/dh-8-en.html) 
surface roughness measurement device.


Building:
---------

<code>
cd /path/to/project/root  
mkdir build && cd build  
cmake ..  
make  
</code>

Setup usb connection:
---------------------

Connect the DH-8 via usb to your host-pc.
The command `'dmesg | tail'` shall give a similar output:
<code>
usb 7-1: new full speed USB device number 3 using uhci_hcd  
usb 7-1: New USB device found, idVendor=10c4, idProduct=ea60  
usb 7-1: New USB device strings: Mfr=1, Product=2, SerialNumber=3  
usb 7-1: Product: CP2101 USB to UART Bridge Controller  
usb 7-1: Manufacturer: Silicon Labs  
usb 7-1: SerialNumber: 0001  
cp210x 7-1:1.0: cp210x converter detected  
usb 7-1: reset full speed USB device number 3 using uhci_hcd  
usb 7-1: cp210x converter now attached to ttyUSB0  
</code>

Check the current permissions of the created device:
<code>
ls -lha /dev/ttyUSB0  
crw-rw---T 1 root dialout 188, 0 Jan 10 09:54 /dev/ttyUSB0  
</code>

The root user and the group dialout have read/write permissions to 
the serial device. Make sure that your user "yourusername" is member 
of the group dialout:
<code>
grep dialout /etc/group  
dialout:x:20:yourusername  
</code>

If necessary add your user to the group dialout:
<code>
sudo usermod -a -G dialout yourusername  
</code>

The serial terminal configuration is:
<code>
8 data bit  
1 stop bit  
no parity  
115200 baud  
</code>

Example usage:
--------------
<code>
./src/main --dev /dev/ttyUSB0 --file ../data/20120208_surface04.m --lt 0.48 --lc 0.08   
</code>


Dependencies:
-------------

1. cmake
    cross plattform make 
    http://www.cmake.org/
2. boost program options
    http://www.boost.org/doc/libs/1_48_0/doc/html/program_options.html

These dependencies are provided by following debian (SQUEEZE) packages:
- cmake
- libboost-program-options-dev



LICENSE
-------
This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by 
the Free Software Foundation, either version 3 of the License, or 
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program. If not, see [gpl](www.gnu.org/licenses/).


CONTACT
-------
Please use the contact-form provided by github.

