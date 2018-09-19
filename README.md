------------
Description:
------------

This is an example of capturing Boson Video (USB) using V4L2 (Video for Linux 2) and OpenCV to display the video.
Boson USB has two modes, AGC-8 bits and RAW-16bits.
Video in 16 bits is not paintable so a linear transformation needs to happen before displaying the image. We call this
process AGC, and in this example we use the most basic one (LINEAR). Then we use standard call to paint in GREY.
Video is 8 bits is directly paintable, linear transformation happens inside the camera, not further actions need to happen in SW.

To Display Boson data we are using OpenCV to convert from YUV to RGB.

Additionally BosonUSB can display frames in YUYV format from ordinary camera built in laptops  

--------------
How to use it:
--------------
```
 BosonUSB [-r][-y][-z [2..4]][-s [bBv]][-f name][-d [num]]
	r	  : raw16 bits video input (default)
	y	  : agc-8 bits video input
	z [2..4]  : zoom factor
	f <name>  : record TIFFS in Folder <name>
	s [bBv]   : camera size : b=boson320, B=boson640, v=video 640x480
	d [num]   : linux video port

./BosonUSB               -> opens Boson320 /dev/video0 in RAW16 mode
./BosonUSB -r            -> opens Boson320 /dev/video0 in RAW16 mode
./BosonUSB -y            -> opens Boson320 /dev/video0 in AGC-8bits mode
./BosonUSB -sB -d1       -> opens Boson640 /dev/video1 in RAW16 mode
./BosonUSB -sB -y -d2    -> opens Boson640 /dev/video2 in AGC-8bits mode
./BosonUSB -f cap        -> creates a folder named 'cap' and inside TIFF files (raw16, agc, yuv) will be located.
```
----------
To compile
----------

This SW uses some libraries as v4l2 and OpenCV, they need to be installed first in the PC.
They are not part of this package.

Running cmake to create the build system in the current directory.
```
cmake .
make
```

(if CMakeCache.txt exists remove it first time)


-----------------------------
How to clean the full project
-----------------------------
```
make clean
rm -rf CMakeFiles
rm CMakeCache.txt
rm cmake_install.cmake
rm Makefile
```
-----------------------------------
Compilation using provided makefile
-----------------------------------
Program can be compiled using provided simple makefile instead CMake:
```
make -f MakefileBoson
```
or cleaned:
```
make -f MakefileBoson clean
```
