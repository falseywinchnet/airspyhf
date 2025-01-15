# User mode driver for Airspy HF+ 

This repository contains host software (Linux/Windows) for Airspy HF+, a high performance software defined radio for the HF and VHF bands. This version is modified to make it able to compile and run appropriately on MUSL linux, the latest versions of raspberry pi OS(with the latest libusb 1.0.27), and with as few warnings or errors as possible.

## Please use the official version at https://github.com/airspy/airspyhf if you dont know c++. this modded version is not supported.


It is also refactored for a performance boost and has very vectored code.
it has been changed to no longer perform any kind of tapering or buffer skipping internally, choosing instead to trust SDR software writers. This can result in a large initial DC bump, so the first few hundred MS after starting the radio or changing IQ frequency should be dropped or tapered appropriately. 

2. different window function commented, you can try it
3. some other tweaks to taste - possibly more decodes on most bands - adjusts iq constantly

http://www.airspy.com/airspy-hf-plus

## How to build host software on Windows:

### For VisualStudio 2022 or later:

* `git clone https://github.com/airspy/airspyhf.git host`
* Download https://github.com/libusb/libusb/releases/download/v1.0.27/libusb-1.0.27.7z
* Extract **libusb-1.0.27.7z** to host directory
  * You should have **host\libusb-1.0.27**
* Download ftp://mirrors.kernel.org/sourceware/pthreads-win32/pthreads-w32-2-9-1-release.zip
* Extract **pthreads-w32-2-9-1-release.zip** to host directory
  * You should have **host\libpthread-2-9-1-win**
* Navigate to **src** and Launch **airspyhf.sln** with VisualStudio 2013 or later
* In Visual Studio, choose **Release**, **x86** or **x64** then **Build Solution**
*if visual studio cannot find the libraries, manually edit the c++ and linker includes to use the appropriate /include/ directory
*for libusb, use the "{arch}/dll" folder if compiling with clang, and copy the dll into the directory with your airspyhf.dll-
*the version of libusb this compiles against is not the same as the one which ships with sdrsharp

## How to build the host software on Linux:

### Prerequisites for Linux (Debian/Ubuntu/Raspbian):


* `sudo apt-get install build-essential cmake pkg-config libudev-dev` 
you'll need to make libusb if your platform's libusb isnt up to date yet. it probably isn't.
* `wget https://github.com/airspy/airspyhf/archive/master.zip`
* `tar -xfj https://github.com/libusb/libusb/releases/download/v1.0.27/libusb-1.0.27.tar.bz2`
* `cd libusb-1.0.27`
except that hilariously, thanks to systemd, which i wish would stop existing, there isnt even headers anymore- it ate them
* `wget https://raw.githubusercontent.com/systemd/systemd/main/src/libudev/libudev.h`
* `sudo mv libudev.h /usr/include/`
* `./configure --prefix=/usr --disable-static`
* `make`
* `sudo make install`
* `cd .../`

I initially modified some of the c to make it compile on musl. i still think it will work fine there, but some of these instructions may differ for you.
spyserver depends on a handful of libraries as part of glibc which are not properly emulated with gcompat, so you'll have to make do
### Build host software on Linux:

* `wget https://github.com/airspy/airspyhf/archive/master.zip`

* `unzip master.zip`

* `cd airspyhf-master`

* `mkdir build`

* `cd build`
* `cmake ../ -DLIBUSB_LIBRARIES="/usr/lib/libusb-1.0.so" -DLIBUSB_INCLUDE_DIRS="/usr/include/libusb-1.0" -DINSTALL_UDEV_RULES=ON`
* `make`
* `sudo make install`
* `sudo ldconfig`

## Clean CMake temporary files/dirs:

`cd airspyhf-master/build`

`rm -rf *`

## requirement to make it work
"LD_PRELOAD=/usr/lib/libusb-1.0.so airspyhf_info/spyserver/etc" if your system has an old libusb it depends on, you'll have to manually bind to this on linux


note that if you blindly followed the instructions above you have just installed the normal airspy library and not the one in this repo since i did not change the wget directive. 

## Principal authors:

Ian Gilmour <ian@sdrsharp.com> and Youssef Touil <youssef@airspy.com> 


http://www.airspy.com

This file is part of Airspy HF (with user mode driver based on Airspy R2, itself based on HackRF project see http://greatscottgadgets.com/hackrf/).
