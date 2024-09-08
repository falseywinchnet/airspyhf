# User mode driver for Airspy HF+ 

This repository contains host software (Linux/Windows) for Airspy HF+, a high performance software defined radio for the HF and VHF bands. This version is modified to make it able to compile and run appropriately on MUSL linux. It is also hinted for pragmas and modern c++ versions for a bit more performance on modern systems.

http://www.airspy.com/airspy-hf-plus

## How to build host software on Windows:

### For VisualStudio 2013 or later:

* `git clone https://github.com/airspy/airspyhf.git host`
* Download https://github.com/libusb/libusb/releases/download/v1.0.27/libusb-1.0.27.7z
* Extract **libusb-1.0.27.7z** to host directory
  * You should have **host\libusb-1.0.27**
* Download ftp://mirrors.kernel.org/sourceware/pthreads-win32/pthreads-w32-2-9-1-release.zip
* Extract **pthreads-w32-2-9-1-release.zip** to host directory
  * You should have **host\libpthread-2-9-1-win**
* Navigate to **src** and Launch **airspyhf.sln** with VisualStudio 2013 or later
* In Visual Studio, choose **Release**, **x86** or **x64** then **Build Solution**

### For MinGW:

`git clone https://github.com/airspy/airspyhf.git host`

`cd host`

`mkdir build`

`cd build`

Normal version:

`cmake ../ -G "MSYS Makefiles" -DLIBUSB_INCLUDE_DIR=/usr/local/include/libusb-1.0/`

Debug version:

`cmake ../ -G "MSYS Makefiles" -DCMAKE_BUILD_TYPE=Debug -DLIBUSB_INCLUDE_DIR=/usr/local/include/libusb-1.0/`

`make`

`make install`


## How to build the host software on Linux:

### Prerequisites for Linux (Debian/Ubuntu/Raspbian):


`sudo apt-get install build-essential cmake libusb-1.0-0-dev pkg-config libudev-dev` 
you'll need to make libusb if your platform's libusb isnt up to date yet.
`wget https://github.com/airspy/airspyhf/archive/master.zip`
`tar -xfj https://github.com/libusb/libusb/releases/download/v1.0.27/libusb-1.0.27.tar.bz2"
`cd libusb-1.0.27`
except that hilariously, thanks to systemd, which i wish would stop existing, there isnt even headers anymore- it ate them
`wget https://raw.githubusercontent.com/systemd/systemd/main/src/libudev/libudev.h`
`sudo mv libudev.h /usr/include/`
`./configure --prefix=/usr --disable-static`
 `make`
 `sudo make install`
 `cd .../`

### Build host software on Linux:

`wget https://github.com/airspy/airspyhf/archive/master.zip`

`unzip master.zip`

`cd airspyhf-master`

`mkdir build`

`cd build`

`cmake ../ -DINSTALL_UDEV_RULES=ON`

`make`

`sudo make install`

`sudo ldconfig`

## Clean CMake temporary files/dirs:

`cd airspyhf-master/build`

`rm -rf *`


## Principal authors:

Ian Gilmour <ian@sdrsharp.com> and Youssef Touil <youssef@airspy.com> 


http://www.airspy.com

This file is part of Airspy HF (with user mode driver based on Airspy R2, itself based on HackRF project see http://greatscottgadgets.com/hackrf/).
