# Native Windows x64 package

`build-x64.sh` cross-builds the readable driver as a real Windows
`airspy.dll`. It is not the Wine TCP shim: libusb runs inside the Windows
process and talks directly to the radio.

The workspace preserves an x64 `libusb-1.0.dll` but not its import library.
`libusb-1.0.def` records the subset used by libairspy, and the build generates
the MinGW import library from that definition. MinGW's winpthreads supplies the
existing driver thread ABI. `airspy.def` freezes the complete field export set,
including the historically visible gain tables and receiver-mode function; the
build fails if the resulting DLL differs from the common symbol manifest.

From the driver directory:

```sh
./windows/build-x64.sh
```

Copy every DLL from `windows/out/x64/` beside the Windows application's
executable. The Airspy must use a libusb-compatible Windows device driver
(normally WinUSB through Zadig); merely copying the DLL does not change the
kernel driver assigned to the USB device.

This package preserves the fixed 16-transfer by 256-KiB geometry and the public
libairspy C ABI. It still requires qualification on an actual Windows host
before release.

`airspy-load-smoke.exe` verifies that Windows can load the complete DLL
dependency set and call the version ABI without opening a radio.
