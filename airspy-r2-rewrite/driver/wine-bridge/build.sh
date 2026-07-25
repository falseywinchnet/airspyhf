#!/bin/sh
set -eu

HERE=$(CDPATH= cd -- "$(dirname "$0")" && pwd)
UPSTREAM=${AIRSPY_SOURCE_ROOT:-"$HERE/../../../airspy-r2-research/upstream/airspyone_host"}
NATIVE_BUILD=${AIRSPY_NATIVE_BUILD:-/tmp/airspyone-host-build}
OUT="$HERE/out"
MINGW=${MINGW:-x86_64-w64-mingw32-gcc}

mkdir -p "$OUT"

cc -O2 -Wall -Wextra -pthread \
    -I"$HERE" -I"$UPSTREAM/libairspy/src" \
    -I/opt/homebrew/opt/libusb/include/libusb-1.0 \
    "$HERE/helper.c" \
    -L"$NATIVE_BUILD/libairspy/src" -lairspy \
    -L/opt/homebrew/opt/libusb/lib -lusb-1.0 \
    -o "$OUT/airspy-helper"

cp "$NATIVE_BUILD/libairspy/src/libairspy.1.0.12.dylib" "$OUT/libairspy.0.dylib"
install_name_tool -id @loader_path/libairspy.0.dylib "$OUT/libairspy.0.dylib"
install_name_tool -change /usr/local/lib/libairspy.0.dylib \
    @loader_path/libairspy.0.dylib "$OUT/airspy-helper"

cc -O2 -Wall -Wextra -Wpedantic -fobjc-arc \
    -I"$HERE" "$HERE/counter_window.m" \
    -framework Cocoa -o "$OUT/airspy-counters"
mkdir -p "$OUT/AirspyCounters.app/Contents/MacOS"
cp "$OUT/airspy-counters" \
    "$OUT/AirspyCounters.app/Contents/MacOS/airspy-counters"
cp "$HERE/AirspyCounters-Info.plist" \
    "$OUT/AirspyCounters.app/Contents/Info.plist"

"$MINGW" -O2 -Wall -Wextra -shared \
    -I"$HERE" -I"$UPSTREAM/libairspy/src" \
    -o "$OUT/airspy.dll" "$HERE/shim.c" \
    -lws2_32 -Wl,--out-implib,"$OUT/libairspy.dll.a"

"$MINGW" -O2 -Wall -Wextra \
    -I"$UPSTREAM/libairspy/src" \
    -o "$OUT/test-airspy.exe" "$HERE/test_win.c" \
    -L"$OUT" -lairspy

file "$OUT/airspy-helper" "$OUT/airspy-counters" \
    "$OUT/airspy.dll" "$OUT/test-airspy.exe"
