#!/bin/sh
set -eu

HERE=$(CDPATH= cd -- "$(dirname "$0")" && pwd)
DRIVER_ROOT=${AIRSPY_DRIVER_ROOT:-"$HERE/.."}
AIRSPY_HEADERS=${AIRSPY_SOURCE_ROOT:-"$DRIVER_ROOT/current"}
READABLE_BUILD=${AIRSPY_READABLE_BUILD:-"$DRIVER_ROOT/.build/readable-release"}
OUT="$HERE/out"
MINGW=${MINGW:-x86_64-w64-mingw32-gcc}

mkdir -p "$OUT"

cmake -S "$DRIVER_ROOT/readable" -B "$READABLE_BUILD" \
    -DCMAKE_BUILD_TYPE=Release
cmake --build "$READABLE_BUILD"

cc -O2 -Wall -Wextra -pthread \
    -I"$HERE" -I"$AIRSPY_HEADERS/libairspy/src" \
    -I/opt/homebrew/opt/libusb/include/libusb-1.0 \
    "$HERE/helper.c" \
    "$READABLE_BUILD/libairspy_readable.dylib" \
    -L/opt/homebrew/opt/libusb/lib -lusb-1.0 \
    -o "$OUT/airspy-helper"

cp "$READABLE_BUILD/libairspy_readable.dylib" "$OUT/libairspy.0.dylib"
install_name_tool -id @loader_path/libairspy.0.dylib "$OUT/libairspy.0.dylib"
install_name_tool -change @rpath/libairspy_readable.dylib \
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
    -I"$HERE" -I"$AIRSPY_HEADERS/libairspy/src" \
    -o "$OUT/airspy.dll" "$HERE/shim.c" \
    -lws2_32 -Wl,--out-implib,"$OUT/libairspy.dll.a"

"$MINGW" -O2 -Wall -Wextra \
    -I"$AIRSPY_HEADERS/libairspy/src" \
    -o "$OUT/test-airspy.exe" "$HERE/test_win.c" \
    -L"$OUT" -lairspy

file "$OUT/airspy-helper" "$OUT/airspy-counters" \
    "$OUT/airspy.dll" "$OUT/test-airspy.exe"
otool -L "$OUT/airspy-helper"
shasum -a 256 "$OUT/libairspy.0.dylib" \
    > "$OUT/libairspy.0.dylib.sha256"
