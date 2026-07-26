#!/bin/sh
set -eu

HERE=$(CDPATH= cd -- "$(dirname "$0")" && pwd)
DRIVER_ROOT=$(CDPATH= cd -- "$HERE/.." && pwd)
WORKSPACE_ROOT=$(CDPATH= cd -- "$DRIVER_ROOT/../../.." && pwd)
MINGW_CXX=${MINGW_CXX:-x86_64-w64-mingw32-g++}
MINGW_CC=${MINGW_CC:-x86_64-w64-mingw32-gcc}
MINGW_DLLTOOL=${MINGW_DLLTOOL:-x86_64-w64-mingw32-dlltool}
LIBUSB_INCLUDE=${LIBUSB_INCLUDE:-/opt/homebrew/opt/libusb/include/libusb-1.0}
LIBUSB_DLL=${LIBUSB_DLL:-"$DRIVER_ROOT/current/libairspy/vc/libs/x64/libusb-1.0.dll"}
BUILD_DIR=${AIRSPY_WINDOWS_BUILD:-"$DRIVER_ROOT/.build/windows-x64"}
OUT_DIR=${AIRSPY_WINDOWS_OUT:-"$DRIVER_ROOT/windows/out/x64"}

mkdir -p "$BUILD_DIR" "$OUT_DIR"
rm -f "$OUT_DIR"/*.dll "$OUT_DIR"/*.a

"$MINGW_DLLTOOL" \
    --input-def "$HERE/libusb-1.0.def" \
    --dllname libusb-1.0.dll \
    --output-lib "$BUILD_DIR/libusb-1.0.dll.a"

"$MINGW_CXX" \
    -shared -O2 -DNDEBUG -std=gnu++20 \
    -Wall -Wextra -Wpedantic \
    -I"$DRIVER_ROOT/current/libairspy/src" \
    -I"$DRIVER_ROOT/model/include" \
    -I"$LIBUSB_INCLUDE" \
    "$DRIVER_ROOT/readable/src/legacy_translation_unit.cpp" \
    "$DRIVER_ROOT/model/src/buffer_pool.cpp" \
    "$DRIVER_ROOT/model/src/legacy_unpack.cpp" \
    "$DRIVER_ROOT/model/src/legacy_unpack_x86.cpp" \
    "$DRIVER_ROOT/model/src/sample_conversion.cpp" \
    "$DRIVER_ROOT/model/src/transfer_lifecycle.cpp" \
    "$DRIVER_ROOT/current/libairspy/src/iqconverter_float.c" \
    "$DRIVER_ROOT/current/libairspy/src/iqconverter_int16.c" \
    "$HERE/airspy.def" \
    "$BUILD_DIR/libusb-1.0.dll.a" \
    -Wl,--out-implib,"$OUT_DIR/libairspy.dll.a" \
    -Wl,--strip-all \
    -static-libgcc -static-libstdc++ \
    -lwinpthread \
    -o "$OUT_DIR/airspy.dll"

cp "$LIBUSB_DLL" "$OUT_DIR/libusb-1.0.dll"

for runtime in libwinpthread-1.dll
do
    path=$("$MINGW_CXX" -print-file-name="$runtime")
    if [ "$path" = "$runtime" ] || [ ! -f "$path" ]
    then
        sysroot=$("$MINGW_CXX" -print-sysroot)
        candidate="$sysroot/x86_64-w64-mingw32/bin/$runtime"
        if [ -f "$candidate" ]
        then
            path=$candidate
        fi
    fi
    if [ "$path" != "$runtime" ] && [ -f "$path" ]
    then
        cp "$path" "$OUT_DIR/$runtime"
    fi
done

"$MINGW_CC" \
    -O2 \
    -I"$DRIVER_ROOT/current/libairspy/src" \
    "$HERE/load-smoke.c" \
    -L"$OUT_DIR" -lairspy \
    -o "$OUT_DIR/airspy-load-smoke.exe"

file "$OUT_DIR/airspy.dll" "$OUT_DIR/libusb-1.0.dll"
x86_64-w64-mingw32-objdump -p "$OUT_DIR/airspy.dll" |
    awk '/DLL Name:/ {print}'

expected_exports=$(mktemp)
actual_exports=$(mktemp)
cleanup()
{
    rm -f "$expected_exports" "$actual_exports"
}
trap cleanup EXIT HUP INT TERM
sed '/^#/d; /^$/d' "$DRIVER_ROOT/docs/legacy-exported-symbols.txt" \
    > "$expected_exports"
x86_64-w64-mingw32-objdump -p "$OUT_DIR/airspy.dll" |
    sed -n '/\[Ordinal\/Name Pointer\]/,/PE File Base Relocations/p' |
    awk '{for (field = 1; field <= NF; ++field) {
        if ($field ~ /^airspy_[A-Za-z0-9_]+$/) print $field
    }}' |
    sort -u > "$actual_exports"
diff -u "$expected_exports" "$actual_exports"

cp "$HERE/README.md" "$OUT_DIR/README.txt"
(cd "$OUT_DIR" && zip -q -FS ../airspy-readable-windows-x64.zip \
    airspy.dll libusb-1.0.dll libwinpthread-1.dll \
    airspy-load-smoke.exe README.txt)

shasum -a 256 "$OUT_DIR"/*.dll
shasum -a 256 "$DRIVER_ROOT/windows/out/airspy-readable-windows-x64.zip"
