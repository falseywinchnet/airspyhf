#!/bin/sh
# Build the AirSpy HF Wine bridge:
#   - airspyhf-helper : native host executable, links the installed libairspyhf
#   - airspyhf.dll    : PE x86_64 shim, loaded by SDR# inside Wine
set -e
cd "$(dirname "$0")"
mkdir -p out

PREFIX="${PREFIX:-/usr/local}"
CC="${CC:-cc}"
MINGW="${MINGW:-x86_64-w64-mingw32-gcc}"

echo ">> building native helper"
PC_INCLUDEDIR="$(pkg-config --variable=includedir libairspyhf)"
HELPER_CFLAGS="$(pkg-config --cflags libairspyhf)"
HELPER_LIBS="$(pkg-config --libs libairspyhf)"
"$CC" -O2 -Wall -Wextra \
    -I. -I"$PC_INCLUDEDIR" \
    helper.c $HELPER_CFLAGS $HELPER_LIBS -lpthread \
    -o out/airspyhf-helper
echo "   out/airspyhf-helper"

echo ">> building PE shim (x86_64 airspyhf.dll)"
"$MINGW" -O2 -Wall -Wextra -shared \
    -I. -I../src \
    -o out/airspyhf.dll shim.c \
    -lws2_32 \
    -Wl,--out-implib,out/libairspyhf.dll.a
echo "   out/airspyhf.dll"

if [ "${1:-}" = "install" ]; then
    echo ">> installing into $PREFIX"
    install -d "$PREFIX/bin" "$PREFIX/lib/airspyhf/wine"
    install -m 0755 out/airspyhf-helper "$PREFIX/bin/airspyhf-helper"
    install -m 0644 out/airspyhf.dll "$PREFIX/lib/airspyhf/wine/airspyhf.dll"
fi

echo ">> done"
ls -la out
