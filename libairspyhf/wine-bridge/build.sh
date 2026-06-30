#!/bin/sh
# Build the AirSpy HF Wine bridge:
#   - airspyhf-helper : native macOS (arm64), links the installed libairspyhf
#   - airspyhf.dll    : PE x86_64, loaded by SDR# inside Wine
set -e
cd "$(dirname "$0")"
mkdir -p out

echo ">> building native helper (arm64)"
HELPER_FLAGS="$(pkg-config --cflags --libs libairspyhf)"
cc -O2 -Wall -Wextra -I. helper.c $HELPER_FLAGS -lpthread -o out/airspyhf-helper
echo "   out/airspyhf-helper"

echo ">> building PE shim (x86_64 airspyhf.dll)"
MINGW="${MINGW:-x86_64-w64-mingw32-gcc}"
"$MINGW" -O2 -Wall -Wextra -shared \
    -I. -I../src \
    -o out/airspyhf.dll shim.c \
    -lws2_32 \
    -Wl,--out-implib,out/libairspyhf.dll.a
echo "   out/airspyhf.dll"

echo ">> done"
ls -la out
