#!/usr/bin/env bash
#
# Build and install the Rust AirspyHF+ driver and its command-line tools.
#
# Usage:
#   ./install.sh                 # build + install for the local platform
#   PREFIX=/opt ./install.sh     # install under a custom prefix (default /usr/local)
#   TARGET=<triple> ./install.sh # cross-compile for another platform (staged, not installed)
#   AIRSPYHF_NATIVE=1 ./install.sh   # optimize for the local CPU (-C target-cpu=native)
#   AIRSPYHF_DLL_NAME=experimentalairspyhf ./install.sh   # rename the installed Windows DLL
#
# Cross-compiling: set TARGET to a rustc triple (e.g. x86_64-pc-windows-gnu,
# x86_64-unknown-linux-gnu). When TARGET names a different OS than the host, the
# artifacts are staged under ./dist/<triple> instead of being installed into the
# system, so a Linux/macOS box can produce binaries for the other platforms.
#
# NOTE ON DLL NAMING: SDR consumers load "airspyhf.dll" on Windows (NOT
# "libairspyhf.dll"). The crate's [lib] name is "airspyhf", so cargo already
# emits airspyhf.dll / libairspyhf.so / libairspyhf.dylib. This script installs
# those names unchanged.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$SCRIPT_DIR"

PREFIX="${PREFIX:-/usr/local}"
TARGET="${TARGET:-}"
NATIVE="${AIRSPYHF_NATIVE:-0}"

command -v cargo >/dev/null 2>&1 || { echo "error: cargo not found on PATH" >&2; exit 1; }

HOST_TRIPLE="$(rustc -vV | sed -n 's/^host: //p')"
TRIPLE="${TARGET:-$HOST_TRIPLE}"

os_of() {
    case "$1" in
        *windows*) echo windows ;;
        *apple*|*darwin*) echo macos ;;
        *linux*|*android*) echo linux ;;
        *) echo unknown ;;
    esac
}
HOST_OS="$(os_of "$HOST_TRIPLE")"
TARGET_OS="$(os_of "$TRIPLE")"

echo ">> host:   $HOST_TRIPLE ($HOST_OS)"
echo ">> target: $TRIPLE ($TARGET_OS)"

# ---- cross toolchain preflight -------------------------------------------
# The *-pc-windows-gnu targets link through the MinGW-w64 cross GCC, which is
# not part of the Rust toolchain. Check for it up front and, where we can,
# install it automatically instead of failing deep inside cargo's linker step.
ensure_mingw() {
    local linker="$1" brew_pkg="mingw-w64" apt_pkg="gcc-mingw-w64"
    command -v "$linker" >/dev/null 2>&1 && return 0

    echo ">> cross linker '$linker' not found (needed for $TRIPLE)"
    if [ "$HOST_OS" = macos ] && command -v brew >/dev/null 2>&1; then
        echo ">> installing $brew_pkg via Homebrew ..."
        brew install "$brew_pkg"
    elif command -v apt-get >/dev/null 2>&1; then
        echo ">> installing $apt_pkg via apt-get (sudo) ..."
        sudo apt-get update && sudo apt-get install -y "$apt_pkg"
    fi

    command -v "$linker" >/dev/null 2>&1 && return 0
    echo "error: '$linker' still not on PATH. Install the MinGW-w64 cross toolchain:" >&2
    echo "         macOS:         brew install $brew_pkg" >&2
    echo "         Debian/Ubuntu: sudo apt-get install $apt_pkg" >&2
    echo "         Fedora:        sudo dnf install mingw64-gcc  (or mingw32-gcc)" >&2
    exit 1
}

if [ "$TARGET_OS" = windows ] && [ "$HOST_OS" != windows ]; then
    case "$TRIPLE" in
        x86_64-pc-windows-gnu)  ensure_mingw x86_64-w64-mingw32-gcc ;;
        i686-pc-windows-gnu)    ensure_mingw i686-w64-mingw32-gcc ;;
        *-pc-windows-msvc)
            echo "error: cannot cross-compile the *-windows-msvc target from $HOST_OS." >&2
            echo "       Use a *-pc-windows-gnu triple (e.g. x86_64-pc-windows-gnu)." >&2
            exit 1 ;;
    esac
fi

# ---- build ---------------------------------------------------------------
CARGO_ARGS=(build --release)
[ -n "$TARGET" ] && CARGO_ARGS+=(--target "$TARGET")

if [ "$NATIVE" = "1" ]; then
    echo ">> building (native CPU tuning) ..."
    RUSTFLAGS="${RUSTFLAGS:-} -C target-cpu=native" cargo "${CARGO_ARGS[@]}"
else
    echo ">> building (portable) ..."
    cargo "${CARGO_ARGS[@]}"
fi

if [ -n "$TARGET" ]; then
    OUT="target/$TARGET/release"
else
    OUT="target/release"
fi

# ---- resolve artifact names ----------------------------------------------
case "$TARGET_OS" in
    macos)   LIB_SRC="libairspyhf.dylib"; LIB_DEST="libairspyhf.dylib"; EXE="" ;;
    linux)   LIB_SRC="libairspyhf.so";    LIB_DEST="libairspyhf.so";    EXE="" ;;
    windows) LIB_SRC="airspyhf.dll";      LIB_DEST="${AIRSPYHF_DLL_NAME:-airspyhf}.dll"; EXE=".exe" ;;
    *) echo "error: unsupported target OS: $TARGET_OS" >&2; exit 1 ;;
esac

TOOLS=(airspyhf_lib_version airspyhf_info airspyhf_gpio airspyhf_calibrate airspyhf_rx)

# ---- cross-compile: stage only -------------------------------------------
if [ "$TARGET_OS" != "$HOST_OS" ]; then
    STAGE="dist/$TRIPLE"
    echo ">> cross build: staging into $STAGE (not installing to the system)"
    mkdir -p "$STAGE/lib" "$STAGE/bin" "$STAGE/include/libairspyhf"
    cp "$OUT/$LIB_SRC" "$STAGE/lib/$LIB_DEST"
    if [ "$TARGET_OS" = "windows" ]; then
        # Import libraries for linking C programs against the DLL.
        for implib in airspyhf.dll.lib libairspyhf.dll.a; do
            [ -f "$OUT/$implib" ] && cp "$OUT/$implib" "$STAGE/lib/"
        done
    fi
    for t in "${TOOLS[@]}"; do cp "$OUT/$t$EXE" "$STAGE/bin/"; done
    cp "$REPO_ROOT"/libairspyhf/src/airspyhf.h \
       "$REPO_ROOT"/libairspyhf/src/airspyhf_commands.h \
       "$REPO_ROOT"/libairspyhf/src/iqbalancer.h "$STAGE/include/libairspyhf/"
    echo ">> staged. Copy $STAGE/{lib,bin,include} onto the $TARGET_OS machine."
    exit 0
fi

# ---- native install ------------------------------------------------------
LIBDIR="$PREFIX/lib"
BINDIR="$PREFIX/bin"
INCDIR="$PREFIX/include/libairspyhf"

# Wrap mutating commands in sudo only if the nearest existing ancestor of the
# prefix is not writable by the current user.
SUDO=""
probe="$PREFIX"
while [ ! -e "$probe" ]; do probe="$(dirname "$probe")"; done
if [ ! -w "$probe" ]; then
    if command -v sudo >/dev/null 2>&1; then SUDO="sudo"; fi
fi

echo ">> installing into $PREFIX (SUDO='${SUDO:-none}')"
$SUDO mkdir -p "$LIBDIR" "$BINDIR" "$INCDIR"
$SUDO install -m 0755 "$OUT/$LIB_SRC" "$LIBDIR/$LIB_DEST"
if [ "$TARGET_OS" = "macos" ] && command -v install_name_tool >/dev/null 2>&1; then
    $SUDO install_name_tool -id "@rpath/$LIB_DEST" "$LIBDIR/$LIB_DEST"
fi
for t in "${TOOLS[@]}"; do
    $SUDO install -m 0755 "$OUT/$t$EXE" "$BINDIR/$t$EXE"
done
$SUDO install -m 0644 \
    "$REPO_ROOT"/libairspyhf/src/airspyhf.h \
    "$REPO_ROOT"/libairspyhf/src/airspyhf_commands.h \
    "$REPO_ROOT"/libairspyhf/src/iqbalancer.h \
    "$INCDIR/"

if [ "$TARGET_OS" = "linux" ]; then
    if [ -f "$REPO_ROOT/tools/52-airspyhf.rules" ]; then
        echo ">> installing udev rules"
        $SUDO install -m 0644 "$REPO_ROOT/tools/52-airspyhf.rules" /etc/udev/rules.d/52-airspyhf.rules
        $SUDO udevadm control --reload-rules 2>/dev/null || true
        $SUDO udevadm trigger 2>/dev/null || true
    fi
    if command -v ldconfig >/dev/null 2>&1; then
        $SUDO ldconfig || true
    fi
fi

# ---- pkg-config file -------------------------------------------------------
# The C++/CMake build installs libairspyhf.pc; this one doesn't, which is why
# tools that locate the library via `pkg-config --cflags --libs libairspyhf`
# (e.g. libairspyhf/wine-bridge/build.sh) only work after the C++ variant has
# been installed at least once. Install the same file here so this install.sh
# alone is sufficient.
if [ "$TARGET_OS" != "windows" ]; then
    VER_MAJOR="$(sed -n 's/.*AIRSPYHF_VER_MAJOR: u32 = \([0-9]*\);.*/\1/p' "$SCRIPT_DIR/libairspyhf/src/lib.rs")"
    VER_MINOR="$(sed -n 's/.*AIRSPYHF_VER_MINOR: u32 = \([0-9]*\);.*/\1/p' "$SCRIPT_DIR/libairspyhf/src/lib.rs")"
    PC_RPATH=""
    [ "$TARGET_OS" = "macos" ] && PC_RPATH='-Wl,-rpath,${libdir}'
    PCDIR="$LIBDIR/pkgconfig"
    PC_TMP="$(mktemp)"
    cat > "$PC_TMP" <<EOF
prefix=\${pcfiledir}/../..
exec_prefix=\${prefix}
libdir=\${exec_prefix}/lib
includedir=\${prefix}/include

Name: AirSpy HF+ Library
Description: C Utility Library (Rust implementation)
Version: ${VER_MAJOR}.${VER_MINOR}
Cflags: -I\${includedir}/libairspyhf
Libs: -L\${libdir} -lairspyhf ${PC_RPATH}
EOF
    $SUDO mkdir -p "$PCDIR"
    $SUDO install -m 0644 "$PC_TMP" "$PCDIR/libairspyhf.pc"
    rm -f "$PC_TMP"
    echo ">> installed pkg-config file: $PCDIR/libairspyhf.pc"
fi

# ---- Wine bridge -----------------------------------------------------------
# Install the same helper/shim pair used by the C++ tree. The helper links
# against the library installed above, so C++ vs Rust behavior is selected by
# whichever install path last populated PREFIX.
if [ "$TARGET_OS" != "windows" ]; then
    BRIDGE_DIR="$REPO_ROOT/libairspyhf/wine-bridge"
    BRIDGE_TMP="$(mktemp -d)"

    echo ">> building Wine bridge helper against $LIBDIR/$LIB_DEST"
    cc -O2 -Wall -Wextra \
        -I"$BRIDGE_DIR" -I"$PREFIX/include" \
        "$BRIDGE_DIR/helper.c" \
        -L"$LIBDIR" -lairspyhf -Wl,-rpath,"$LIBDIR" -lpthread \
        -o "$BRIDGE_TMP/airspyhf-helper"
    $SUDO install -m 0755 "$BRIDGE_TMP/airspyhf-helper" "$BINDIR/airspyhf-helper"
    echo ">> installed Wine bridge helper: $BINDIR/airspyhf-helper"

    if command -v x86_64-w64-mingw32-gcc >/dev/null 2>&1; then
        echo ">> building Wine airspyhf.dll shim"
        x86_64-w64-mingw32-gcc -O2 -Wall -Wextra -shared \
            -I"$BRIDGE_DIR" -I"$REPO_ROOT/libairspyhf/src" \
            -o "$BRIDGE_TMP/airspyhf.dll" "$BRIDGE_DIR/shim.c" \
            -lws2_32
        $SUDO mkdir -p "$LIBDIR/airspyhf/wine"
        $SUDO install -m 0644 "$BRIDGE_TMP/airspyhf.dll" "$LIBDIR/airspyhf/wine/airspyhf.dll"
        echo ">> installed Wine shim: $LIBDIR/airspyhf/wine/airspyhf.dll"
    else
        echo ">> x86_64-w64-mingw32-gcc not found; skipping Wine airspyhf.dll shim"
    fi

    rm -rf "$BRIDGE_TMP"
fi

echo
echo ">> Installed:"
echo "     $LIBDIR/$LIB_DEST"
for t in "${TOOLS[@]}"; do echo "     $BINDIR/$t$EXE"; done
if [ "$TARGET_OS" != "windows" ]; then
    echo "     $BINDIR/airspyhf-helper"
    [ -f "$LIBDIR/airspyhf/wine/airspyhf.dll" ] && echo "     $LIBDIR/airspyhf/wine/airspyhf.dll"
fi
echo "     $INCDIR/{airspyhf.h,airspyhf_commands.h,iqbalancer.h}"
echo
echo ">> Reminder: SDR consumers expect the shared library named 'airspyhf' on"
echo "   Windows (airspyhf.dll). Do not rename it to libairspyhf.dll."
