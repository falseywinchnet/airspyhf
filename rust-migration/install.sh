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

echo
echo ">> Installed:"
echo "     $LIBDIR/$LIB_DEST"
for t in "${TOOLS[@]}"; do echo "     $BINDIR/$t$EXE"; done
echo "     $INCDIR/{airspyhf.h,airspyhf_commands.h,iqbalancer.h}"
echo
echo ">> Reminder: SDR consumers expect the shared library named 'airspyhf' on"
echo "   Windows (airspyhf.dll). Do not rename it to libairspyhf.dll."
