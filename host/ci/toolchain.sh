#!/bin/sh
# Build a bare cross toolchain (binutils + the C compiler only, no libc, no
# libgcc) from the upstream GNU release tarballs.  This is the recipe in
# docs/toolchain.md as a script, for CI and for anyone who wants it done in
# one go.
#
#     host/ci/toolchain.sh fr30-elf       $HOME/opt/fr30-elf
#     host/ci/toolchain.sh arm-none-eabi  $HOME/opt/arm-none-eabi
#
# Needs a host C/C++ compiler, make, curl, xz, and the GMP / MPFR / MPC and
# zlib development files (Debian/Ubuntu: build-essential libgmp-dev
# libmpfr-dev libmpc-dev zlib1g-dev).  BINUTILS_VERSION / GCC_VERSION /
# JOBS / WORK override the defaults below.
set -eu

target=${1:?usage: toolchain.sh <target> <prefix>}
prefix=${2:?usage: toolchain.sh <target> <prefix>}
binutils=${BINUTILS_VERSION:-2.43.1}
gcc=${GCC_VERSION:-14.2.0}
jobs=${JOBS:-$(getconf _NPROCESSORS_ONLN 2>/dev/null || echo 2)}
work=${WORK:-$(mktemp -d)}
mirror=${GNU_MIRROR:-https://ftpmirror.gnu.org/gnu}

mkdir -p "$work"
cd "$work"

fetch() {
    # ftpmirror.gnu.org redirects to a nearby mirror; ftp.gnu.org is the
    # fallback when that mirror is having a bad day
    [ -f "$2" ] || curl -fsSL --retry 3 -o "$2" "$mirror/$1/$2" ||
        curl -fsSL --retry 3 -o "$2" "https://ftp.gnu.org/gnu/$1/$2"
}

fetch binutils "binutils-$binutils.tar.xz"
fetch "gcc/gcc-$gcc" "gcc-$gcc.tar.xz"
[ -d "binutils-$binutils" ] || tar xf "binutils-$binutils.tar.xz"
[ -d "gcc-$gcc" ] || tar xf "gcc-$gcc.tar.xz"

# makeinfo is often missing; MAKEINFO=true keeps the doc step from failing.
# The bundled zlib breaks stdio.h on macOS, so use the system one.
rm -rf build-binutils && mkdir build-binutils && cd build-binutils
"../binutils-$binutils/configure" --target="$target" --prefix="$prefix" \
    --disable-gdb --disable-sim --disable-nls --disable-werror \
    --with-system-zlib
make -j"$jobs" MAKEINFO=true all-binutils all-gas all-ld
make MAKEINFO=true install-binutils install-gas install-ld
cd ..

# all-gcc only: the firmware is -ffreestanding and needs neither libgcc nor
# a libc, and `make all` would stop trying to build libgcc
rm -rf build-gcc && mkdir build-gcc && cd build-gcc
PATH="$prefix/bin:$PATH" "../gcc-$gcc/configure" --target="$target" \
    --prefix="$prefix" --enable-languages=c --without-headers --with-newlib \
    --with-system-zlib --disable-nls --disable-shared --disable-threads \
    --disable-libssp --disable-libquadmath --disable-libatomic \
    --disable-libgomp --disable-werror --disable-bootstrap
PATH="$prefix/bin:$PATH" make -j"$jobs" MAKEINFO=true all-gcc
make MAKEINFO=true install-gcc
cd ..

"$prefix/bin/$target-gcc" --version | head -n 1
