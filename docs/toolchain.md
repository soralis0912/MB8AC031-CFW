# ツールチェイン（`fr30-elf` のビルド）

ファームウェアは C で書いてあるので、`fr30-elf` の binutils と GCC が
必要です。FR30 は binutils でも GCC でも現在もサポートされているターゲットなので、上流の
配布物からそのままビルドできます（パッチは不要です）。

必要なものは、ここに書いた手順だけで揃います。

## 探す場所

`host/build.py` は、次の順に探します。

| 順 | 場所 |
|---|---|
| 1 | 環境変数 `FR30_TOOLCHAIN`（`$FR30_TOOLCHAIN/bin/fr30-elf-gcc`） |
| 2 | `PATH` |
| 3 | `~/opt/fr30-elf/bin` |
| 4 | `/usr/local/fr30-elf/bin` |
| 5 | `/opt/fr30-elf/bin` |

見つからないときは、`build.py` が `NoToolchain` で止まります。テストのほうは
「ツールチェインが無いので飛ばす」と表示して終わります。

以下は `~/opt/fr30-elf` にインストールする例です。

下の手順は [`../host/ci/toolchain.sh`](../host/ci/toolchain.sh) にまとめてあり、
1 回の実行でビルドできます（CI もこれを使います）。

```
host/ci/toolchain.sh fr30-elf      $HOME/opt/fr30-elf
host/ci/toolchain.sh arm-none-eabi $HOME/opt/arm-none-eabi
```

バージョンは環境変数 `BINUTILS_VERSION`・`GCC_VERSION` で変えられます。Debian / Ubuntu では
先に `build-essential libgmp-dev libmpfr-dev libmpc-dev zlib1g-dev xz-utils` をインストールしてください。

## binutils

```
curl -O https://ftp.gnu.org/gnu/binutils/binutils-2.43.1.tar.xz
tar xf binutils-2.43.1.tar.xz
mkdir build-binutils && cd build-binutils
../binutils-2.43.1/configure --target=fr30-elf --prefix=$HOME/opt/fr30-elf \
    --disable-gdb --disable-sim --disable-nls --disable-werror
make MAKEINFO=true all-binutils all-gas all-ld
make MAKEINFO=true install-binutils install-gas install-ld
```

* `MAKEINFO=true` は、`makeinfo` が無い環境でドキュメントの生成だけが失敗するのを
  避けるためのものです。成果物には影響しません
* `gdb` と `sim` は使わないので外してあります

### `-b binary` と `--endian=big`

`fr30-elf-objdump` に生のバイト列を渡すときは、`--endian=big` を付けてください。
`-b binary` ではエンディアンがホストの既定になるので、付けないと 16 bit の
命令語のバイトが入れ替わり、別の命令として表示されます（`stm1 (r8,r9)` が `*unknown*` になります）。
FR はビッグエンディアンです。

## GCC

`fr30-*-elf` は GCC master でも現在サポートされています（`gcc/config.gcc` にあり、obsolete にも
なっていません）。C を FR のコードにコンパイルするのに必要なのは `all-gcc` だけで、
libc も libgcc も要りません。

```
curl -O https://ftp.gnu.org/gnu/gcc/gcc-14.2.0/gcc-14.2.0.tar.xz
tar xf gcc-14.2.0.tar.xz
mkdir build-gcc && cd build-gcc
../gcc-14.2.0/configure --target=fr30-elf --prefix=$HOME/opt/fr30-elf \
    --enable-languages=c --without-headers --with-newlib \
    --with-system-zlib --disable-nls
make MAKEINFO=true all-gcc
make MAKEINFO=true install-gcc
```

* macOS では `--with-system-zlib` が必要です。同梱の zlib が `fdopen` を
  マクロで置き換えてしまい、`stdio.h` が壊れます
* `--without-headers --with-newlib` で、ホストの libc を参照しないようにします。
  ファームウェアは `-ffreestanding` でビルドし、必要な関数は `src/main/util.c` にあります
* `make all-gcc` / `install-gcc` だけで足ります。`make all` まで実行すると
  libgcc をビルドしようとして止まりますが、libgcc は使いません
* prefix を移しても動きます。GCC は自分の位置からの相対パスで `cc1` や `as` を
  探すので、ビルドしたツリーをそのまま別の場所へ移せます

## 動作の確認

```
export FR30_TOOLCHAIN=$HOME/opt/fr30-elf
$FR30_TOOLCHAIN/bin/fr30-elf-gcc --version
python3 host/build.py            # 全部ビルドする
python3 tests/test_host.py       # 空の場所へ写して組み直し、同じバイトが出るか
```

## ARM のイメージ用の `arm-none-eabi`

ARM のコアに載せるイメージ（[`../src/armcore/`](../src/armcore/)、[`arm.md`](arm.md)）は、
別のツールチェイン `arm-none-eabi` でビルドします。これが無くても、残りはすべてビルドできます。
その場合、`build.py` は ARM のイメージだけを飛ばし（`arm skipped`）、`tests/test_host.py` は
`armcore.bin` を比べません。

探す順は fr30 と同じ形です。

| 順 | 場所 |
|---|---|
| 1 | 環境変数 `ARM_TOOLCHAIN`（`$ARM_TOOLCHAIN/bin/arm-none-eabi-gcc`） |
| 2 | `PATH` |
| 3 | `~/opt/arm-none-eabi/bin` |
| 4 | `/usr/local/arm-none-eabi/bin`・`/opt/arm-none-eabi/bin`・`/opt/homebrew/bin` |

ビルドの方法は fr30 と同じで、`--target=arm-none-eabi` に変えます。macOS（Apple Silicon、
Homebrew の gmp・mpfr・libmpc あり）では、下の手順でインストールできます。
ビッグエンディアンの組み合わせは、既定の multilib に無くても構いません。libgcc も
libc も使わないためです（`-nostdlib -ffreestanding`）。

```
../binutils-2.43.1/configure --target=arm-none-eabi --prefix=$HOME/opt/arm-none-eabi \
    --disable-gdb --disable-sim --disable-nls --disable-werror --with-system-zlib
make -j8 MAKEINFO=true all-binutils all-gas all-ld
make MAKEINFO=true install-binutils install-gas install-ld

PATH=$HOME/opt/arm-none-eabi/bin:$PATH ../gcc-14.2.0/configure \
    --target=arm-none-eabi --prefix=$HOME/opt/arm-none-eabi \
    --with-gmp=/opt/homebrew/opt/gmp --with-mpfr=/opt/homebrew/opt/mpfr \
    --with-mpc=/opt/homebrew/opt/libmpc --with-system-zlib \
    --enable-languages=c --without-headers --with-newlib --disable-nls \
    --disable-shared --disable-threads --disable-libssp --disable-libquadmath \
    --disable-libatomic --disable-libgomp --disable-werror --disable-bootstrap
PATH=$HOME/opt/arm-none-eabi/bin:$PATH make -j8 MAKEINFO=true all-gcc
make MAKEINFO=true install-gcc
```

* macOS では、binutils にも `--with-system-zlib` が必要です。付けないと、GCC のときと同じように
  同梱の zlib が `stdio.h` を壊してビルドが止まります

使うコンパイラのフラグは [`../config/armcore.json`](../config/armcore.json) にあります
（`-mcpu=arm926ej-s -marm -mbig-endian`、リンクは `-EB`）。

## CI

GitHub Actions（[`../.github/workflows/firmware.yml`](../.github/workflows/firmware.yml)）で、
push と pull request のたびにビルドします。

1. `host/ci/toolchain.sh` で `fr30-elf` と `arm-none-eabi` をビルドします。ビルドしたものはキャッシュに置くので、
   ビルドし直すのは初回と、バージョンかスクリプトが変わったときだけです
2. `python3 host/build.py` ですべてビルドします
3. `tests/test_*.py` をすべて実行します（`test_ts_dione.py` だけで 20 分ほどかかります）
4. `frc1-*.frc`・`auxcore.bin`・`armcore.bin`・`*.map` と `SHA256SUMS`・`BUILDINFO`
   （コミットとツールチェインのバージョン）を、成果物 `cfw-<コミット>` に置きます

`v` で始まるタグを push すると、同じものを GitHub のリリースに添付します。

Linux でビルドした `.frc`・`auxcore.bin`・`armcore.bin` は、macOS でビルドしたものと同じバイトになります（実測）。
GCC のビルドはメモリを多く使うので、メモリの少ない環境では `JOBS` を下げてください
（Docker の既定の割り当てなら `JOBS=3` にします）。
