# 仕様

このファームウェアの仕様書です。実装を読まなくても、何を作っているものかが
分かるところまでを書いてあります。

| | 中身 |
|---|---|
| [`frq1.md`](frq1.md) | ホストとの電文。64 バイトの枠、コマンド表、状態と誤りの空間、誰がどこを書くか |
| [`frc1.md`](frc1.md) | 配布の形式。`FRC1` コンテナの struct、読み手が確かめること、入れる順番。実機で通っているのはこの経路です |
| [`image.md`](image.md) | イメージの形。256 KiB のメモリマップ、リンカの契約、2 つの入口 |
| [`modules.md`](modules.md) | モジュールの契約。返り値、振り分け、`FW_ENOTIMPL` を返すモジュール |
| [`devices.md`](devices.md) | 機種表。`config/` の各列、裏取りの段階、機種を増やす手順 |
| [`i2c.md`](i2c.md) | I2C の仕様。同じコントローラがこのチップに 2 つあるので、共通の状態機械と、実体ごとの扱い |
| [`demod.md`](demod.md) | 復調。アドレス表、ロックと C/N のレジスタ、チューナーへの経路（レジスタ `0xFE` のリピータ） |
| [`arm.md`](arm.md) | ARM の 2 基。番地の 2 つの見え方（ITCM `0`・DTCM `0x00100000`）、制御帯、郵便受け、帯 `0x60` の電文、試す順。ARM から見た番地も載せています。こちらのイメージは GV-M2TV の ARM-E・ARM-C と PIX-DT295 の ARM-E で動きます（実測） |
| [`card.md`](card.md) | カード I/F。レジスタ、電源投入の 3 段、ATR の後、1 ブロックの送受信、帯 `0x50` の電文。時間の管理と T=1 はホストが受け持ちます |
| [`ts.md`](ts.md) | TS の取り込み。リングに書くのは機器なので、リングの処理は 2 本のレジスタの間の算術になります。入口と出口で残っていることも書いてあります |
| [`toolchain.md`](toolchain.md) | `fr30-elf` をビルドする手順。binutils と GCC、探す場所。ARM のイメージに使う `arm-none-eabi` も |

## 概要

ホストが 64 バイトの `FRQ1` の枠を 1 つ渡してくるので、ファームウェアはそれに答えて戻ります。

入口は 2 つあり、どちらも同じ `frq1_dispatch` に入ります。実機で通っているのは FRC1 の経路です。

```
FRC1（平文。実機で通っている経路）
  ホスト ──0xBC の制御転送──> ローダの hook ──> frc1_entry 0x000E1000
                                                 └─> frq1_dispatch

call 0x80380000
  ホスト ──USB 制御転送──> servicer ──call 0x80380000──> _start
                                                 └─> frq1_dispatch
```

2 本目は入口だけが残っています（`_start` は 24 バイト）。

* 主 FR の入口のアセンブリは [`../src/main/entry.S`](../src/main/entry.S)・
  [`../src/main/entry_frc1_selene.S`](../src/main/entry_frc1_selene.S)・
  [`../src/main/entry_frc1_dione.S`](../src/main/entry_frc1_dione.S) の 3 本です。
  主 FR の残りはすべて C で書いてあります（ビルドの手順は [`toolchain.md`](toolchain.md)）。
  補助コアと ARM コアのイメージには、それぞれ入口のアセンブリが 1 本あります
  （`../src/auxcore/entry.S`・`../src/armcore/vectors.S`）。
* 機種ごとの違い（I2C の届け方、復調、チューナー）は、C ではなく
  [`../config/`](../config) の表で決まります（[`devices.md`](devices.md)）。
* 配布の形式は [`FRC1`](frc1.md) です。ローダの 8 バイトを差し替えるので、
  servicer を通らずに枠が届きます。ホスト側のサンプルは
  [`../sample/`](../sample/) にあります（ファームローダ、TS の取り出し、ARM の道具、カードの IFD ハンドラ）。
* 中身の無いモジュールや、その機種で番地の分かっていない部品は、`FW_ENOTIMPL`（-1）を
  返します（[`modules.md`](modules.md)）。

## 検証の状況

確かめてあるのは次のところまでです。

* FR のエミュレータ（[`../tests/fr/fremu.py`](../tests/fr/fremu.py)）で
  実際に走らせて返ったバイト
* モデル化した I2C コントローラと取り込みリングに対して、手順が文書どおりに
  出ること
* 実機では、GV-M2TV と PIX-DT295 に `FRC1` が入り、TS・カード・ARM まで動いたこと

どこまでが実測で、どこからが机上の設計かは、
[`modules.md`](modules.md) の状態の表と、一部の `.c` の先頭の注記に書いてあります。
