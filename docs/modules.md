# モジュールの契約

実装は [`../src/shared/fw.h`](../src/shared/fw.h) で、この文書はその説明です。

## 世代で分かれるものの名前

`selene`（MB8AC031 / MB86M0x）と `dione`（MB8AC018 / MB86H5x）で中身が
違うファイルは、名前の後ろに `_selene` / `_dione` を付けます。

| 何 | selene | dione |
|---|---|---|
| FRC1 の入口 | `entry_frc1_selene.S` | `entry_frc1_dione.S` |
| 置き場所 | `link_selene.ld` | `link_dione.ld` |
| TS の番地 | `regs_selene.h` | `regs_dione.h` |
| TS の入口・リング・出口 | `tsif_selene.c`・`ring_selene.c`・`tsout_selene.c` | `ts_dione.c` |
| 前段の電源と I2C の経路 | （`fe.c` と `i2c/`） | `fe_dione.c` |
| ARM の帯を起こす | `arm_selene.c` | `arm_dione.c` |
| 検査 | `test_ring_selene.py`・`test_ts_selene.py`・`test_ts_dt295.py`・`test_entry_selene.py` | `test_ts_dione.py`・`test_i2c_dione.py`・`test_entry_dione.py` |

両方で同じものには名前を付けません（`mb86m01.h`・`stream.c`・`usbep.c` など）。
`stream.c`・`arm.c` は、イメージの世代を見て処理を振り分けます。

## 返り値

| 値 | 名前 | 意味 |
|---|---|---|
| `0` | `FW_OK` | できた |
| `-1` | `FW_ENOTIMPL` | まだ書いていない / この機種では分かっていない |
| `-2` | `FW_EIO` | 機器が答えなかった |
| `-3` | `FW_ETIMEOUT` | 待ったが来なかった |
| `-4` | `FW_EINVAL` | 引数がおかしい |
| `-5` | `FW_EAGAIN` | 誤りではないが、まだ足りない |
| `-6` | `FW_EBADCMD` | 知らないコマンド（振り分けだけが返す） |
| `-7` | `FW_EBUSY` | その周辺は別のコアが使っている（カードを ARM に渡している間の `card.c` など） |

この表の値がそのまま枠に出ます。状態欄（`+0x0C`）は 1 語で、`+1` が成功、
負の値はこの表の値です（[`frq1.md`](frq1.md)）。ホスト側の定義は
[`../sample/frq1.h`](../sample/frq1.h) にあります。

## `FW_ENOTIMPL` を返すモジュール

どのモジュールにも `.c` があります。中身の無いモジュールと、その機種で番地の
分かっていない部品の関数は `FW_ENOTIMPL` を返し、レジスタには何も書きません。
`frq1.c` の振り分けはすべてのコマンドを持っているので、ホストには、止まるのではなく
`FW_ENOTIMPL` の答えが返ります。

`card.c`・`ts_dione.c`・`arm.c`・`demod/tc9052x.c`・`shared/fjcore.c` と ARM の `armmain.c`・`svc_card.c` は、
先頭の注記（WHAT IS KNOWN / WHAT IS NOT）に、分かっていることと分かっていないことを分けて書いています。
ほかの `.c` には、この形の注記はありません。

## モジュールの一覧

| | 状態 | 根拠 |
|---|---|---|
| `frq1.c` | 動く | 枠の仕様（[`frq1.md`](frq1.md)） |
| `device.c` | 動く | 枠の仕様 |
| `probe.c` | 動く | チップを調べる道具（帯 `0x40`。PEEK・POKE・RING・I2CSCAN・EPDMA）。ファームウェアのほかの部分からは呼ばない（[`frq1.md`](frq1.md)） |
| `usbep.c` | 実機で動く（PIX-DT295） | 端点の FIFO へ 1 バースト送る。`tsout_selene.c` と `probe.c` が使う。手順は `mb86m01.h` の `USB_EP_*`（[`ts.md`](ts.md)） |
| `util.c` | 動く | `-ffreestanding` に要る分 |
| `gpio.c` | 動く | ストラップ（`0x00F10004`） |
| `clk.c` | 未実装 | `clk_init` は空の入口だけです |
| `i2c.c` + `shared/fjcore.c` | 実機で動く（GV-M2TV・PIX-DT295） | 状態機械は両世代で共通。主 FR の上で動く。補助コアのプログラムも同じものを使う（[`i2c.md`](i2c.md)） |
| `i2c/fujitsu.c` | 実機で動く（GV-M2TV・PIX-DT295） | `0x00FD8FE0` の実体。selene では前段のリセットと端子、両世代でゲート。順まで実測 |
| `i2c/auxslot.c` | 実機で動いた（PIX-DT295）。今はどの行も選ばない | `0x01684FE0` の操作を、補助コアに載せたこちらのプログラムに任せる。主 FR はプログラムを置いて起こし、枠で依頼する。DT295 の行は `i2c/fujitsu.c` を使う（[`i2c.md`](i2c.md)） |
| `demod.c` + `demod/tc9052x.c` | 読み出しは動く | ロックと C/N は読み出しだけ（地上波は GV-M2TV で実測、衛星は未確認）。`init` は復調 IC がいるかを見るだけで、立ち上げはホストが `I2CXFER` で送る（[`demod.md`](demod.md)） |
| `fe.c` | 報告は実機で動く（GV-M2TV・PIX-DT295） | ロックと C/N を枠で返す。選局のコマンドは無い。ホストが `I2CXFER` で選局する（[`frq1.md`](frq1.md)） |
| `ring_selene.c` | 実機で動く（PIX-DT295） | 記録 192 B・パケット 188 B・0x47 の探し方・空きの式。エミュレータの検査は通る（[`ts.md`](ts.md)） |
| `tsif_selene.c` | 実機で動く（PIX-DT295） | 立ち上げと停止の書き込みの順番と値。意味は未確認（[`ts.md`](ts.md)） |
| `tsout_selene.c` | 実機で動く（PIX-DT295） | 1 バーストの書き込みの列。エミュレータの検査は通る（[`ts.md`](ts.md)） |
| `ts_dione.c` | 実機で動く（GV-M2TV） | dione の取り込みと送出。`STREAM` だけで TS がホストに届く（[`ts.md`](ts.md)） |
| `fe_dione.c` | 実機で動く（GV-M2TV） | 前段の電源と `I2CXFER`。ホストの選局の列を 1 記録 1 枠で処理する（[`i2c.md`](i2c.md)） |
| `stream.c` | 実機で動く（PIX-DT295） | 1 要求ぶん流して報告する。主ループが無いので、止めるまで流し続ける形にはできない（[`frq1.md`](frq1.md)） |
| `arm.c` | 実機で動く（GV-M2TV の ARM-E・ARM-C、PIX-DT295 の ARM-E） | 帯 `0x60` のコマンド。両世代で共通（[`arm.md`](arm.md)） |
| `arm_ipc.c` | 実機で動く（GV-M2TV の ARM-E・ARM-C、PIX-DT295 の ARM-E。`ARM_CANCEL` は GV-M2TV だけ） | ARM の仕事のやり取りの主 FR 側。`ARM_CALL`・`ARM_REAP`・`ARM_CANCEL`（止め語）・`ARM_PUT`・`ARM_GET`（置き場）。中継するだけで待たない。カードの持ち主を数える（[`../src/armcore/README.md`](../src/armcore/README.md)） |
| `arm_dione.c` | 実機で動く（GV-M2TV） | `0x00F10118` の上半分を `0xFFFF` にして 2 基の帯を起こす |
| `arm_selene.c` | 実機で動く（PIX-DT295、ARM-E） | `0x00F10118` を開けて `+0x404` の bit2 を立てる。門は戻さない（戻すと帯が見えなくなる。[`arm.md`](arm.md) の「selene」） |
| `card.c` | 実機で動く（GV-M2TV・PIX-DT295） | 中から ATR と T=1 の 1 往復まで。PC/SC のリーダーもこれで動く。ARM が持っている間は `FW_EBUSY`（[`card.md`](card.md)） |
| `pm.c` | 実機で動く（PIX-DT295） | 軽い SUSPEND。取り込みを止めるだけ。チップの省電力は、書き込む先のレジスタが無い（`frq1.md` の SUSPEND 節） |
| `health.c` | 実機で動く（GV-M2TV） | `HEALTH`（`0x14`）。段階・旗・部品ごとの最初の失敗 |
| `hif.c` `sys.c` | 枠だけ | 番地は実測、方針が未決 |

## 振り分けと機種差

I2C と復調は、機種ごとに別のものになります。そのため
`i2c.c` / `demod.c` は機器を触らない振り分けだけを受け持ち、実体は
`src/main/i2c/`・`src/main/demod/` にあります。どれを使うかは
[`../config/devices/`](../config/devices) で決まります
（[`devices.md`](devices.md)）。

```
fw.h の i2c_* / demod_*       ← ほかのモジュールが呼ぶ口
          |
     i2c.c / demod.c            ← 振り分けだけ
          |
   drivers.h の i2c_ops / demod_ops
          |
   src/main/i2c/*.c  src/main/demod/*.c

チューナーは振り分けも実体も無い（ホストが I2CXFER で選局する）
```

どのモジュールも製品名を持っていません。分かっていない部品の関数は
ヌルポインタで、振り分けがそれを `FW_ENOTIMPL` に変えます。

### ドライバからの I2C の呼び出し

ドライバは `b->i2c->xfer` を直接呼ばず、`i2c.c` を通します。`i2c.c` が経路の
準備（`i2c_init`）を受け持っていて、それを 1 度だけ走らせるためです。

富士通のコントローラでは、クロックが入る前の書き込みは反映されません。準備を飛ばして
`xfer` を呼ぶと、バスに何も出ないまま、来ない `bit0` を待ち続けます。
`tests/test_i2c.py` は、ドライバが準備を通してから転送することを確かめます。

## 規約

* `-O2 -fno-builtin -ffreestanding -Wall -Wextra` でビルドし、警告 0 で通します。
* `.data` は作りません（[`image.md`](image.md)）。`const` か実行時の代入にします。
* static 変数は使えます。置き場は、リンカが不変領域の外に配置します。
* レジスタの定義は [`../src/shared/mb86m01.h`](../src/shared/mb86m01.h) にあり、
  各行に出所が書いてあります。
* 待ちにはすべて上限があります。I2C の上限は 1 バイトあたり `0x20000` 回です。
  実機に合わせた値ではなく、十分に大きく取った値で、ここに達するのはバスが塞がったときです。
