# GV-M2TV・PIX-DT295 のカードの差し込み口を PC/SC のリーダーにする（macOS・Linux）

CFW のカードの枠（帯 `0x50`–`0x57`、[`../../docs/card.md`](../../docs/card.md)）を、
macOS と Linux（pcsc-lite）の PC/SC から ISO 7816 のスマートカードリーダーとして見せる
IFD ハンドラです。

バンドルは GV-M2TV（`04bb:053a`）と PIX-DT295（`06b8:103b`）の両方を名乗ります。
`card_tool` などの道具では、環境変数 `CFW_CARD_DEV=m2tv|dt295` で機種を選びます
（指定が無ければ、開けるものを順に探します）。ARM の経路は `CFW_CARD_ARM=0|1` で選びます
（`M2TV_CARD_ARM` という名前でも受け付けます）。

PIX-DT295 では、`card_tool atr` が主 FR と ARM-E の両方の経路で通ります（実測）。
DT295 でもカードは専用の郵便受け（`0x003B0040`）を使うので、
`ts_grab` と同時に使えます（[`../../docs/frq1.md`](../../docs/frq1.md)）。
PC/SC のリーダーとしても動きます（ATR・抜き差し・TS との同時使用。macOS で実測）。

## IFD ハンドラを使う理由

macOS 26（Darwin 25）でも、CryptoTokenKit の `com.apple.ifdreader` は
他社の IFD バンドルを読み込みます。動きは次のとおりです（macOS 26 で実測）。

- 探す場所は `/usr/local/libexec/SmartCardServices/drivers/` です
  （`com.apple.ifdreader` の文字列にあります）
- launchd は、どの USB 機器が接続されても（`IOUSBHostDevice`）ifdreader を
  起動します。ifdreader はバンドルの `Info.plist` の `ifdVendorID` /
  `ifdProductID` で機器を割り当てます。GV-M2TV は CCID のクラス（`0x0B`）を
  名乗りませんが、それでも割り当てられます
- バンドルを読み込むのは XPC サービス `com.apple.ifdbundle` です。これは
  `com.apple.security.cs.disable-library-validation` を持つので、
  アドホック署名のバンドルでも読み込めます
- 呼ばれる関数は、pcsc-lite の IFD ハンドラ v3 と同じ名前です
  （`IFDHCreateChannelByName`・`IFDHPowerICC`・`IFDHTransmitToICC`・
  `IFDHICCPresence` など）

機器の USB 記述子はローダ状態のときのままなので、機器は CCID のリーダーとしては見えません。
PC/SC には、この IFD ハンドラがリーダーとして見せます。

## 層

| 層 | どこ |
|---|---|
| PC/SC（APDU） | [`ifd_m2tv.c`](ifd_m2tv.c) |
| T=1（順序番号・連鎖・再送・WTX・IFS） | [`t1.h`](t1.h)。すべてホストで処理します |
| カードの枠（`CARD_ON`・`PARAM`・`PUT`・`SEND`・`RECV`・`GET`） | [`card_link.h`](card_link.h) |
| `0xBC` の窓、libusb | [`../chipio.h`](../chipio.h)、[`../frq1.h`](../frq1.h) |

ATR を読んだら、そこから F・D・CWI・BWI を決めて `CARD_PARAM` を送り、
S(IFS 要求、IFSD = 251) を 1 往復させます。PPS は行いません。TA2 があれば
（特定モード）TA1 の F・D で、無ければ既定の 372・1 で動かします。
対応するのは T=1 だけです。CRC（TC3 bit0）を使うカードは受け付けません。

## 動作

- PC/SC のリーダーとして見えるだけです。機器のカードの差し込み口を PC/SC に見せ、
  APDU を中継します。カードの中身には触れません
- APDU は既定では通しません。電源投入・ATR・S(IFS) は常に通ります。`IFDHTransmitToICC` は
  APDU を 1 つも送らず、先頭の 4 バイトを記録に残して断ります。
  macOS はカードを見つけるたびに自分で APDU を送る（トークンの照合）
  ので、既定では止めてあります。通すには、インストールしたバンドルの `Info.plist` の
  `CFWAllowAPDU` を `true` にして、機器を接続し直します
- USB は呼ばれたときだけ開きます。同じ窓を `frc1_load` と `ts_grab` も
  使います。1 つの枠は約 34 回の制御転送なので、2 つのプログラムが同じ郵便受けに
  枠を送ると互いに壊してしまいます。このドライバは郵便受け 1（GV-M2TV は `0x000C5040`、
  PIX-DT295 は `0x003B0040`）を使い、`ts_grab` は郵便受け 0（`0x000C5000`・`0x003B0000`）を
  使います。そのため `ts_grab` で録画しながらカードを使えます（CFW の郵便受けは 4 つです。
  [`../../docs/frq1.md`](../../docs/frq1.md)）。機器ごとの番地は [`../cfwdev.h`](../cfwdev.h) にあります。
  郵便受けが 1 つだった古い CFW では、このドライバの枠に誰も答えません
- 枠を送る前に、hook（GV-M2TV は `0x000E9F80`、PIX-DT295 は `0x000EA6FC`）を読んで
  確かめます（読むだけです）。CFW の
  stub（`ldi:20 #entry, r12` / `jmp @r12` / `nop`）でなければ枠を
  送りません。そのため `frc1_load install` の途中に割り込むことはありません
- 電源投入が間に合わなかったときは、3 秒間だけ「カードなし」と答えます。
  macOS は答えなかったカードを「応答なし」（`SCARD_STATE_MUTE`）にして、
  抜き差しを検出するまで再び試しません。抜き差しがあったように見せないと、挿し直すまで
  使えなくなります
- 挿抜の検出は、既定では 1 回だけです。GV-M2TV のカードは抜けない構造です。CFW が
  `CARD_EVENT` bit10 = 1 を一度返したら、その後は問い合わせずに「ある」と答えます。
  それまでは 3 秒に 1 回問い合わせます。この判断は [`presence.h`](presence.h) にまとめてあります
- カードを抜ける機種では `CFWCardRemovable` を使います。`Info.plist` に真偽値のキー
  `CFWCardRemovable` を `true` で追加すると、カードを検出した後も 500 ms に 1 回問い合わせ直します。
  bit10 が 0 の答えが来たら「無い」と答え、電源も切れたものとして扱います（次の
  送信は失敗し、`pcscd` が次のカードに電源を入れ直します）。答えが来ない間は、
  前の答えのままにします

## ビルド

```
make            # バンドル・card_tool・pcsc_atr を作り、T=1 と在否の検査を回す
make test       # 検査だけ。機材も libusb も要らない
```

libusb-1.0（`brew install libusb`）の `.a` をバンドルに静的にリンクします。

Linux でも同じ `make` でビルドできます。`uname` が `Linux` なら、バンドルの中身は
`Contents/Linux/libifd_m2tv.so`（pcsc-lite の形）になり、`Info.plist` の
`CFBundleExecutable` もそれに合わせて書き換えます。必要なパッケージは
`apt install build-essential pkg-config libusb-1.0-0-dev libpcsclite-dev` です。
記録は syslog（名前 `ifd-cfw-m2tv`）に出ます。`Info.plist` のキーは同じものを
自前で読みます（`<true/>` と `<integer>0|1</integer>` だけに対応しています）。

| 出来るもの | 何か |
|---|---|
| `ifd-cfw-m2tv.bundle` | IFD ハンドラです。アドホック署名です |
| `card_tool` | 同じコードを端末から使います。`status`（読むだけ）と `atr`（電源投入・ATR・S(IFS)・切断）があります。APDU は送りません |
| `pcsc_atr` | 最小の PC/SC クライアントです。リーダーを列挙し、カードを待ち、接続して ATR を表示し、切断します。APDU は送りません |
| `test_t1` | 模擬カードで T=1 を検査します（連鎖・WTX・壊れたブロック・消えたブロック）。ATR の段階で、T=0 だけのカードと CRC のカードに印が付くことも確かめます |
| `test_presence` | 模擬の差し込み口で在否を検査します。抜けない口（一度検出したら問い合わせない）と、抜ける口（抜くと 500 ms 以内に「無い」になり電源も切る・戻すと「ある」・答えが無い間は前のまま・問い合わせは 500 ms に 1 回）を確かめます |

## インストール（sudo が必要）

先に CFW が動いていることを確かめます（`card_tool status` が答えるかどうか）。

インストールしたあとは、macOS の PC/SC（`com.apple.ifdreader`）がバンドルを読み込み、
カードの在否を問い合わせ続けます。バンドルと `card_tool` は同じ郵便受けを使うので、
その間に `card_tool atr` を使うと、相手の答えを読んで `CFW did not answer` になることがあります
（GV-M2TV で実測）。インストールしたあとは `pcsc_atr` で確かめるか、バンドルを外してから
`card_tool` を使ってください。Linux の `pcscd` でも同じことが起きると推定していますが、未確認です。
コマンドはリポジトリの最上位から打つ形で書いてあります。

```
make -C sample/ifd
sudo mkdir -p /usr/local/libexec/SmartCardServices/drivers
sudo cp -R sample/ifd/ifd-cfw-m2tv.bundle /usr/local/libexec/SmartCardServices/drivers/
sudo chown -R root:wheel /usr/local/libexec/SmartCardServices/drivers/ifd-cfw-m2tv.bundle
```

ifdreader がバンドルを選ぶのは、機器が接続されたときです。そのため、インストールした後に
機器を接続し直します。接続し直すと CFW が消えるので、もう一度書き込みます
（PIX-DT295 なら `build/frc1-dt295.frc`）。

```
sample/frc1_load install build/frc1-m2tv.frc
```

CFW がまだ無くても、ドライバは待っています（3 秒に 1 回 hook を読みます）。

## 動作の確認

```
sample/ifd/pcsc_atr GV-M2TV
```

リーダー `GV-M2TV CFW card slot`（PIX-DT295 なら `PIX-DT295 CFW card slot`）が表示され、
カードがあり、ATR が出れば成功です。
macOS では、ドライバの記録を次のコマンドで見られます。

```
log stream --info --predicate 'subsystem == "cfw.ifd"'
```

ARM を経由させる場合は、ARM のイメージを走らせてから（`sample/arm_probe hold`・
`sample/arm_probe image build/armcore.bin`・`sample/arm_probe run`）、
`CFW_CARD_ARM=0 sample/ifd/card_tool atr` を実行します。
カードの待ちは ARM-E の中で済みます（[`../../docs/card.md`](../../docs/card.md) の
「ARM の経路」）。この経路で ATR と S(IFS) が取れることは、GV-M2TV と PIX-DT295 の
ARM-E で実測です。

ドライバも ARM の経路で動きます（GV-M2TV の ARM-E・ARM-C で実測）。選んだ ARM の
イメージが走っていないと、電源投入が `IFD_COMMUNICATION_ERROR`（612）になります。

`pcscd` に環境変数を渡す方法が無いので、`Info.plist` のキーで選びます。macOS の
PC/SC（`ctkpcscd`）はシステムが起動するので、環境変数を設定できません。バンドルの
`Info.plist` に整数のキー `CFWCardARM`（`0` ARM-E・`1` ARM-C）を追加すると、
ドライバはカードの待ちをその ARM に任せます。キーが無ければ主 FR を使います。環境変数
`CFW_CARD_ARM`（または `M2TV_CARD_ARM`）が設定されていれば、そちらが優先されます
（手で試すときのためです）。

```
/usr/libexec/PlistBuddy -c "Add :CFWCardARM integer 1" ifd-cfw-m2tv.bundle/Contents/Info.plist
codesign --force --sign - ifd-cfw-m2tv.bundle     # plist を変えたら署名し直す
```

経路の選び方は次のとおりです（GV-M2TV で実測）。

| 設定 | 経路 |
|---|---|
| キーも環境変数も無い | 主 FR |
| `CFWCardARM = 0` / `1` | ARM-E / ARM-C |
| 環境変数 `CFW_CARD_ARM`（`M2TV_CARD_ARM`）がある | 環境変数の値。キーより優先します |

`pcscd` が読み込むインストール済みのドライバで `CFWCardARM` を使う形は未確認です。
入れ替えに `sudo` が必要です。

`CFWCardRemovable` を `true` にすると、記録に「asked every 500 ms」と出て、電源を入れた後も
問い合わせ直します。カードが挿さっている間の答えは、キーが無いときと同じです（GV-M2TV で実測）。
PIX-DT295 では、カードを抜くと `pcsc_atr` が「無い」（state `0x12`）を返し、挿し直すと同じ ATR を
返します（実測）。

インストールしたバンドルを書き換えたときは、`com.apple.ifdreader` を起動し直します。動いている
ifdreader は起動したときの表を使い続けるので、たとえば `0x103B` を追加しても DT295 にドライバを
割り当てません。`sudo killall com.apple.ifdreader` では止まらないので、`sudo kill -9 <pid>` で
止めてから機器を接続し直します（macOS 26 で実測）。書き換えた後は
`sudo codesign --force --sign -` で署名し直します。

### Linux

`make -C sample/ifd` で `libifd_m2tv.so` ができます。バンドルを pcsc-lite の
ドライバの置き場（`/usr/lib/pcsc/drivers/` など。ディストリビューションによって違います）に置きます。
ビルドと、`pcscd` が `.so` を読み込むことは実測です。USB の機器を接続した Linux での動作は未確認です。

記録は syslog に、名前 `ifd-cfw-m2tv` で出ます。systemd の journal があれば、次のコマンドで見られます。

```
journalctl -f -t ifd-cfw-m2tv
```

## アンインストール

```
sudo rm -rf /usr/local/libexec/SmartCardServices/drivers/ifd-cfw-m2tv.bundle
```

その後、機器を接続し直します。
