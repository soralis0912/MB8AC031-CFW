# MB8AC031 CFW

Fujitsu / Socionext MB8AC031（MB86M01 / 02 / 03）と、1 世代前の MB8AC018 を積んだテレビチューナー用の
自作ファームウェアです。ライセンスは [MIT](LICENSE) です。

ホストは 64 バイトの `FRQ1` の枠を機器に送り、ファームウェアはそれに答えを書いて返します。
主 FR のイメージは、入口のアセンブリ 3 本を除いて C で書いてあり、`fr30-elf` の GCC でビルドします。
補助コアと ARM コアのイメージにも、入口のアセンブリが 1 本ずつあります（`src/auxcore/entry.S`・`src/armcore/vectors.S`）。
機種ごとの違い（I2C の経路、復調 IC、チューナー）は `config/` の表に書きます
（[`docs/devices.md`](docs/devices.md)）。

## 対応機種と現状

| 機種 | USB ID | 系統 | 状態 |
|---|---|---|---|
| PIX-DT295 | `06b8:103b` | selene（MB8AC031） | 実機で TS とカードを確認 |
| GV-M2TV | `04bb:053a` | dione（MB8AC018） | 実機で TS とカードを確認 |

どちらの機種でも、純正のファームウェアを使わずに生の TS が取れ、カードの差し込み口が
PC/SC のカードリーダーとして見えます（実測）。

選局はホストが行います。ファームウェアには選局のコマンドがなく、ホストが `I2CXFER` で
チューナーと復調 IC を設定し、`STATUS` でロックを確かめてから `STREAM` で TS を受け取ります。
サンプルの `ts_grab` には、2 機種の選局の手順が入っています（`sample/fe_m2tv.h`・`sample/fe_dt295.h`）。
モジュールごとの状態は [`docs/modules.md`](docs/modules.md) にまとめてあります。

## ビルド

`fr30-elf` の binutils と GCC が必要です。手順は [`docs/toolchain.md`](docs/toolchain.md) にあります。

```
python3 host/build.py --list          # ターゲットと機種表を表示する
python3 host/build.py                 # すべてビルドする（出力は build/）
python3 host/build.py frc1            # ターゲットを名指す（いまは frc1 だけ）
python3 host/build.py --boards dt295  # コンテナを 1 機種のぶんだけ作る
python3 host/build.py --map           # どこに何が配置されたかを表示する
```

ターゲットは `frc1` の 1 つです（`config/fw.json` の `targets`）。出力は機種ごとに 1 本のコンテナ
`FRC1`（`build/frc1-<id>.frc`）で、実機に入れるのはこれです。

補助コアのイメージ（`config/auxcore.json`、`build/auxcore.bin`）と ARM コアのイメージ
（`config/armcore.json`、`build/armcore.bin`）は、ターゲットの指定によらず毎回ビルドします。
ARM コアは、`arm-none-eabi` が無ければ飛ばします。

push のたびに GitHub Actions がビルドとテストを行い、`.frc` などを成果物として残します。
`v` で始まるタグを push すると、リリースにも添付されます（[`docs/toolchain.md`](docs/toolchain.md) の「CI」）。

## テスト

テストに実機は要りません。FR のエミュレータと、機器の動きを Python で再現したモデルを使います。

```
for t in tests/test_*.py; do python3 "$t" || break; done
```

`tests/` には 16 本の検査があります。主なものは次のとおりです。

```
python3 tests/test_frq1.py            # 枠への応答（FR のエミュレータ）
python3 tests/test_frc1.py            # コンテナの書き手と、docs/frc1.md の struct
python3 tests/test_i2c.py             # 機種ごとの I2C の経路
python3 tests/test_ring_selene.py     # selene の取り込みリングの読み出し
python3 tests/test_ts_dione.py        # dione の TS の経路（20 分ほどかかります）
python3 tests/test_i2c_dione.py       # dione の I2C（I2CXFER）
python3 tests/test_card.py            # カード I/F
python3 tests/test_arm.py             # ARM コアの制御（dione・selene）
python3 tests/test_host.py            # このリポジトリだけでビルドできるか
```

## サンプル

`sample/` に、ホスト側のサンプルがあります。ファームウェアを機器に入れる `frc1_load`、
TS を取り出す `ts_grab`、チップのレジスタを読む道具などです（[`sample/README.md`](sample/README.md)）。

```
make -C sample
```

カードリーダーとして使うための PC/SC のドライバは `sample/ifd/` にあります。
ATR と APDU を中継するだけで、カードの中身は扱いません。

## ディレクトリ

| | |
|---|---|
| `src/main/` | 主 FR のイメージ（各モジュールと 3 本の入口） |
| `src/auxcore/` | 補助コアのイメージ（主 FR とは別の CPU） |
| `src/armcore/` | ARM コア 2 基のイメージ（`arm-none-eabi` でビルド。[`docs/arm.md`](docs/arm.md)） |
| `src/shared/` | 複数のイメージで共有するもの（レジスタ、枠の定義、I2C の状態機械） |
| `config/` | ビルドの設定と機種表 |
| `host/` | ホストで動く道具（ビルド、コンテナの読み書き） |
| `sample/` | ホスト側のサンプル |
| `tests/` | テストと FR のエミュレータ |
| `docs/` | 仕様書。入口は [`docs/README.md`](docs/README.md) |
