# `host/`（ホスト側で動くもの）

`src/` はチップの上で動くもの、ここはホストの上で動くものを置いています。

```
python3 host/build.py           # 全部ビルドする
python3 host/genboards.py       # 機種表を検査して一覧する
```

| | |
|---|---|
| [`build.py`](build.py) | ビルドドライバです。`config/` を読んで `src/` をビルドし、`build/` に出力します |
| [`genboards.py`](genboards.py) | `config/` の機種表を読んで検査し、1 機種の行を `build/board_<id>.c` に書き出します。`build.py` が import します |
| [`frcontainer.py`](frcontainer.py) | `FRC1` の読み書きです。`build.py` が import します |

（ローダのイメージは置いていません。下の「ローダの指紋」を参照してください）

出力先はここではなく `../build/` です。`config/` も `src/` も
リポジトリのルートから見た位置のままで、`build.py` の `_HERE` はルート
（このディレクトリの親）を指します。ここに置いたのは実行するファイルの置き場所を分けるためで、ツリーの
基準点は変えていません。

## ローダの指紋

`frc1` をビルドするには、書き込み先のローダの `crc32` が必要です。インストールのときに
「このローダ向けにビルドしたものか」を確かめるための欄で、違えばインストーラが
拒否します。

値は `config/devices/<id>.json` の
`loader.crc32` に 16 進の文字列（`"0x76894854"` の形）で入っています。

| 機種 | 長さ | `crc32` |
|---|---|---|
| PIX-DT295 | `0x3910` | `0xf6ecc1ab` |
| GV-M2TV（dione） | `0x2620` | `0x76894854` |

新しい機種の値を求めるには、その機種のチップからローダを読み出し、先頭 `bytes` バイトの `zlib.crc32` を計算します。`null` を
書くと検査をしません。

## 外部の依存

* `fr30-elf` のツールチェイン。ビルドの方法と探す場所は
  [`../docs/toolchain.md`](../docs/toolchain.md) にあります
* `arm-none-eabi` のツールチェイン（無くてもよい）。ARM のイメージ `armcore.bin` だけに使います。
  無ければ `build.py` はそのイメージを飛ばします（[`../docs/toolchain.md`](../docs/toolchain.md)）
