# 機種表

[`../config/devices/`](../config/devices) に機種ごとの行、
[`../config/drivers.json`](../config/drivers.json) に共有のドライバの表があります。
[`../host/genboards.py`](../host/genboards.py) が 1 機種の行を C に変換し、
[`../src/main/drivers.h`](../src/main/drivers.h) にインターフェースの定義があります。

イメージは機種ごとに 1 本です。FRC1 のコンテナが機種ごとに 1 本なので、中のイメージも
その機種の行だけを持ちます。ファームウェアは、動いている機器を調べません。

## 表にする理由

この LSI は復調を行いません。復調 IC もチューナーも外付けのモジュールの中にあり、
機種によって違うものが載っています。

さらに、I2C の準備も機種によって違います。PIX-DT295 と GV-M2TV では、
`0x00FD8FE0` の富士通のコントローラを主 FR から動かします（実測）。DT295 では、その前に前段のリセットを
解く必要があります（[`i2c.md`](i2c.md) の「DT295 の主 FR からの経路」）。

同じ設計の I2C コントローラが、このチップに 2 つ載っています。
MB8AC031 では `0x01684FE0` の実体を補助ユニット `0x017` の側からも動かせます
（DT295 で実測。今はどの行もこの経路を選んでいません）。状態機械は共通で、基底アドレスだけが引数です（[`i2c.md`](i2c.md)）。

そのため、I2C の経路も、復調 IC やチューナーと同じく機種の行で選びます。

## 構成

```
              config/devices/<id>.json
                           |
           +---------------+---------------+
           |                               |
        i2c_ops                        demod_ops
     src/main/i2c/                  src/main/demod/
```

チューナーのドライバはありません。ファームウェアはチューナーを操作せず、選局は
ホストが `I2CXFER` で行います（[`frq1.md`](frq1.md)）。行の `tuner` は、
品番・アドレス・確かさを記録しているだけです。

`build.py` がイメージをビルドするとき、`genboards.py` がその機種の行を
`build/board_<id>.c`（`const struct board fw_board` 1 件と、使う ops の
extern 宣言）に書き出し、そのイメージにだけリンクします。生成物は `build/` に置き、
コミットしません。中身は `python3 host/genboards.py --c <id>` で確認できます。

ドライバのソースは、すべてどのイメージにも入ります。どの機種も選んでいないドライバも
コンパイルとリンクを通るので、壊れたまま気づかれないということがありません。

## struct

実装は [`../src/main/drivers.h`](../src/main/drivers.h) にあります。生成される行は
`struct board` 1 件（`fw_board`）で、分かっていない部品はヌルポインタ、
分かっていないアドレスは 0 になります。振り分けの処理がそれを `FW_ENOTIMPL` に変えます。

```c
#define FW_SRC_T        0u      /* 地上波 1 本目（ISDB-T）              */
#define FW_SRC_T2       1u      /* 地上波 2 本目                        */
#define FW_SRC_S        2u      /* 衛星 1 本目（ISDB-S）                */
#define FW_SRC_S2       3u      /* 衛星 2 本目                          */
#define FW_SRC_COUNT    4u

/* 裏取りの段階。表が持っていて、半分しか分かっていない機材を
 * 駆動しないための判断材料にする */
#define EV_UNKNOWN      0u
#define EV_INFERRED     1u
#define EV_MEASURED     2u

struct board;

/* 経路。1 転送（n バイト書いて m バイト読む）だけが原始的な操作 */
struct i2c_ops {
    const char *name;
    int (*init)(const struct board *b);
    int (*xfer)(const struct board *b, unsigned addr,
                const unsigned char *wb, unsigned wn,
                unsigned char *rb, unsigned rn);
};

struct demod_ops {
    const char *name;
    int (*init)(const struct board *b, unsigned source);
    int (*locked)(const struct board *b, unsigned source, int *locked);
    int (*cnr_raw)(const struct board *b, unsigned source, unsigned *raw);
};

/* 受信帯 1 つ（IDENTIFY の頁 1、docs/frq1.md） */
#define FW_BANDS        2u
#define BAND_HOST_TUNED 0x01u   /* ホストが I2CXFER で選局する          */
#define BAND_LOCK_SEEN  0x02u   /* 帯の中でロックを実測した             */
struct band {
    unsigned char       source;         /* FW_SRC_*                     */
    unsigned char       flags;          /* BAND_*                       */
    unsigned short      record;         /* 1 記録のバイト数。TS は 188  */
    unsigned            lo_khz, hi_khz; /* 中心周波数の下限・上限       */
    unsigned            step_hz;        /* チューナーの刻み             */
};

/* config/devices/<id>.json 1 本。genboards.py が fw_board に落とす */
struct board {
    unsigned            usb;            /* vid<<16 | pid、0 = 製品なし  */
    const struct i2c_ops    *i2c;       /* 0 = 経路が分からない         */
    const struct demod_ops  *demod;
    unsigned char       demod_addr[FW_SRC_COUNT];   /* 7 bit、0 = 未確定 */
    unsigned char       tuner_addr;
    unsigned char       tuners;
    unsigned char       ev_demod;       /* EV_*                         */
    unsigned char       ev_tuner;
    unsigned char       card_slot;      /* 0 / 1 / CARD_UNKNOWN         */
    unsigned char       card_polarity;  /* 0 / 1 / CARD_UNKNOWN         */
    unsigned char       ev_card;        /* EV_*                         */
    unsigned char       nbands;         /* bands[] の有効な数           */
    struct band         bands[FW_BANDS];
    unsigned            ts_in0;         /* 0x01901000 に使う値          */
    unsigned            ts_in1;         /* 0x01901004 に書く値          */
    unsigned char       ev_ts;          /* EV_MEASURED のときだけ取り込む */
};

#define CARD_UNKNOWN    0xffu

extern const struct board fw_board;     /* この像の機種の行           */
```

## `drivers` の節

ドライバを登録する節です。`source`（ソースの位置）、`ops`（C の構造体の名前）、
`evidence` を持ちます。

| 種類 | インターフェース | 持つもの |
|---|---|---|
| `i2c` | `struct i2c_ops` | `init` と `xfer`（n バイト書いて m バイト読む 1 転送） |
| `demod` | `struct demod_ops` | `init` / `locked` / `cnr_raw` |

経路の操作が `xfer` の 1 つだけなのは、経路が持つ原始的な操作がこれだけで足りるためです。
「レジスタ 1 本の読み書き」のような操作は、`i2c.c` がその上に実装します。

## `boards` の節

| 列 | 中身 |
|---|---|
| `id` | 短い名前です。ファイル名と同じで、`build/frc1-<id>.frc` と `build.py --boards <id>` に使われます |
| `name` | 製品名 |
| `usb` | `VID:PID`。コンテナのターゲットと、IDENTIFY の `+0x08` になります。`null` は製品を指定しない行です（`unknown`、ちょうど 1 つ） |
| `i2c` | 経路のドライバ名 |
| `demod` | `driver` / `part` / `module` / `addr_t` / `addr_s` / `addr_t2` / `addr_s2` / `evidence`。`addr_t2`・`addr_s2` は 2 台目の地上・衛星のアドレスで、無ければ `null` です |
| `tuner` | `part` / `addr` / `evidence`。記録だけです（`driver` は置きません。あると `genboards.py` が拒否します） |
| `tuners` | チューナーの本数 |
| `bands` | 受信帯の表です。2 つまで書けます。省くと帯は 0 個です（[`../config/README.md`](../config/README.md) の `bands`） |
| `ts` | `in0` / `in1` / `evidence`。TS の取り込みの入口の値です（下） |
| `card` | `slot` / `polarity` / `evidence`。`slot` は差し込み口があるかで、`true` / `false` / `null`（分からない）のどれかです。`polarity` は `0` / `1` / `null` です。省くと「分からない」扱いになり、カードのコマンドは `FW_ENOTIMPL` を返します（[`card.md`](card.md)） |
| `loader` | 常駐ローダの番地と指紋です。これがある行が FRC1 コンテナのターゲットになります（[`../config/README.md`](../config/README.md) の `loader`） |
| `evidence` | その行自体の裏取りです。USB ID とローダの指紋をどこまで実測したかを表します。動作を確かめたかどうかではありません |

`genboards.py` は `card.slot` の `true` / `false` / `null` を、`card_slot` の
`1` / `0` / `CARD_UNKNOWN` に変えます。`polarity` も同じで、`null` は `CARD_UNKNOWN` になります。

### `ts`（TS の取り込みの入口）

selene の取り込み（`tsif_selene.c`、[`ts.md`](ts.md)）が使う値です。

| 鍵 | 中身 |
|---|---|
| `in0` | `0x01901000` に使う値です。上位 16 bit を半語で書きます |
| `in1` | `0x01901004` に書く値です |
| `evidence` | `measured` か `unknown` だけです。`inferred` は `genboards.py` が拒否します |

`evidence` が `measured` の行は、`in0` と `in1` を両方持つ必要があります。`measured` 以外の行
（`ts` を省いた行も含む）では、`tsif_init` も `tsif_start` も何も書かずに `FW_ENOTIMPL` を返します。
今は PIX-DT295 だけが `measured` で、値は `0x06070000` / `0x04000000` です（実測）。

### 裏取りの段階

| 印 | 意味 |
|---|---|
| `measured` | 実機で確認した（USB の記述子・メモリのダンプ・I2C の応答） |
| `inferred` | 基板の刻印や同系の機種からの推定。チップでは確かめていない |
| `unknown` | 分かっていない |

生成した表はこの段階を持っており、分かっていない欄は 0 になり、ドライバは
`FW_ENOTIMPL` を返します。これにより、推定の I2C アドレスを使って機器に書き込むことはありません。

例として、GV-M2TV には衛星の復調 IC が無いので `addr_s` が `null` です。DT295 で実測した
`0x11` は流用していません。そのため GV-M2TV では、衛星の `demod_init` が
バスに何も出さずに `FW_ENOTIMPL` を返します。

`genboards.py` は表を読むときに形を検査し、次のものを拒否します。

* ドライバが無いのに `evidence` が `unknown` でない行
  （分からないものは `unknown` にします）
* `card.slot` が `true` / `false` / `null` でない行、`card.polarity` が `0` / `1` / `null` でない行、
  `card.evidence` が `unknown` でないのに `polarity` が `null` の行
* `ts.evidence` が `inferred` の行、`measured` なのに `in0`・`in1` が欠けた行
* 帯が 3 つ以上ある行、帯の鍵が欠けた行、`tuned_by` が `host` でない帯
* USB ID の重複
* `usb` が `null` の行が 1 件でない表

## 使われる行の決まり方

使われる行はビルドのときに決まります。`frc1-<id>.frc` のイメージは `<id>` の行で
リンクしてあり、コンテナはその機種のローダにしか書き込めません
（`usb` とローダの指紋で `frc1_load` が選びます）。ほとんどのテスト用のイメージは `unknown` の行でリンクします。

ファームウェアは動いている機器を調べる方法を使いません。ストラップ（`0x00F10004` の
bit10-11）が指す先（`0x0800` なら `0x017A3D08`）に USB 記述子がある、という
見当はありますが、FRC1 の経路では `0x017A3D08` の値は雑音です（selene の別の機器で実測）。
1 コンテナが 1 機種に対応しているので、調べる必要がありません。

## 機種の追加

`config/untested/` を作り、ファイルを 1 つ追加します。C のコードは変更しません。
`loader` を書くと `frc1-<id>.frc` が増えます。そのイメージが実機で動いたら
`config/devices/` へ移します（[`../config/README.md`](../config/README.md)）。
品番が分かっていない場合は、`null` と `"unknown"` のままにしておきます。

```
python3 host/genboards.py              # 表を検査して一覧する
python3 host/genboards.py --c dt295    # その機種の行の C を見る
python3 host/build.py --boards dt295   # その機種のコンテナだけを組む
```

## 今の表

| 機種 | USB | 経路 | 復調 | チューナー |
|---|---|---|---|---|
| GV-M2TV | `04BB:053A` | `fujitsu-fd8f` | TC90517、7 bit `0x18`、モジュールは Sony SMT-MJ111（すべて推定） | Sony SMT モジュールの中（品番は不明） |
| PIX-DT295 | `06B8:103B` | `fujitsu-fd8f`（実測。主 FR が `0x00FD8FE0` を動かす） | TC90532XBG、`0x10`/`0x11`（実測） | MxL661（推定） |
| 製品なし（`unknown`） | — | — | — | — |
