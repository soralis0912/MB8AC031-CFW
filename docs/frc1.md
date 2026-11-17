# `FRC1` コンテナ

このファームウェアを配る形式です。書き手は
[`../host/frcontainer.py`](../host/frcontainer.py)、ビルドするのは
[`../host/build.py`](../host/build.py) の `frc1` ターゲット、機器に入れるのは
[`../sample/frc1_load.c`](../sample/frc1_load.c)、読み手のヘッダは
[`../sample/frc1.h`](../sample/frc1.h) です。

## この形式を使う理由

実機で通っている経路はこれだけです。平文の経路では、
ホストが `0xBC` の窓から RAM にバイトを書き、常駐ローダの 8 バイトを
こちらへ向けます。署名も servicer も通りません。

`FRC1` は、「このバイト列をこの番地へ。このチップで、ビルドしたときの相手の
ローダであるときだけ」という内容を 1 ファイルにまとめたものです。

署名・暗号・圧縮・位置独立は持っていません。セグメントは絶対
番地を持ちます（ホストがその番地へ直接書くため）。番地はリンク時に解決されるので、
リロケーション表もありません。

## 全体の形

すべてビッグエンディアンです（チップに合わせています）。

```
+---------------------------+  0x00
| ヘッダ 64 B               |
+---------------------------+  0x40
| ターゲット表 64 B × n     |  どの石に入れられるか
+---------------------------+
| セグメント表 32 B × m     |  どのバイト列をどこへ
+---------------------------+
| 文字列表                  |  NUL 区切りの UTF-8
+---------------------------+  （4 バイト境界に揃える）
| ペイロード                |  セグメントの中身が順に
+---------------------------+
```

## struct

そのまま使える形にしてあります。サンプルの実装は
[`../sample/frc1.h`](../sample/frc1.h) にあります。
[`../tests/test_frc1.py`](../tests/test_frc1.py) が C の読み手と Python の
書き手を突き合わせるので、ここの定義がずれるとテストが失敗します。

どの欄もビッグエンディアンです。リトルエンディアンのホストでは、
`uint32_t` として直に読まず、下の `be32()` を通してください。
`_Static_assert` は、詰め物が入って大きさが狂っていないかを検査します。

```c
#include <stdint.h>
#include <string.h>

#define FRC1_MAGIC          "FRC1"
#define FRC1_VERSION        1u
#define FRC1_HEADER_BYTES   64u
#define FRC1_TARGET_BYTES   64u
#define FRC1_SEGMENT_BYTES  32u
#define FRC1_MAX_TARGETS    32u     /* target_mask が 32 bit だから */

/* ビッグエンディアンの 32 bit。整列も仮定しない */
static uint32_t be32(const void *p)
{
    const uint8_t *b = (const uint8_t *)p;
    return ((uint32_t)b[0] << 24) | ((uint32_t)b[1] << 16) |
           ((uint32_t)b[2] << 8) | (uint32_t)b[3];
}

/* ---- ヘッダ ---------------------------------------------------- 64 B */
struct frc1_header {
    char     magic[4];          /* +0x00  "FRC1"                        */
    uint32_t version;           /* +0x04  1                             */
    uint32_t header_bytes;      /* +0x08  64                            */
    uint32_t total_bytes;       /* +0x0c  ファイルの大きさと一致する    */
    uint32_t flags;             /* +0x10  0                             */
    uint32_t crc32;             /* +0x14  **この欄を 0 にした全体の crc32** */
    uint32_t target_offset;     /* +0x18                                */
    uint32_t target_count;      /* +0x1c                                */
    uint32_t segment_offset;    /* +0x20                                */
    uint32_t segment_count;     /* +0x24                                */
    uint32_t string_offset;     /* +0x28                                */
    uint32_t string_bytes;      /* +0x2c                                */
    uint32_t build_id;          /* +0x30  unix 秒。0 でよい             */
    uint32_t name;              /* +0x34  文字列表の中の位置            */
    uint32_t reserved[2];       /* +0x38  0                             */
};
_Static_assert(sizeof(struct frc1_header) == FRC1_HEADER_BYTES, "header");

/* ---- ターゲット ------------------------------------------------ 64 B
 *
 * 1 行 = 入れられる石 1 つと、ペイロードが知っていなければならない番地。
 * **これは機材の常駐ローダについての事実**で、こちらのコードの話ではない。
 * だからソースではなくコンテナに入っている。
 *
 * **ローダのバイト列は入っていない。** 潰す 8 バイトは指紋（`hook_crc32`）で
 * 照合するだけで、バイトそのものは `install` がチップから読む。`restore`
 * 用の控えは `<file>.frc.hook` に落ちる。無ければ抜き挿しで戻る ——
 * ローダは RAM にあるので、電源を入れ直せば元のバイトが flash から戻る。
 */
struct frc1_target {
    uint32_t usb_vendor;        /* +0x00                                */
    uint32_t usb_product;       /* +0x04                                */
    uint32_t hook;              /* +0x08  差し替える番地（0x000EA6FC）  */
    uint32_t hook_bytes;        /* +0x0c  8                             */
    uint32_t hook_crc32;        /* +0x10  元の 8 バイトの指紋           */
    uint32_t reserved0;         /* +0x14  0                             */
    uint32_t entry;             /* +0x18  hook の飛び先。セグメントの 1 つ */
    uint32_t resume_plain;      /* +0x1c  副命令 0 でローダへ返す先     */
    uint32_t resume_mask;       /* +0x20  副命令 1 でローダへ返す先     */
    uint32_t complete;          /* +0x24  制御転送を終わらせる          */
    uint32_t staging;           /* +0x28  64 バイトの OUT バッファ      */
    uint32_t state;             /* +0x2c  **0 で読めないと入れられない** */
    uint32_t loader_base;       /* +0x30  0x000E9000                    */
    uint32_t loader_bytes;      /* +0x34  0x3910                        */
    uint32_t loader_crc32;      /* +0x38  0 = 確かめない                */
    uint32_t name;              /* +0x3c  文字列表の中の位置            */
};
_Static_assert(sizeof(struct frc1_target) == FRC1_TARGET_BYTES, "target");

/* ---- セグメント ------------------------------------------------ 32 B */
struct frc1_segment {
    uint32_t address;           /* +0x00  石から見た番地                */
    uint32_t bytes;             /* +0x04                                */
    uint32_t file_offset;       /* +0x08  このファイルの中の位置        */
    uint32_t crc32;             /* +0x0c  そのバイト列の crc32          */
    uint32_t flags;             /* +0x10  FRC1_SEG_*                    */
    uint32_t target_mask;       /* +0x14  bit i = target[i] で使う      */
    uint32_t name;              /* +0x18  文字列表の中の位置            */
    uint32_t reserved;          /* +0x1c  0                             */
};
_Static_assert(sizeof(struct frc1_segment) == FRC1_SEGMENT_BYTES, "segment");

#define FRC1_SEG_CODE    1u    /* 命令（報告の見た目だけに効く）        */
#define FRC1_SEG_VERIFY  2u    /* 書いたあと読み直して比べる            */
#define FRC1_SEG_WINDOW  4u    /* 24 bit の窓では届かない番地           */
#define FRC1_SEG_ENTRY   8u    /* どれかのターゲットの入口を含む        */
```

文字列表は NUL 区切りの UTF-8 で、位置 0 は必ず空文字列です。
`name` が 0 なら名前はありません。

## 読み手が確かめること

サンプル（[`../sample/frc1_load.c`](../sample/frc1_load.c)）は、次の順で確かめます。
どれも、壊れたコンテナでチップに書き始めないための検査です。

| | |
|---|---|
| 1 | `magic` が `"FRC1"`、`version` が 1、`header_bytes` が 64 |
| 2 | `total_bytes` がファイルの大きさと一致する |
| 3 | `crc32` が、その欄を 0 に置き換えた全体の crc32 と一致する |
| 4 | 3 つの表と文字列表が、どれもファイルの中に収まっている |
| 5 | `target_count` が 1 以上 32 以下（マスクが 32 bit） |
| 6 | 各ターゲットの `hook_bytes` が 8 |
| 7 | 各セグメントの `file_offset + bytes` がファイルの中に収まっている |
| 8 | 各セグメントの `crc32` が中身と一致する |
| 9 | 同じターゲットで使うセグメントの番地が重なっていない |

## 入れる順番

この順番は、ホスト側から実機に入れたときの実測に基づいて決めています
（[`../sample/frc1_load.c`](../sample/frc1_load.c) が同じ順で入れます）。

1. チップの `state` が 0 で読めること。0 でなければ別のファームウェアが
   起動していて、`0x000E9000` の RAM には平文ローダではなく、起動したファームウェアが載って
   います
2. 走っているローダの crc32 が、ビルドしたときの相手と一致すること。
   同じ selene のローダでも 1 語だけ違うことがあります（`0x000E953C`）。ペイロードは
   その中の番地へ飛ぶので、違う相手に入れると違う命令へ飛びます。
   `loader_crc32` が 0 のコンテナはこの検査を飛ばします
   - 2b. 同じビルドが既に入っていれば、何も書きません。hook が既にこの
     コンテナの入口へ飛んでいて、コードのセグメント（`boot`・`main`）が
     ビルドしたとおりに読み戻せたら、そこで終わります（`.bss` は走ると変わるので
     比べません）。入れ直すときは `--force` を付けます。違うビルドなら 3〜5 を通ります
     （GV-M2TV で実測）
3. 元の 8 バイトを先に戻すこと。すでに hook が入っているときは、その
   飛び先のコードが、いまこの転送を処理しているコードそのものです。そこを上書きすると、
   EP0 が固まり、物理的に抜き差ししないと戻らなくなります（実測）
4. セグメントを書き、読み直して比べること
5. hook は最後に書くこと。半分だけ書けた状態で入口を向けないためです。hook の読み直し
   自体が `0xBC` の転送なので、その時点で新しいファームウェアを通ります。読み直しの
   答えが返ってくれば、ゲートが働いています

## `CFW` の載せ方

`python host/build.py frc1` を実行すると、機種ごとに 1 本、
`build/frc1-<id>.frc` ができます（いまは
`frc1-dt295.frc`・`frc1-m2tv.frc`）。ファイル自体がどの機器のものかを示すので、入れるときに別の機種の
hook を選ぶことはできません。形式としては 32 bit のターゲットマスクを持っていて、
`frcontainer.build` もリストを受け取るので、複数機種を 1 本に入れることもできます。
今のビルドではそうしていません。

セグメントは 3 つです（番地は `selene` のもの。`dione` は下の世代の表にあります）。

| 名前 | 番地 | 中身 | 理由 |
|---|---|---|---|
| `boot` | `0x000E1000` | `.boot`（selene 116 B = `0x74`、dione 134 B = `0x86`） | hook の stub は `ldi:20`（即値 20 bit）なので、飛び先は `0x100000` 未満でなければなりません。`0x000E1000` の枠は容量 `0x800` です |
| `main` | `0x00380000` | `.text` | ファームウェア本体。リンクは `0x80380000`（キャッシュ側のビュー）で行い、これは同じメモリです。ホストが書くのは素の番地です |
| `bss` | `0x003B0000` | 0 | これが無いと、static 変数が電源投入時の SRAM の値で始まります。この経路は宣言したバイトしか書かないので、`.bss` を明示的な 0 のセグメントにしています |

`boot` の番地は、機種のファイルの `loader.boot` から来ます
（[`../config/README.md`](../config/README.md)）。置き場そのものを決めるのは
リンカスクリプト（[`../src/main/link_selene.ld`](../src/main/link_selene.ld)・
[`../src/main/link_dione.ld`](../src/main/link_dione.ld)）の `BOOT_BASE` で、`build.py` が
両方を突き合わせ、食い違えばビルドを止めます。イメージは機種ごとにその行でリンクしますが、
リンカスクリプトは世代ごとに 1 本なので、同じ世代の機種は同じ `BOOT_BASE` を使います。
違う番地が要る機種には、別のリンカスクリプトが要ります。

入口は `frc1_entry`（`0x000E1000`）です。処理の中身は
[`../src/main/entry_frc1_selene.S`](../src/main/entry_frc1_selene.S) にあります。
副命令・長さ・番地のゲートを通してから `frq1_dispatch` を呼び、
答えたら `complete` へ、こちらの枠でなければローダの元の経路へ返します。

番地は stub に焼き込んであります。ただし、ローダの世代によって値も入口も違います。

| 世代 | 機種 | hook | 記述子 | `staging` | body |
|---|---|---|---|---|---|
| `selene` | MB8AC031（DT295） | `0x000EA6FC` | `r11`、番地 +0x00・副命令 +0x46 | `0x000C4620` | `0x80380000` |
| `dione` | MB8AC018（GV-M2TV） | `0x000E9F80` | `r10`、番地 +0x44・副命令 +0x40 | `0x000C5000` | `0x000E1800`（内部 RAM） |

`build.py` は機種の `loader.family` で `entry_frc1_selene.S` / `entry_frc1_dione.S` と
`link_selene.ld` / `link_dione.ld` を選びます。焼き込んだ番地と表が食い違う
コンテナはビルドを拒むので、気づかないうちにずれることはありません。dione は
`resume_mask` も自分で持ちます（ローダの read-modify-write 経路へ返すため）。
dione の body が `0x80380000` でないのは、ローダ状態では外部メモリー
コントローラが落ちていて、窓からも DRAM に届かないためです（実機で確認）。

ターゲットになるのは、`loader` が `null` でない機種です
（[`../config/devices/`](../config/devices)）。対象は、ローダの hook と 8 バイトを
確かめた 2 機種です。hook の番地と置き換える 8 バイトが合っていないと
別のものを書き潰すので、推定では増やしていません。値は機種ごとに持っています。
同じ世代の機器どうしで `loader` の値が一致しても、それは測った結果が一致したということで、
世代として決まっているわけではありません。
値をまとめると、測っていない機種が黙って値を引き継ぐことになります。
GV-M2TV は世代が違うので、実際に全部の値が違います。USB ID は機種の行から取るので、
ターゲットと機種で食い違うことはありません。

## 2 つの入口

1 つのファームウェアに入口が 2 つあります。

| 経路 | 入口 | 入り方 | 実機 |
|---|---|---|---|
| `FRC1`（平文） | `frc1_entry` `0x000E1000` | 常駐ローダの hook から入ります。selene は `r11` に記述子、dione は `r10` に記述子・`r11` に setup packet | 通っている（GV-M2TV・PIX-DT295） |
| 常駐 | `_start` `0x80380000` | servicer が `call` します。r4 = 枠 | 入口だけ |

どちらも同じ `frq1_dispatch` に入ります（[`frq1.md`](frq1.md)）。

## 確かめた点と未確認の点

* GV-M2TV（`frc1-m2tv.frc`）と PIX-DT295（`frc1-dt295.frc`）は、冷起動から入り、
  IDENTIFY がその行の USB ID を返します（実測）。
* 中のイメージは、その機種の行でリンクしてあります。FRC1 の経路では
  身元の語（`0x017A3D08`）が雑音なので、ファームウェアが走りながら機種を
  調べることはしません。コンテナが機種ごとなので、機種はビルドで決まります
  （[`devices.md`](devices.md)）。どの行のイメージかは、IDENTIFY の `+0x08` で
  ホストから見えます
* ゲートの定数は、ローダを確かめた機種の行だけが持ちます。ほかの機種で同じ値かどうかは
  未確認なので、行を足すときは `loader` を `null` にします
