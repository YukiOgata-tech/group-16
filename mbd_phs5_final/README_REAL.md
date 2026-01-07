# mbd_phs5 実機用プログラム

## 概要

mbd_phs5 のシミュレーターで検証した制御アルゴリズムを、Raspberry Pi 実機用に移植したプログラムです。

### 制御方式

**固定行列マッピング**
- パラメータ調整不要
- シンプルで安定した動作
- 交差点検出機能搭載
- ラインロスト時の探索動作を追加

## チューニング（本番コース向け）

走行速度やセンサーしきい値は `tuning.py` で一括調整できます。

- 速度: `BASE_SPEED`, `MIN_BASE_SPEED`, `SPEED_SCALE`
- 旋回感度: `OUTER_GAIN`, `INNER_GAIN`, `TURN_SLOWDOWN`
- 交差点/ロスト: `BLACK_THRESHOLD`, `CROSS_BLACK_COUNT`, `LOST_BLACK_THRESHOLD`

### ファイル構成

```
mbd_phs5/
├── real_line_follower.py    # 実機用メインプログラム ★
├── test_sensors.py           # センサー動作確認
├── test_motors.py            # モーター動作確認
├── README_REAL.md            # このファイル
│
# 以下はシミュレーター用（実機では不使用）
├── main_mils_line_follower.py
├── mils_line_follower_ctrl.py
├── mils_line_follower_body.py
└── mils_line_follower_phrf.py
```

---

## ハードウェア要件

### 必須コンポーネント

| 部品 | 型番 | 数量 | 接続 |
|------|------|------|------|
| マイコン | Raspberry Pi Zero W | 1 | - |
| ADC | MCP3004 (10bit 4ch) | 1 | SPI0 |
| モータードライバー | DRV8835 (2ch) | 1 | GPIO 5,6,26,27 |
| フォトリフレクタ | LBR-127HLD | 4 | MCP3004 CH0-3 |
| DCモーター | FA-130RA | 2 | DRV8835 |
| 電源 | 単3 NiMH電池 | 4 | - |

### ピン配置

#### モータードライバー（DRV8835）

| GPIO | 機能 | モーター |
|------|------|----------|
| 6 | AIN1 | 左モーター正転 |
| 5 | AIN2 | 左モーター逆転 |
| 26 | BIN1 | 右モーター正転 |
| 27 | BIN2 | 右モーター逆転 |

#### ADC（MCP3004）

| チャンネル | センサー位置 |
|-----------|-------------|
| CH0 | 左端 |
| CH1 | 中左 |
| CH2 | 中右 |
| CH3 | 右端 |

---

## セットアップ

### 1. 必要なライブラリのインストール

```bash
# gpiozero と numpy をインストール
sudo apt-get update
sudo apt-get install python3-gpiozero python3-numpy

# または pip 経由
pip3 install gpiozero numpy
```

### 2. SPI の有効化

MCP3004 を使用するには SPI を有効化する必要があります。

```bash
# raspi-config を起動
sudo raspi-config

# Interface Options → SPI → Yes を選択
# 再起動
sudo reboot
```

### 3. 配線確認

test_sensors.py と test_motors.py でハードウェアの動作を確認してください。

---

## 使い方

### センサー動作確認

```bash
python3 test_sensors.py
```

**確認項目**:
- [ ] 白い紙の上で 0.8〜1.0 の値
- [ ] 黒い線の上で 0.0〜0.3 の値
- [ ] 4つすべてのセンサーが反応

**出力例**:
```
センサー値: CH0:0.92  CH1:0.88  CH2:0.15  CH3:0.90  | ████████████████████ ██████████████████░░ ███░░░░░░░░░░░░░░░░░ ████████████████████
```

### モーター動作確認

```bash
python3 test_motors.py
```

**確認項目**:
- [ ] 前進で両輪が前に回る
- [ ] 左旋回で右に曲がる
- [ ] 右旋回で左に曲がる
- [ ] 速度変化が機能する

### ライントレース実行

```bash
python3 real_line_follower.py
```

**停止方法**:
- `Ctrl+C` を押す

---

## 制御アルゴリズム詳細

### 行列マッピング

```python
mat_A = [
    [-1.0, -0.2, 0.2, 1.0],   # 左モーター
    [ 1.0,  0.2, -0.2, -1.0],  # 右モーター
]

モーター指令 = mat_A × センサー値 + 0.2
```

### センサー値とモーター動作の関係

| センサー状態 | 検出 | 左モーター | 右モーター | 動作 |
|-------------|------|-----------|-----------|------|
| [白, 黒, 黒, 白] | 中央 | 0.2 | 0.2 | 直進 |
| [黒, 黒, 白, 白] | 左寄り | -0.6 | 0.6 | 左旋回 |
| [白, 白, 黒, 黒] | 右寄り | 0.6 | -0.6 | 右旋回 |
| [黒, 黒, 黒, 白] | 交差点 | 0.35 | 0.35 | 直進 |

### 交差点検出

```python
# 条件: 3本以上黒 & 中央2本が黒
if num_black >= 3 and is_black[1] and is_black[2]:
    return (0.35, 0.35)  # 直進
```

---

## トラブルシューティング

### 問題1: モーターが動かない

**確認項目**:
- [ ] 電源電圧が十分か（3.0V以上）
- [ ] DRV8835 の配線が正しいか
- [ ] `test_motors.py` で動作確認したか

**対策**:
1. 電池を新品に交換
2. 配線を確認
3. GPIO番号が正しいか確認

### 問題2: センサー値が反応しない

**確認項目**:
- [ ] SPI が有効化されているか
- [ ] MCP3004 の配線が正しいか
- [ ] `test_sensors.py` で動作確認したか

**対策**:
```bash
# SPI 有効化確認
ls /dev/spi*
# /dev/spidev0.0 が表示されるはず

# 再度 SPI を有効化
sudo raspi-config
```

### 問題3: ラインから外れる

**症状別対策**:

#### 外側に外れる
```python
# real_line_follower.py の行列係数を変更
mat_A = np.array([
    [-1.2, -0.3, 0.3, 1.2],  # 係数を大きく
    [1.2, 0.3, -0.3, -1.2],
])
```

#### 内側に入り込む
```python
# 係数を小さく
mat_A = np.array([
    [-0.8, -0.1, 0.1, 0.8],
    [0.8, 0.1, -0.1, -0.8],
])
```

#### 速度が速すぎる
```python
# オフセットを下げる
vec_mtrs = np.dot(mat_A, vec_prs) + 0.15  # 0.2 → 0.15
```

#### 速度が遅すぎる
```python
# オフセットを上げる
vec_mtrs = np.dot(mat_A, vec_prs) + 0.3  # 0.2 → 0.3
```

### 問題4: 交差点で暴走する

**対策**:
```python
# 交差点判定の閾値を変更
if num_black >= 2 and is_black[1] and is_black[2]:  # 3 → 2
```

### 問題5: 左右どちらかに偏る

**確認項目**:
- [ ] モーターの左右が逆接続されていないか
- [ ] センサーの配置が CH0(左) → CH3(右) の順か

**対策**:
```python
# モーターの左右を入れ替え
motors = Robot(left=(PIN_BIN1, PIN_BIN2), right=(PIN_AIN1, PIN_AIN2))
```

---

## パラメータ調整ガイド

### 基本的な調整手順

1. **センサー配置を確認** - CH0(左端)〜CH3(右端)
2. **test_sensors.py** でセンサー値確認
3. **test_motors.py** でモーター動作確認
4. **real_line_follower.py** で実走行
5. 挙動に応じて調整

### 調整可能なパラメータ

#### 1. 行列係数（旋回感度）

```python
# real_line_follower.py:72-76
mat_A = np.array([
    [-1.0, -0.2, 0.2, 1.0],   # 外側の値を大きく → 旋回強
    [1.0, 0.2, -0.2, -1.0],   # 内側の値を大きく → 応答速
])
```

| パラメータ | 効果 | 推奨範囲 |
|-----------|------|----------|
| 外側係数 (±1.0) | 旋回力 | 0.8〜1.2 |
| 内側係数 (±0.2) | 微調整力 | 0.1〜0.4 |

#### 2. 速度オフセット

```python
# real_line_follower.py:78
vec_mtrs = np.dot(mat_A, vec_prs) + 0.2  # 基本速度
```

| 値 | 速度 | 用途 |
|----|------|------|
| 0.1 | 遅い | 複雑なコース |
| 0.2 | 標準 | デフォルト |
| 0.3 | 速い | 簡単なコース |

#### 3. 交差点判定閾値

```python
# real_line_follower.py:70
if num_black >= 3 and is_black[1] and is_black[2]:
```

| 閾値 | 感度 | 特徴 |
|------|------|------|
| 2 | 高感度 | 誤検知しやすい |
| 3 | 標準 | バランス良好 |
| 4 | 低感度 | 検出漏れ |

---

## デバッグモード

### センサー値とモーター指令の表示

```python
# real_line_follower.py:93-94 のコメントを解除

# デバッグ出力（必要に応じてコメント解除）
print(f"Motors: L={left:+.2f}, R={right:+.2f}")
```

**出力例**:
```
Motors: L=+0.20, R=+0.20
Motors: L=-0.15, R=+0.55
Motors: L=+0.60, R=-0.10
```

---

## シミュレーターとの違い

| 項目 | シミュレーター | 実機 |
|------|---------------|------|
| センサー | pygame 画像読取 | MCP3004 ADC |
| モーター | 物理シミュレーション | gpiozero Robot |
| 制御ロジック | mils_line_follower_ctrl.py | real_line_follower.py |
| UI | pygameウィンドウ | なし |
| パラメータ調整 | 不要 | 微調整が必要な場合あり |

---

## 性能比較

| 制御方式 | 完走確率 | 速度 | 調整難易度 | 推奨度 |
|---------|---------|------|-----------|--------|
| **mbd_phs5** | 70〜80% | 中 | 低 | ★★★★★ |
| mbd_phs4 (PD) | 10〜30% | 高 | 高 | ★★ |
| eiclab_advanced | 20〜40% | 中 | 中 | ★★★ |

---

## ライセンス

MIT License

Copyright (c) 2019-2025 Shogo MURAMATSU / EicDesignLab contributors

---

## 参考資料

- [gpiozero Documentation](https://gpiozero.readthedocs.io/)
- [プロジェクトWiki](https://github.com/msiplab/EicDesignLab/wiki)
- MCP3004 データシート
- DRV8835 データシート
