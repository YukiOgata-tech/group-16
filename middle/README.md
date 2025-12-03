# line-tracer-check

Raspberry Pi + MCP3004(4ch) + DRV8835(2chモータ) でフォトリフレクタ4個の信号からモータを制御する最小実装。

中間発表用の楕円コース走行プログラムです。

## ハードウェア構成

### 部品リスト
- Raspberry Pi Zero W
- MCP3004 (10bit 4ch A/Dコンバータ)
- DRV8835 (2chモータドライバ)
- LBR-127HLD フォトリフレクタ × 4個
- FA-130RA DCモータ × 2個（ツインモーターギアボックス）
- 単3 NiMH電池 × 4本（RPi: 3.6V, モータ: 2.4V）

### ピン配線

#### モータ（DRV8835, IN/INモード）
- 左モータ: BCM 6, 5
- 右モータ: BCM 26, 27

#### センサー（MCP3004, SPI0経由）
- SPI0 CE0: BCM 8
- SPI0 MOSI: BCM 10
- SPI0 MISO: BCM 9
- SPI0 SCLK: BCM 11

#### フォトリフレクタのチャンネル割り当て
- CH0: 左端
- CH1: 左中
- CH2: 右中
- CH3: 右端

## セットアップ

### 1. 必要なパッケージのインストール

```bash
sudo apt-get update
sudo apt-get install python3-gpiozero python3-numpy
```

または pip を使う場合:

```bash
pip3 install -r requirements.txt
```

### 2. SPIの有効化

Raspberry Pi の SPI インターフェースを有効にする必要があります：

```bash
sudo raspi-config
```

- `Interface Options` を選択
- `SPI` を選択
- `Yes` で有効化
- 再起動

確認コマンド:
```bash
ls /dev/spi*
# /dev/spidev0.0 /dev/spidev0.1 が表示されればOK
```

### 3. プログラムのコピー

このフォルダ全体を Raspberry Pi にコピーします：

```bash
# ローカルPC から
scp -r middle/ pi@raspberrypi.local:~/

# または GitHub経由で
cd ~/EicDesignLab
git pull
```

## 使用方法

### 手動実行

```bash
cd ~/middle
python3 line_follower_main.py
```

停止する場合は `Ctrl+C` を押してください。

### テスト用スクリプト

動作確認用のスクリプトが `scripts/` フォルダにあります：

```bash
# センサー値の確認（モータは動かない）
python3 scripts/00_adc_watch.py

# モータのテスト動作
python3 scripts/01_motors_watch.py

# シンプルなラインフォロワー
python3 scripts/10_line_follower.py
```

### シミュレーション（実機なしでテスト）

ハードウェアがまだ完成していない場合、シミュレーターで制御アルゴリズムをテストできます：

```bash
# 依存パッケージのインストール（初回のみ）
pip3 install pygame scipy

# シミュレーターの起動
cd ~/middle
python3 simulator.py
```

**操作方法:**
1. マウスドラッグで車体を配置
2. マウスクリックで車体の向きを設定
3. クリックでシミュレーション開始
4. ESC で停止、q で終了

**注意:**
- シミュレーターは実機の物理特性を完全には再現しません
- 実機では微調整が必要な場合があります
- `config.py` で設定したパラメータがそのまま使用されます

### 自動起動設定

起動時に自動でラインフォロワーを開始する場合は、systemd サービスを使用します：

```bash
# サービスファイルをコピー
sudo cp line_follower.service /etc/systemd/system/

# サービスを有効化
sudo systemctl enable line_follower.service

# サービスを開始（すぐに起動）
sudo systemctl start line_follower.service

# 状態確認
sudo systemctl status line_follower.service

# 自動起動を無効化する場合
sudo systemctl disable line_follower.service
```

## パラメータ調整

`config.py` で各種パラメータを調整できます：

### 速度設定
```python
BASE_SPEED = 0.25    # 基本速度（0-1）
MIN_SPEED = 0.12     # 最低速度
CLAMP = 0.70         # 最大出力制限
```

### ステアリング
```python
K_STEER = 0.40       # ステアリング感度（大きいほど急旋回）
POSITION_WEIGHTS = [-1.5, -0.5, 0.5, 1.5]  # センサー位置の重み
```

### センサーフィルタ
```python
NORM_ALPHA = 0.02    # 正規化の学習率
LPF_ALPHA = 0.3      # ローパスフィルタ係数
READ_HZ = 50         # 読み取り周波数
```

## ファイル構成

```
middle/
├── line_follower_main.py        # メイン実行プログラム（中間発表用）
├── simulator.py                 # シミュレーター（実機なしテスト用）
├── config.py                    # 設定パラメータ
├── sensors.py                   # センサー制御（PhotoArray クラス）
├── motors.py                    # モータ制御（MotorDriver クラス）
├── controller.py                # 制御アルゴリズム
├── sensor_to_motors_basic.py   # 最もシンプルな実装例
├── generate_course_image.py    # 楕円コース画像生成スクリプト
├── ellipse_course.png           # 楕円コース画像（シミュレーター用）
├── line_follower.service        # systemd 自動起動設定
├── requirements.txt             # Python依存パッケージ
├── README.md                    # このファイル
└── scripts/                    # テスト・デバッグ用スクリプト
    ├── 00_adc_watch.py         # センサー値モニタ
    ├── 01_motors_watch.py      # モータテスト
    └── 10_line_follower.py     # シンプルなラインフォロワー
```

## トラブルシューティング

### SPI エラーが出る場合

```bash
# SPI が有効か確認
ls /dev/spi*

# 無ければ raspi-config で有効化
sudo raspi-config
```

### モータが動かない場合

1. 電池電圧を確認（3V以上必要）
2. モータドライバの配線を確認
3. テストスクリプトで動作確認：
   ```bash
   python3 scripts/01_motors_watch.py
   ```

### センサーが反応しない場合

1. センサー値を確認：
   ```bash
   python3 scripts/00_adc_watch.py
   ```
2. 白い面と黒い面で値が変化するか確認
3. 必要に応じて `sensor_to_motors_basic.py` の `WHITE_IS_HIGH` と `THRESH` を調整

### ラインを外れてしまう場合

`config.py` で以下を調整：

- `BASE_SPEED` を下げる（より遅く）
- `K_STEER` を下げる（マイルドな旋回）
- `POSITION_WEIGHTS` を調整（センサー位置の重み）

## 制御アルゴリズム

このプログラムは重み付き位置推定による比例制御を使用しています：

1. 4つのセンサーから黒線の強さ（0-1）を取得
2. 位置重み（-1.5, -0.5, 0.5, 1.5）との内積で現在位置を推定
3. 推定位置に比例したステアリング量を計算
4. 左右モータの速度差でステアリングを実現

詳細は `controller.py` と `sensors.py` を参照してください。

## 開発者向け情報

### モジュール構成

- **sensors.py**: `PhotoArray` クラス
  - センサー値の読み取り
  - 自動正規化（白/黒の自動キャリブレーション）
  - ローパスフィルタ

- **motors.py**: `MotorDriver` クラス
  - gpiozero の Robot クラスのラッパー
  - 安全なモータ制御

- **controller.py**: 制御関数
  - `compute_steer()`: 位置推定
  - `mixer()`: 差動ステアリング

### テスト

gpiozero のモックピンファクトリを使えば、実機なしでテストできます：

```bash
export GPIOZERO_PIN_FACTORY=mock
python3 -m unittest discover tests
```

（※ 現時点では tests/ フォルダは未実装）

## ライセンス

新潟大学工学部 電子情報通信設計製図 講義教材

## 参考資料

- [EicDesignLab リポジトリ](https://github.com/msiplab/EicDesignLab)
- [gpiozero ドキュメント](https://gpiozero.readthedocs.io/)
- MCP3004 データシート
- DRV8835 データシート
