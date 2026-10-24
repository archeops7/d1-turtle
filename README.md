# TurtleBot3 Visual Navigation with LiquidAI/d1-3B

Gazebo 上の TurtleBot3 Waffle Pi が、車載カメラの画像をローカルのマルチモーダル意思決定モデル [LiquidAI/d1-3B](https://huggingface.co/LiquidAI/d1-3B) に渡し、椅子の正面で停止するまでの行動を選択するデモです。

## タスクと設計意図

単純な距離センサや色抽出だけでは解けないタスクとして、以下の要素を持つ部屋を構成しています。

- **木目の椅子（目標）と木の箱（ダミー）**: 同じ材質・同程度の幅を持ち、形状（背もたれと脚の有無）から「座れるもの」を視覚的に見分ける必要があります。
- **床の低い小物**: 高さ 0.04 m の円柱で、2D LiDAR（走査高さ約 0.12 m）の下をすり抜けるためカメラ画像でのみ検知できます。

### 1 回の推論で評価する 4 つの質問

1. `maneuver` (`choice`): `forward` / `turn_left` / `turn_right` / `stop` / `search` の選択と確信度
2. `facing_sittable` (`noul`): 箱ではなく椅子の正面に近づいている確率
3. `crush_risk` (`noul`): そのまま前進すると床の小物を踏む確率
4. `progress` (`score`): 椅子の正面での停止にどれだけ近づいたか（0〜3 の 4 段階）

## 動作環境と起動手順

RTX等を搭載した Windows マシン上の WSL2（Ubuntu）+ WSLg を対象としています。

### 1. Docker と NVIDIA Container Toolkit の準備（未導入の場合）

```bash
curl -fsSL https://get.docker.com | sudo sh
sudo usermod -aG docker "$USER"
# 再ログイン後
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey \
  | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
distribution=$(. /etc/os-release; echo $ID$VERSION_ID)
curl -fsSL https://nvidia.github.io/libnvidia-container/$distribution/libnvidia-container.list \
  | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' \
  | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
sudo apt-get update && sudo apt-get install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo service docker restart
```

### 2. ビルドと実行

```bash
xhost +local:root
docker compose up --build
```

- 初回起動時に `LiquidAI/d1-3B`（BF16、約 6 GB）を Docker volume `hf-cache` にダウンロードします。
- Gazebo の GUI ウィンドウにロボットの動きが表示され、ターミナルに `label=... cmd=... sittable=... crush=... progress=... confidence=... range=...` の推論結果が出力されます。
- 終了するときは `Ctrl+C` の後に `docker compose down`（モデルのキャッシュも削除する場合は `docker compose down -v`）を実行してください。

## ファイル構成

| ファイル | 役割 |
| --- | --- |
| `app/d1_driver.py` | カメラ画像と LiDAR を購読し、d1-3B の推論結果から `/cmd_vel` を出力する ROS 2 ノード |
| `app/worlds/*.world` | 椅子・木箱・床の小物・机・俯瞰カメラを備えた Gazebo ワールド |
| `app/bringup.launch.py` | Gazebo サーバ/クライアント、状態配信、初期姿勢付きスポーン、ドライバノードの一括起動 |
| `Dockerfile` | ROS 2 Humble Desktop Full + TurtleBot3 Gazebo + CUDA 版 PyTorch 環境の定義 |
| `docker-compose.yml` | GPU・WSLg・WSL ドライバ・モデルキャッシュ volume のマウント設定 |
| `entrypoint.sh` | WSL2 の CUDA ローダー設定と ROS 2 launch の起動スクリプト |
