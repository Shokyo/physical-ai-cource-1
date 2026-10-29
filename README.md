# GP8 Sort — YOLO-World × MoveIt 2 で安川 GP8 が物体を仕分ける

東大 松尾研究室の Physical AI 講座 2026 応用編1 最終課題「AI×ロボティクスによる創造的タスクの実現」の提出物です．

安川電機の産業用ロボット **MOTOMAN-GP8** を Gazebo 上に構築し，天井カメラの画像を
**YOLO（シミュレータの真値から自動生成したデータで fine-tune）** で認識して，机上の物体を
種類ごとにトレイへ仕分けます．動作計画は **MoveIt 2**，制御は **ros2_control** です．

```
 overhead camera ──image──▶ yolo_detector (YOLO-World) ──/detections (JSON)──▶ pick_place
                                                                                  │ MoveIt 2 (pymoveit2)
                                                                                  ▼
                                              Gazebo ◀── ros2_control ◀── gp8_arm_controller
                                                ▲
                                                └── /gp8/attach|detach/<obj>  (真空グリッパを模擬)
```

## 環境

| 項目 | 内容 |
|---|---|
| OS / ROS | Ubuntu 24.04, ROS 2 Kilted |
| シミュレータ | Gazebo (gz-sim Ionic, ros_gz) |
| 動作計画 | MoveIt 2 (OMPL / Pilz), ros2_control + gz_ros2_control |
| AI | Ultralytics YOLO11n を自動ラベル付けデータで fine-tune（YOLO-World のゼロショットも選択可） |
| ロボットモデル | [Yaskawa-Global/motoman_ros2_support_packages](https://github.com/Yaskawa-Global/motoman_ros2_support_packages) の GP8 (BSD-3-Clause) |

## パッケージ構成

| ディレクトリ | 内容 |
|---|---|
| `src/yaskawa/` | 安川公式の GP8 URDF/メッシュ（vendor，無改変） |
| `src/gp8_sort_description/` | GP8 + 真空グリッパの URDF．慣性の追加，ros2_control，Gazebo プラグイン（DetachableJoint） |
| `src/gp8_sort_moveit_config/` | 手書きの MoveIt 2 設定（SRDF，kinematics，joint_limits，controllers）と launch |
| `src/gp8_sort_gazebo/` | Gazebo ワールド（机・トレイ・物体・天井カメラ）とシミュレーション一括起動 launch |
| `src/gp8_sort_ai/` | `yolo_detector`（検出ノード），`pick_place`（仕分けタスク），`gen_dataset`（自動ラベル付けデータ生成），`scene_tool`（シーンのリセット/ランダム化），`record_topics`・`image_view`（動画用） |
| `src/pymoveit2/` | MoveIt 2 の Python クライアント（[AndrejOrsula/pymoveit2](https://github.com/AndrejOrsula/pymoveit2), BSD-3-Clause） |
| `ai/` | YOLO 用 Python 環境の定義，学習スクリプト，学習済み重み |
| `docs/` | レポート（`report.md` / `report.pdf`），fine-tune 手順（`finetune.md`） |
| `scripts/` | デモ起動・停止・録画・動画合成スクリプト |
| `media/` | 提出動画（MP4 は容量の都合でリポジトリに含めず，YouTube で公開） |

## セットアップ

```bash
# ROS 2 Kilted + 依存パッケージ
sudo apt install -y ros-kilted-moveit ros-kilted-moveit-configs-utils \
  ros-kilted-moveit-planners-ompl ros-kilted-pilz-industrial-motion-planner \
  ros-kilted-ros2-control ros-kilted-ros2-controllers ros-kilted-gz-ros2-control \
  ros-kilted-ros-gz ros-kilted-joint-state-publisher-gui python3-colcon-common-extensions

# ワークスペース
git clone <this repo> ~/pai_final_ws
cd ~/pai_final_ws
source /opt/ros/kilted/setup.bash
colcon build --symlink-install

# YOLO 用 Python 環境（uv を使用．pip でも可）
cd ai && uv venv --python /usr/bin/python3 .venv && \
  VIRTUAL_ENV=$PWD/.venv uv pip install -r requirements.txt
```

学習済み重み `ai/models/gp8_sort_yolo.pt` はリポジトリに含めています．自分で学習し直す場合は
「データセット生成と学習」を参照してください．

## 実行

ターミナル 1: Gazebo + ros2_control + MoveIt + RViz

```bash
source ~/pai_final_ws/install/setup.bash
ros2 launch gp8_sort_gazebo sim.launch.py
```

ターミナル 2: YOLO 検出ノード（venv の Python で起動）

```bash
source ~/pai_final_ws/install/setup.bash
~/pai_final_ws/ai/.venv/bin/python -m gp8_sort_ai.yolo_detector --ros-args -p use_sim_time:=true
```

ターミナル 3: 仕分けタスク

```bash
source ~/pai_final_ws/install/setup.bash
ros2 run gp8_sort_ai pick_place --ros-args -p use_sim_time:=true
```

シーンをリセット / ランダム配置する（デモ前に）:

```bash
~/pai_final_ws/ai/.venv/bin/python -m gp8_sort_ai.scene_tool reset
~/pai_final_ws/ai/.venv/bin/python -m gp8_sort_ai.scene_tool random
```

### デモ一括実行と動画の作成

```bash
scripts/start_demo.sh      # Gazebo + MoveIt + RViz + 検出ノード + 検出画像ビューアを起動し，ウィンドウを整列
scripts/run_demo.sh 3      # ランダム配置 → 仕分け，を 3 回
scripts/stop_all.sh        # 全部停止
```

提出動画は画面録画ではなく，Gazebo 内の撮影用カメラ（`scene_camera`）と検出画像のトピックを
`gp8_sort_ai.record_topics` で MP4 に保存し，`scripts/compose_video.sh` で左右に並べて作成しています．

```bash
~/pai_final_ws/ai/.venv/bin/python -m gp8_sort_ai.record_topics --out media/rec /scene_camera/image /detections/image &
scripts/run_demo.sh 3
kill %1
scripts/compose_video.sh media/rec media/PAI_final_demo.mp4
```

Gazebo なしで MoveIt だけ試す場合（mock hardware）:

```bash
ros2 launch gp8_sort_moveit_config demo_mock.launch.py
```

## データセット生成と学習

シミュレーション起動中に以下を実行すると，物体をランダム配置した画像と，既知の形状・姿勢を
カメラモデルで投影して得た bounding box（自動アノテーション）から YOLO 形式のデータセットが
生成されます．続けて YOLO11n を fine-tune します（RTX 3060 Laptop で数分）．

```bash
~/pai_final_ws/ai/.venv/bin/python -m gp8_sort_ai.gen_dataset --out ~/pai_final_ws/ai/dataset --train 240 --val 60
~/pai_final_ws/ai/train_yolo.sh ~/pai_final_ws/ai/dataset 40
```

## 仕組み

1. **認識**: 天井カメラ画像を YOLO に入力し，3 クラス（`red cube`, `green cube`, `blue cylinder`）を
   検出する．`task.yaml` の `detector.weights` を `yolov8s-worldv2.pt` にすると，学習なしの
   YOLO-World（テキストプロンプト）に切り替えられる（単純形状では精度が低い）．
2. **座標変換**: カメラは真下を向いた固定カメラなので，検出中心の画素を机平面（高さ既知）に
   逆投影して物体のワールド座標を得る．
3. **動作計画**: 物体上空 → 直線降下 → 吸着 → 持ち上げ → トレイ上空 → 直線降下 → 解放，を
   MoveIt 2（自由空間は OMPL，直線は Cartesian path）で計画・実行する．机とトレイは
   衝突オブジェクトとして登録する．
4. **把持**: Gazebo の `DetachableJoint` プラグインで吸着を模擬し，吸着パッド先端が物体に触れた
   時点で物体をリンクに固定する．

## ライセンス

このリポジトリの自作部分は Apache-2.0．`src/yaskawa` は安川電機の BSD-3-Clause，
`src/pymoveit2` は BSD-3-Clause に従います．
