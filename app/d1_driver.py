#!/usr/bin/env python3
"""ローカルのマルチモーダル意思決定モデル LiquidAI/d1-3B を用いた TurtleBot3 視覚ナビゲーションノード。

車載の前面カメラ（/camera/image_raw）の画像と直近の行動履歴を d1-3B に渡し、4つの質問を同時に評価する。
モデルの回答（確率分布・期待スコア）を速度テーブル（MANEUVERS）と LiDAR 距離（/scan）に組み合わせて、
同じ木目・サイズの箱や小さい小物を回避しながら「座れる椅子」の正面で停止する。
"""

import sys

import numpy as np
import rclpy
import torch
from geometry_msgs.msg import Twist
from PIL import Image as PILImage
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan
from transformers import AutoModel

# モデルに状態（state）として渡すミッション記述。
# 座標やモデル名は与えず、視覚的な意味理解のみで判断させる。
GOAL = (
    "座れるものを探して正面で止まってください。座れないものもあるので、周囲をよく探索して確実に座るものだけを選択してください。箱などの前では止まらずに、他に座れるものがないか探してください。"
    "床に落ちているものは踏まないように、視覚の下方に何かある場合やそのまま前進するとぶつかりそうな場合は、回避できるように旋回して回り込んでください。"
    "家具の下の隙間には入らずに、代わりに方向転換して回り込むようにしてください。"
)

# 行動ラベルと速度の対応表。
# LLM に直接コードや連続値を生成させるのではなく、d1 には意味的な行動選択のみを行わせる。
MANEUVERS = {
    "forward": (0.08, 0.0),
    "turn_left": (0.0, 0.5),
    "turn_right": (0.0, -0.5),
    "stop": (0.0, 0.0),
    "search": (0.0, 0.3),
}

# d1-3B の system_one API に渡す質問定義。
# - choice : 各選択肢の確率分布・最尤選択肢（choice）・確信度（confidence）を返す
# - noul   : Yes/No の二値質問に対する肯定確率（0.0〜1.0）を返す
# - score  : 順序付き基準（0〜3）に対する確率重み付き期待スコア（0.0〜3.0）を返す
QUESTIONS = {
    "maneuver": {
        "type": "choice",
        "instructions": (
            "The image is the robot's forward camera. "
            "Which maneuver approaches the chair, which a person could sit on, "
            "and avoids the similar solid wooden box?"
        ),
        "criteria": {
            "forward": "The chair is near the center and the floor ahead is clear down to the bottom edge of the image",
            "turn_left": "The chair, or the open view toward it, is to the left",
            "turn_right": "The chair, or the open view toward it, is to the right",
            "stop": "A wooden box, a small floor object low in the frame or cut off at the bottom, or the gap under a table is centered",
            "search": "The chair is not visible",
        },
    },
    "facing_sittable": {
        "type": "noul",
        "instructions": (
            "Is the robot facing a chair close to the camera and near the center, "
            "rather than the solid wooden box?"
        ),
    },
    "crush_risk": {
        "type": "noul",
        "instructions": (
            "Would moving forward now hit a small object on the floor "
            "or enter the gap under furniture? "
            "An object low in the image, or cut off by the bottom edge, is already close enough to hit. "
            "A tiny speck near the horizon is still far."
        ),
    },
    "progress": {
        "type": "score",
        "instructions": "How close is the robot to stopping in front of the chair?",
        "criteria": [
            "The chair is not visible",
            "The chair is visible but far or off to the side",
            "The chair is near the center but still far",
            "The chair is close, large, and centered",
        ],
    },
}


def disable_fused_attention():
    """d1-3B のハイブリッド層で Flash Attention を無効化し、明示的な Attention 計算に切り替える。

    d1-3B の `hybrid.py` は幹（状態）と枝（各質問）の Attention 出力を log-sum-expで結合する。
    コンテナ内の PyTorch 2.6（cu124）環境では Flash Attention カーネルが返す LSE テンソルの末尾次元が
    枝のトークン長（例: 263）ではなくヘッド次元（128）になって形状不一致を起こすため、`_fused` 判定を常に False にする。
    """
    patched = False
    for module in sys.modules.values():
        path = getattr(module, "__file__", None) or ""
        if path.endswith("hybrid.py") and callable(getattr(module, "_fused", None)):
            module._fused = lambda q: False
            patched = True
    if not patched:
        raise RuntimeError("d1 hybrid.py did not load")


def load_model():
    """LiquidAI/d1-3B モデルを読み込み、推論用の設定を適用して返す。
    """
    device = "cuda" if torch.cuda.is_available() else "cpu"
    dtype = torch.float32 if device == "cpu" else torch.bfloat16
    print(f"loading LiquidAI/d1-3B on {device}", flush=True)
    model = AutoModel.from_pretrained(
        "LiquidAI/d1-3B", trust_remote_code=True, dtype=dtype
    ).to(device)
    model.eval()
    disable_fused_attention()
    print("d1-3B ready", flush=True)
    return model


def image_to_pil(msg):
    """ROS 2 の sensor_msgs/Image メッセージを PIL Imageに変換する。
    """
    channels = 3
    raw = np.frombuffer(msg.data, dtype=np.uint8)
    row = msg.width * channels
    if msg.step == row:
        frame = raw.reshape(msg.height, msg.width, channels)
    else:
        frame = raw.reshape(msg.height, msg.step)[:, :row].reshape(msg.height, msg.width, channels)
    if msg.encoding == "bgr8":
        frame = frame[:, :, ::-1]
    return PILImage.fromarray(frame.copy())


def front_distance(msg):
    """2D LiDAR（/scan）からロボット前方 ±30 度以内の最短距離 [m] を算出する。

    TurtleBot3 の LaserScan は 0 度が正面で反時計回りに 360 度格納されるため、
    配列の先頭（0〜+30 度）と末尾（-30〜0 度）を取り出して結合する。
    自身の車体への反射や無効値（0.12 m 以下、NaN、Inf）は除外する。
    高さ 0.12 m 未満の床の小物（floor_object）はこの走査面の下にあるため LiDAR には映らない。
    """
    ranges = np.asarray(msg.ranges, dtype=np.float32)
    width = max(1, int(ranges.size * 30 / 360))
    sector = np.concatenate((ranges[:width], ranges[-width:]))
    sector = sector[np.isfinite(sector) & (sector > 0.12)]
    if sector.size == 0:
        return 99.0
    return float(sector.min())


def decide(answers):
    """d1-3B の4つの回答から (並進速度, 旋回速度, 完了フラグ, 状態ラベル) を決定する。

    条件分岐順位:
    1. 椅子の正面に十分近づいた場合（facing_sittable > 0.75 かつ progress >= 2.5）はタスク完了として停止。
    2. 行動選択の確信度（confidence）が 0.4 未満と低い場合は、誤進を防ぐためその場で探索旋回。
    3. 床の小物への衝突リスク（crush_risk > 0.6）がある場合は、速度を低下して回避旋回。
    4. 上記以外は、d1 が選んだ maneuver ラベルに対応する固定速度をそのまま返す。
    """
    name = answers["maneuver"]["choice"]
    confidence = float(answers["maneuver"]["confidence"])
    facing = float(answers["facing_sittable"]["noul"])
    crush = float(answers["crush_risk"]["noul"])
    progress = float(answers["progress"]["score"])

    if facing > 0.75 and progress >= 2.5:
        return 0.0, 0.0, True, "done"
    if confidence < 0.4:
        return 0.0, 0.3, False, "uncertain_search"

    linear, angular = MANEUVERS.get(name, (0.0, 0.0))
    if crush > 0.6 and linear > 0.0:
        return 0.1, 0.4, False, "crush_avoid"
    return linear, angular, False, name


class Driver(Node):
    """カメラ画像と LiDAR を購読し、d1-3B の判断に基づいて /cmd_vel を配信する ROS 2 ノード。"""

    def __init__(self, model):
        super().__init__("d1_driver")
        self.model = model
        self.front = None          # 前面カメラの最新フレーム（PIL Image）
        self.front_range = 99.0    # 前方 ±30 度の LiDAR 最短距離 [m]
        self.history = []          # 直近に選択した行動ラベルの履歴（状態文脈として d1 に渡す）
        self.command = (0.0, 0.0)  # 現在保持している速度指令 (linear_x, angular_z)
        self.hold_until = self.get_clock().now()  # 現在の速度指令を保持する終了時刻
        self.done = False          # 椅子の正面への到着完了フラグ
        self.chair_flag = False # 接近中に椅子を視覚的に捕捉済みかどうかのフラグ
        self.announced_wait = False

        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Image, "/camera/image_raw", self.on_front, qos_profile_sensor_data)
        self.create_subscription(LaserScan, "/scan", self.on_scan, qos_profile_sensor_data)
        # 0.2 秒周期で制御ループを回し、推論で決定した行動は 0.4 秒間保持する
        self.create_timer(0.2, self.tick)

    def on_front(self, msg):
        self.front = image_to_pil(msg)

    def on_scan(self, msg):
        self.front_range = front_distance(msg)

    def _publish(self, linear, angular):
        """指定した並進速度・旋回速度を /cmd_vel トピックに送信する。"""
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.pub.publish(msg)

    def _query_d1(self):
        """d1-3B で現在の画像と状態を評価し、椅子を捉えていればフラグを更新する。

        椅子の直前まで近づくと、車載の近距離カメラの画角から椅子の全体像が外れてしまい、
        モデルが「椅子が見えない（search）」と判定してその場で回転し続けることがある。
        これを防ぐため、前方距離 1.5 m 以内で椅子らしさ（facing > 0.5 または progress >= 2.0）を
        一度でも確認したら `chair_flag = True` として記憶しておく。
        """
        result = self.model.system_one(
            {"mission": GOAL, "recent_actions": self.history[-3:]},
            QUESTIONS,
            images=[self.front],
        )
        answers = result["answers"]
        facing = float(answers["facing_sittable"]["noul"])
        progress = float(answers["progress"]["score"])
        if self.front_range < 1.5 and (facing > 0.5 or progress >= 2.0):
            self.chair_flag = True
        return answers, facing, progress

    def _arrive(self, now, facing, progress):
        """目標の椅子の正面に到着したと判定し、ロボットを完全停止させてタスクを終了する。"""
        self.command = (0.0, 0.0)
        self._publish(0.0, 0.0)
        self.done = True
        self.hold_until = now + Duration(seconds=0.4)
        self.get_logger().info(
            "arrived in front of the chair range=%.2f sittable=%.2f progress=%.2f"
            % (self.front_range, facing, progress)
        )

    def _near_obstacle(self, now):
        """前方に障害物があるときの近接処理。

        TurtleBot3 の LiDAR は車体中央にあり距離 0.42 m 以内で前進を続けると椅子の脚に乗り上げる。
        目の前の障害物が椅子（フラグ済み、または facing > 0.4）であれば到着として停止し、
        木箱や壁などの場合はその場で旋回して回避する。
        """
        answers, facing, progress = self._query_d1()
        if self.chair_flag or facing > 0.4:
            self._arrive(now, facing, progress)
            return

        name = answers["maneuver"]["choice"]
        _, angular = MANEUVERS.get(name, (0.0, 0.4))
        if abs(angular) < 0.1:
            angular = 0.4
        self.command = (0.0, angular)
        self._publish(0.0, angular)
        self.history.append("near_turn")
        self.hold_until = now + Duration(seconds=0.4)
        self.get_logger().info(
            "too close to drive range=%.2f sittable=%.2f turn=%.2f" % (self.front_range, facing, angular)
        )

    def tick(self):
        """0.2 秒ごとに呼ばれるメイン制御ループ。"""
        now = self.get_clock().now()
        # 行動保持期間中や完了後も、Gazebo の diff_drive プラグイン向けに現在の速度を再送し続ける
        self._publish(*self.command)
        if self.done or now < self.hold_until:
            return
        if self.front is None:
            if not self.announced_wait:
                self.get_logger().info("waiting for /camera/image_raw")
                self.announced_wait = True
            return

        # すでに椅子を捕捉した状態で前方 0.5 m 未満まで近づいたら、脚に接触する前に到着完了とする
        if self.chair_flag and self.front_range < 0.5:
            self._arrive(now, 0.0, 0.0)
            return
        # 前方 0.42 m 未満に障害物がある場合は、乗り上げ防止のため近接判定に切り替え
        if self.front_range < 0.42:
            self._near_obstacle(now)
            return

        answers, facing, progress = self._query_d1()
        linear, angular, done, label = decide(answers)
        # 椅子を捕捉済みのまま 0.55 m 未満まで接近し、近すぎて画角から外れた場合も到着とみなす
        if self.chair_flag and label == "search" and self.front_range < 0.55:
            self._arrive(now, facing, progress)
            return

        self.command = (linear, angular)
        self._publish(linear, angular)
        self.done = done
        self.history.append(label)
        self.hold_until = now + Duration(seconds=0.4)
        self.get_logger().info(
            "label=%s cmd=(%.2f, %.2f) sittable=%.2f crush=%.2f progress=%.2f confidence=%.2f range=%.2f"
            % (
                label,
                linear,
                angular,
                facing,
                float(answers["crush_risk"]["noul"]),
                progress,
                float(answers["maneuver"]["confidence"]),
                self.front_range,
            )
        )
        if done:
            self.get_logger().info("mission complete, stopped")


def main():
    model = load_model()
    rclpy.init()
    node = Driver(model)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
